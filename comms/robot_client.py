"""
Cliente Modbus TCP para el robot Borunte.

Implementa SOLO lo confirmado en el manual "伯朗特Modbus_TCP通信协议说明书V1_0"
y "伯朗特驱控一体MODBUS_通讯协议解析". Ver docs/MODBUS_REGISTER_MAP.md para el
detalle de cada dirección.

Requiere: pip install pymodbus
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from enum import IntEnum

from pymodbus.client import ModbusTcpClient


# --------------------------------------------------------------------------
# Direcciones confirmadas (ver docs/MODBUS_REGISTER_MAP.md)
# --------------------------------------------------------------------------

ADDR_MOVEMENT_STATUS = 2470          # 0x09A6 — 0=parado, 1=en movimiento
ADDR_AXIS_COUNT = 2267               # 0x08DB
ADDR_AXIS_POSITION_BASE = 2268        # 0x08DC
ADDR_WORLD_POSITION_BASE = 2347      # 0x091C
ADDR_ALARM_CODE = 2396               # 0x095C

ADDR_CMD_STOP = 20000                # 0x4E20
ADDR_CMD_PAUSE = 20001               # 0x4E21
ADDR_CMD_SINGLE_LOOP = 20002         # 0x4E22
ADDR_CMD_START_BUTTON = 20003        # 0x4E23  (comportamiento a confirmar)
ADDR_CMD_STOP_BUTTON = 20004         # 0x4E24
ADDR_CMD_CLEAR_ALARM_NEXT = 20005    # 0x4E25
ADDR_CMD_CLEAR_ALARM_CONTINUE = 20006  # 0x4E26
ADDR_GLOBAL_SPEED = 20200            # 0x4EE8

ADDR_OPEN_VAR_BASE = 21900           # 0x558C == allpara[800]
OPEN_VAR_SLOT_START = 800
OPEN_VAR_SLOT_END = 900              # exclusivo, rango usable documentado

# Fixed-point: todas las posiciones vienen/van escaladas x1000 (3 decimales)
POSITION_SCALE = 1000


class MovementStatus(IntEnum):
    STOPPED = 0
    MOVING = 1


@dataclass
class Pose:
    """6 valores de pose — ejes (grados) o mundo (mm), según el contexto."""
    a: float
    b: float
    c: float
    d: float
    e: float
    f: float

    def to_scaled_ints(self) -> list[int]:
        return [round(v * POSITION_SCALE) for v in (self.a, self.b, self.c, self.d, self.e, self.f)]

    @classmethod
    def from_scaled_ints(cls, values: list[int]) -> "Pose":
        return cls(*(v / POSITION_SCALE for v in values))


class BorunteModbusError(Exception):
    pass


class BorunteRobotClient:
    """
    Wrapper de alto nivel sobre pymodbus para hablar con el Borunte.

    El robot expone Modbus TCP en el puerto 502 (según manual, "Modbus" en
    el selector de puerto del pad). Unit id confirmado en los ejemplos: 1.
    """

    def __init__(
        self,
        host: str,
        port: int = 502,
        unit_id: int = 1,
        timeout: float = 2.0,
        client: object | None = None,
    ):
        """
        `client`: opcional. Si se pasa (por ejemplo un SimulatedBorunteRobot
        de comms/robot_simulator.py), se usa en vez de crear un ModbusTcpClient
        real. Esto es lo que permite testear toda la lógica de arriba sin
        hardware — ver docs/SIMULATION.md.
        """
        self.host = host
        self.port = port
        self.unit_id = unit_id
        self.client = client if client is not None else ModbusTcpClient(host, port=port, timeout=timeout)

    # -- conexión ----------------------------------------------------------

    def connect(self) -> bool:
        return self.client.connect()

    def close(self) -> None:
        self.client.close()

    def __enter__(self) -> "BorunteRobotClient":
        if not self.connect():
            raise BorunteModbusError(f"No se pudo conectar a {self.host}:{self.port}")
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    # -- lectura de telemetría ----------------------------------------------

    def read_movement_status(self) -> MovementStatus:
        rr = self.client.read_holding_registers(ADDR_MOVEMENT_STATUS, count=1, slave=self.unit_id)
        self._check(rr)
        return MovementStatus(rr.registers[0])

    def read_axis_count(self) -> int:
        rr = self.client.read_holding_registers(ADDR_AXIS_COUNT, count=1, slave=self.unit_id)
        self._check(rr)
        return rr.registers[0]

    def read_world_position(self) -> Pose:
        """Posición cartesiana actual (X,Y,Z,U,V,W) en mm/grados, en vivo."""
        rr = self.client.read_holding_registers(ADDR_WORLD_POSITION_BASE, count=12, slave=self.unit_id)
        self._check(rr)
        values = self._registers_to_int32_list(rr.registers)
        return Pose.from_scaled_ints(values)

    def read_axis_position(self) -> Pose:
        """Posición articular actual (J1..J6) en grados, en vivo."""
        rr = self.client.read_holding_registers(ADDR_AXIS_POSITION_BASE, count=12, slave=self.unit_id)
        self._check(rr)
        values = self._registers_to_int32_list(rr.registers)
        return Pose.from_scaled_ints(values)

    def read_alarm_code(self) -> int:
        rr = self.client.read_holding_registers(ADDR_ALARM_CODE, count=1, slave=self.unit_id)
        self._check(rr)
        return rr.registers[0]

    # -- comandos tipo "botón remoto" ---------------------------------------

    def cmd_stop(self) -> None:
        self._write_command(ADDR_CMD_STOP)

    def cmd_pause(self) -> None:
        self._write_command(ADDR_CMD_PAUSE)

    def cmd_start_button(self) -> None:
        # ADVERTENCIA: comportamiento no confirmado contra hardware real.
        # Ver docs/MODBUS_REGISTER_MAP.md.
        self._write_command(ADDR_CMD_START_BUTTON)

    def cmd_clear_alarm_and_continue(self) -> None:
        self._write_command(ADDR_CMD_CLEAR_ALARM_CONTINUE)

    def set_global_speed_percent(self, percent: float) -> None:
        value = round(max(0.0, min(100.0, percent)) * 10)  # 0-1000 = 0.0-100.0%
        rq = self.client.write_register(ADDR_GLOBAL_SPEED, value, slave=self.unit_id)
        self._check(rq)

    # -- envío de posición (bloque 800-890) ----------------------------------

    def send_target_pose(self, pose: Pose, slot: int = 0) -> None:
        """
        Escribe una pose en el bloque de variables abiertas (800-890).

        `slot` es un offset dentro del rango abierto (0 = allpara[800]).
        NO ejecuta el movimiento por sí solo — según el manual hace falta
        además seleccionar la referencia de trayectoria (free path / posture
        line). Ese paso está pendiente de confirmar si es 100% remoto
        (ver docs/ARCHITECTURE.md, sección "pendiente de validar").
        """
        if not (0 <= slot < (OPEN_VAR_SLOT_END - OPEN_VAR_SLOT_START)):
            raise ValueError(f"slot fuera de rango abierto 800-900: {slot}")

        address = ADDR_OPEN_VAR_BASE + slot * 2  # cada allpara[] = 2 registros (32 bits)
        registers = self._int32_list_to_registers(pose.to_scaled_ints())
        rq = self.client.write_registers(address, registers, slave=self.unit_id)
        self._check(rq)

    def wait_until_stopped(self, poll_interval_s: float = 0.05, timeout_s: float = 30.0) -> None:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if self.read_movement_status() == MovementStatus.STOPPED:
                return
            time.sleep(poll_interval_s)
        raise BorunteModbusError("Timeout esperando que el robot termine el movimiento")

    # -- helpers internos -----------------------------------------------------

    @staticmethod
    def _registers_to_int32_list(registers: list[int]) -> list[int]:
        """Convierte pares de registros (hi, lo) en una lista de enteros de 32 bits."""
        out = []
        for i in range(0, len(registers), 2):
            hi, lo = registers[i], registers[i + 1]
            value = (hi << 16) | lo
            if value & 0x80000000:
                value -= 1 << 32
            out.append(value)
        return out

    @staticmethod
    def _int32_list_to_registers(values: list[int]) -> list[int]:
        out = []
        for v in values:
            v &= 0xFFFFFFFF
            out.append((v >> 16) & 0xFFFF)
            out.append(v & 0xFFFF)
        return out

    def _write_command(self, address: int, value: int = 1) -> None:
        rq = self.client.write_register(address, value, slave=self.unit_id)
        self._check(rq)

    @staticmethod
    def _check(response) -> None:
        if response.isError():
            raise BorunteModbusError(str(response))


if __name__ == "__main__":
    # Ejemplo mínimo — ajustar IP antes de correr contra hardware real.
    with BorunteRobotClient(host="192.168.1.10") as robot:
        print("Ejes en uso:", robot.read_axis_count())
        print("Posición mundo actual:", robot.read_world_position())
        print("Posición articular actual:", robot.read_axis_position())
        print("Estado de movimiento:", robot.read_movement_status())
