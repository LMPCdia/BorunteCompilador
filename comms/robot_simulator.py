"""
Simulador del robot Borunte.

IMPORTANTE: esto implementa una HIPÓTESIS de comportamiento, no el
comportamiento real confirmado. Específicamente asume que:

  1. Escribir una pose en el bloque 800-890 (0x558C+) + escribir el comando
     "Activar botón Start" (0x4E23) dispara un movimiento hacia esa pose.
  2. Durante el movimiento, 0x09A6 (estado de movimiento) vale 1; al llegar,
     vuelve a 0.
  3. El comando "Detener acción actual" (0x4E20) frena el movimiento en seco.

Estos tres puntos están marcados como "no confirmado" en
docs/MODBUS_REGISTER_MAP.md. Este simulador existe para poder desarrollar y
probar compiler/ + runtime/ + gui/ SIN hardware. Cuando llegue el robot real,
hay que:
  (a) confirmar/corregir estos supuestos contra el hardware,
  (b) actualizar este simulador para que refleje la realidad,
  (c) re-correr toda la batería de tests — si algo asumido acá estaba mal,
      el problema se detecta en un solo lugar (este archivo), no desparramado
      por todo el compiler/VM.
"""

from __future__ import annotations

import threading
import time

from comms.fake_modbus import FakeModbusClient
from comms.robot_client import (
    ADDR_AXIS_POSITION_BASE,
    ADDR_CMD_CLEAR_ALARM_CONTINUE,
    ADDR_CMD_START_BUTTON,
    ADDR_CMD_STOP,
    ADDR_MOVEMENT_STATUS,
    ADDR_OPEN_VAR_BASE,
    ADDR_WORLD_POSITION_BASE,
    Pose,
)


class SimulatedBorunteRobot(FakeModbusClient):
    def __init__(self, move_duration_s: float = 0.3, initial_pose: Pose | None = None) -> None:
        super().__init__()
        self._lock = threading.Lock()
        self._move_duration_s = move_duration_s
        self._move_thread: threading.Thread | None = None
        self._stop_requested = threading.Event()

        pose = initial_pose or Pose(0, 0, 0, 0, 0, 0)
        self._set_world_pose(pose)
        self._set_axis_pose(pose)  # simplificación: en el simulador, ejes == mundo
        self.registers[ADDR_MOVEMENT_STATUS] = 0

    # -- reacciona a escrituras (ver supuestos en el docstring del módulo) ----

    def _on_write(self, address: int, values: list[int]) -> None:
        if address == ADDR_CMD_START_BUTTON:
            self._start_move_to_pending_target()
        elif address == ADDR_CMD_STOP:
            self._stop_requested.set()
        elif address == ADDR_CMD_CLEAR_ALARM_CONTINUE:
            pass  # sin modelo de alarmas todavía en el simulador

    # -- movimiento simulado ---------------------------------------------------

    def _read_target_pose(self) -> Pose:
        """Lee la última pose escrita en el bloque abierto 800-890 (slot 0)."""
        base = ADDR_OPEN_VAR_BASE
        raw = [self.registers.get(base + i, 0) for i in range(12)]
        values = []
        for i in range(0, len(raw), 2):
            hi, lo = raw[i], raw[i + 1]
            v = (hi << 16) | lo
            if v & 0x80000000:
                v -= 1 << 32
            values.append(v)
        return Pose.from_scaled_ints(values)

    def _start_move_to_pending_target(self) -> None:
        target = self._read_target_pose()
        self._stop_requested.clear()
        if self._move_thread and self._move_thread.is_alive():
            return  # ya se está moviendo; ignorar (comportamiento a confirmar)
        self._move_thread = threading.Thread(target=self._run_move, args=(target,), daemon=True)
        self.registers[ADDR_MOVEMENT_STATUS] = 1
        self._move_thread.start()

    def _run_move(self, target: Pose) -> None:
        start = self._get_world_pose()
        steps = max(1, int(self._move_duration_s / 0.02))
        for i in range(1, steps + 1):
            if self._stop_requested.is_set():
                self.registers[ADDR_MOVEMENT_STATUS] = 0
                return
            t = i / steps
            interp = Pose(*(s + (e - s) * t for s, e in zip(
                (start.a, start.b, start.c, start.d, start.e, start.f),
                (target.a, target.b, target.c, target.d, target.e, target.f),
            )))
            self._set_world_pose(interp)
            self._set_axis_pose(interp)
            time.sleep(self._move_duration_s / steps)
        self.registers[ADDR_MOVEMENT_STATUS] = 0

    # -- helpers de lectura/escritura directa de pose --------------------------

    def _set_world_pose(self, pose: Pose) -> None:
        self._write_pose_block(ADDR_WORLD_POSITION_BASE, pose)

    def _set_axis_pose(self, pose: Pose) -> None:
        self._write_pose_block(ADDR_AXIS_POSITION_BASE, pose)

    def _get_world_pose(self) -> Pose:
        raw = [self.registers.get(ADDR_WORLD_POSITION_BASE + i, 0) for i in range(12)]
        values = []
        for i in range(0, len(raw), 2):
            hi, lo = raw[i], raw[i + 1]
            v = (hi << 16) | lo
            if v & 0x80000000:
                v -= 1 << 32
            values.append(v)
        return Pose.from_scaled_ints(values)

    def _write_pose_block(self, base_address: int, pose: Pose) -> None:
        for i, v in enumerate(pose.to_scaled_ints()):
            v &= 0xFFFFFFFF
            self.registers[base_address + i * 2] = (v >> 16) & 0xFFFF
            self.registers[base_address + i * 2 + 1] = v & 0xFFFF
