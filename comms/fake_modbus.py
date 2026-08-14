"""
Implementación mínima en memoria de la interfaz de pymodbus que usa
BorunteRobotClient (read_holding_registers / write_register / write_registers
/ connect / close). No abre sockets — todo vive en un dict de registros.

Sirve de base para comms/robot_simulator.py, que le agrega comportamiento
(simular movimiento, etc.) por encima de este almacenamiento.
"""

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class _Response:
    registers: list[int] = field(default_factory=list)
    _error: bool = False

    def isError(self) -> bool:  # noqa: N802 (mantiene el nombre de pymodbus)
        return self._error


class FakeModbusClient:
    """Reemplazo drop-in de ModbusTcpClient para tests, sin red real."""

    def __init__(self) -> None:
        # Modbus holding registers: dirección (int) -> valor de 16 bits (0-65535)
        self.registers: dict[int, int] = {}

    def connect(self) -> bool:
        return True

    def close(self) -> None:
        pass

    # -- API compatible con pymodbus ------------------------------------------

    def read_holding_registers(self, address: int, count: int = 1, slave: int = 1) -> _Response:
        values = [self.registers.get(address + i, 0) for i in range(count)]
        return _Response(registers=values)

    def write_register(self, address: int, value: int, slave: int = 1) -> _Response:
        self.registers[address] = value & 0xFFFF
        self._on_write(address, [value & 0xFFFF])
        return _Response()

    def write_registers(self, address: int, values: list[int], slave: int = 1) -> _Response:
        for i, v in enumerate(values):
            self.registers[address + i] = v & 0xFFFF
        self._on_write(address, values)
        return _Response()

    # -- hook para subclases (ej. el simulador de robot) -----------------------

    def _on_write(self, address: int, values: list[int]) -> None:
        """Las subclases sobreescriben esto para reaccionar a escrituras
        (ej. disparar un movimiento simulado). No-op acá."""
