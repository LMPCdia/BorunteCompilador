"""
Simulador de E/S digital para la VM de referencia. (El nombre viene de cuando
la E/S iba a estar en un PLC; hoy representa las E/S del robot.) Esto le permite a un test "apretar un botón" (set_input) mientras
la VM está corriendo, y luego revisar qué salidas activó el programa.
"""

from __future__ import annotations

import threading
import time


class PlcIoSimulator:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._inputs: dict[int, bool] = {}
        self._outputs: dict[int, bool] = {}
        self.output_log: list[tuple[float, int, bool]] = []  # (timestamp, número, estado)

    def set_input(self, number: int, state: bool) -> None:
        with self._lock:
            self._inputs[number] = state

    def read_input(self, number: int) -> bool:
        with self._lock:
            return self._inputs.get(number, False)

    def set_output(self, number: int, state: bool) -> None:
        with self._lock:
            self._outputs[number] = state
            self.output_log.append((time.monotonic(), number, state))

    def read_output(self, number: int) -> bool:
        with self._lock:
            return self._outputs.get(number, False)

    def wait_input(self, number: int, timeout_s: float, poll_interval_s: float = 0.01) -> bool:
        """Devuelve True si la entrada se activó dentro del timeout."""
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if self.read_input(number):
                return True
            time.sleep(poll_interval_s)
        return False
