"""
Poll del estado de la VM del PLC en un hilo aparte.

Va en un hilo y no en el de la UI por la misma razón que `gui/vm_worker.py`:
cada `read_status()` es una transacción Modbus sobre la red, y con hardware real
eso puede tardar. Un `QTimer` en el hilo de la UI congelaría la ventana en cada
lectura lenta, y con un PLC que no responde la dejaría colgada del todo.

**El hilo vive una sola vez por conexión y se PAUSA, no se destruye.** Esto no es
una optimización, es corrección: el panel necesita el socket para su cuenta cada
vez que manda un comando o carga el programa, y la primera versión daba de baja
el hilo en cada una de esas operaciones. Dos problemas con eso:

1. Las señales ya encoladas hacia el hilo de la UI se entregaban **después** de
   que el worker fuera recolectado, y Qt terminaba tocando un objeto C++
   destruido. Eso crashea el proceso, no levanta una excepción.
2. Levantar y bajar un `QThread` por cada click es caro y llena el log de Qt.

Con pausa hay un solo punto de sincronización que importa: el panel no puede
tocar el socket hasta que el worker confirme que **no está en el medio de una
transacción**. Para eso está `is_idle()`.
"""

from __future__ import annotations

from PySide6.QtCore import QObject, QThread, Signal

from comms.plc_client import CoolmayPlcClient

# Después de tres fallos seguidos se corta. Uno aislado puede ser un timeout
# puntual; tres son un problema de verdad, y no tiene sentido seguir golpeando la
# red ni llenar la ventana de mensajes con la misma línea repetida.
MAX_CONSECUTIVE_ERRORS = 3


class PlcStatusWorker(QObject):
    """Lee estado + banco de variables en loop, hasta que se le pide parar."""

    status_read = Signal(object, object)  # (VmStatus, list[int] de variables)
    failed = Signal(str)
    stopped = Signal()

    def __init__(
        self,
        plc: CoolmayPlcClient,
        interval_ms: int = 200,
        variable_count: int = 16,
    ) -> None:
        """
        `variable_count`: cuántas variables del banco leer. No se lee el banco
        completo de 100 por default: son 100 registros más por vuelta y el panel
        muestra las primeras. Un programa compilado casi nunca usa más.
        """
        super().__init__()
        self.plc = plc
        self.interval_ms = interval_ms
        self.variable_count = variable_count
        # Todos flags booleanos, escritos por un hilo y leídos por el otro. No
        # hace falta lock: en CPython una asignación a un atributo es atómica y
        # acá no hay invariantes entre dos flags que haya que mantener juntas.
        self._running = False
        self._paused = False
        self._in_transaction = False
        self._consecutive_errors = 0

    # -- control desde el hilo de la UI ------------------------------------------

    def request_stop(self) -> None:
        self._running = False

    def set_paused(self, paused: bool) -> None:
        self._paused = paused

    def is_idle(self) -> bool:
        """True si el worker no va a tocar el socket: está pausado y fuera de
        transacción. Es la condición que el panel tiene que esperar antes de
        hablarle al PLC por su cuenta."""
        return self._paused and not self._in_transaction

    def is_running(self) -> bool:
        return self._running

    # -- el loop -------------------------------------------------------------------

    def run(self) -> None:
        self._running = True
        self._consecutive_errors = 0

        while self._running:
            if self._paused:
                QThread.msleep(20)
                continue

            self._in_transaction = True
            try:
                status = self.plc.read_status()
                variables = self.plc.read_variables(self.variable_count)
            except Exception as e:  # noqa: BLE001
                self._consecutive_errors += 1
                if self._consecutive_errors >= MAX_CONSECUTIVE_ERRORS:
                    self._in_transaction = False
                    self.failed.emit(f"Se perdió la comunicación con el PLC: {e}")
                    break
            else:
                self._consecutive_errors = 0
                self.status_read.emit(status, variables)
            finally:
                self._in_transaction = False

            # msleep de Qt: es el sleep del hilo actual y no depende de time.
            QThread.msleep(self.interval_ms)

        self._running = False
        self.stopped.emit()
