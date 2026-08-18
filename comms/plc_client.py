"""
Cliente Modbus TCP para el PLC Coolmay CX3G — el PLC en sí, no el robot.

Espejo de `comms/robot_client.py` pero del otro lado del sistema de 2 niveles
(ver `docs/ARCHITECTURE.md`): acá se carga el bytecode compilado en los
registros `D` del PLC, se arranca/para la VM que corre en ladder, y se lee su
estado en vivo (Program Counter, banco de variables).

⚠️  TODO ESTE MÓDULO ES HIPÓTESIS SIN CONFIRMAR. A diferencia de
`robot_client.py`, que implementa direcciones tomadas de los manuales del
Borunte, acá no hay manual: el mapa de registros lo definimos nosotros en
`docs/INSTRUCTION_SET.md` y **nadie lo implementó todavía en ladder**. Lo que
falta validar con el CX3G delante, en orden de riesgo:

1. **Que el registro `Dn` del PLC se lea/escriba como holding register Modbus
   número `n`.** Está parametrizado en `d_register_base` para poder corregirlo
   en un solo lugar si el CX3G los expone con un offset.
2. **Que los enteros de 32 bits se guarden con el word bajo primero**
   (convención FX de Mitsubishi, que es lo que hereda el Coolmay). Ojo que es
   **al revés que el robot Borunte**, donde el ejemplo confirmado del manual
   manda el word alto primero. Las dos convenciones conviven a propósito, cada
   una del lado que le corresponde — no unificar sin confirmar las dos.
   **Esta es la que más conviene validar primero**, porque si está al revés
   todo carga "bien" y ejecuta cualquier cosa.
3. Que el mapa de control (`D0-D109`) y las bases de las tablas (`D1000`,
   `D4000`) sean los del contrato.
4. Que el handshake de comandos funcione como está documentado: el host
   escribe en `D3` y la VM lo devuelve a `0` al aceptarlo.

Requiere: pip install pymodbus
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from enum import IntEnum

from pymodbus.client import ModbusTcpClient

from comms.robot_client import Pose
from runtime.bytecode import OPCODE_NAMES, OPCODE_NUMBERS, Instruction, Program

# --------------------------------------------------------------------------
# Mapa de registros — ver "Registros de control de la VM" en
# docs/INSTRUCTION_SET.md. Todos los números son OFFSETS de registro D, no
# direcciones Modbus: la traducción pasa por CoolmayPlcClient._d().
# --------------------------------------------------------------------------

D_PC = 0                    # Program Counter actual
D_STATE = 1                 # Estado de la VM
D_ERROR = 2                 # Código de error
D_COMMAND = 3               # Comando del host (handshake)
D_INSTRUCTION_COUNT = 4     # Cantidad de instrucciones cargadas
D_POINT_COUNT = 5           # Cantidad de puntos cargados
D_CONTRACT_VERSION = 6      # Versión del contrato que implementa la VM
D_RESERVED = 7

CONTROL_BLOCK_SIZE = 8      # D0-D7, pensado para leerse en UNA transacción
CONTRACT_VERSION = 2        # la que describe docs/INSTRUCTION_SET.md v0.2

D_VAR_BASE = 10             # banco de variables de usuario
VAR_BANK_SIZE = 100         # D10-D109

D_BYTECODE_BASE = 1000      # tabla de bytecode
WORDS_PER_INSTRUCTION = 8
MAX_INSTRUCTIONS = 375      # D1000-D3999 / 8

D_POINTS_BASE = 4000        # tabla de puntos
WORDS_PER_POINT = 12        # 6 valores de 32 bits
MAX_POINTS = 333            # D4000-D7995 / 12


class PlcVmState(IntEnum):
    STOPPED = 0
    RUNNING = 1
    ERROR = 2
    PAUSED = 3
    FINISHED = 4


class PlcCommand(IntEnum):
    NONE = 0
    START = 1
    STOP = 2
    PAUSE = 3
    RESUME = 4
    RESET = 5


class PlcVmError(IntEnum):
    NONE = 0
    UNKNOWN_OPCODE = 1
    POINT_INDEX_OUT_OF_RANGE = 2
    VAR_INDEX_OUT_OF_RANGE = 3
    WAIT_IN_TIMEOUT = 4
    MOVE_TIMEOUT = 5
    ROBOT_ALARM = 6
    RET_WITHOUT_CALL = 7
    CALL_STACK_OVERFLOW = 8


@dataclass
class VmStatus:
    """Foto del bloque de control, leída en una sola transacción."""
    pc: int
    state: PlcVmState
    error: int
    command: int
    instruction_count: int
    point_count: int
    contract_version: int

    @property
    def is_running(self) -> bool:
        return self.state == PlcVmState.RUNNING

    @property
    def error_name(self) -> str:
        try:
            return PlcVmError(self.error).name
        except ValueError:
            return f"DESCONOCIDO({self.error})"


class CoolmayModbusError(Exception):
    pass


class CoolmayPlcClient:
    """
    Wrapper de alto nivel sobre pymodbus para hablar con el PLC Coolmay CX3G.

    Igual que `BorunteRobotClient`, acepta un `client` inyectado para poder
    testear toda la lógica de arriba (codificación, troceado, handshake) sin
    hardware ni red — ver `tests/test_plc_client.py`.
    """

    def __init__(
        self,
        host: str,
        port: int = 502,
        unit_id: int = 1,
        timeout: float = 2.0,
        client: object | None = None,
        d_register_base: int = 0,
        max_registers_per_transaction: int = 100,
    ):
        """
        `d_register_base`: dirección Modbus del registro `D0`. El default 0
        asume que `Dn` es el holding register `n`, que es la hipótesis 1 del
        docstring del módulo. Si el CX3G los expone corridos, se corrige acá y
        todo el resto del módulo sigue andando.

        `max_registers_per_transaction`: tope de registros por lectura o
        escritura. Modbus limita una escritura a 123 registros por trama, y
        cargar un programa entero son miles — así que todas las
        lecturas/escrituras de tabla se trocean. 100 deja margen.
        """
        self.host = host
        self.port = port
        self.unit_id = unit_id
        self.d_register_base = d_register_base
        self.max_registers_per_transaction = max_registers_per_transaction
        self.client = (
            client if client is not None
            else ModbusTcpClient(host, port=port, timeout=timeout)
        )

    def _d(self, offset: int) -> int:
        """Offset de registro D -> dirección Modbus."""
        return self.d_register_base + offset

    # -- conexión ------------------------------------------------------------

    def connect(self) -> bool:
        return self.client.connect()

    def close(self) -> None:
        self.client.close()

    def __enter__(self) -> "CoolmayPlcClient":
        if not self.connect():
            raise CoolmayModbusError(f"No se pudo conectar a {self.host}:{self.port}")
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    # -- estado de la VM -----------------------------------------------------

    def read_status(self) -> VmStatus:
        """Lee todo el bloque de control en UNA transacción.

        Importa que sea una sola: si se leyera registro por registro, el PC y
        el estado podrían venir de momentos distintos y mostrar cosas
        imposibles (estado=parada con un PC que sigue avanzando).
        """
        regs = self._read(self._d(D_PC), CONTROL_BLOCK_SIZE)
        return VmStatus(
            pc=regs[D_PC],
            state=self._decode_state(regs[D_STATE]),
            error=regs[D_ERROR],
            command=regs[D_COMMAND],
            instruction_count=regs[D_INSTRUCTION_COUNT],
            point_count=regs[D_POINT_COUNT],
            contract_version=regs[D_CONTRACT_VERSION],
        )

    @staticmethod
    def _decode_state(value: int) -> PlcVmState:
        try:
            return PlcVmState(value)
        except ValueError as exc:
            raise CoolmayModbusError(
                f"Estado de VM desconocido en D{D_STATE}: {value}. "
                f"¿El ladder del PLC implementa el contrato v{CONTRACT_VERSION}?"
            ) from exc

    def read_pc(self) -> int:
        return self._read(self._d(D_PC), 1)[0]

    # -- comandos con handshake ----------------------------------------------

    def send_command(
        self,
        command: PlcCommand,
        wait_ack: bool = True,
        timeout_s: float = 1.0,
        poll_interval_s: float = 0.02,
    ) -> None:
        """Escribe el comando en `D3` y espera que la VM lo devuelva a `0`.

        Ese `0` es el único acuse que tenemos de que del otro lado hay un
        ladder que entiende el contrato. Si no llega, es mejor fallar que
        seguir a ciegas: sin ack no sabemos si el comando se ejecutó, se
        ignoró, o si el PLC está corriendo un programa completamente distinto.
        """
        self._write(self._d(D_COMMAND), [int(command)])
        if not wait_ack:
            return

        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if self._read(self._d(D_COMMAND), 1)[0] == PlcCommand.NONE:
                return
            time.sleep(poll_interval_s)
        raise CoolmayModbusError(
            f"La VM del PLC no acusó el comando {command.name} en {timeout_s}s "
            f"(D{D_COMMAND} sigue en {int(command)}). Revisar que el ladder esté "
            f"corriendo y que implemente el handshake del contrato."
        )

    def start(self, **kw) -> None:
        self.send_command(PlcCommand.START, **kw)

    def stop(self, **kw) -> None:
        self.send_command(PlcCommand.STOP, **kw)

    def pause(self, **kw) -> None:
        self.send_command(PlcCommand.PAUSE, **kw)

    def resume(self, **kw) -> None:
        self.send_command(PlcCommand.RESUME, **kw)

    def reset(self, **kw) -> None:
        self.send_command(PlcCommand.RESET, **kw)

    # -- banco de variables --------------------------------------------------

    def read_variable(self, index: int) -> int:
        self._check_var_index(index)
        return self._to_signed16(self._read(self._d(D_VAR_BASE + index), 1)[0])

    def write_variable(self, index: int, value: int) -> None:
        self._check_var_index(index)
        self._write(self._d(D_VAR_BASE + index), [int(value) & 0xFFFF])

    def read_variables(self, count: int = VAR_BANK_SIZE) -> list[int]:
        """Lee el banco completo (troceado si hace falta)."""
        if not (0 < count <= VAR_BANK_SIZE):
            raise ValueError(f"count fuera del banco de {VAR_BANK_SIZE}: {count}")
        regs = self._read(self._d(D_VAR_BASE), count)
        return [self._to_signed16(r) for r in regs]

    @staticmethod
    def _check_var_index(index: int) -> None:
        if not (0 <= index < VAR_BANK_SIZE):
            raise ValueError(
                f"Índice de variable fuera del banco de {VAR_BANK_SIZE}: {index}"
            )

    # -- carga del programa ---------------------------------------------------

    def upload_program(self, program: Program) -> None:
        """Carga bytecode + tabla de puntos en el PLC.

        Las tablas se escriben ANTES que los contadores (`D4`/`D5`) a
        propósito: así la VM nunca ve un contador que apunte a una tabla a
        medio escribir. Si la conexión se corta en el medio, quedan los
        contadores viejos y una tabla nueva incompleta — que es peor que
        inconsistente, pero al menos la VM no arranca creyendo que hay 300
        instrucciones válidas cuando se escribieron 12.
        """
        n_instr = len(program.instructions)
        n_points = len(program.points)
        if n_instr > MAX_INSTRUCTIONS:
            raise CoolmayModbusError(
                f"El programa tiene {n_instr} instrucciones y en el PLC caben "
                f"{MAX_INSTRUCTIONS} (D{D_BYTECODE_BASE}-D{D_BYTECODE_BASE + MAX_INSTRUCTIONS * WORDS_PER_INSTRUCTION - 1})"
            )
        if n_points > MAX_POINTS:
            raise CoolmayModbusError(
                f"El programa tiene {n_points} puntos y en el PLC caben "
                f"{MAX_POINTS}. Ojo que la tabla de puntos no se deduplica: "
                f"cada MOVEJ/MOVEL agrega una entrada aunque repita un POINT."
            )

        words: list[int] = []
        for instr in program.instructions:
            words.extend(self.encode_instruction(instr))
        if words:
            self._write_chunked(self._d(D_BYTECODE_BASE), words)

        point_words: list[int] = []
        for pose in program.points:
            point_words.extend(self.encode_point(pose))
        if point_words:
            self._write_chunked(self._d(D_POINTS_BASE), point_words)

        # recién ahora los contadores
        self._write(self._d(D_INSTRUCTION_COUNT), [n_instr])
        self._write(self._d(D_POINT_COUNT), [n_points])

    def read_back_program(self) -> tuple[list[Instruction], list[Pose]]:
        """Relee y decodifica lo que hay cargado en el PLC.

        Sirve para verificar la carga (`verify_program`) y, más importante,
        para chequear la hipótesis del orden de words: si el PLC guardara los
        32 bits al revés de lo que asumimos, la relectura devuelve valores
        absurdos y se nota enseguida.
        """
        status = self.read_status()
        instructions: list[Instruction] = []
        if status.instruction_count:
            words = self._read_chunked(
                self._d(D_BYTECODE_BASE),
                status.instruction_count * WORDS_PER_INSTRUCTION,
            )
            for i in range(status.instruction_count):
                chunk = words[i * WORDS_PER_INSTRUCTION:(i + 1) * WORDS_PER_INSTRUCTION]
                instructions.append(self.decode_instruction(chunk))

        points: list[Pose] = []
        if status.point_count:
            words = self._read_chunked(
                self._d(D_POINTS_BASE), status.point_count * WORDS_PER_POINT
            )
            for i in range(status.point_count):
                chunk = words[i * WORDS_PER_POINT:(i + 1) * WORDS_PER_POINT]
                points.append(self.decode_point(chunk))

        return instructions, points

    def verify_program(self, program: Program) -> None:
        """Relee el PLC y compara contra lo que se quiso cargar.

        Levanta `CoolmayModbusError` con el primer desvío encontrado. No
        devuelve bool a propósito: un `if not verify(...)` que nadie mira es
        exactamente el error que esto tiene que evitar.
        """
        instructions, points = self.read_back_program()

        if len(instructions) != len(program.instructions):
            raise CoolmayModbusError(
                f"Se cargaron {len(program.instructions)} instrucciones pero el PLC "
                f"reporta {len(instructions)}"
            )
        if len(points) != len(program.points):
            raise CoolmayModbusError(
                f"Se cargaron {len(program.points)} puntos pero el PLC reporta "
                f"{len(points)}"
            )

        for i, (want, got) in enumerate(zip(program.instructions, instructions)):
            if (want.opcode, want.a, want.b, want.c, want.d) != (
                got.opcode, got.a, got.b, got.c, got.d
            ):
                raise CoolmayModbusError(
                    f"La instrucción {i} volvió distinta del PLC: "
                    f"se escribió {want!r}, se leyó {got!r}"
                )

        for i, (want, got) in enumerate(zip(program.points, points)):
            if want.to_scaled_ints() != got.to_scaled_ints():
                raise CoolmayModbusError(
                    f"El punto {i} volvió distinto del PLC: "
                    f"se escribió {want}, se leyó {got}"
                )

    # -- codificación de la tabla de 8 words ----------------------------------

    @classmethod
    def encode_instruction(cls, instr: Instruction) -> list[int]:
        """Instrucción -> 8 words, según "Formato de instrucción" del contrato."""
        if instr.opcode not in OPCODE_NUMBERS:
            raise CoolmayModbusError(
                f"Opcode sin número asignado en el contrato: {instr.opcode!r}. "
                f"Agregalo a OPCODE_NUMBERS en runtime/bytecode.py."
            )
        return [
            OPCODE_NUMBERS[instr.opcode],       # word 0
            instr.a & 0xFFFF,                   # word 1
            *cls._int32_to_words(instr.b),      # words 2-3
            *cls._int32_to_words(instr.c),      # words 4-5
            instr.d & 0xFFFF,                   # word 6
            0,                                  # word 7 (reservado)
        ]

    @classmethod
    def decode_instruction(cls, words: list[int]) -> Instruction:
        if len(words) != WORDS_PER_INSTRUCTION:
            raise CoolmayModbusError(
                f"Una instrucción son {WORDS_PER_INSTRUCTION} words, llegaron {len(words)}"
            )
        opcode_num = words[0]
        if opcode_num not in OPCODE_NAMES:
            raise CoolmayModbusError(f"Opcode desconocido leído del PLC: {opcode_num:#04x}")
        return Instruction(
            opcode=OPCODE_NAMES[opcode_num],
            a=words[1],
            b=cls._words_to_int32(words[2], words[3]),
            c=cls._words_to_int32(words[4], words[5]),
            d=words[6],
        )

    @classmethod
    def encode_point(cls, pose: Pose) -> list[int]:
        """Pose -> 12 words (6 enteros de 32 bits escalados x1000)."""
        words: list[int] = []
        for value in pose.to_scaled_ints():
            words.extend(cls._int32_to_words(value))
        return words

    @classmethod
    def decode_point(cls, words: list[int]) -> Pose:
        if len(words) != WORDS_PER_POINT:
            raise CoolmayModbusError(
                f"Un punto son {WORDS_PER_POINT} words, llegaron {len(words)}"
            )
        values = [
            cls._words_to_int32(words[i], words[i + 1])
            for i in range(0, WORDS_PER_POINT, 2)
        ]
        return Pose.from_scaled_ints(values)

    # -- helpers de 32 bits ---------------------------------------------------
    #
    # WORD BAJO PRIMERO (convención FX de Mitsubishi). Es la hipótesis 2 del
    # docstring del módulo, y es al revés que _int32_list_to_registers() de
    # BorunteRobotClient, que manda el word alto primero porque así lo muestra
    # el ejemplo confirmado del manual del robot. Si alguna vez hay que
    # unificarlas, confirmar LAS DOS contra hardware antes de tocar nada.

    @staticmethod
    def _int32_to_words(value: int) -> list[int]:
        v = int(value) & 0xFFFFFFFF
        return [v & 0xFFFF, (v >> 16) & 0xFFFF]

    @staticmethod
    def _words_to_int32(low: int, high: int) -> int:
        value = ((high & 0xFFFF) << 16) | (low & 0xFFFF)
        if value & 0x80000000:
            value -= 1 << 32
        return value

    @staticmethod
    def _to_signed16(value: int) -> int:
        value &= 0xFFFF
        return value - 0x10000 if value & 0x8000 else value

    # -- transacciones Modbus (con troceado) ----------------------------------

    def _read(self, address: int, count: int) -> list[int]:
        rr = self.client.read_holding_registers(address, count=count, slave=self.unit_id)
        self._check(rr)
        return list(rr.registers)

    def _write(self, address: int, values: list[int]) -> None:
        rq = self.client.write_registers(address, list(values), slave=self.unit_id)
        self._check(rq)

    def _read_chunked(self, address: int, count: int) -> list[int]:
        out: list[int] = []
        step = self.max_registers_per_transaction
        for offset in range(0, count, step):
            out.extend(self._read(address + offset, min(step, count - offset)))
        return out

    def _write_chunked(self, address: int, values: list[int]) -> None:
        step = self.max_registers_per_transaction
        for offset in range(0, len(values), step):
            self._write(address + offset, values[offset:offset + step])

    @staticmethod
    def _check(response) -> None:
        if response.isError():
            raise CoolmayModbusError(str(response))


if __name__ == "__main__":  # pragma: no cover
    # Ejemplo mínimo — ajustar IP antes de correr contra hardware real.
    # OJO: nada de esto se probó contra un CX3G. Ver el docstring del módulo.
    from compiler.codegen import compile_source

    programa = compile_source("SET_OUT(Y10, ON)\nWAIT 1s\nSET_OUT(Y10, OFF)\n")
    with CoolmayPlcClient(host="192.168.1.20") as plc:
        print("Estado antes de cargar:", plc.read_status())
        plc.upload_program(programa)
        plc.verify_program(programa)
        print("Carga verificada. Arrancando...")
        plc.start()
        print("Estado:", plc.read_status())
