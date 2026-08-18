"""
Representación del bytecode en memoria (lado Python).

Esto es el equivalente "en objetos" de la tabla de 8 words por instrucción
descripta en docs/INSTRUCTION_SET.md. Cuando exista la VM real en el PLC,
va a hacer falta un serializador que convierta esto a la codificación de
registros D/R — pero para desarrollar y probar el compiler y la VM de
referencia en software, esta representación alcanza y sobra.
"""

from __future__ import annotations

from dataclasses import dataclass, field

from comms.robot_client import Pose

# Nombres de opcode — deben coincidir 1:1 con docs/INSTRUCTION_SET.md
NOP = "NOP"
MOVEJ = "MOVEJ"
MOVEL = "MOVEL"
WAIT_IN = "WAIT_IN"
SET_OUT = "SET_OUT"
WAIT_TIME = "WAIT_TIME"
JUMP = "JUMP"
JUMP_IF_ZERO = "JUMP_IF_ZERO"
JUMP_IF_VAR_NEQ_CONST = "JUMP_IF_VAR_NEQ_CONST"  # agregado al implementar codegen de IF — ver docs/INSTRUCTION_SET.md
JUMP_IF_VAR_NEQ_VAR = "JUMP_IF_VAR_NEQ_VAR"      # v0.2 del contrato — IF a == b
COPY_VAR = "COPY_VAR"                            # v0.2 del contrato — a = b y parámetros de PROC
CALL = "CALL"
RET = "RET"
SET_VAR = "SET_VAR"
ADD_VAR = "ADD_VAR"
COPY_POINT_OFFSET = "COPY_POINT_OFFSET"
ALARM_CLEAR_CONTINUE = "ALARM_CLEAR_CONTINUE"
END = "END"

# Códigos numéricos de la tabla de docs/INSTRUCTION_SET.md.
#
# Hasta la v0.2 estos números vivían SOLO en la documentación: el compiler y la
# VM de referencia trabajan con los nombres (strings), que es más legible para
# debug. Pero para escribir el bytecode en los registros D del PLC hace falta el
# número, así que la tabla tiene que existir en código. Si agregás un opcode al
# contrato, agregalo también acá — hay un test que verifica que las dos listas
# de opcodes coincidan.
OPCODE_NUMBERS: dict[str, int] = {
    NOP: 0x00,
    MOVEJ: 0x01,
    MOVEL: 0x02,
    WAIT_IN: 0x03,
    SET_OUT: 0x04,
    WAIT_TIME: 0x05,
    JUMP: 0x06,
    JUMP_IF_ZERO: 0x07,
    CALL: 0x08,
    RET: 0x09,
    SET_VAR: 0x0A,
    ADD_VAR: 0x0B,
    COPY_POINT_OFFSET: 0x0C,
    JUMP_IF_VAR_NEQ_CONST: 0x0D,
    JUMP_IF_VAR_NEQ_VAR: 0x0E,
    COPY_VAR: 0x0F,
    ALARM_CLEAR_CONTINUE: 0xFE,
    END: 0xFF,
}

OPCODE_NAMES: dict[int, str] = {num: name for name, num in OPCODE_NUMBERS.items()}


@dataclass
class Instruction:
    opcode: str
    a: int = 0
    b: int = 0
    c: int = 0
    d: int = 0
    # metadata solo para debug/legibilidad (no existe en el PLC real)
    comment: str = ""

    def __repr__(self) -> str:  # pragma: no cover
        base = f"{self.opcode} a={self.a} b={self.b} c={self.c} d={self.d}"
        return f"{base}  ; {self.comment}" if self.comment else base


@dataclass
class Program:
    instructions: list[Instruction] = field(default_factory=list)
    points: list[Pose] = field(default_factory=list)          # tabla de puntos, indexada
    point_names: dict[str, int] = field(default_factory=dict)  # nombre -> índice, solo debug
    proc_addresses: dict[str, int] = field(default_factory=dict)  # nombre de PROC -> PC
    var_names: dict[str, int] = field(default_factory=dict)    # nombre de VAR -> índice
    # nombre de PROC -> nombres de sus parámetros, en orden. Los slots en
    # var_names se llaman "<proc>.<param>" (ver "Calling convention de PROC"
    # en docs/INSTRUCTION_SET.md).
    proc_params: dict[str, list[str]] = field(default_factory=dict)

    def dump(self) -> str:  # pragma: no cover
        lines = [f"{i:4d}: {instr!r}" for i, instr in enumerate(self.instructions)]
        return "\n".join(lines)
