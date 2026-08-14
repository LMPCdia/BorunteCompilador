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
CALL = "CALL"
RET = "RET"
SET_VAR = "SET_VAR"
ADD_VAR = "ADD_VAR"
ALARM_CLEAR_CONTINUE = "ALARM_CLEAR_CONTINUE"
END = "END"


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

    def dump(self) -> str:  # pragma: no cover
        lines = [f"{i:4d}: {instr!r}" for i, instr in enumerate(self.instructions)]
        return "\n".join(lines)
