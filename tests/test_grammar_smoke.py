"""
Smoke test: confirma que la gramática v0.1 al menos parsea sin explotar.
Esto NO valida semántica ni codegen — solo que la sintaxis está bien formada.

Requiere: pip install lark
"""

from pathlib import Path

import pytest
from lark import Lark

GRAMMAR_PATH = Path(__file__).parent.parent / "compiler" / "grammar.lark"

EXAMPLE_PROGRAM = """POINT p_home = WORLD(-255.639, 900.000, 855.380, -157.418, 6.509, -28.706)
POINT p_entrada = p_home + OFFSET(0, 198.679, -155.380, 0, 0, 0)

VAR offset_z : REAL = 0.0
VAR pieza : INT

TIMER t_espera = 10.0s

PROC soldar_pieza(pieza)
WAIT_IN(X010, 10)
MOVEJ p_home SPEED 80
MOVEL p_entrada SPEED 50
IF pieza == 1 THEN
SET_OUT(Y10, ON)
ELSE
SET_OUT(Y11, ON)
ENDIF
WAIT UNTIL MOVE_DONE
SET_OUT(Y10, OFF)
MOVEJ p_home SPEED 80
ENDPROC
"""


@pytest.fixture(scope="module")
def parser() -> Lark:
    grammar_text = GRAMMAR_PATH.read_text(encoding="utf-8")
    return Lark(grammar_text, parser="lalr")


def test_grammar_loads(parser: Lark) -> None:
    assert parser is not None


def test_example_program_parses(parser: Lark) -> None:
    tree = parser.parse(EXAMPLE_PROGRAM)
    assert tree is not None


def test_joint_points_parse_and_keep_their_frame():
    from compiler.ast_builder import build_ast

    ast = build_ast("POINT casa = JOINT(0, 45, -45, 0, -75, 0)\nPOINT p = WORLD(1, 2, 3, 0, 0, 0)\n")
    assert [s.expr.frame for s in ast.statements] == ["JOINT", "WORLD"]
