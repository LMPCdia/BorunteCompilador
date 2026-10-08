"""
Resaltado de sintaxis del DSL sobre el editor.

Las listas de palabras están derivadas A MANO de `compiler/grammar.lark`. No se
leen de la gramática porque Lark no expone los literales de una forma estable
que valga la pena parsear para esto. Consecuencia: **si agregás una keyword a
la gramática, agregala también acá** — hay un test que verifica que las
keywords resaltadas existan en la gramática, pero no puede detectar lo que
falta (una keyword nueva sin resaltar no rompe nada, solo se ve gris).

Sobre los colores: hay DOS paletas, y se elige según el color real del fondo
del editor. La primera versión estaba pensada para fondo blanco y sobre el tema
oscuro de Windows las keywords en azul marino quedaban ilegibles. Cada paleta
declara el fondo contra el que se diseñó, y `tests/test_gui_smoke.py` verifica
que todos sus colores tengan contraste mínimo 3.0:1 (WCAG AA para texto grande)
contra ese fondo.
"""

from __future__ import annotations

import re
from dataclasses import dataclass, field

from PySide6.QtCore import QRegularExpression
from PySide6.QtGui import (
    QColor,
    QFont,
    QPalette,
    QSyntaxHighlighter,
    QTextCharFormat,
    QTextDocument,
)

# --- vocabulario (derivado de compiler/grammar.lark) ------------------------

KEYWORDS = [
    "POINT", "VAR", "TIMER", "PROC", "ENDPROC",
    "IF", "THEN", "ELSE", "ENDIF",
    "WAIT_IN", "WAIT", "UNTIL", "MOVE_DONE", "SET_OUT", "SPEED", "TOOL", "COORD",
]
MOVE_KINDS = ["MOVEJ", "MOVEL"]
POINT_FUNCTIONS = ["WORLD", "JOINT", "OFFSET"]
TYPES = ["INT", "REAL", "BOOL"]
STATES = ["ON", "OFF"]

# Nombres de E/S: X010 (entrada), Y034 (salida), M100 (marca interna).
IO_PATTERN = r"\b[XYM][0-9]+\b"
NUMBER_PATTERN = r"\b-?[0-9]+(\.[0-9]+)?\b"
COMMENT_PATTERN = r";[^\n]*"


@dataclass
class HighlightPalette:
    """Una paleta y el fondo contra el que se diseñó.

    `reference_background` no es decorativo: es lo que hace verificable el
    contraste en un test. Sin él no habría contra qué medir.
    """
    name: str
    reference_background: str
    keyword: str
    move: str
    point_function: str
    type_: str
    state: str
    io: str
    number: str
    comment: str

    def colors(self) -> dict[str, str]:
        return {
            "keyword": self.keyword,
            "move": self.move,
            "point_function": self.point_function,
            "type": self.type_,
            "state": self.state,
            "io": self.io,
            "number": self.number,
            "comment": self.comment,
        }


LIGHT_PALETTE = HighlightPalette(
    name="claro",
    reference_background="#ffffff",
    keyword="#7f0055",
    move="#0033b3",
    point_function="#00627a",
    type_="#795e26",
    state="#116644",
    io="#a31515",
    number="#0d7040",
    comment="#5f6b76",
)

DARK_PALETTE = HighlightPalette(
    name="oscuro",
    reference_background="#1e1e1e",
    keyword="#c586c0",
    move="#569cd6",
    point_function="#4ec9b0",
    type_="#dcdcaa",
    state="#7ec699",
    io="#ce9178",
    number="#b5cea8",
    comment="#6a9955",
)

ALL_PALETTES = [LIGHT_PALETTE, DARK_PALETTE]


def relative_luminance(color: str) -> float:
    """Luminancia relativa de WCAG 2.1, para poder medir contraste."""
    rgb = QColor(color)
    channels = []
    for raw in (rgb.redF(), rgb.greenF(), rgb.blueF()):
        channels.append(raw / 12.92 if raw <= 0.03928 else ((raw + 0.055) / 1.055) ** 2.4)
    r, g, b = channels
    return 0.2126 * r + 0.7152 * g + 0.0722 * b


def contrast_ratio(foreground: str, background: str) -> float:
    """Relación de contraste WCAG entre dos colores (1.0 a 21.0)."""
    lum_a = relative_luminance(foreground)
    lum_b = relative_luminance(background)
    lighter, darker = max(lum_a, lum_b), min(lum_a, lum_b)
    return (lighter + 0.05) / (darker + 0.05)


def choose_palette(base_color: QColor) -> HighlightPalette:
    """Elige la paleta según el fondo REAL del editor.

    Se mira `QPalette.Base` (el fondo de los campos de texto), no
    `QPalette.Window`: en varios temas no son el mismo color, y el que importa
    acá es el de atrás del texto.
    """
    return DARK_PALETTE if base_color.lightness() < 128 else LIGHT_PALETTE


class DslSyntaxHighlighter(QSyntaxHighlighter):
    def __init__(
        self,
        document: QTextDocument,
        palette: HighlightPalette | None = None,
        qt_palette: QPalette | None = None,
    ) -> None:
        super().__init__(document)
        if palette is None:
            base = (
                qt_palette.color(QPalette.ColorRole.Base)
                if qt_palette is not None
                else QColor("#ffffff")
            )
            palette = choose_palette(base)
        self.palette_used = palette
        self._rules: list[tuple[QRegularExpression, QTextCharFormat]] = []
        self._build_rules(palette)

    def _build_rules(self, palette: HighlightPalette) -> None:
        def fmt(color: str, bold: bool = False, italic: bool = False) -> QTextCharFormat:
            f = QTextCharFormat()
            f.setForeground(QColor(color))
            if bold:
                f.setFontWeight(QFont.Weight.Bold)
            if italic:
                f.setFontItalic(True)
            return f

        def word_rule(words: list[str], text_format: QTextCharFormat) -> None:
            # \b...\b para no pintar "ON" dentro de "POINT". Las palabras se
            # ordenan de más larga a más corta para que WAIT_IN gane sobre WAIT.
            pattern = r"\b(" + "|".join(
                re.escape(w) for w in sorted(words, key=len, reverse=True)
            ) + r")\b"
            self._rules.append((QRegularExpression(pattern), text_format))

        word_rule(KEYWORDS, fmt(palette.keyword, bold=True))
        word_rule(MOVE_KINDS, fmt(palette.move, bold=True))
        word_rule(POINT_FUNCTIONS, fmt(palette.point_function))
        word_rule(TYPES, fmt(palette.type_))
        word_rule(STATES, fmt(palette.state, bold=True))
        self._rules.append((QRegularExpression(IO_PATTERN), fmt(palette.io)))
        self._rules.append((QRegularExpression(NUMBER_PATTERN), fmt(palette.number)))
        # El comentario va ÚLTIMO a propósito: pisa cualquier regla anterior,
        # así `; MOVEJ p SPEED 50` se ve todo como comentario y no medio
        # coloreado como si fuera código.
        self._rules.append((QRegularExpression(COMMENT_PATTERN), fmt(palette.comment, italic=True)))

    def highlightBlock(self, text: str) -> None:  # noqa: N802 (API de Qt)
        for regex, text_format in self._rules:
            it = regex.globalMatch(text)
            while it.hasNext():
                match = it.next()
                self.setFormat(match.capturedStart(), match.capturedLength(), text_format)
