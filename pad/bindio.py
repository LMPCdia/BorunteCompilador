"""
`bindIOInfo` + `distance` de un movimiento: «una salida que cambia según la
distancia al punto» (en la soldadura, prender y apagar el arco cerca de los
puntos del cordón).

Cómo se decodificó (octubre 2026): el pad puede exportar el programa como
HTML (la misma lista que muestra en pantalla). Cada movimiento con
`bindIOInfo` trae ahí una línea como

    whiledistanceend pointthe distancelimit then 1.000mm then output Y034 on
    whiledistancestart pointthe distancemore then 2.000mm then output Y034 off

Cruzando los 327 movimientos del respaldo con esas líneas, los valores salen
de combinar:

    bindIOInfo = (point << 10) | 0x40 | (0x4 si es el punto FINAL) | (0x1 si
                 está habilitado) | (0x100000 si la salida pasa a ON)

    punto de la salida   point = Y034 -> 20 (numeración octal desde 010)
    referencia           0x4 prendido: punto final ("end point ... limit");
                         apagado: punto inicial ("start point ... more")
    estado               0x100000 prendido: la salida queda en ON
    habilitado           0x1; apagado = la configuración está pero el pad no
                         muestra nada (3 casos, todos con distancia 0)
    `distance`           milímetros

Valores vistos: 0x5041, 0x5045, 0x105041, 0x105045 y 0x105044. **Deducido, no
probado en el robot**: es lo que muestra el texto del propio pad. El bit 0x40
siempre estuvo prendido y no se sabe qué es. Cualquier otro valor se deja
pasar crudo (`decode` devuelve None) para no perder nada.
"""

from __future__ import annotations

from dataclasses import dataclass

_BASE = 0x40
_END = 0x4
_ENABLED = 0x1
_ON = 0x100000
_POINT_SHIFT = 10
_KNOWN_LOW = _BASE | _END | _ENABLED
_MAX_POINT = 0x3FF            # 10 bits entre _POINT_SHIFT y _ON


@dataclass(frozen=True)
class BindIO:
    point: int            # la salida, como `point` de SET_OUT (Y034 = 20)
    end: bool             # True: distancia al punto final; False: al inicial
    on: bool              # True: la salida pasa a ON; False: a OFF
    enabled: bool = True

    def encode(self) -> int:
        if not 0 <= self.point <= _MAX_POINT:
            raise ValueError(f"salida {self.point}: fuera de rango")
        return (self.point << _POINT_SHIFT) | _BASE | (_END if self.end else 0) \
            | (_ENABLED if self.enabled else 0) | (_ON if self.on else 0)

    @property
    def relation(self) -> str:
        """El pad dice «limit» para el punto final y «more» para el inicial."""
        return "limit" if self.end else "more"


def decode(value: int) -> BindIO | None:
    """None si el valor tiene bits que no entendemos (o es 0: sin enlace)."""
    if not value or value < 0:
        return None
    low = value & ((1 << _POINT_SHIFT) - 1)
    high = value >> _POINT_SHIFT
    on = bool(high & (_ON >> _POINT_SHIFT))
    point = high & _MAX_POINT
    if value & ~(_ON | (_MAX_POINT << _POINT_SHIFT) | _KNOWN_LOW):
        return None
    if low & ~_KNOWN_LOW or not low & _BASE:
        return None
    return BindIO(point=point, end=bool(low & _END), on=on, enabled=bool(low & _ENABLED))


def pad_text(bind: BindIO, distance_mm: float, out_name: str) -> str:
    """La línea que el pad muestra para este enlace (solo si está habilitado)."""
    ref = "end point" if bind.end else "start point"
    return (f"whiledistance{ref}the distance{bind.relation} then {distance_mm:.3f}mm "
            f"then output {out_name} {'on' if bind.on else 'off'}")
