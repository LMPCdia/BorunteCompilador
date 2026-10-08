# Simulador cinemático (`sim/`)

Ejecuta **el mismo respaldo del pad que se exporta** sobre un modelo
geométrico del robot y avisa, antes de llevar el programa al robot:

- ejes fuera de rango (`MOVEJ` y cada tramo de un `MOVEL`);
- `MOVEL` inalcanzables (fuera de alcance, o la recta pasa por donde el brazo
  no llega);
- saltos bruscos de un eje en un `MOVEL` (singularidad de muñeca);
- tiempo de ciclo **estimado**.

```bash
python -m sim.check programa.krlb
python -m sim.check HCBackupRobot_*.zip --model BRTIRUS1820A --input X012=1
```

## Piezas

| Archivo | Qué hace |
|---|---|
| `sim/models/<MODELO>.json` | Medidas, rangos y velocidades de un modelo. Agregar un modelo = agregar un JSON |
| `sim/kinematics.py` | Cinemática directa (ejes → brida) e inversa (brida → ejes, Levenberg-Marquardt buscando cerca de la configuración actual). Python puro |
| `sim/pad_sim.py` | Intérprete de las acciones del pad: MOVEJ, MOVEL, WAIT, SET_OUT, IF→GOTO, CALL |
| `sim/check.py` | Línea de comandos |

## Modelo geométrico

Todos los BRTIRUS vistos tienen esta forma (cotas del 1820A):

```
          d4 = 825.5           d6 = 154
 J3 ●────────────────────● J5 ──────▸ brida
    │ a3 = 100            (centro de muñeca = "P point" del plano)
 J3 ●
    │
    │ a2 = 730
    │
 J2 ●  ← a1 = 170 adelante de J1, d1 = 494.6 sobre la base
 ═══╧═══ base, J1 vertical
```

Posición cero: brazo vertical, antebrazo horizontal hacia +X (como en el
plano).

## Qué está confirmado y qué no

| Dato | Estado | Evidencia |
|---|---|---|
| Medidas `d1, a1, a2, a3, d4, d6` del 1820A | **Verificado contra el plano** | Alcance 170 + 730 + √(825.5² + 100²) = 1731.53 (cota 1731.5) y altura 494.6 + 730 + 831.53 = 2056.1 (cota 2056) |
| Rangos y velocidades | Tabla del fabricante | La tabla enviada no dice el modelo: confirmar que es la del 1820A |
| Cero de cada eje = pose del plano | Hipótesis | — |
| J2 positivo = hacia adelante; J3 positivo = antebrazo hacia abajo | Hipótesis | El HOME de un programa real (J2 = 45.9°, J3 = -44.9°) da brazo inclinado y antebrazo horizontal, la pose típica |
| J5 invertido (`sign: -1`) | Deducido | Con ese HOME (J5 = -76°) la herramienta queda **hacia abajo**; con el otro sentido apuntaría hacia arriba. Un foro de RoboDK también reporta ejes 4-6 invertidos en Borunte |
| J4, J6 | Sin evidencia | Se dejan en `sign: 1` |
| Convención U, V, W | Hipótesis | `R = Rz(W) · Ry(V) · Rx(U)` (ángulos fijos X-Y-Z) |
| Zona de trabajo inferior del plano (cotas 1036, 775, R480) | **No coincide** | Con los rangos de la tabla, el centro de la muñeca no baja de ≈ -90 mm. Puede ser otra referencia del "P point", o que J3 se mida respecto de la horizontal |
| Tiempo de ciclo | Estimado | Velocidad = % de la máxima de cada eje, sin aceleraciones ni suavizado |

**Cómo se valida con el robot:** llevarlo a 3-5 poses distintas y anotar, de
la pantalla del pad, los ángulos J1-J6 y la posición X, Y, Z, U, V, W
(herramienta 0, coordenadas 0). Si `fk(ángulos)` da esa posición, el modelo
está bien; si no, la diferencia dice qué eje está invertido o desfasado.

## Limitaciones

- Los `MOVEL` con herramienta o coordenadas distintas de 0 se saltean (hace
  falta cargar esos marcos del pad).
- Sin colisiones todavía (llega con el visor 3D y el layout).
- `WAIT_IN` no existe en el pad todavía; los `IF` usan los estados de entrada
  que se pasan con `--input`.
