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

En la app: pestaña **Simulación 3D** → elegir el robot, escribir las
entradas activas (`X012=1`) y **Simular**. El robot se anima con ▶ o con la
barra de tiempo; los problemas van a la ventana de mensajes.

## Layout de la celda

**Importar objeto…** acepta **STEP**, STL y OBJ (mm). La pieza se apoya en el
piso (z = 0) y se ubica con la tabla: X, Y, Z respecto de la base del robot y
giro alrededor del eje vertical. **Guardar layout…** escribe un
`.layout.json` con rutas relativas, así la carpeta se puede mover entera.

Los STEP se convierten a triángulos con **gmsh** (dependencia nueva, trae
OpenCascade; suma ~60 MB al `.exe`). La malla es gruesa a propósito: alcanza
para ver la celda, no para medir.

## Modelos 3D del robot

Mientras no estén los modelos del fabricante, el robot se dibuja con
cilindros a partir de las cotas. Para usar los modelos reales: exportar del
CAD **un archivo por eslabón** (base, J1…J6) **en la posición cero**, en mm
y en el sistema de la base, guardarlos en `sim/models/<MODELO>/` y listarlos
en el JSON del modelo:

```json
"meshes": ["base.stl", "j1.stl", "j2.stl", "j3.stl", "j4.stl", "j5.stl", "j6.stl"]
```

Con producto de exponenciales cada eslabón dibujado en la posición cero se
ubica solo; no hace falta ningún ajuste extra.

## Piezas

| Archivo | Qué hace |
|---|---|
| `sim/models/<MODELO>.json` | Medidas, rangos y velocidades de un modelo. Agregar un modelo = agregar un JSON |
| `sim/kinematics.py` | Cinemática directa (ejes → brida) e inversa (brida → ejes, Levenberg-Marquardt buscando cerca de la configuración actual). Python puro |
| `sim/pad_sim.py` | Intérprete de las acciones del pad: MOVEJ, MOVEL, WAIT, SET_OUT, IF→GOTO, CALL |
| `sim/check.py` | Línea de comandos |
| `sim/meshes.py` | Carga de STEP (gmsh), STL y OBJ; primitivas |
| `sim/scene.py` | Eslabones del robot, layout (JSON) y línea de tiempo |
| `gui/sim_view.py`, `gui/viewport3d.py` | Pestaña "Simulación 3D" y vista Qt3D |

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
| J2 y J3 invertidos (`sign: -1`: J2 positivo = brazo hacia atrás) | Deducido | De los 25 MOVEJ de un respaldo real: con esta elección el HOME queda recogido (brida en 509, 1, 969) en vez de casi estirado, las poses "Safty" de las 3 filas quedan a la misma altura sobre las mesas con la torcha hacia abajo, y el único MOVEL en coordenadas del mundo queda a ~110 mm del MOVEJ vecino (con la otra elección, a ~1180 mm). También acerca la zona de trabajo a las cotas del plano |
| J5 invertido (`sign: -1`) | Deducido | Con ese HOME (J5 = -76°) la herramienta queda **hacia abajo**; con el otro sentido apuntaría hacia arriba. Un foro de RoboDK también reporta ejes 4-6 invertidos en Borunte |
| J4, J6 | Sin evidencia | Se dejan en `sign: 1` |
| Convención U, V, W | Hipótesis | `R = Rz(W) · Ry(V) · Rx(U)` (ángulos fijos X-Y-Z) |
| Zona de trabajo inferior del plano (cotas 1036, 775, R480) | **No coincide** | Con los rangos de la tabla, el centro de la muñeca no baja de ≈ -90 mm. Puede ser otra referencia del "P point", o que J3 se mida respecto de la horizontal |
| Tiempo de ciclo | Estimado | Velocidad = % de la máxima de cada eje, sin aceleraciones ni suavizado |

**Cómo se valida con el robot:** llevarlo a 3-5 poses distintas y anotar, de
la pantalla del pad, los ángulos J1-J6 y la posición X, Y, Z, U, V, W
(herramienta 0, coordenadas 0). Si `fk(ángulos)` da esa posición, el modelo
está bien; si no, la diferencia dice qué eje está invertido o desfasado.

## Cómo se comporta (decisiones de la revisión con 3 agentes, octubre 2026)

- **MOVEL = recta de la PUNTA** de la herramienta (no de la brida), con la
  orientación interpolada por el camino más corto.
- **Pose inicial desconocida:** el robot aparece en el primer MOVEJ, sin
  tiempo ni trayectoria. Un MOVEL antes de cualquier MOVEJ no se evalúa.
- **Sin errores en cadena:** si un MOVEL falla, se dibuja en rojo hasta donde
  llegó y el robot sigue desde el destino (si alguna configuración llega);
  si no, los MOVEL siguientes quedan "sin evaluar" hasta el próximo MOVEJ.
  Lo mismo pasa con los MOVEL salteados por falta de herramienta o sistema.
- **Bucles:** con las entradas fijas, volver a la misma etiqueta con el
  robot en la misma pose es un bucle infinito seguro: se simula una vuelta
  y se informa. Además hay un tope de 20 000 acciones y se puede cancelar.
- **Ejes:** dentro de una recta manda la continuidad (si J4 pasa de 180° es
  un error real, con el valor y el rango en el mensaje). En los puntos
  sueltos se elige la vuelta de cada eje que cae en su rango.
- **Tiempo:** por eje, al % de su velocidad máxima; cada muestra guarda su
  tiempo real (cerca de una singularidad la animación se frena donde el
  robot se frenaría).

## Limitaciones

- Los `MOVEL` con una herramienta o un sistema de coordenadas que no se
  cargó en la pestaña se saltean (un solo aviso, y el botón "Cargar las que
  faltan" agrega las filas).
- Sin detección de colisiones todavía: el layout se ve, pero no se chequea
  contra el robot.
- La vista 3D necesita OpenGL. Sin él (máquinas virtuales, escritorio
  remoto viejo) la pestaña muestra un aviso y la simulación funciona igual.
- `WAIT_IN` no existe en el pad todavía; los `IF` usan los estados de entrada
  de las casillas (o `--input` en la línea de comandos), fijos toda la
  simulación.
- Sin velocidad lineal máxima del robot: el tiempo de un MOVEL rápido puede
  salir corto.
