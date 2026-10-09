# Simulador cinemático (`sim/`)

Ejecuta **el mismo respaldo del pad que se exporta** sobre un modelo
geométrico del robot y avisa, antes de llevar el programa al robot:

- ejes fuera de rango (`MOVEJ` y cada tramo de un `MOVEL`);
- `MOVEL` inalcanzables (fuera de alcance, o la recta pasa por donde el brazo
  no llega);
- saltos bruscos de un eje en un `MOVEL` (singularidad de muñeca);
- **choques** del brazo y la herramienta contra las piezas de la celda, el
  piso y el mismo brazo (ver "Choques");
- tiempo de ciclo **estimado**.

```bash
python -m sim.check programa.krlb
python -m sim.check HCBackupRobot_*.zip --model BRTIRUS1820A --input X012=1
```

En la app: pestaña **Simulación 3D** → elegir el robot, escribir las
entradas activas (`X012=1`) y **Simular**. El robot se anima con ▶ o con la
barra de tiempo; los problemas van a la ventana de mensajes.

## Navegación 3D (como Inventor / AutoCAD)

| Acción | Mouse / teclado |
|---|---|
| Zoom hacia el cursor | rueda |
| Desplazar | botón del medio (o derecho), flechas |
| Orbitar alrededor del centro de la vista | Shift + botón del medio (o izquierdo) |
| Encuadrar todo | doble clic con el botón del medio, F6, botón *Encuadrar* |
| Vistas estándar (encuadradas) | botones Iso, Arriba, Frente, Lado, Atrás, Izquierda |

La órbita mantiene Z para arriba (como el 3DORBIT de AutoCAD). *Simular*
pasó a F9 (F6 es encuadrar, como en Inventor). Código: `gui/camera_nav.py`.

## Layout de la celda

**Importar objeto…** acepta **STEP**, STL y OBJ (mm). La pieza se apoya en el
piso (z = 0) y se ubica con la tabla: X, Y, Z respecto de la base del robot y
giro alrededor del eje vertical. **Guardar layout…** escribe un
`.layout.json` con rutas relativas, así la carpeta se puede mover entera.

### Ubicar piezas por distancias (`sim/placement.py`)

Pestaña **Piezas**: elegí la pieza en la tabla (una recién importada ya queda
elegida) y en **Ubicar**:

| Modo | Qué se indica |
|---|---|
| A una distancia del robot | distancia al eje de J1 (hasta la cara más cercana o hasta el centro) y ángulo (0° = adelante, +X; 90° = a la izquierda, +Y) |
| Corrida respecto de… | ΔX, ΔY, ΔZ desde la base del robot o desde otra pieza |
| Al lado de… | lado (+X, -X, +Y, -Y) y separación entre caras, centrada y en el mismo piso |
| Encima de… | apoyada arriba de otra pieza, con corrimiento ΔX, ΔY |

Todo se mide sobre la caja de la pieza ya girada; el punto de apoyo es el
centro de su base. Abajo se ve al instante el tamaño, la distancia al eje del
robot y la separación por eje con cada pieza; **Medir distancias** calcula la
distancia mínima real entre superficies (python-fcl) al robot en la pose que
se ve y a cada pieza.

Los STEP se convierten a triángulos con **gmsh** (dependencia nueva, trae
OpenCascade; suma ~60 MB al `.exe`). La malla es gruesa a propósito: alcanza
para ver la celda, no para medir.

## Biblioteca de Google Drive (`sim/library.py`, `gui/library_view.py`)

Pestaña **Biblioteca**: lee EN LÍNEA la carpeta de Drive (por defecto
*RobotsBoruntesSimulador*; se puede pegar el enlace de otra), sin cuenta de
Google: la carpeta tiene que estar compartida como **«Cualquier persona con
el enlace»**. Se lee al abrir la pestaña y con *Actualizar*.

Clasificación **por carpeta** (no por nombre de archivo):

| Carpeta | Tipo | Doble clic / botón |
|---|---|---|
| `Robots/…` o un STEP cuyo nombre diga `BRTIRUSxxxx` | robot | *Usar este robot*: lo importa del STEP (un par de minutos, `sim/robot_import.py`) y lo elige |
| `Herramientas`, `Grippers`, `Pinzas`, `Antorchas`, `Ventosas`… | herramienta | *Montar en la brida* |
| cualquier otra (`Bases`, `Mesas`, `Piezas`…) | pieza | *Insertar en la celda* (queda elegida para ubicarla) |

- Formatos: STEP, STL, OBJ. Los nativos (SolidWorks `.SLDPRT`, Inventor
  `.ipt`…) se cuentan y se avisa que hay que exportarlos a STEP.
- Los archivos se bajan al usarlos a `~/BorunteDSL/biblioteca/<id>/` y se
  vuelven a bajar solo si en Drive cambió el tamaño o la fecha. Sin red, se
  usa lo ya bajado.
- La celda guarda el ID de Drive de cada pieza y de la herramienta: abierta
  en otra PC, las baja sola.
- Un robot nuevo se importa con los rangos y velocidades del BRTIRUS1510A
  (hipótesis, se avisa): hay que reemplazarlos por su tabla en
  `~/BorunteDSL/modelos/<MODELO>.json`.
- Límite: la vista web de Drive muestra hasta unos cientos de archivos por
  carpeta.
- Revisar la carpeta desde la consola: `python -m sim.library` muestra el
  árbol y lo que está fuera de lugar (archivos sueltos en la raíz, robots
  fuera de `Robots/`, CAD nativo sin un STEP al lado). Las reglas para quien
  sube archivos están en el doc *LEEME - cómo cargar modelos* de la carpeta.

## Robot = CAD + planilla de parámetros (`sim/robot_params.py`)

Cada robot de la biblioteca es una carpeta `Robots/<MODELO>/` con:

- el **STEP del ensamble** del fabricante → geometría (cotas) y mallas;
- la **planilla de parámetros** (Google Sheet o CSV; la del 1510A es
  *Parámetros BRTIRUS1510A*, copiarla para otro robot) → por eje: rango,
  velocidad máxima, aceleración máxima, sentido de giro, si está confirmado;
  y datos generales: alcance, carga, velocidad lineal máxima, cotas del plano;
- opcional, el PDF del datasheet (la app no lo lee: es para quien completa la
  planilla). Un PDF no se interpreta solo a propósito: un número mal leído de
  un datasheet en chino terminaría moviendo el robot real.

**Usar este robot** (pestaña Biblioteca) hace todo solo:

1. lee la planilla (siempre, de la web: si alguien la corrigió, se usa lo nuevo);
2. baja e importa el STEP **solo si el robot no está o si el STEP cambió**
   en Drive (se compara tamaño y fecha; un robot que viene con la app no se
   reimporta);
3. arma el modelo: la geometría del CAD manda; las cotas y el alcance de la
   planilla se comparan con el CAD y se avisa si difieren (más de 2 mm o 2 %);
4. **verifica la cinemática inversa**: 60 poses al azar dentro de los rangos,
   directa → inversa → directa, tienen que cerrar a menos de 0.1 mm;
5. lo guarda en `~/BorunteDSL/modelos/<MODELO>.json` y lo elige. Lo que la
   planilla marca sin confirmar queda como hipótesis en las notas del modelo.

Con aceleraciones en la planilla, la casilla **del datasheet** (al lado de
*Aceleración*) usa la aceleración máxima de cada eje: cada movimiento tarda
en acelerar lo que necesite el eje más exigido. Con velocidad lineal máxima,
los MOVEL no llevan la punta más rápido que esa velocidad × SPEED %
(hipótesis: que el % del MOVEL sea sobre la velocidad lineal).

Desde la consola: `python -m sim.robot_params planilla.csv --model BRTIRUS1510A
[--write]` (y `-` en vez del CSV imprime la plantilla).

## Gráficas de movimiento (`sim/motion.py`, `gui/motion_charts.py`)

Botón **Gráficas…** (o *Simulación → Gráficas de movimiento*, Ctrl+G):

- por eje: posición (°), velocidad (°/s) y aceleración (°/s²);
- de la punta de la herramienta: velocidad y aceleración lineal (mm/s, mm/s²);
- tabla con el recorrido, la velocidad máxima de cada eje (y qué % de su
  límite es) y la aceleración máxima;
- franjas rojas donde hay choque y naranjas donde pasa más cerca que el margen;
- una línea sigue a la animación, y un clic en la gráfica lleva la animación ahí.
- Clic en la leyenda: muestra/oculta un eje.

**Aceleración (hipótesis).** El respaldo del pad trae `speed` y `smooth`
pero no la aceleración del controlador. El simulador usa un perfil
trapezoidal: cada movimiento tarda **Aceleración** segundos (0.25 s por
defecto, campo al lado de la barra de tiempo) en llegar a su velocidad y lo
mismo en frenar, arrancando y terminando quieto. Con 0, velocidad constante
y las aceleraciones no significan nada. Cambia el tiempo de ciclo: +1
tiempo de aceleración por movimiento. No se simula `smooth` (el robot real
probablemente redondea las esquinas sin frenar).

Las derivadas son numéricas, con una ventana de 40 ms; no cruzan los
"saltos" del simulador (recuperaciones), que no son movimiento.

## Choques (`sim/collision.py`, python-fcl)

Se activa en la pestaña **Choques** (activado por defecto). Después de
simular, se recorre la trayectoria y se revisa:

| Qué | Contra qué | Margen |
|---|---|---|
| Eslabones J1…J6 | cada pieza del layout | sí (aviso si pasa más cerca; error si toca) |
| Herramienta montada en la brida | cada pieza del layout | sí, salvo en las piezas marcadas **Se trabaja**: ahí solo cuenta tocarla |
| Eslabones J2…J6 y herramienta | el piso (z = 0, el plano de apoyo de la base) | sí |
| Eslabones a 3+ ejes de distancia, herramienta contra J1…J4 | el mismo brazo | no: solo si se tocan |

- **Las piezas son sólidos.** fcl compara superficies; para que un eslabón
  metido entero adentro de una pieza cuente, cada vez que el robot aparece de
  golpe en una pose (el primer punto, después de un salto del simulador) se
  prueba si quedó adentro (rayo contra la malla). Entre medio el movimiento es
  continuo y para meterse tiene que cruzar la superficie, que sí se ve.
  Supone mallas cerradas (un STEP de sólidos lo es).
- **No se escapa nada entre muestras.** Avance conservador: si los ejes
  cambian Δq, ningún punto de una parte se mueve más que Σ |Δq_i|·R_i (R_i:
  cota de la distancia de la parte al eje i, que no depende de la pose). Si la
  distancia medida en los extremos cubre eso, el tramo está libre; si no, se
  parte al medio hasta 3 mm. Un giro rápido de J1 no "saltea" un poste.
- **La base no se revisa contra las piezas**: lo que la toca es el pedestal o
  la mesa donde está atornillada.
- **Herramienta:** el STEP/STL/OBJ de la antorcha o la pinza, dibujado en
  coordenadas de la brida (Z saliendo de la brida). Si en el CAD no está así,
  el campo **Montaje** (X, Y, Z, U, V, W respecto de la brida) lo acomoda. Se
  dibuja pegada al robot. Sin herramienta cargada, se revisa hasta la brida.
- Cada choque o cercanía es un problema con su instante: clic y la animación
  va ahí. Durante la animación, la pieza se pinta de **rojo** mientras el
  robot la toca y de **naranja** mientras está más cerca que el margen.
- Margen, herramienta, montaje y qué pieza se trabaja se guardan en el
  `.layout.json`.

**Qué NO es exacto** (además de lo de "Qué está confirmado y qué no"):

- **La forma del robot.** Mientras el modelo no traiga las mallas del
  fabricante, los eslabones son cilindros aproximados y los choques son
  estimaciones; la app lo avisa. Con las mallas reales (ver abajo) se usan esas.
- **La trayectoria entre puntos**: el controlador real puede redondear
  esquinas o interpolar distinto. El margen está para eso.
- **Cables, mangueras y el alimentador de alambre** no están en ningún modelo.

Dependencia: **python-fcl** (trae numpy). Sin ella el simulador funciona
igual y la pestaña dice que los choques no se revisan. En el `.exe`, sus DLL
(`ccd`, `octomap`) las agrega el spec (`FCL_BINARIES`) y el self-test hace un
choque de prueba.

## Modelos 3D del robot

| Modelo | Forma | Cotas | Rangos y velocidades |
|---|---|---|---|
| **BRTIRUS1510A** (el de la celda, por defecto) | mallas del fabricante | medidas del STEP del fabricante: alcance 1511 mm | tabla "Basic Parameters": velocidades confirmadas por el usuario; rangos **sin confirmar** |
| BRTIRUS1820A | cilindros aproximados | plano "BASIC SIZE" | la misma tabla |

### Importar un robot desde el STEP del fabricante

```bash
python -m sim.robot_import "BRTIRUS1510A 六轴机器人模型（21版本）.STEP" \
    --name BRTIRUS1510A --joints-from BRTIRUS1820A --out sim/models
```

(sin `--out` va a `~/BorunteDSL/modelos`, que la app también lee). Tarda un
par de minutos. Qué hace:

1. **Separa el ensamble en partes** (`sim/step_assembly.py`): lee el grafo del
   STEP, escribe un STEP chico por parte y la ubica con la transformación del
   ensamble. Hace falta porque gmsh junta todo el ensamble y los nombres de
   Borunte vienen en GBK, que OpenCascade descarta. Las caras que gmsh no
   puede mallar (pasa con superficies periódicas del CAD) se dejan afuera y se
   avisa; en el 1510A fueron 2 de ~8000.
2. **Lleva cada eslabón a la posición cero** (`sim/robot_import.py`). El
   ensamble viene en cualquier pose (el del 1510A: J1 -1.25°, J2 3.5°, J3
   -3.5°, J4 72°). Cada eje se ubica con los cilindros de los rodamientos y se
   "desgira" de J1 a J6: es la inversa exacta del producto de exponenciales.
3. **Mide las cotas** (d1, a1, a2, a3, d4, d6) de la posición cero. Control:
   el alcance (a1 + a2 + √(d4² + a3²)) da 1511 mm para el 1510A.
4. **Simplifica las mallas** (grilla de 6 mm, 2-3 mm en muñeca y brida):
   ~90 mil triángulos, 4.7 MB.

Las partes siguen la nomenclatura de Borunte: `PBR6US..A000` base, `B000`
J1 (转座), `C000` brazo (大臂), `D000` J3 (三轴), `E000` J4 (四轴), `F000`
muñeca y la brida sin código (六轴装配体). Dónde está cada eje dentro de cada
parte (`BORUNTE_RECIPE`) se midió en el 1510A: **para otro modelo hay que
verificarlo** (si el alcance medido no da el del nombre, la receta no sirve).

Lo que el STEP no dice y queda como hipótesis:

- **Rangos, velocidades y sentidos de giro**: se copian de otro modelo
  (`--joints-from`). Reemplazarlos por la tabla del robot.
- **El cero de J6**: la brida es casi simétrica. Se toma el giro que deja la
  pieza alineada.
- **El 1510A tenía el antebrazo corrido 8.6 mm** a lo largo del eje de J3. Se
  centró, porque centrado el antebrazo queda simétrico, así que era la unión
  del CAD y no el robot.

Para dibujar un robot con otras mallas a mano: exportar del CAD **un archivo
por eslabón** (base, J1…J6) **en la posición cero**, en mm y en el sistema de
la base, guardarlos en `sim/models/<MODELO>/` y listarlos en el JSON:

```json
"meshes": ["base.stl", "j1.stl", "j2.stl", "j3.stl", "j4.stl", "j5.stl", "j6.stl"]
```

Con mallas reales, los choques ya no se marcan como "aproximados".

## Piezas

| Archivo | Qué hace |
|---|---|
| `sim/models/<MODELO>.json` | Medidas, rangos y velocidades de un modelo. Agregar un modelo = agregar un JSON |
| `sim/kinematics.py` | Cinemática directa (ejes → brida) e inversa (brida → ejes, Levenberg-Marquardt buscando cerca de la configuración actual). Python puro |
| `sim/pad_sim.py` | Intérprete de las acciones del pad: MOVEJ, MOVEL, WAIT, SET_OUT, IF→GOTO, CALL |
| `sim/check.py` | Línea de comandos |
| `sim/meshes.py` | Carga de STEP (gmsh), STL y OBJ; primitivas |
| `sim/scene.py` | Eslabones del robot, layout (JSON) y línea de tiempo |
| `sim/collision.py` | Choques con python-fcl sobre la trayectoria simulada |
| `sim/step_assembly.py`, `sim/robot_import.py` | Ensamble STEP del fabricante -> modelo de robot |
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
- Choques: ver las aproximaciones en "Choques". Sin profundidad de
  penetración (se informa "toca", no cuánto se mete).
- La vista 3D necesita OpenGL. Sin él (máquinas virtuales, escritorio
  remoto viejo) la pestaña muestra un aviso y la simulación funciona igual.
- `WAIT_IN` no existe en el pad todavía; los `IF` usan los estados de entrada
  de las casillas (o `--input` en la línea de comandos), fijos toda la
  simulación.
- Sin velocidad lineal máxima del robot: el tiempo de un MOVEL rápido puede
  salir corto, y la velocidad de la punta en las gráficas, alta.
