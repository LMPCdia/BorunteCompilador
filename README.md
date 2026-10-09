# borunte-dsl

Lenguaje tipo KRL para programar un robot Borunte desde la PC. El programa se
compila a un **respaldo del pad** (`HCBackupRobot_<fecha>.zip`), se importa en
el teach pendant por pendrive y lo ejecuta el controlador del robot solo.

Leer primero `docs/ARCHITECTURE.md` y `docs/PAD_FORMAT.md`.

## Estado actual

- [x] Gramática del DSL (`compiler/grammar.lark`) con puntos `WORLD(...)`
      (X,Y,Z,U,V,W) y `JOINT(...)` (ángulos de eje)
- [x] AST de dos pasadas (`compiler/ast_nodes.py`, `compiler/ast_builder.py`)
- [x] **Formato del respaldo del pad decodificado** a partir de un respaldo
      real (`pad/backup.py`, `docs/PAD_FORMAT.md`): leer y reescribir da el
      mismo contenido byte a byte
- [x] **Generador para el pad** (`compiler/pad_codegen.py`): `MOVEJ`/`MOVEL`,
      `SET_OUT`, `WAIT`, `PROC`/llamadas e `IF` sobre entradas
- [x] Listado legible de cualquier respaldo (`python -m pad.listing`)
- [x] VM de referencia en Python (`runtime/vm.py`) para simular la lógica en la
      PC, contra el simulador del robot (`comms/robot_simulator.py`)
- [x] Cliente Modbus del robot (`comms/robot_client.py`) para digitalizar puntos
- [x] GUI (PySide6) con "Exportar para el pad" (Ctrl+E)
- [x] **Simulador 3D** (`sim/`, pestaña "Simulación 3D"): ejecuta el respaldo
      del pad sobre el modelo del robot, avisa ejes fuera de rango, puntos
      inalcanzables y singularidades, estima el tiempo de ciclo, y arma el
      layout importando STEP/STL/OBJ. Busca **choques** del brazo y la
      herramienta contra las piezas (sólidos), el piso y el propio brazo
      (python-fcl), gráficas de ejes y de la punta, navegación como Inventor,
      ubicación de piezas por distancias, el robot corrido o girado en la
      celda (pedestal, pared, techo) y biblioteca de modelos en Google
      Drive: sus robots aparecen en la lista Robot y se importan solos del
      STEP del fabricante (ejes encontrados en los cilindros del CAD).
      Robots: **BRTIRUS1510A** (el de la celda, con la forma real sacada del
      STEP del fabricante) y BRTIRUS1820A. Ver `docs/SIMULATOR.md`
- [x] Ejecutable de Windows de un solo archivo, construido en GitHub Actions
- [ ] **Probar en el pad** un respaldo generado (nadie lo hizo todavía)
- [ ] Espera de entrada, `ELSE` y variables en el pad (falta un ejemplo del
      pad para decodificarlos)

## Lo que NO está confirmado

Nada de esto se probó todavía en el pad. Está marcado como hipótesis en el
código y en `docs/PAD_FORMAT.md`, que le pone un grado de confianza a cada
dato. Lo más importante:

1. **Que el pad importe lo que generamos.** El formato reproduce byte a byte
   un respaldo real, pero un respaldo nuevo nunca se importó.
2. **La numeración de las E/S**: octal empezando en `010` (`Y034` = salida 20).
   Salió de solo 3 casos.
3. **`WAIT`** (acción `100`) e **`IF` sobre entrada en OFF** (`pointStatus: 0`).
4. **Herramienta y coordenadas**: con `TOOL n` / `COORD n` en el programa; el
   simulador necesita sus valores (copiados del pad) en las pestañas
   Herramientas y Coordenadas.

**Probá cada programa nuevo primero a velocidad baja.**

## Limitaciones conocidas (a propósito, no bugs escondidos)

Para el pad, lo que no sabemos expresar es un error de compilación claro:

- `MOVEJ` necesita un punto `JOINT(...)` y `MOVEL` uno `WORLD(...)`. El MOVEJ
  del pad guarda ángulos de eje: mandarle X/Y/Z movería el robot a cualquier
  lado.
- Sin `VAR`, asignaciones, `ELSE`, `WAIT_IN` ni `PROC` con parámetros.
- `IF` solo pregunta por una entrada: `IF X010 == 0 THEN` (forma confirmada).
  `IF X010 == 1` y llamar un PROC desde otro PROC nunca aparecieron en un
  respaldo real: se exportan solo con **Programa → Permitir instrucciones sin
  confirmar en el pad** (en la simulación se permiten siempre).
- `TOOL n` / `COORD n` cambian la herramienta y el sistema de coordenadas de
  los movimientos que siguen; cada PROC arranca de nuevo con los valores de
  exportación (0/0 desde la app), no hereda los del que lo llama.
- Los errores dicen la línea; los avisos (lo que se descarta, como `WAIT
  UNTIL MOVE_DONE`) aparecen al exportar.
- Los offsets de puntos (`p + OFFSET(...)`) se resuelven al compilar.

De la VM de referencia (solo simulación): `IF` solo con `==`, `PROC` sin pila
de frames, y la tabla de puntos no se deduplica. En la VM un `IF X010 == 1`
lee una variable, no la entrada.

## Setup

```bash
python -m venv venv
source venv/bin/activate   # o venv\Scripts\activate en Windows
pip install -r requirements.txt
pytest tests/ -v -s
```

Los tests de GUI necesitan `QT_QPA_PLATFORM=offscreen` si no hay display:

```bash
QT_QPA_PLATFORM=offscreen pytest tests/ -v
```

El `-s` muestra el bytecode generado y el trace de ejecución de
`test_end_to_end_against_simulator` — es la forma más rápida de ver todo el
pipeline funcionando.

Para abrir la GUI:

```bash
python -m gui.app
```

Se abre con un programa de ejemplo ya cargado.

- **Ctrl+E — Exportar para el pad**: genera `HCBackupRobot_<fecha>.zip` en la
  carpeta que elijas y muestra el listado en la pestaña "Pad". Se copia a la
  raíz de un pendrive y se importa desde el pad.
- Para probar la lógica en la PC: "Conectar" (dejá "Simulador" tildado) →
  **F7** compilar → **F5** ejecutar. **Shift+F5** para.
- **F8** digitaliza el punto actual del robot (necesita conexión Modbus).

Para ver cualquier respaldo del pad en texto:

```bash
python -m pad.listing HCBackupRobot_20260814213843.zip
```

Para verificar la instalación sin abrir ventana:

```bash
python -m gui.app --self-test
```

## Construir el ejecutable

**Sin Windows a mano:** cada push a `main` o a una rama `claude/**` (o
"Run workflow" en la pestaña Actions) dispara `.github/workflows/build-exe.yml`,
que corre este mismo script en un runner Windows y deja `BorunteDSL.exe`
descargable en *Actions → la corrida → Artifacts*.

En una PC con Windows:

```powershell
.\packaging\build_exe.ps1
```

Genera `dist/BorunteDSL.exe` (un solo archivo, ~50 MB), copiable a otra PC con
Windows sin instalar Python. **Hay que construirlo EN Windows**: PyInstaller no
compila cruzado.

El script corre los tests, empaqueta, y **no da el `.exe` por bueno hasta que
pase `--self-test` sobre el binario ya construido**. Eso no es ceremonia:
`compiler/grammar.lark` y `pad/template.fnc` se leen como archivos en tiempo
de ejecución, así que un
empaquetado incompleto produce un ejecutable que abre bien y falla al compilar
el primer programa — y con `console=False` ese error no se ve en ninguna parte.

## Estructura

```
compiler/    gramática (Lark) + AST + pad_codegen.py (respaldo del pad)
             + codegen.py (bytecode para la VM de referencia)
sim/         kinematics.py, pad_sim.py (simulador), meshes.py, scene.py,
             models/*.json (un archivo por modelo de robot)
pad/         backup.py (leer/escribir HCBackupRobot_*.zip), listing.py
             (listado legible), template.fnc
comms/       robot_client.py (Modbus del robot), robot_simulator.py +
             fake_modbus.py (para probar sin hardware)
runtime/     bytecode.py, vm.py (VM de referencia), plc_io_simulator.py (E/S
             simuladas)
gui/         main_window.py + paneles, vm_worker.py, app.py (--self-test)
packaging/   spec de PyInstaller, build_exe.ps1, datafiles.py
docs/        ARCHITECTURE.md, PAD_FORMAT.md, INSTRUCTION_SET.md,
             MODBUS_REGISTER_MAP.md, SIMULATION.md
tests/
```

## Próximos pasos sugeridos, en orden

1. **Probar en el pad**, en este orden: importar un respaldo reescrito sin
   cambios, después uno generado, a velocidad baja. Ver `docs/PAD_FORMAT.md`,
   "Para confirmar".
2. **Exportar del pad un respaldo de prueba** con una espera de entrada, un
   salto incondicional y una variable, para poder agregar `WAIT_IN`, `ELSE` y
   `VAR` al generador.
3. **Elegir herramienta y coordenadas desde la GUI** (hoy van en 0/0) y usar
   un respaldo del propio robot como plantilla (`PadOptions.template`).
4. **Digitalizar en `JOINT`** además de `WORLD`, para poder usar los puntos
   digitalizados en un `MOVEJ`.
5. **Persistencia de los puntos digitalizados** (hoy viven en memoria).
