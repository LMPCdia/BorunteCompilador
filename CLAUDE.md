# CLAUDE.md — Contexto del proyecto para Claude Code

Este archivo se lee automáticamente al abrir Claude Code en esta carpeta.
Contiene lo que un colaborador nuevo necesitaría saber antes de tocar código.

## Qué es esto

Lenguaje tipo KRL para programar un robot Borunte. El programa se compila a un
**respaldo del pad** (`HCBackupRobot_<fecha>.zip`) que se importa en el teach
pendant por pendrive, y lo ejecuta el controlador del robot solo. LEER
PRIMERO: `docs/ARCHITECTURE.md`, después `docs/PAD_FORMAT.md`,
`docs/INSTRUCTION_SET.md`, `docs/MODBUS_REGISTER_MAP.md`, `docs/SIMULATION.md`.

**Ya no hay PLC.** Hasta octubre 2026 el plan era una VM en ladder sobre un
Coolmay CX3G; se descartó y se borró ese código (`comms/plc_client.py`,
`plc_vm/`). Si algo todavía habla de "PLC", es un resto a limpiar.

No hay pad ni robot disponibles todavía. El formato del pad salió de UN
respaldo real; la lógica se prueba contra `comms/robot_simulator.py`.

## Reglas del proyecto (leer antes de escribir código)

1. **Nunca inventes comportamiento del hardware real como si estuviera
   confirmado.** Si algo no está en `docs/PAD_FORMAT.md` o
   `docs/MODBUS_REGISTER_MAP.md` como confirmado, tratalo como hipótesis y
   decilo explícitamente en el código (comentario) y en tu respuesta.
2. **El generador del pad no adivina.** Lo que no sabemos expresar en el
   formato del pad es un `CompileError` claro, nunca una acción "probable":
   el robot ejecuta lo que se le carga sin nadie mirando. Lo que sabemos
   expresar pero el respaldo real nunca mostró va detrás de
   `PadOptions.allow_unverified` (apagado por defecto al exportar). Para
   agregar algo hace falta un respaldo real del pad que lo muestre.
3. **`docs/PAD_FORMAT.md` e `docs/INSTRUCTION_SET.md` son contratos.** Si tu
   tarea cambia el formato generado o un opcode, avisá antes de seguir.
4. **Todo cambio en `compiler/` o `runtime/` debe dejar
   `pytest tests/ -v` en verde antes de darse por terminado.** Si agregás
   funcionalidad nueva, agregá un test que la cubra en `tests/`.
5. **`comms/robot_client.py` es la única puerta de entrada al robot.**
   Nada en `compiler/`, `runtime/` o `gui/` debe hablar Modbus directamente
   ni importar `pymodbus`. Si hace falta un método nuevo del robot, se
   agrega ahí, con su dirección documentada en
   `docs/MODBUS_REGISTER_MAP.md`.
6. **No agregues dependencias nuevas sin decirlo explícitamente en tu
   resumen final.** El stack es: `lark` (parser), `pymodbus` (Modbus),
   `PySide6` (GUI y 3D con Qt3D), `gmsh` (lectura de STEP), `python-fcl` +
   `numpy` (choques del simulador), `pytest` (tests). Cualquier librería nueva debe
   justificarse.

## Estado actual (ver también README.md, que puede estar más actualizado)

**Tests en verde** (`QT_QPA_PLATFORM=offscreen` para los de GUI sin display).

- **`pad/`**: `backup.py` lee y escribe respaldos byte a byte (acciones como
  `dict` crudos); `listing.py` los muestra legibles; `template.fnc` es el
  `.fnc` del respaldo real, que se copia en los respaldos nuevos.
- **`compiler/pad_codegen.py`**: AST -> respaldo del pad. `MOVEJ` exige punto
  `JOINT(...)` y `MOVEL` `WORLD(...)`. Sin `VAR`, `ELSE`, `WAIT_IN` ni
  parámetros (error de compilación).
- **`compiler/codegen.py` + `runtime/vm.py`**: bytecode y VM de referencia,
  solo para simular la lógica en la PC.
- **`comms/robot_client.py`**: Modbus del robot, para digitalizar puntos y
  simular contra el robot real.
- **`sim/`**: simulador cinemático del respaldo del pad. Robot por defecto:
  **BRTIRUS1510A** (el de la celda), con cotas y mallas sacadas del STEP del
  fabricante (`python -m sim.robot_import`). Velocidades confirmadas por el usuario;
  rangos de la misma tabla, sin confirmar. También está el BRTIRUS1820A
  (cilindros, del plano). Pestaña "Simulación 3D" con layout de
  piezas STEP/STL/OBJ y búsqueda de choques (`sim/collision.py`, python-fcl:
  piezas como sólidos, piso, el propio brazo, herramienta montada en la
  brida), gráficas de movimiento (perfil de aceleración supuesto), navegación
  tipo Inventor, ubicación de piezas por distancias y biblioteca en línea de
  Google Drive (`sim/library.py`). Ver `docs/SIMULATOR.md`.
- **GUI** (`gui/`): "Exportar para el pad" (Ctrl+E) + pestaña "Pad" con el
  listado; compilar/ejecutar en la VM; digitalizar puntos.
- **`packaging/`** + `.github/workflows/build-exe.yml`: `.exe` de un solo
  archivo construido en un runner Windows, con `--self-test` como criterio.

**Skill `actualizar-simulador`** (`.claude/skills/`): revisar la carpeta de
Drive (`python -m sim.library`), importar un robot nuevo, publicar el `.exe`.

## Cosas que ya se aprendieron a golpes (no repetirlas)

1. **`compiler/grammar.lark` y `pad/template.fnc` se leen como archivos en
   runtime.** Cualquier forma
   de empaquetar o mover el proyecto tiene que llevarlo. Sin él el programa
   abre bien y falla al compilar, y con `console=False` el error no se ve.
   `python -m gui.app --self-test` lo detecta.
2. **No soltar la referencia a un `QThread` que todavía está saliendo.** Qt
   destruye el objeto C++ por debajo y el proceso *crashea* — no es una
   excepción, se lleva puesto al pytest sin dejar ni el resumen. Hacer
   `quit()` + `wait()` antes de poner la referencia en `None`.
3. **No guardar dicts en los datos de un item de Qt** si el orden importa: se
   convierten a `QVariantMap`, que está ordenado por clave. Usar lista de
   pares.
4. **En modo windowed PyInstaller deja `sys.stdout` en `None`** y un `print()`
   pelado revienta. Ver `_emit()` en `gui/app.py`.
5. **PowerShell no espera a un ejecutable sin consola.** `& $exe` devuelve
   `$LASTEXITCODE` 0 pase lo que pase; hay que usar
   `Start-Process -Wait -PassThru` y mirar `ExitCode`.
6. **Qt3D sin OpenGL hace caer el proceso** (segmentation fault, no
   excepción). `gui/viewport3d.py: opengl_available()` se consulta SIEMPRE
   antes de crear la vista, y los tests nunca la construyen. Para verla en un
   contenedor: `xvfb-run` + `QT_QPA_PLATFORM=xcb`.
7. **gmsh carga su DLL con ctypes**: PyInstaller no la ve. El spec la agrega
   a mano (`GMSH_BINARIES`) y el self-test importa un STEP para verificarlo.
   Lo mismo con las DLL de python-fcl (`FCL_BINARIES`); si la extensión no
   carga, `fcl` se importa vacío (sin excepción): `sim.collision.available()`
   lo detecta.
8. **El resaltado necesita dos paletas.** Una sola pensada para fondo blanco
   queda ilegible sobre el tema oscuro de Windows. Hay un test de contraste
   WCAG 3.0:1.

## Tareas priorizadas

### Probar en el pad (bloqueada hasta tener el pad)

Primero importar un respaldo reescrito sin cambios por `pad/backup.py`;
después uno generado, a velocidad baja. Ver `docs/PAD_FORMAT.md`.

### Decodificar lo que falta del pad (necesita respaldos de ejemplo)

Espera de entrada (`WAIT_IN`), salto incondicional (`ELSE`), variables. Cada
uno necesita un respaldo exportado del pad que lo use.

### Herramienta, coordenadas y plantilla desde la GUI

Hoy se exporta con herramienta 0 / coordenadas 0 y el `.fnc` del respaldo
analizado. `PadOptions` ya acepta `tool`, `coord` y `template`; falta la UI.

### Digitalizar en JOINT

F8 lee la posición cartesiana; para usar un punto digitalizado en un `MOVEJ`
hace falta leer los ejes (`read_axis_position()`) y escribir `JOINT(...)`.

### Otros

- Persistencia de puntos digitalizados (JSON en la PC).
- Recordar la disposición de los paneles (`QMainWindow.saveState`).
- Renombrar `runtime/plc_io_simulator.py` (resto del PLC).

### Completadas

Tareas A-D (codegen de dos pasadas, GUI, afinado). Tarea C (cliente del PLC) y
E (panel del PLC) quedaron sin efecto al descartar el PLC.

## Cómo correr todo

```bash
python -m venv venv && source venv/bin/activate
pip install -r requirements.txt
QT_QPA_PLATFORM=offscreen pytest tests/ -v
python -m gui.app --self-test
```

Para construir el `.exe` (solo en Windows): `.\packaging\build_exe.ps1`
