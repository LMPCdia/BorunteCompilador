# borunte-dsl

Lenguaje tipo KRL para orquestar un robot Borunte a través de un PLC Coolmay
CX3G vía Modbus. Ver `docs/ARCHITECTURE.md` primero, y `docs/SIMULATION.md`
si no tenés el hardware a mano (spoiler: no lo necesitás para seguir
avanzando).

## Estado actual

- [x] Arquitectura de 2 niveles definida y documentada
- [x] Mapa de registros Modbus del Borunte consolidado
- [x] **Contrato de bytecode v0.2** (`docs/INSTRUCTION_SET.md`, DRAFT sin
      validar contra hardware): opcodes, formato de 8 words, calling
      convention de `PROC`, y el bloque de registros de control de la VM
- [x] Cliente Modbus del robot (`comms/robot_client.py`)
- [x] **Simulador del robot** (`comms/robot_simulator.py`) — permite probar
      todo sin hardware; supuestos documentados en `docs/SIMULATION.md`
- [x] Gramática v0.2 del DSL (`compiler/grammar.lark`)
- [x] **AST propio de dos pasadas** (`compiler/ast_nodes.py` +
      `compiler/ast_builder.py`) — reemplaza el Transformer de una sola
      pasada que no podía resolver saltos hacia adelante
- [x] **Codegen v0.3 completo**: `POINT`, `VAR`, `MOVEJ`/`MOVEL`, `WAIT_IN`,
      `SET_OUT`, `WAIT`, `IF`/`ELSE` (con `VAR == CONST` **y `VAR == VAR`**),
      `PROC`/llamadas **con parámetros** (incluye llamadas hacia adelante), y
      asignación entre variables
- [x] VM de referencia en Python (`runtime/vm.py`), con parada
      (`request_stop()`)
- [x] **Cliente Modbus del PLC CX3G** (`comms/plc_client.py`): carga el
      bytecode, arranca/para la VM del ladder y lee su estado en vivo.
      ⚠️ Todo el nivel PLC es propuesta **sin confirmar** — ver abajo.
- [x] **GUI v0.2 (PySide6)** con paneles acoplables al estilo WorkVisual:
      editor con resaltado de sintaxis, estructura del proyecto navegable,
      propiedades, ventana de mensajes, campos de trabajo, digitalización de
      puntos, y ejecutar/parar en un hilo aparte con log en vivo
- [x] **Empaquetado**: `dist/BorunteDSL.exe` de un solo archivo, con
      `--self-test` como criterio de aceptación
- [x] **154 tests en verde**
- [ ] Panel de la GUI para la VM del PLC — `comms/plc_client.py` existe pero
      todavía nadie lo usa desde la interfaz (ver "Próximos pasos")
- [ ] VM en ladder/IL para el CX3G real (bloqueada hasta tener hardware)

## Lo que NO está confirmado contra hardware

Nada de esto se probó contra un robot o un PLC real. Está marcado como
hipótesis en el código y en la documentación, no como hecho.

**Del PLC** (ver el docstring de `comms/plc_client.py`, que las lista
ordenadas por riesgo):

1. Que el registro `Dn` del PLC se lea/escriba como holding register Modbus
   número `n`. Parametrizado en `d_register_base` para corregirlo en un solo
   lugar.
2. Que los enteros de 32 bits se guarden con el **word bajo primero**
   (convención FX de Mitsubishi) — **al revés que el robot Borunte**, donde el
   ejemplo confirmado del manual manda el word alto primero. Las dos
   convenciones conviven a propósito. **Es la que más conviene validar
   primero**: si está al revés, todo carga "bien" y ejecuta cualquier cosa.
3. Que el mapa de control (`D0-D109`) y las bases de las tablas (`D1000`,
   `D4000`) sean los del contrato — que todavía nadie implementó en ladder.
4. Que el handshake de comandos funcione como está documentado (el host
   escribe en `D3`, la VM lo devuelve a `0` al aceptarlo).

**Del robot** (ya estaba en `docs/SIMULATION.md`): que escribir una pose en el
bloque `800-890` y disparar "Start" mueva el robot, y con qué latencia por
transacción.

## Limitaciones conocidas (documentadas a propósito, no bugs escondidos)

Del compilador:

- `IF` solo soporta `==`: `VAR == CONST` o `VAR == VAR`, no expresiones
  compuestas ni otros operadores. Intentarlo tira `CompileError` clara, no
  bytecode incorrecto.
- La calling convention de `PROC` usa **slots fijos, sin pila de frames**: no
  es recursiva ni reentrante, y los parámetros son por valor. Está documentado
  en el contrato para que `plc_vm/` respete las mismas reglas.
- La tabla de puntos **no se deduplica**: cada `MOVEJ`/`MOVEL` agrega una
  entrada aunque mueva a un `POINT` ya declarado (el programa de ejemplo da 5
  entradas para 2 puntos). La GUI lo advierte al compilar. Importa porque en el
  PLC caben 333.
- Los offsets de puntos (`p + OFFSET(...)`) se resuelven en tiempo de
  compilación, no en runtime.
- No hay bucles (`WHILE`/`FOR`), y `TIMER` se parsea pero no emite nada.

De la parada:

- `PARAR` corta en el próximo límite de instrucción y **no interrumpe el
  movimiento en curso**. Un paro de emergencia de verdad va por una línea
  física al robot, no por Modbus ni por ese botón.

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

Se abre con un programa de ejemplo ya cargado. Flujo: "Conectar" (dejá
"Simulador" tildado si no tenés hardware) → **F7** compilar → **F5** ejecutar.
**Shift+F5** para, **F8** digitaliza el punto actual del robot.

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
`compiler/grammar.lark` se lee como archivo en tiempo de ejecución, así que un
empaquetado incompleto produce un ejecutable que abre bien y falla al compilar
el primer programa — y con `console=False` ese error no se ve en ninguna parte.

## Estructura

```
compiler/    parser (Lark) + AST de dos pasadas + codegen → bytecode
comms/       robot_client.py (robot), plc_client.py (PLC CX3G),
             robot_simulator.py + fake_modbus.py (para probar sin hardware)
runtime/     bytecode.py (Instruction/Program/opcodes), vm.py (VM de
             referencia), plc_io_simulator.py (E/S simulada del PLC)
pad/         backup.py (leer/escribir HCBackupRobot_*.zip del pad),
             listing.py (listado legible) — ver docs/PAD_FORMAT.md
gui/         main_window.py + paneles (project_tree, properties_panel,
             message_window, work_fields, connection_panel,
             syntax_highlighter), vm_worker.py, app.py (--self-test)
packaging/   spec de PyInstaller, build_exe.ps1, datafiles.py
plc_vm/      SOLO documentación — la VM real se escribe a mano en
             GX Developer/Works2 una vez validado el contrato con hardware
tests/
docs/        ARCHITECTURE.md, MODBUS_REGISTER_MAP.md, INSTRUCTION_SET.md,
             SIMULATION.md, PAD_FORMAT.md
```

## Próximos pasos sugeridos, en orden

1. **Panel de la GUI para la VM del PLC.** `comms/plc_client.py` sabe cargar
   el programa, arrancarlo, pararlo y leer el Program Counter en vivo, pero
   nadie lo usa: hoy el "Ejecutar" de la barra corre la VM de referencia en la
   PC, no en el CX3G. Falta el panel que cargue el programa compilado y muestre
   PC + estado + banco de variables en vivo. El poll del estado va en un hilo
   aparte, como `gui/vm_worker.py`, no en el hilo de la UI.
2. **Deduplicar la tabla de puntos en el codegen.** Cambia los índices del
   bytecode, así que hay que hacerlo con los tests delante.
3. **Persistencia de los puntos digitalizados** (decisión de diseño 3 de
   `docs/ARCHITECTURE.md`: SQLite/JSON en la PC). Hoy viven solo en memoria de
   la GUI y se pierden al cerrarla.
4. Recordar la disposición de los paneles entre sesiones
   (`QMainWindow.saveState`).
5. Cuando llegue el hardware: seguir `docs/SIMULATION.md` → "Próximo hito
   cuando llegue el hardware", y validar primero la hipótesis 2 del PLC (orden
   de los words de 32 bits).

`plc_vm/` sigue sin ser delegable a un agente: necesita a alguien con
GX Developer/Works2 y, eventualmente, el hardware real.
