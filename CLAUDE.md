# CLAUDE.md — Contexto del proyecto para Claude Code

Este archivo se lee automáticamente al abrir Claude Code en esta carpeta.
Contiene lo que un colaborador nuevo necesitaría saber antes de tocar código.

## Qué es esto

Lenguaje tipo KRL para programar un robot Borunte.

**El backend de compilación PRIMARIO es `compiler/act_backend.py`**, que emite
el formato nativo `.act` del robot. Modbus quedó reducido a lo que ya estaba
documentado: orquestación remota del programa ya importado y coordinación con
E/S de celda.

LEER PRIMERO: `docs/ARCHITECTURE.md` y `docs/PAD_PROGRAM_FORMAT.md`, después
`docs/MODBUS_REGISTER_MAP.md`, `docs/INSTRUCTION_SET.md`, `docs/SIMULATION.md`.

No hay hardware físico disponible todavía. Todo se desarrolla y prueba
contra `comms/robot_simulator.py`, un simulador en software cuyos supuestos
de comportamiento están documentados (y marcados como hipótesis, no hechos)
en `docs/SIMULATION.md`.

## Reglas del proyecto (leer antes de escribir código)

1. **Nunca inventes comportamiento del hardware real como si estuviera
   confirmado.** Si algo no está en `docs/MODBUS_REGISTER_MAP.md` como
   confirmado, tratalo como hipótesis y decilo explícitamente en el código
   (comentario) y en tu respuesta al usuario.
2. **`plc_vm/` no se toca con código.** Es la VM que se escribe a mano en
   GX Developer/Works2 contra el PLC real. Esa carpeta es solo documentación
   y pseudocódigo. No generes ladder ni intentes automatizarlo.
3. **`docs/INSTRUCTION_SET.md` es un contrato.** Si tu tarea requiere
   cambiarlo (agregar un opcode, cambiar el formato de instrucción), parate
   y avisá antes de seguir — ese archivo lo consumen `compiler/`, `runtime/`
   y (eventualmente) `plc_vm/`. Un cambio silencioso rompe a los otros dos.
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
   `PySide6` (GUI), `pytest` (tests). Cualquier librería nueva debe
   justificarse.

## Estado actual (ver también README.md, que puede estar más actualizado)

**275 tests en verde.** Los de GUI necesitan `QT_QPA_PLATFORM=offscreen` si no
hay display.

- **Backend `.act`** (`compiler/act_backend.py`, PRIMARIO) + lector
  (`compiler/act_reader.py`). Compila el DSL al formato nativo del robot. El
  formato salió de ingeniería inversa sobre un export real de 338 KB y está
  documentado en `docs/PAD_PROGRAM_FORMAT.md`, con cada afirmación marcada como
  confirmada o hipótesis. Restricciones del formato: sin `ELSE` (no hay salto
  incondicional), `IF` solo sobre entradas físicas, sin variables internas,
  `PROC` con id explícito y sin parámetros.
- **Contrato v0.2** (`docs/INSTRUCTION_SET.md`, backend anterior del PLC): opcodes hasta `0x0F`, formato
  de 8 words con los 32 bits en **word bajo primero** del lado del PLC,
  calling convention de `PROC`, y el bloque de registros de control de la VM
  (`D0-D7`, handshake por `D3`, bases `D1000`/`D4000`).
- **Compilador v0.3** (`compiler/`): `IF` con `VAR == CONST` y `VAR == VAR`,
  `PROC` con parámetros por slots fijos, asignación entre variables,
  comentarios y líneas en blanco en cualquier parte, archivo sin salto de línea
  final. Pasada 0 (`_collect_proc_signatures`) para que las llamadas hacia
  adelante también carguen sus parámetros.
- **`runtime/vm.py`**: soporta todos los opcodes del contrato y tiene
  `request_stop()` para cortar en el próximo límite de instrucción.
- **`comms/plc_client.py`**: `CoolmayPlcClient`, espejo de
  `BorunteRobotClient` pero para el PLC. Carga bytecode + puntos, handshake de
  comandos, banco de variables, relectura para verificar, troceado de
  transacciones.
- **GUI v0.2** (`gui/`): `QMainWindow` con paneles acoplables — estructura del
  proyecto (navega al código), propiedades (muestra el registro `D` destino),
  ventana de mensajes, campos de trabajo, resaltado de sintaxis con dos
  paletas, digitalización de puntos, y ejecutar/parar en un hilo aparte.
- **`comms/plc_simulator.py`**: CX3G simulado que EJECUTA el bytecode. No es
  el ladder (`plc_vm/` sigue sin código, regla 2) — es la especificación
  ejecutable de lo que ese ladder tiene que hacer. Cada rama de su `_step()`
  debería tener un equivalente en una red de ladder.
- **Panel de la VM del PLC** (`gui/plc_panel.py` + `gui/plc_status_worker.py`,
  Tarea E): carga el programa en el PLC y lo ejecuta ahí, con el poll del
  estado en un hilo aparte.
- **`packaging/`**: `.exe` de un solo archivo, con `--self-test` como criterio
  de aceptación.

## Cosas que ya se aprendieron a golpes (no repetirlas)

1. **`compiler/grammar.lark` se lee como archivo en runtime.** Cualquier forma
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
6. **El resaltado necesita dos paletas.** Una sola pensada para fondo blanco
   queda ilegible sobre el tema oscuro de Windows. Hay un test de contraste
   WCAG 3.0:1.
7. **No dar de baja un hilo de poll en cada operación: pausarlo.** Bajarlo deja
   señales encoladas que se entregan cuando el worker ya fue recolectado, y Qt
   toca un objeto C++ destruido. Ver `gui/plc_status_worker.py` y el
   `is_idle()` que el panel espera antes de tocar el socket.
8. **El `.act` tiene el `program` de la biblioteca codificado DOS veces**: es
   un string con JSON adentro. Y el orden de ejecución es el del array, NO el
   campo `insertedIndex` (ese es el orden en que el operario insertó las líneas
   en el pad).
9. **El exit code que reporta el shell de Bash acá no es confiable** (devuelve
   127 con la salida completa y exit real 0). Si hace falta el código de salida
   de verdad, medirlo con PowerShell + `Start-Process -Wait -PassThru`. Un
   crash real se reconoce distinto: corta la salida a la mitad, sin resumen.

## Tareas priorizadas

### Tarea F — Deduplicar la tabla de puntos en el codegen

Cada `MOVEJ`/`MOVEL` registra una entrada nueva en `program.points` aunque
mueva a un `POINT` ya declarado. En el programa de ejemplo: **5 entradas para 2
puntos declarados**. No es un error de corrección, pero se come los 333 lugares
que tiene el PLC. Cambia los índices del bytecode, así que hay que hacerlo con
cuidado y con los tests delante.

### Tarea G — Persistencia de puntos digitalizados

Decisión de diseño 3 de `docs/ARCHITECTURE.md` (SQLite/JSON en la PC). Hoy
viven solo en memoria de la GUI y se pierden al cerrarla.

### Otros

- Recordar la disposición de los paneles entre sesiones
  (`QMainWindow.saveState`).
- Del lenguaje: no hay bucles (`WHILE`/`FOR`), `IF` solo soporta `==`, `TIMER`
  se parsea pero no emite nada, y los `OFFSET` se resuelven en compilación.
- `plc_vm/` (la VM en ladder) sigue bloqueada hasta tener el CX3G delante, y no
  es delegable: necesita GX Developer/Works2.

### Completadas

Tarea A (codegen de dos pasadas), Tarea B (GUI v0.1), Tarea C (cliente del
PLC), Tarea D (afinar la GUI), Tarea E (panel de la VM del PLC).

## Cómo correr todo

```bash
python -m venv venv && source venv/bin/activate
pip install -r requirements.txt
QT_QPA_PLATFORM=offscreen pytest tests/ -v
python -m gui.app --self-test
```

Para construir el `.exe` (solo en Windows): `.\packaging\build_exe.ps1`
