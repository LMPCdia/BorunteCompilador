# CLAUDE.md — Contexto del proyecto para Claude Code

Este archivo se lee automáticamente al abrir Claude Code en esta carpeta.
Contiene lo que un colaborador nuevo necesitaría saber antes de tocar código.

## Qué es esto

Lenguaje tipo KRL para programar un robot Borunte a través de un PLC Coolmay
CX3G, vía Modbus. Arquitectura de 2 niveles — LEER PRIMERO:
`docs/ARCHITECTURE.md`, después `docs/MODBUS_REGISTER_MAP.md`,
`docs/INSTRUCTION_SET.md`, `docs/SIMULATION.md`.

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

- Gramática v0.1 (`compiler/grammar.lark`) parsea el subconjunto completo
  del lenguaje, incluyendo `IF`/`PROC`.
- **Codegen v0.2 (`compiler/codegen.py`) YA soporta `IF`/`ELSE` y
  `PROC`/`CALL`** (incluye llamadas hacia adelante). Ver limitaciones
  documentadas en el docstring de ese archivo (condiciones solo
  `VAR == CONST`, sin calling convention para parámetros de PROC).
- `compiler/ast_nodes.py` + `compiler/ast_builder.py`: AST propio de dos
  pasadas (reemplazó al Transformer de una sola pasada que no podía
  resolver saltos hacia adelante).
- `runtime/vm.py` (VM de referencia en Python) soporta todos los opcodes de
  `docs/INSTRUCTION_SET.md`.
- 15/15 tests en verde (`tests/test_end_to_end.py` + `tests/test_control_flow.py`
  + `tests/test_gui_smoke.py`).
- **GUI v0.1 ya existe** (`gui/main_window.py`, `gui/connection_panel.py`,
  `gui/vm_worker.py`, `gui/app.py`): editor, compilar, panel de conexión
  (simulador/real), ejecutar en background thread con log en vivo. Probada
  headless (offscreen) en `tests/test_gui_smoke.py`. Falta: resaltado de
  sintaxis, captura de puntos en vivo desde el robot ("digitalizar").

## Tareas priorizadas

### Tarea A — ~~Codegen de dos pasadas~~ COMPLETADA
### Tarea B — ~~GUI v0.1~~ COMPLETADA (ver arriba lo que falta afinar)

Ver `compiler/ast_nodes.py`, `compiler/ast_builder.py`, `compiler/codegen.py`.
Si tocás estos archivos, correr `pytest tests/ -v` antes de dar por
terminado — hay 15 tests que cubren lineal + IF/ELSE + PROC/CALL + GUI.

Pendiente dentro del compiler, si hay tiempo: soportar `VAR == VAR` en `IF`
(hoy solo `VAR == CONST`) y una calling convention real para parámetros de
`PROC` (hoy se parsean pero no hacen nada).

### Tarea D — Afinar la GUI

- Resaltado de sintaxis (`QSyntaxHighlighter`) sobre el editor.
- Botón "Digitalizar punto": con el robot conectado, leer
  `robot.read_world_position()` y agregarlo a una tabla de puntos con
  nombre editable por el usuario — hoy la tabla de puntos es de solo
  lectura (se llena desde `program.points` después de compilar).
- Correr `pytest tests/test_gui_smoke.py -v` (con `QT_QPA_PLATFORM=offscreen`
  si no hay display) antes de dar por terminada cualquier tarea acá.

### Tarea C — Cliente Modbus del PLC (CX3G)

Espejo de `comms/robot_client.py` pero para el PLC en sí (no el robot) —
necesario para cuando la VM real corra en el CX3G y haya que
leer/escribir su estado desde la GUI (arrancar/parar la VM, ver el PC
actual, etc.). Definir el mapeo de registros de control de la VM
(`docs/INSTRUCTION_SET.md`, sección "Registros de control de la VM") antes
de escribir el cliente.

## Cómo correr todo

```bash
python -m venv venv && source venv/bin/activate
pip install -r requirements.txt
pytest tests/ -v -s
```
