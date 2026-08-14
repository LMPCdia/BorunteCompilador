# borunte-dsl

Lenguaje tipo KRL para orquestar un robot Borunte a través de un PLC Coolmay
CX3G vía Modbus. Ver `docs/ARCHITECTURE.md` primero, y `docs/SIMULATION.md`
si no tenés el hardware a mano (spoiler: no lo necesitás para seguir
avanzando).

## Estado actual

- [x] Arquitectura de 2 niveles definida y documentada
- [x] Mapa de registros Modbus del Borunte consolidado
- [x] Contrato de bytecode v0.2 (DRAFT, sin validar contra hardware — se le
      agregó `JUMP_IF_VAR_NEQ_CONST` respecto de la v0.1 original)
- [x] Cliente Modbus del robot (`comms/robot_client.py`)
- [x] **Simulador del robot** (`comms/robot_simulator.py`) — permite probar
      todo sin hardware; supuestos documentados en `docs/SIMULATION.md`
- [x] Gramática v0.1 del DSL (`compiler/grammar.lark`)
- [x] **AST propio de dos pasadas** (`compiler/ast_nodes.py` +
      `compiler/ast_builder.py`) — reemplaza el Transformer de una sola
      pasada que no podía resolver saltos hacia adelante
- [x] **Codegen v0.2 completo**: `POINT`, `VAR`, `MOVEJ`/`MOVEL`,
      `WAIT_IN`, `SET_OUT`, `WAIT`, **`IF`/`ELSE`**, **`PROC`/llamadas**
      (incluye llamadas hacia adelante — el PROC puede definirse después
      del CALL en el texto)
- [x] VM de referencia en Python (`runtime/vm.py`)
- [x] Tests de punta a punta: lineal (`test_end_to_end.py`) + control de
      flujo `IF`/`PROC` (`test_control_flow.py`) — **15/15 en verde**
- [x] **GUI v0.1 (PySide6)**: editor + botón Compilar (muestra bytecode y
      tabla de puntos, o el error de compilación) + panel de conexión
      (simulador o robot real por IP) + botón Ejecutar (corre en un hilo
      aparte, log en vivo) — probada headless en `tests/test_gui_smoke.py`
- [ ] VM en ladder/IL para el CX3G real (bloqueada hasta tener hardware)
- [ ] GUI: resaltado de sintaxis, captura de puntos en vivo desde el robot
      (falta la función de "digitalizar", ver `docs/ARCHITECTURE.md`)

## Limitaciones conocidas del codegen v0.2 (documentadas a propósito, no bugs escondidos)

- `IF` solo soporta condiciones de la forma `VAR == CONST` — no
  `VAR == VAR` ni expresiones compuestas. Intentarlo tira `CompileError`
  clara, no bytecode incorrecto.
- `PROC` no tiene calling convention: los parámetros se parsean pero no se
  bindean a nada. Usá `VAR` globales si necesitás pasar datos a un `PROC`
  por ahora.
- Los offsets de puntos (`p + OFFSET(...)`) se resuelven en tiempo de
  compilación, no en runtime.

## Setup

```bash
python -m venv venv
source venv/bin/activate   # o venv\Scripts\activate en Windows
pip install -r requirements.txt
pytest tests/ -v -s
```

El `-s` muestra el bytecode generado y el trace de ejecución de
`test_end_to_end_against_simulator` — es la forma más rápida de ver todo el
pipeline funcionando.

Para abrir la GUI:

```bash
python -m gui.app
```

Se abre con un programa de ejemplo ya cargado. Flujo: "Conectar" (dejá
"Simulador" tildado si no tenés hardware) → "Compilar" → "Ejecutar". El tab
"Log de ejecución" muestra el trace en vivo.

## Estructura

```
compiler/    parser (Lark) + codegen → bytecode (ver limitación de IF/PROC)
comms/       cliente Modbus real + simulador del robot
runtime/     bytecode.py (Instruction/Program), vm.py (VM de referencia),
             plc_io_simulator.py (E/S simulada del PLC)
gui/         (vacío todavía)
plc_vm/      SOLO documentación — la VM real se escribe a mano en
             GX Developer/Works2 una vez validado el contrato con hardware
tests/
docs/        ARCHITECTURE.md, MODBUS_REGISTER_MAP.md, INSTRUCTION_SET.md,
             SIMULATION.md
```

## Próximos pasos sugeridos, en orden

1. Resaltado de sintaxis en el editor de la GUI (`gui/main_window.py`,
   `QSyntaxHighlighter` sobre el `QPlainTextEdit`).
2. Función de "digitalizar punto": botón en la GUI que, con el robot
   conectado, lea la posición actual (`robot.read_world_position()`) y la
   agregue a una tabla editable de puntos con nombre — hoy la tabla de
   puntos es de solo lectura, poblada desde el bytecode compilado.
3. Cuando llegue el hardware: seguir la guía de `docs/SIMULATION.md` →
   "Próximo hito cuando llegue el hardware".

## Trabajo en paralelo (Claude Code)

- Sesión A → resaltado de sintaxis + digitalización de puntos en `gui/`
- Sesión B → soporte de `VAR == VAR` en `IF` y calling convention real para
  `PROC` en `compiler/codegen.py`
- Sesión C → cliente Modbus del PLC (CX3G), en espejo de
  `comms/robot_client.py`

`plc_vm/` sigue sin ser delegable a un agente: necesita a alguien con
GX Developer/Works2 y, eventualmente, el hardware real.
