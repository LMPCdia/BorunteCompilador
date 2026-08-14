# borunte-dsl

Lenguaje tipo KRL para orquestar un robot Borunte a través de un PLC Coolmay
CX3G vía Modbus. Ver `docs/ARCHITECTURE.md` primero, y `docs/SIMULATION.md`
si no tenés el hardware a mano (spoiler: no lo necesitás para seguir
avanzando).

## Estado actual

- [x] Arquitectura de 2 niveles definida y documentada
- [x] Mapa de registros Modbus del Borunte consolidado
- [x] Contrato de bytecode v0.1 (DRAFT, sin validar contra hardware)
- [x] Cliente Modbus del robot (`comms/robot_client.py`)
- [x] **Simulador del robot** (`comms/robot_simulator.py`) — permite probar
      todo sin hardware; supuestos documentados en `docs/SIMULATION.md`
- [x] Gramática v0.1 del DSL (`compiler/grammar.lark`)
- [x] Codegen para el subconjunto lineal: `POINT`, `VAR` (init constante),
      `MOVEJ`/`MOVEL`, `WAIT_IN`, `SET_OUT`, `WAIT`
- [x] VM de referencia en Python (`runtime/vm.py`)
- [x] **Test de punta a punta**: DSL → parser → bytecode → VM → robot
      simulado, pasando (`tests/test_end_to_end.py`)
- [ ] Codegen para `IF`/`ELSE` — **incompleto a propósito, ver abajo**
- [ ] Codegen para `PROC`/llamadas a subrutina — **incompleto a propósito**
- [ ] VM en ladder/IL para el CX3G real (bloqueada hasta tener hardware)
- [ ] GUI (PySide6) — no arrancada todavía

## Limitación conocida: `IF` y `PROC` no compilan

El `Transformer` de Lark que usamos en `compiler/codegen.py` procesa el
árbol de abajo hacia arriba (bottom-up). Eso funciona perfecto para
sentencias lineales, pero se rompe para `IF/ELSE` y `PROC` porque necesitan
saber la dirección (PC) de instrucciones que todavía no se emitieron. Ambos
casos están marcados con `raise CompileError(...)` y una explicación en el
código — a propósito, para no generar bytecode incorrecto en silencio.

**La solución** es pasar a un compilador de dos pasadas: primero armar un
AST real (con una clase propia, no el árbol crudo de Lark), y recién ahí
generar bytecode con resolución de saltos por backpatching. Es un cambio
acotado a `compiler/codegen.py` — no afecta a `runtime/`, `comms/` ni al
diseño del bytecode en sí.

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

1. Arreglar el compiler de dos pasadas para que `IF`/`PROC` compilen.
2. Arrancar `gui/` — puede desarrollarse contra el simulador sin esperar
   nada más.
3. Cuando llegue el hardware: seguir la guía de `docs/SIMULATION.md` →
   "Próximo hito cuando llegue el hardware".

## Trabajo en paralelo (Claude Code)

Con el estado actual, ya se puede repartir:

- Sesión A → arreglar `compiler/codegen.py` (dos pasadas, IF/PROC)
- Sesión B → arrancar `gui/` contra `comms/robot_simulator.py`
- Sesión C → ampliar `comms/` con el cliente Modbus del PLC (CX3G), en
  espejo de `robot_client.py`

`plc_vm/` sigue sin ser delegable a un agente: necesita a alguien con
GX Developer/Works2 y, eventualmente, el hardware real.
