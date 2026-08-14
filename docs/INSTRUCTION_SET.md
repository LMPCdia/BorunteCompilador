# Instruction Set v0.1 (DRAFT — a validar contra hardware real)

Este es el contrato entre `compiler/` (que lo genera), `comms/` (que lo
transporta) y `plc_vm/` (que lo ejecuta en ladder/IL). Cualquier cambio acá
rompe a los tres.

## Formato de instrucción

Cada instrucción ocupa un registro fijo de **8 words** (16 bytes) en el PLC,
para que el Program Counter sea `PC * 8` y la dirección sea trivial de
calcular en ladder (sin aritmética variable de longitud).

```
word 0:      opcode
word 1:      operand A (uso depende del opcode)
word 2-3:    operand B (32 bits — típicamente un valor o índice)
word 4-5:    operand C (32 bits)
word 6:      operand D (16 bits)
word 7:      reservado / flags
```

Base de la tabla de bytecode: **a definir** (propuesta: `D1000` en adelante,
dejando `D0-D999` libres para uso general del PLC).

## Tabla de puntos

Separada del bytecode. Cada punto = 6 valores de 32 bits (X,Y,Z,U,V,W o
J1-J6) = **12 words por punto**.

Base propuesta: `D4000` en adelante (soporta ~250 puntos antes de tocar
`D8000`; si hace falta más, usar registros `R` extendidos).

## Opcodes v0.1

| Opcode | Nombre | Operandos | Descripción |
|---|---|---|---|
| `0x00` | `NOP` | — | No hace nada |
| `0x01` | `MOVEJ` | B=índice de punto, D=velocidad | Mueve por articulaciones. Escribe punto en bloque 800-890, dispara, hace poll de `0x09A6` |
| `0x02` | `MOVEL` | B=índice de punto, D=velocidad | Mueve en línea recta (cartesiano) |
| `0x03` | `WAIT_IN` | A=número de entrada, B=timeout(ms) | Espera entrada del PLC (no del robot) |
| `0x04` | `SET_OUT` | A=número de salida, B=estado (0/1) | Fuerza salida |
| `0x05` | `WAIT_TIME` | B=milisegundos | Timer bloqueante |
| `0x06` | `JUMP` | B=PC destino | Salto incondicional |
| `0x07` | `JUMP_IF_ZERO` | A=índice de variable, B=PC destino | Salto condicional (variable == 0). Reservado para futuro uso; el codegen v0.1 usa `0x0D` en su lugar (ver nota abajo) |
| `0x0D` | `JUMP_IF_VAR_NEQ_CONST` | A=índice de variable, B=constante, C=PC destino | Salta a C si `var != const`. **Agregado en la iteración del codegen (no estaba en el diseño original v0.1)** — es lo que permite compilar `IF var == const THEN ... ELSE ... ENDIF`. Limitación actual: el codegen SOLO soporta esta forma exacta de condición (`VAR == CONST`), no `VAR == VAR` ni expresiones compuestas. |
| `0x08` | `CALL` | B=PC destino | Llama subrutina (usa pila de retorno del PLC) |
| `0x09` | `RET` | — | Vuelve de subrutina |
| `0x0A` | `SET_VAR` | A=índice var, B=valor inmediato | Asigna constante |
| `0x0B` | `ADD_VAR` | A=índice var, B=operando | var += operando |
| `0x0C` | `COPY_POINT_OFFSET` | A=punto origen, B=punto destino, C=offset (referencia a 6 words) | Calcula punto_destino = punto_origen + offset |
| `0xFE` | `ALARM_CLEAR_CONTINUE` | — | Manda `0x4E26` al robot |
| `0xFF` | `END` | — | Fin de programa |

> Este set es intencionalmente mínimo. Se amplía SOLO después de validar el
> PoC (fase 2 del roadmap) — agregar opcodes antes de probar la VM real es
> desperdiciar trabajo.

## Registros de control de la VM (a coordinar con `plc_vm/`)

| Registro (propuesto) | Uso |
|---|---|
| `D0` | Program Counter actual |
| `D1` | Estado de la VM (0=parado, 1=corriendo, 2=error) |
| `D2` | Código de error actual |
| `D10-D109` | Banco de 100 variables de usuario (`VAR`) |

## Qué falta definir en el PoC (fase 2)

- Cómo se codifica exactamente `MOVEJ` vs `MOVEL` en la escritura al bloque
  800-890 del robot (ver duda abierta sobre "free path"/"posture line").
- Timeout máximo razonable de poll para `0x09A6` antes de considerarlo error.
- Cómo se maneja una alarma del robot en medio de una secuencia (¿la VM se
  detiene, reintenta, o llama `ALARM_CLEAR_CONTINUE` automáticamente?).
