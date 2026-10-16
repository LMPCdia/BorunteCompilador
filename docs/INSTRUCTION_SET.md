# Instruction Set v0.2 — bytecode de la VM de referencia

Este bytecode lo genera `compiler/codegen.py` y lo ejecuta `runtime/vm.py`
**en la PC**, para simular un programa contra `comms/robot_simulator.py` (o
contra el robot real por Modbus) antes de exportarlo.

> **Ya no es el formato que corre en producción.** Desde que se descartó el
> PLC, el programa lo ejecuta el controlador del Borunte a partir del respaldo
> que genera `compiler/pad_codegen.py` — ver `docs/PAD_FORMAT.md` y
> `docs/ARCHITECTURE.md`. Las dos salidas comparten gramática y AST, pero no
> soportan exactamente lo mismo: el pad no tiene (todavía) variables, `ELSE`
> ni `WAIT_IN`.

Las instrucciones son objetos `Instruction` (`runtime/bytecode.py`) con un
opcode y operandos `a`, `b`, `c`, `d`. Los puntos van en una tabla aparte
(`Program.points`), referenciados por índice.

## Opcodes v0.2

| Opcode | Nombre | Operandos | Descripción |
|---|---|---|---|
| `0x00` | `NOP` | — | No hace nada |
| `0x01` | `MOVEJ` | B=índice de punto, D=velocidad | Mueve por articulaciones. Escribe punto en bloque 800-890, dispara, hace poll de `0x09A6` |
| `0x02` | `MOVEL` | B=índice de punto, D=velocidad | Mueve en línea recta (cartesiano) |
| `0x03` | `WAIT_IN` | A=número de entrada, B=timeout(ms) | Espera una entrada |
| `0x04` | `SET_OUT` | A=número de salida, B=estado (0/1) | Fuerza salida |
| `0x05` | `WAIT_TIME` | B=milisegundos | Timer bloqueante |
| `0x06` | `JUMP` | B=PC destino | Salto incondicional |
| `0x07` | `JUMP_IF_ZERO` | A=índice de variable, B=PC destino | Salto condicional (variable == 0). Reservado para futuro uso; el codegen usa `0x0D`/`0x0E` en su lugar |
| `0x08` | `CALL` | B=PC destino | Llama subrutina |
| `0x09` | `RET` | — | Vuelve de subrutina |
| `0x0A` | `SET_VAR` | A=índice var, B=valor inmediato | Asigna constante |
| `0x0B` | `ADD_VAR` | A=índice var, B=operando | var += operando |
| `0x0C` | `COPY_POINT_OFFSET` | A=punto origen, B=punto destino, C=offset (referencia a 6 words) | Calcula punto_destino = punto_origen + offset |
| `0x0D` | `JUMP_IF_VAR_NEQ_CONST` | A=índice de variable, B=constante, C=PC destino | Salta a C si `var != const`. Es lo que permite compilar `IF var == const THEN ... ELSE ... ENDIF` |
| `0x0E` | `JUMP_IF_VAR_NEQ_VAR` | A=índice var izquierda, B=índice var derecha, C=PC destino | Salta a C si `var[A] != var[B]`. **Agregado en v0.2** — permite `IF a == b` entre dos variables |
| `0x0F` | `COPY_VAR` | A=índice var destino, B=índice var origen | `var[A] = var[B]`. **Agregado en v0.2** — lo usan la asignación entre variables (`a = b`) y la carga de parámetros de `PROC` |
| `0xFE` | `ALARM_CLEAR_CONTINUE` | — | Manda `0x4E26` al robot |
| `0xFF` | `END` | — | Fin de programa |

> Este set es intencionalmente mínimo. Se amplía SOLO después de validar el
> PoC (fase 2 del roadmap) — agregar opcodes antes de probar la VM real es
> desperdiciar trabajo.

## Calling convention de `PROC`

Los parámetros **no van por pila**: cada parámetro de cada `PROC` tiene un
**slot fijo y propio** en el banco de variables, con el nombre reservado
`<proc>.<param>`. El punto no es un carácter válido en un identificador del
DSL, así que un slot de parámetro nunca puede colisionar con una `VAR` que
declare el usuario.

El **llamador** carga los slots antes del `CALL`:

- argumento constante → `SET_VAR` sobre el slot
- argumento variable  → `COPY_VAR` sobre el slot

El **cuerpo** del `PROC` lee y escribe ese mismo slot como si fuera una
variable local.

```
PROC apilar(altura)          ; slot reservado: "apilar.altura"
  SET_OUT(Y10, ON)
ENDPROC

apilar(5)                    ; -> SET_VAR  a=slot("apilar.altura") b=5
                             ;    CALL     b=PC(apilar)
apilar(nivel)                ; -> COPY_VAR a=slot("apilar.altura") b=slot("nivel")
                             ;    CALL     b=PC(apilar)
```

Limitaciones (son consecuencia directa de no tener frames):

1. **No es recursiva ni reentrante.** Un `PROC` que se llame a sí mismo, o
   dos llamadas anidadas al mismo `PROC`, se pisan los slots. El compilador
   no lo detecta hoy.
2. **Los parámetros son por valor.** Escribir el parámetro dentro del `PROC`
   modifica el slot, no la variable del llamador.
3. La aridad y la forma de los argumentos (solo constante o variable, no
   expresiones compuestas) se validan **en tiempo de compilación**.
