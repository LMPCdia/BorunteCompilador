# Instruction Set v0.2 (DRAFT — a validar contra hardware real)

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

Los operandos de 32 bits (`B` y `C`) se guardan con el **word bajo primero**
(convención FX de Mitsubishi, que es lo que hereda el Coolmay CX3G). Ojo que
esto es **al revés que el robot Borunte**, donde el ejemplo confirmado del
manual manda el word alto primero — ver `docs/MODBUS_REGISTER_MAP.md`. Las
dos convenciones conviven a propósito, cada una del lado que le corresponde.
**Hipótesis sin confirmar**, y es la primera que conviene validar con el CX3G
delante.

Base de la tabla de bytecode: **`D1000`** en adelante, dejando `D0-D999`
libres para el bloque de control y uso general del PLC. Con 8 words por
instrucción, el espacio `D1000-D3999` da **375 instrucciones**.

## Tabla de puntos

Separada del bytecode. Cada punto = 6 valores de 32 bits (X,Y,Z,U,V,W o
J1-J6) = **12 words por punto**, con la misma convención de word bajo
primero que los operandos.

Base: **`D4000`** en adelante. El espacio `D4000-D7995` da **333 puntos**
antes de tocar `D8000`; si hace falta más, usar registros `R` extendidos.

Los valores van en fixed-point escalado x1000 (3 decimales), igual que las
poses del robot (ver `POSITION_SCALE` en `comms/robot_client.py`).

## Opcodes v0.2

| Opcode | Nombre | Operandos | Descripción |
|---|---|---|---|
| `0x00` | `NOP` | — | No hace nada |
| `0x01` | `MOVEJ` | B=índice de punto, D=velocidad | Mueve por articulaciones. Escribe punto en bloque 800-890, dispara, hace poll de `0x09A6` |
| `0x02` | `MOVEL` | B=índice de punto, D=velocidad | Mueve en línea recta (cartesiano) |
| `0x03` | `WAIT_IN` | A=número de entrada, B=timeout(ms) | Espera entrada del PLC (no del robot) |
| `0x04` | `SET_OUT` | A=número de salida, B=estado (0/1) | Fuerza salida |
| `0x05` | `WAIT_TIME` | B=milisegundos | Timer bloqueante |
| `0x06` | `JUMP` | B=PC destino | Salto incondicional |
| `0x07` | `JUMP_IF_ZERO` | A=índice de variable, B=PC destino | Salto condicional (variable == 0). Reservado para futuro uso; el codegen usa `0x0D`/`0x0E` en su lugar |
| `0x08` | `CALL` | B=PC destino | Llama subrutina (usa pila de retorno del PLC) |
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

Limitaciones que **`plc_vm/` tiene que respetar** (son consecuencia directa
de no tener frames):

1. **No es recursiva ni reentrante.** Un `PROC` que se llame a sí mismo, o
   dos llamadas anidadas al mismo `PROC`, se pisan los slots. El compilador
   no lo detecta hoy.
2. **Los parámetros son por valor.** Escribir el parámetro dentro del `PROC`
   modifica el slot, no la variable del llamador.
3. La aridad y la forma de los argumentos (solo constante o variable, no
   expresiones compuestas) se validan **en tiempo de compilación**.

## Registros de control de la VM (a coordinar con `plc_vm/`)

Bloque de 8 words, pensado para leerse en **una sola transacción Modbus**:

| Registro | Uso |
|---|---|
| `D0` | Program Counter actual |
| `D1` | Estado de la VM (ver tabla abajo) |
| `D2` | Código de error actual (ver tabla abajo) |
| `D3` | Comando del host (handshake — ver abajo) |
| `D4` | Cantidad de instrucciones cargadas |
| `D5` | Cantidad de puntos cargados |
| `D6` | Versión del contrato que implementa la VM (`2` para este documento) |
| `D7` | Reservado |
| `D10-D109` | Banco de 100 variables de usuario (`VAR` y slots `<proc>.<param>`) |

### Estado de la VM (`D1`)

| Valor | Significado |
|---|---|
| `0` | Parada (lista para arrancar) |
| `1` | Corriendo |
| `2` | Error (ver `D2`) |
| `3` | Pausada |
| `4` | Terminada (llegó a `END`) |

### Handshake de comandos (`D3`)

El host escribe el código de comando en `D3`. La VM lo ejecuta y **devuelve
`D3` a `0`** para acusar recibo. El host espera ese `0` con un timeout; si no
llega, considera que la VM no está corriendo el ladder esperado y falla, en
vez de seguir a ciegas.

| Valor | Comando |
|---|---|
| `0` | Ninguno / acuse de la VM |
| `1` | `START` (arrancar desde PC=0) |
| `2` | `STOP` |
| `3` | `PAUSE` |
| `4` | `RESUME` |
| `5` | `RESET` (limpia el error y pone PC=0) |

### Códigos de error (`D2`)

| Valor | Significado |
|---|---|
| `0` | Sin error |
| `1` | Opcode desconocido |
| `2` | Índice de punto fuera de rango |
| `3` | Índice de variable fuera de rango |
| `4` | Timeout esperando una entrada (`WAIT_IN`) |
| `5` | Timeout esperando el fin de movimiento del robot |
| `6` | Alarma del robot |
| `7` | `RET` sin `CALL` (pila de retorno vacía) |
| `8` | Desborde de la pila de retorno |

> ⚠️ **Todo el bloque de control de esta sección es propuesta sin
> confirmar**: nadie lo implementó todavía en ladder, y no se validó que el
> registro `Dn` del CX3G se exponga como holding register Modbus número `n`.
> `comms/plc_client.py` lo parametriza en `d_register_base` para poder
> corregirlo en un solo lugar.

## Qué falta definir en el PoC (fase 2)

- Cómo se codifica exactamente `MOVEJ` vs `MOVEL` en la escritura al bloque
  800-890 del robot (ver duda abierta sobre "free path"/"posture line").
- Timeout máximo razonable de poll para `0x09A6` antes de considerarlo error.
- Cómo se maneja una alarma del robot en medio de una secuencia (¿la VM se
  detiene, reintenta, o llama `ALARM_CLEAR_CONTINUE` automáticamente?).
- Si la pila de retorno del PLC alcanza para el anidamiento de `CALL` que
  necesitamos, y cuál es su profundidad real en el CX3G.
