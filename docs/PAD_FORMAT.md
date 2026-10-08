# Formato de respaldo del pad Borunte (`HCBackupRobot_*.zip`)

> ⚠️ **Deducido, no confirmado por Borunte.** Todo lo de este documento sale
> de analizar **un solo** respaldo real exportado desde el pad (programa de
> soldadura de 9 módulos, ~1100 acciones, agosto 2026). Cada dato lleva su
> grado de confianza:
>
> - **Verificado**: se comprueba mecánicamente (p. ej. leer y reescribir da el
>   mismo archivo byte a byte).
> - **Deducido**: encaja con todos los casos del respaldo, pero no se probó
>   cargando un archivo modificado en el pad.
> - **Hipótesis**: encaja con pocos casos, o hay una explicación alternativa.
> - **Desconocido**: se conserva tal cual, sin interpretar.
>
> Herramientas: `pad/backup.py` (leer/escribir) y
> `python -m pad.listing <zip>` (listado legible).

## Contenedor

| Dato | Confianza |
|---|---|
| Zip (deflate), creado en Unix, permisos `0644`, sin carpetas | Verificado |
| Nombre `HCBackupRobot_AAAAMMDDhhmmss.zip`; el pad no reimporta otro nombre | Deducido (foro de RoboDK + nuestro respaldo) |
| Un programa por zip: `<Nombre>.act`, `.fnc`, `.counters`, `.palletStyle`, `.timers`, `.variables` | Verificado |
| `.counters`, `.palletStyle`, `.timers`, `.variables` vacíos si el programa no usa contadores, paletizado, timers ni variables | Deducido |
| `.fnc`: texto, una línea `clave, valor` por parámetro (191 líneas, claves 214–219 y 357–541). Solo 2 valores no nulos: `357 = 131071` (`0x1FFFF`) y `429 = 268435456` (`0x10000000`) | Desconocido — configuración del programa, se copia de un respaldo existente |

## El `.act`

Texto UTF-8, **una línea JSON por bloque**, separadas por `\n`, sin `\n`
final. JSON compacto (`","` y `":"` sin espacios), sin escapar acentos.
Reescribir con `json.dumps(..., separators=(",", ":"), ensure_ascii=False)`
reproduce el archivo byte a byte (**verificado**).

| Línea | Contenido | Confianza |
|---|---|---|
| 1 | Programa principal: lista de acciones | Deducido |
| 2–9 | 8 listas con solo `END` (`60000`) | Desconocido (¿subprogramas en paralelo?) |
| 10 | `{}` | Desconocido |
| 11 | Lista de módulos: `{"id", "name", "program"}`, donde `program` es un **string** con la lista de acciones en JSON | Deducido |
| 12–19 | 8 listas con solo `END` | Desconocido |

### Acciones

Cada acción es un objeto con `action` (código) e `insertedIndex`. Las claves
aparecen en orden alfabético.

- **El orden de ejecución es el orden en la lista**, no `insertedIndex`.
  `insertedIndex` es un identificador único dentro de cada programa: hay
  huecos (acciones borradas) y desorden (acciones insertadas después).
  *Deducido.*

| Código | Nombre propuesto | Campos | Confianza |
|---|---|---|---|
| `4` | `MOVEJ` | igual que `10` | Deducido (los valores `m0..m5` son ángulos de eje) |
| `10` | `MOVEL` | `points[0].pos.m0..m7` (strings con 3 decimales: X,Y,Z mm + U,V,W °; `m6`,`m7` ejes extra), `speed` (% con 1 decimal, string), `toolCoord`, `smooth`, `delay`, `distance`, `bindIOInfo`, `customName` (nombre del punto), y fijos en todo el respaldo: `ckStatus:"63"`, `passTrans:0`, `quotePoint:[0,0,0]`, `relativeType:0` | Deducido |
| `100` | `WAIT` | `limit` = segundos (string), `type:100`, `point:0`, `pointStatus:0`, `isUnlimit:false` | Hipótesis |
| `200` | `SET_OUT` | `point` = `valveID` = salida, `pointStatus` true/false, `delay`, `isWaitInput:false`, `type:0` | Deducido |
| `800` | `COORD n` | `coordID` | Deducido |
| `801` | `TOOL n` | `toolID` | Deducido |
| `10001` | `IF X ON GOTO etiqueta` | `point` = entrada, `pointStatus:1`, `flag` = etiqueta destino, `inout:0`, `limit:"0.000"`, `type:0`. Las 44 del respaldo tienen `pointStatus:1`; `pointStatus:0` ("si X OFF") **nunca se vio** | Deducido |
| `20000` | `CALL módulo` | `module` (id como **string**), `flag` | Deducido |
| `20001` | fin de módulo (`ENDPROC`) | — | Deducido |
| `50000` | comentario | `comment` (puede traer `&nbsp;`); con `commentAction` es una **acción desactivada** y `comment` es el texto que muestra el pad | Deducido |
| `53000` | ? | `addr:98304` (`0x18000`), `data:2561` (`0x0A01`), `op:0`, `specialType:1`, `type:0`, `typeSel:1` | Desconocido (aparece una vez, al principio del principal) |
| `59999` | etiqueta | `comment` = nombre, `flag` = número (único por programa) | Deducido |
| `60000` | `END` del programa | — | Deducido |

### Detalles de campos

- **`toolCoord`** = herramienta en los 16 bits altos, sistema de coordenadas
  en los bajos: `131073 = 0x20001` → herramienta 2, coordenadas 1.
  *Deducido.* Ojo: **no siempre coincide** con el último `800`/`801`: en el
  respaldo real hay 11 movimientos que no coinciden, 3 módulos sin
  `800`/`801`, y el principal no tiene ninguno. Lo que manda parece ser el
  `toolCoord` de cada movimiento (*hipótesis*).
- **Nombres de E/S**: el pad muestra `point` 2 como `X012`, 3 como `X013` y
  20 como `Y034`. Encaja con numeración **octal empezando en 010**
  (`nombre = octal(point + 8)`). *Hipótesis* — solo 3 casos.
- **`CALL` con `flag`**: `-1` (entero) = sin salto. `"0"` (string) aparece
  solo en llamadas que, por la lógica del programa, deberían saltar a la
  etiqueta 0 al volver. *Hipótesis*: "llamar y después ir a la etiqueta".
- **`bindIOInfo` / `distance`**: aparecen juntos en movimientos de
  aproximación (valores `20545 = 0x5041`, `1069125 = 0x105045` y variantes en
  el último dígito). Probablemente "activar una E/S a cierta distancia del
  punto". *Desconocido* — se copian tal cual.
- **`smooth`**: 0–9, probablemente nivel de suavizado (blending). *Hipótesis.*

## Qué genera el compilador y qué no

Ver la tabla de `compiler/pad_codegen.py`. Resumen: por defecto solo se
exportan formas que aparecen en el respaldo real. Dos cosas que nunca se
vieron (`IF X == 1`, que necesita `pointStatus:0`, y llamar un módulo desde
otro módulo) solo se exportan con **Programa → Permitir instrucciones sin
confirmar en el pad**. En el simulador se permiten siempre.

## Para confirmar con el pad

Lo más eficiente es exportar respaldos que difieran en **una sola cosa**:

0. Un `IF` "si la entrada está apagada" y un módulo que llame a otro módulo:
   confirmaría las dos formas que hoy se exportan solo a pedido.
1. Un `SET_OUT` a una salida conocida (p. ej. `Y010`, `Y011`) y un `IF` sobre
   una entrada conocida → confirma la numeración octal.
2. Un `WAIT` de 2 s → confirma la acción `100`.
3. Un movimiento con `bindIOInfo` configurado a mano → decodifica ese campo.
4. Importar un respaldo **reescrito por `pad/backup.py` sin cambios** → prueba
   que el pad acepta lo que generamos (es la primera prueba a hacer).
5. Importar uno con **una coordenada cambiada** → prueba la escritura real.
