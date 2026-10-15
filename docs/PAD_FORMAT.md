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
> - **Visto en el listado del pad**: el pad exporta el programa como HTML (el
>   mismo texto que muestra en pantalla). Se cruzó con el respaldo: ver
>   «Segunda fuente» al final. Es el **rótulo que el propio pad le pone** a cada
>   campo; confirma qué significa lo que ya guarda, **no** que acepte un
>   respaldo generado por nosotros.
>
> Herramientas: `pad/backup.py` (leer/escribir), `python -m pad.listing <zip>`
> (listado legible) y `python -m pad.pad_html <zip> <html>` (cruce con el
> listado del pad).

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
| `4` | `MOVEJ` | igual que `10` | **Visto en el listado del pad**: `Free Path` con `J1…J6` (los valores `m0..m5` son ángulos de eje) |
| `10` | `MOVEL` | `points[0].pos.m0..m7` (strings con 3 decimales: X,Y,Z mm + U,V,W °; `m6`,`m7` ejes extra), `speed` (% con 1 decimal, string), `toolCoord`, `smooth`, `delay`, `distance`, `bindIOInfo`, `customName` (nombre del punto), y fijos en todo el respaldo: `ckStatus:"63"`, `passTrans:0`, `quotePoint:[0,0,0]`, `relativeType:0` | **Visto en el listado del pad**: `Line3D-Pose` con `X,Y,Z,U,V,W`, `V` (velocidad), `DLY` (demora del movimiento), `Smooth`, `tool:n coord:n` |
| `100` | `WAIT` | `limit` = segundos (string), `type:100`, `point:0`, `pointStatus:0`, `isUnlimit:false` | **Visto en el listado del pad**: `DLY:0.300s` (163 casos). Que el robot de verdad espere ese tiempo: sin probar |
| `200` | `SET_OUT` | `point` = `valveID` = salida, `pointStatus` true/false, `delay`, `isWaitInput:false`, `type:0` | **Visto en el listado del pad**: `Output:Y034OFF DLY0.000` |
| `800` | `COORD n` | `coordID` | **Visto en el listado del pad**: `Switch Coord:[CoordID1]NombreDelSistema` |
| `801` | `TOOL n` | `toolID` | **Visto en el listado del pad**: `Switch Tool:[2:NombreDeLaHerramienta]` |
| `10001` | `IF X ON GOTO etiqueta` | `point` = entrada, `pointStatus:1`, `flag` = etiqueta destino, `inout:0`, `limit:"0.000"`, `type:0`. Las 44 del respaldo tienen `pointStatus:1`; `pointStatus:0` ("si X OFF") **nunca se vio** | **Visto en el listado del pad**: `IF:X030ON Lim:0.000 Go to Flag[1]:` = si X030 está ON, ir a la etiqueta 1 (44 casos) |
| `20000` | `CALL módulo` | `module` (id como **string**), `flag` | **Visto en el listado del pad**: `Call Fun[5]:Nombre And then return to next line` (`flag` -1) o `… return to Flag[0]:` (`flag` `"0"`): llamar y después ir a la etiqueta |
| `20001` | fin de módulo (`ENDPROC`) | — | Deducido |
| `50000` | comentario | `comment` (puede traer `&nbsp;`); con `commentAction` es una **acción desactivada** y `comment` es el texto que muestra el pad | Deducido |
| `53000` | `CMD STOP` | `addr:98304` (`0x18000`), `data:2561` (`0x0A01`), `op:0`, `specialType:1`, `type:0`, `typeSel:1` | **Visto en el listado del pad**: `Cmd Set:Stop` (una vez, al principio del principal). Qué hace en el robot: no se sabe |
| `59999` | etiqueta | `comment` = nombre, `flag` = número (único por programa) | Deducido |
| `60000` | `END` del programa | — | Deducido |

### Detalles de campos

- **`toolCoord`** = herramienta en los 16 bits altos, sistema de coordenadas
  en los bajos: `131073 = 0x20001` → herramienta 2, coordenadas 1.
  *Deducido.* Ojo: **no siempre coincide** con el último `800`/`801`: en el
  respaldo real hay 11 movimientos que no coinciden, 3 módulos sin
  `800`/`801`, y el principal no tiene ninguno. Lo que manda parece ser el
  `toolCoord` de cada movimiento (*hipótesis*).
- **Nombres de E/S**: el pad muestra `point` 2 como `X012`, 3 como `X013`, 4
  como `X014`, 16 como `X030`, 17 como `X031` y 20 como `Y034`. Encaja con
  numeración **octal empezando en 010** (`nombre = octal(point + 8)`).
  *Deducido*: 6 puntos distintos y 65 usos en el listado del pad; no se vio
  el resto del rango.
- **`CALL` con `flag`**: `-1` (entero) = sin salto. `"0"` (string) aparece
  solo en llamadas que, por la lógica del programa, deberían saltar a la
  etiqueta 0 al volver. *Hipótesis*: "llamar y después ir a la etiqueta".
- **`bindIOInfo` / `distance`**: **una salida que cambia según la distancia al
  punto** (en la soldadura, el arco). El pad lo muestra como
  `whiledistanceend pointthe distancelimit then 1.000mm then output Y034 on`
  y `whiledistancestart pointthe distancemore then 2.000mm then output Y034 off`.
  Decodificado cruzando 327 movimientos (`pad/bindio.py`):
  `bindIOInfo = (point << 10) | 0x40 | (0x4 si es el punto final) | (0x1 si
  está habilitado) | (0x100000 si la salida pasa a ON)`, y `distance` son mm.
  Valores vistos: `0x5041` (lejos del inicial → OFF), `0x5045`, `0x105041`,
  `0x105045` (cerca del final → ON) y `0x105044` (deshabilitado, distancia 0:
  el pad no muestra nada; 3 casos). El bit `0x40` siempre estuvo prendido y no
  se sabe qué es. *Deducido del texto del pad.*
- **`smooth`**: 0–9, nivel de suavizado. **Visto en el listado del pad**:
  `Smooth:None` (0) o el número (`Smooth:4`).
- **`delay` de un movimiento**: demora propia del movimiento (`DLY1.000` en
  la línea del movimiento; 1 caso). Distinta de la acción `100`.
- **Acciones desactivadas**: una acción `50000` con `commentAction` (la acción
  original). El pad guarda en `comment` **el mismo texto con que la mostraba**
  (`&nbsp;`×8 + `Output:Y034OFF DLY0.000`); en un `IF` agrega el **nombre de
  la etiqueta destino** (`…Go to Flag[0]:RejaDe6Fila2`). En el programa
  analizado: 167, de dos tipos (`SET_OUT` 143, `IF…GOTO` 24). En pantalla el
  pad las lista como la acción normal.

## Segunda fuente: el listado HTML del pad

El pad exporta el programa como HTML (`mainProgram:`, `fuction[n]:Nombre`,
una entrada numerada por instrucción). Cruzado con el respaldo de un programa
de **1032 acciones** (9 módulos): **las 1032 coinciden**, en el mismo orden y
con los mismos valores:

| Qué se cruzó | Casos |
|---|---:|
| Valores de punto (J1…J6 o X…W), velocidad, demora, suavizado, nombre | 556 |
| `toolCoord` = herramienta ≪ 16 \| sistema | 531 |
| Salida por distancia (texto del pad = `bindIOInfo` decodificado) | 324 |
| Sin `bindIOInfo` = sin ese texto | 232 |
| `SET_OUT` (nombre octal, ON/OFF, demora) | 21 |
| `IF…GOTO` (entrada, ON/OFF, etiqueta) | 44 |
| `CALL` (módulo, `return to next line` / `return to Flag[n]`) | 11 |
| `WAIT` (`DLY:n s` = `limit`) | 163 |
| `COORD` / `TOOL` | 6 / 6 |
| Etiquetas (número y nombre) | 45 |
| Acciones desactivadas (texto guardado = texto del pad) | 167 |

Además el listado trae los **nombres** de las herramientas y los sistemas del
programa (`Switch Coord:[CoordID1]MesaSoldadoraDer`, `Switch Tool:[2:…TCP]`),
pero **no sus valores** (desplazamiento del TCP, origen): esos se copian del
pad.

Reproducir: `python -m pad.pad_html RESPALDO.zip LISTADO.html`. El test de
`tests/test_pad_html.py` usa datos sintéticos; si están las variables
`BORUNTE_REAL_BACKUP` y `BORUNTE_REAL_LISTING`, también cruza los archivos
reales (el programa de un cliente no va al repo).

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
1. Una salida y una entrada fuera de las ya vistas (X012, X013, X014, X030,
   X031, Y034) → confirma la numeración octal en el resto del rango.
2. Un `WAIT` de 2 s en un respaldo propio → el rótulo del pad ya lo confirma
   (`DLY`); falta ver que el robot espere.
3. Un movimiento con una salida por distancia armada a mano, con el bit
   `0x40` apagado y con otras salidas → completa `bindIOInfo`.
4. Importar un respaldo **reescrito por `pad/backup.py` sin cambios** → prueba
   que el pad acepta lo que generamos (es la primera prueba a hacer).
5. Importar uno con **una coordenada cambiada** → prueba la escritura real.
