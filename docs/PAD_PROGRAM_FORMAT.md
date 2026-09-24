# Formato `.act` — programa nativo del robot Borunte

> ⚠️ **Esto es ingeniería inversa, no documentación de Borunte.** Todo lo de acá
> salió de analizar un export real del robot. Cada afirmación está marcada como
> **confirmado por observación** (la forma del registro no deja lugar a dudas) o
> **hipótesis** (lo dedujimos del contexto y hay que validarlo con el robot).

**Fuente:** `AberturaSanLorenzo2026.act` — export del backup del robot del
2026-08-14. 338 KB, 1032 acciones, 9 subprogramas, un programa de producción
real.

**Consumidores:** `compiler/act_reader.py` (lee) y `compiler/act_backend.py`
(genera).

## Estructura del archivo

**Confirmado.** No es un único documento JSON: es **JSON Lines**, un documento
por línea. En el export analizado:

| Línea | Contenido |
|---|---|
| 0 | Programa principal — lista de acciones |
| 1-8 | Ocho documentos `[{"action":60000,"insertedIndex":0}]` |
| 9 | `{}` |
| 10 | Biblioteca de subprogramas — lista de `{id, name, program}` |
| 11-18 | Otros ocho `[{"action":60000,"insertedIndex":0}]` |

No sabemos qué son las líneas de relleno ni la 9. `act_backend.py` las reproduce
igual: omitirlas sería apostar a que no hacen falta.

**El campo `program` de la biblioteca no es una lista: es un string que contiene
JSON.** Está codificado dos veces y hay que hacerle `json.loads` aparte.

**Serialización:** claves ordenadas alfabéticamente, sin espacios, sin salto de
línea final. Verificado con un round-trip byte a byte sobre los 338 KB
(`tests/test_act_backend.py::test_real_export_round_trips_byte_for_byte`).

## Orden de ejecución

**Confirmado.** El orden de ejecución es **el orden del array**, NO el campo
`insertedIndex`. En el principal del export real la secuencia de `insertedIndex`
es `..., 13, 15, 14, 16, ...`: ese campo es el orden en que el operario fue
insertando líneas en el pad, no la secuencia. Confundirlos da un programa
reordenado.

## Catálogo de acciones

Las 13 acciones que aparecen en el export real. La columna "campos fijos" son
los que tienen el mismo valor en **todas** sus apariciones.

| `action` | Qué es | Certeza | Campos propios | Campos fijos |
|---|---|---|---|---|
| `4` | Movimiento tipo A | **hipótesis: `MOVEJ`** | `points`, `speed`, `smooth`, `toolCoord`, `distance`, `bindIOInfo`, `customName` | `ckStatus:"63"`, `delay:"0.000"`, `passTrans:0`, `quotePoint:[0,0,0]`, `relativeType:0` |
| `10` | Movimiento tipo B | **hipótesis: `MOVEL`** | idénticos a los de `4` | los mismos |
| `100` | Espera | **hipótesis: temporizada** | `limit` (segundos), `isUnlimit` | `type:100`, `point:0`, `pointStatus:0`, `customName:""` |
| `200` | Forzar salida | confirmado | `valveID`, `point`, `pointStatus` (bool) | `delay:"0.000"`, `isWaitInput:false`, `type:0` |
| `800` | Seleccionar base | confirmado | `coordID` | — |
| `801` | Seleccionar herramienta | confirmado | `toolID` | — |
| `10001` | `IF` sobre entrada | confirmado | `point` (entrada), `pointStatus`, `flag` (etiqueta destino) | `inout:0`, `type:0`, `limit:"0.000"` |
| `20000` | Llamar subprograma | confirmado | `module` (id, **como string**), `flag` | — |
| `20001` | Fin de subprograma | confirmado | — | aparece 1 vez por subprograma |
| `50000` | Comentario | confirmado | `comment`, a veces `commentAction` | — |
| `53000` | Escritura de registro | **desconocido** | `addr`, `data`, `op`, `specialType`, `typeSel`, `type` | aparece 1 sola vez |
| `59999` | Etiqueta | confirmado | `flag` (id), `comment` (nombre) | — |
| `60000` | Fin del programa principal | confirmado | — | aparece 1 vez |

Todas llevan además `action` e `insertedIndex`.

### `4` vs `10` — la hipótesis más riesgosa

Los dos tienen **exactamente los mismos campos**, así que por la forma del
registro no se pueden distinguir. El mapeo actual (`4`=`MOVEJ`, `10`=`MOVEL`) se
decidió por el uso:

| | `4` | `10` |
|---|---|---|
| Apariciones | 25 | 531 |
| Velocidades | 100.0, 50.0, 25.0 | 50.0, 25.0, **2.0** |

Encaja con que el trabajo fino (lineal) sea lo frecuente y lento, y el
reposicionamiento (articular) lo esporádico y rápido. **Si está al revés, el
robot hace trayectorias distintas de las escritas.** Se corrige en
`MOVE_ACTIONS` de `compiler/act_backend.py`, en un solo lugar.

### Poses

Un movimiento lleva `points: [{"pointName": "", "pos": {...}}]`, con `m0` a `m7`
**como strings de 3 decimales**. `m6` y `m7` valen `"0.000"` en las 556 acciones
de movimiento del export, sin una excepción: son los ejes externos, que este
robot no tiene.

La velocidad va con **un** decimal (`"100.0"`), no con tres.

### `toolCoord`

**Hipótesis.** Empaqueta herramienta y base en un entero:

```
toolCoord = (toolID << 16) | coordID
```

En el export `toolCoord` vale 131072, 131073 y 131074 = `2<<16 | 0/1/2`, y en ese
mismo programa `toolID` vale 2 y `coordID` vale 1 o 2. El encaje es exacto, pero
no está confirmado.

## Control de flujo

**Confirmado, y es la restricción más importante del formato.**

Los saltos van **por etiqueta, no por índice**: un `IF` (`10001`) lleva un `flag`
y salta al `LABEL` (`59999`) que tenga el mismo `flag`. Los ids de etiqueta
arrancan de 0 en cada programa y subprograma.

El `IF` nativo salta **hacia adelante cuando la condición se cumple**. El cuerpo
que queda entre el `IF` y su etiqueta se ejecuta cuando la condición es **falsa**:

```
IF entrada 3 == ON  -> etiqueta 0     ; si está ON, saltea lo que sigue
  <cuerpo — corre solo si la entrada NO está ON>
LABEL 0
```

Para compilar `IF cond THEN cuerpo ENDIF` hay que **invertir la condición**.

### No hay salto incondicional

En las 1032 acciones del export **no aparece ninguno**. Eso significa que un
`ELSE` de propósito general **no es expresable**: para saltear la segunda rama
haría falta justamente eso.

El programa real sí encadena ramas, pero con un truco que depende del contenido:
el `flag` de una llamada a subprograma (`20000`) funciona como "al volver, saltar
a esta etiqueta". `flag: -1` = seguir; `flag: "0"` (string) = saltar a la
etiqueta 0. Sirve solo si la rama termina en una llamada, así que no es un salto
incondicional general.

Por eso `compiler/act_backend.py` acepta `IF ... ENDIF` y rechaza `ELSE` con un
error que lo explica, en vez de inventar bytecode.

## Lo que falta averiguar

- [ ] Cuál de `4`/`10` es `MOVEJ` y cuál `MOVEL` — mirando el pad, o exportando
      un programa de dos líneas hecho a mano con una de cada una.
- [ ] Si `100` es la espera temporizada, y qué significan sus `point`/`pointStatus`.
- [ ] Qué hace `53000` (`addr`, `data`, `op`).
- [ ] Si existe algún salto incondicional que este export no usa. Si existiera,
      se podría soportar `ELSE`.
- [ ] Qué son las líneas de relleno y la línea 9 (`{}`).
- [ ] Qué significan `bindIOInfo`, `distance` y `smooth` en un movimiento. El
      compilador los deja en su valor por defecto.
- [ ] Si el robot acepta un `.act` generado por nosotros. **Nada de esto se
      probó importando un archivo al robot.**
