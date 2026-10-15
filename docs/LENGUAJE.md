# Guía del lenguaje — DSL tipo KRL para robot Borunte

Documento autocontenido: alcanza para escribir programas correctos sin mirar el resto del repo.

**Cómo se mantiene al día:** todos los bloques de código de esta guía los compila `tests/test_language_guide.py` contra el compilador de verdad. Un bloque `krl` tiene que compilar; uno `krl-error` tiene que fallar con el mensaje que dice su primera línea; uno `krl-unverified` solo compila si se permiten instrucciones sin confirmar; uno `krl-vm` compila para la VM de la PC pero no para el pad. Si el compilador cambia y esta guía no, el test falla.

> ⚠️ **Nada de lo que genera el compilador se probó todavía en un pad real.** El formato del respaldo sale de analizar UN respaldo exportado del pad (ver `docs/PAD_FORMAT.md`). Lo que esta guía llama «confirmado» quiere decir «visto en ese respaldo», no «probado cargándolo en el robot». Cada cosa dudosa lleva su advertencia; la sección 14 las junta.

## 1. Qué es esto

Un lenguaje chico, de una sentencia por línea, para programar un robot Borunte. El programa se escribe en la PC y se compila de dos maneras:

| Destino | Qué genera | Quién lo ejecuta | Para qué |
|---|---|---|---|
| **Pad** (`compiler/pad_codegen.py`) | Un **respaldo del pad** (`HCBackupRobot_<fecha>.zip`) | El controlador del robot, solo, con sus propias E/S | **Producción.** Se copia a un pendrive y se importa desde el pad |
| **VM de la PC** (`compiler/codegen.py` + `runtime/vm.py`) | Bytecode | Una VM en la PC | Probar lógica (variables, `ELSE`, esperas de entrada) sin robot |

Además hay un **simulador 3D** (pestaña «Simulación 3D») que ejecuta el respaldo del pad sobre el modelo del robot: avisa ejes fuera de rango, puntos inalcanzables, singularidades y choques, y estima el tiempo de ciclo (sección 12).

No hay PLC en el camino: se descartó. El pad ejecuta el programa solo.

Los dos destinos aceptan subconjuntos distintos del lenguaje. No es un descuido: el pad solo sabe hacer lo que se vio en el respaldo real. **Lo que el pad no sabe expresar es un error de compilación claro, nunca una acción «probable»**: el robot ejecuta lo que se le carga sin nadie mirando. La tabla de la sección 11 dice qué anda en cada destino.

Si escribís para el robot (el caso normal), mirá lo que anda en el **pad**.

## 2. Archivos del programa

Un programa son archivos con el mismo nombre, como en KUKA, más uno común a la carpeta:

| Archivo | Qué tiene |
|---|---|
| `paletizado.src` | La **lógica**: movimientos, velocidades, esperas, salidas, `IF`, `PROC`. **Sin `POINT`.** |
| `paletizado.dat` | Los **puntos de ese programa**: solo `POINT` (y comentarios). |
| `config.dat` | Puntos **comunes a toda la carpeta** (HOME, poses de traslado…). Opcional. |

Se juntan en ese orden (`config.dat`, el `.dat`, el `.src`) para compilar, y los errores dicen archivo y línea: `paletizado.src, línea 3: …`.

```krl-config
; config.dat — puntos de toda la celda
POINT p_home = JOINT(0, 0, 0, 0, -90, 0)
```

```krl-dat
; paletizado.dat — puntos de este programa
POINT p_pieza = WORLD(500, 0, 400, 180, 0, 180)
```

```krl-src
; paletizado.src — solo la lógica
PROC tomar()
    MOVEJ p_home SPEED 80
    MOVEL p_pieza SPEED 10
ENDPROC

tomar()
```

Reglas, todas con error de compilación que dice archivo y línea:

- un `.dat` (y `config.dat`) solo tiene `POINT`;
- el `.src` no declara puntos, tampoco dentro de un `PROC` o un `IF`;
- el mismo nombre de punto no puede estar en `config.dat` y en el `.dat`.

```krl-src
; ERROR: se declara en
POINT p_nuevo = WORLD(1, 2, 3, 0, 0, 0)
MOVEL p_nuevo SPEED 10
```
```krl-dat
POINT p_otro = JOINT(0, 0, 0, 0, 0, 0)
```

Un punto del `.dat` que el `.src` no usa es solo un **aviso** (los de `config.dat` no avisan: son para varios programas).

Un programa de **un solo archivo** (`.krlb`, lo de antes) sigue andando igual: ahí los `POINT` van donde estén. En la interfaz, *Archivo → Separar en .src y .dat* lo convierte.

## 3. Reglas básicas

- Una sentencia por línea. No hay separador ni punto y coma final.
- Las palabras clave van en **MAYÚSCULAS**. `MOVEJ` es válido; `movej` es error.
- Comentarios con `;`, hasta el fin de la línea, en su propia línea o al final de una sentencia.
- Las líneas en blanco se ignoran en cualquier parte (también dentro de un `PROC` o un `IF`).
- El archivo puede no terminar en salto de línea.
- Identificadores: letras, dígitos y `_`; no empiezan con dígito. Distinguen mayúsculas.
- Números: enteros o decimales con signo opcional (`1`, `1.5`, `-2.25`). Los decimales van **con punto**, no con coma.

```krl
; esto es un comentario de línea completa
PROC encender()
    SET_OUT(Y010, ON)        ; y esto va al final de una sentencia
ENDPROC

encender()
```

```krl-error
; ERROR: las palabras clave van en mayúscula
POINT p = JOINT(0, 0, 0, 0, 0, 0)
movej p SPEED 50
```

## 4. Puntos

Hay dos tipos de punto, y **cada movimiento exige el suyo** (sección 5):

| Tipo | Valores | Para |
|---|---|---|
| `JOINT(j1, j2, j3, j4, j5, j6)` | ángulos de cada eje, en grados | `MOVEJ` |
| `WORLD(x, y, z, u, v, w)` | posición en mm y orientación en grados, respecto de la base | `MOVEL` |

```krl
POINT p_home  = JOINT(0.347, 45.894, -44.865, -0.792, -75.952, -0.859)
POINT p_pieza = WORLD(1097.1, -150.0, 721.9, 180.0, -10.0, 180.0)

PROC ir()
    MOVEJ p_home SPEED 50
    MOVEL p_pieza SPEED 10
ENDPROC

ir()
```

Cada valor tiene que estar entre -10000 y 10000 (mm o grados). Se revisa cuando el punto se usa en un movimiento.

```krl-error
; ERROR: fuera de rango (máximo ±10000)
POINT p = WORLD(20000, 0, 0, 0, 0, 0)
MOVEL p SPEED 10
```

### Declarar antes de usar

Los puntos se declaran **antes** de usarse, también antes del `PROC` que los usa. Un nombre no se declara dos veces.

```krl-error
; ERROR: Punto no definido: 'p_a'
PROC ir()
    MOVEJ p_a SPEED 50
ENDPROC
POINT p_a = JOINT(0, 0, 0, 0, 0, 0)
ir()
```

```krl-error
; ERROR: ya estaba declarado
POINT p_a = JOINT(0, 0, 0, 0, 0, 0)
POINT p_a = JOINT(1, 1, 1, 1, 1, 1)
```

### Puntos relativos: `OFFSET`

`OFFSET` suma 6 valores a un punto **`WORLD`**. Se resuelve **al compilar**: el robot nunca ve la suma, el programa lleva las coordenadas ya calculadas.

```krl
POINT p_pieza = WORLD(500, 0, 400, 180, 0, 180)
POINT p_sobre = p_pieza + OFFSET(0, 0, 100, 0, 0, 0)

PROC bajar()
    MOVEL p_sobre SPEED 50
    MOVEL p_pieza + OFFSET(0, 0, 20, 0, 0, 0) SPEED 20
    MOVEL p_pieza SPEED 10
ENDPROC

bajar()
```

Sobre un punto `JOINT` no tiene sentido (sumaría grados a los ejes, no milímetros), y da error:

```krl-error
; ERROR: OFFSET sobre un punto JOINT
POINT p_a = JOINT(0, 0, 0, 0, 0, 0)
POINT p_b = p_a + OFFSET(0, 0, 10, 0, 0, 0)
```

### Punto literal en el movimiento

Se puede mover a un punto escrito en la misma línea, aunque conviene declararlo para poder nombrarlo:

```krl
PROC ir()
    MOVEJ JOINT(0, 0, 0, 0, -90, 0) SPEED 50
ENDPROC

ir()
```

## 5. Movimientos

```text
MOVEJ <punto JOINT> SPEED <n>     movimiento articular (por ejes)
MOVEL <punto WORLD>  SPEED <n>    movimiento lineal (cartesiano)
```

`SPEED` es un **porcentaje de 0,1 a 100**. Fuera de ese rango es error.

```krl-error
; ERROR: MOVEJ necesita un punto JOINT(...)
POINT p = WORLD(500, 0, 400, 180, 0, 180)
MOVEJ p SPEED 50
```

```krl-error
; ERROR: MOVEL necesita un punto WORLD(...)
POINT p = JOINT(0, 0, 0, 0, 0, 0)
MOVEL p SPEED 50
```

```krl-error
; ERROR: SPEED 150: en el pad es un % entre 0.1 y 100
POINT p = JOINT(0, 0, 0, 0, 0, 0)
MOVEJ p SPEED 150
```

El pad guarda para `MOVEJ` ángulos de eje y para `MOVEL` coordenadas cartesianas; por eso el compilador no deja mezclarlos: movería el robot a cualquier lado.

Los movimientos son **bloqueantes**: la línea siguiente no arranca hasta que el robot llegó. No hace falta esperar explícitamente.

> ⚠️ **Deducido, no confirmado.** Que la acción `4` del pad sea `MOVEJ` (valores = ángulos de eje) y la `10` sea `MOVEL` (valores = X, Y, Z, U, V, W) se dedujo del respaldo real. Si estuviera al revés, el robot haría trayectorias distintas de las escritas. **Probar cada programa nuevo a velocidad baja.**

## 6. Entradas y salidas

Se escriben **como las muestra el pad**: la letra en mayúscula (`X` entradas, `Y` salidas) y **tres dígitos octales** desde el `010`: `X010`, `X011`, … `X017`, `X020`… El rango va de `010` a `107`. `Y20` o `Y0020` son error; `M…` (marcas) no existe.

```krl
PROC pinza()
    SET_OUT(Y010, ON)
    SET_OUT(Y017, OFF)
    SET_OUT(Y020, ON)
ENDPROC

pinza()
```

```krl-error
; ERROR: no es una salida (Y) del pad
SET_OUT(Y20, ON)
```

```krl-error
; ERROR: las E/S del pad van de Y010 a Y107
SET_OUT(Y110, ON)
```

> ⚠️ **Hipótesis.** La numeración octal desde 010 (`nombre = octal(punto + 8)`) se dedujo de **3 casos** vistos en el respaldo real. Confirmar con una salida y una entrada conocidas.

Una entrada se lee **solo dentro de un `IF`** (sección 9). No existe «guardar una entrada en una variable» en el pad.

## 7. Herramienta y sistema de coordenadas

```text
TOOL <n>      herramienta (0 = brida)
COORD <n>     sistema de coordenadas (0 = base del robot)
```

(No existe `BASE`: es `COORD`.) Cambian el estado para los movimientos que siguen: cada movimiento generado lleva adentro la herramienta y el sistema vigentes, así que el orden importa. El número va de 0 a 15 (rango conservador: en el pad se vieron hasta el 2).

```krl
POINT p_a = WORLD(500, 0, 400, 180, 0, 180)
POINT p_b = WORLD(500, 100, 400, 180, 0, 180)

PROC trabajar()
    COORD 1
    TOOL 2
    MOVEL p_a SPEED 50     ; sale con coordenadas 1 y herramienta 2
    COORD 2
    MOVEL p_b SPEED 50     ; sale con coordenadas 2
ENDPROC

trabajar()
```

```krl-error
; ERROR: no se esperaba «1» en «BASE 1»
BASE 1
```

No se pueden poner **dentro de un `IF`**: van antes del `IF`, o dentro de un `PROC` que llame al `IF`.

```krl-error
; ERROR: TOOL dentro de un IF no se puede exportar
PROC a()
    IF X010 == 0 THEN
        TOOL 1
    ENDIF
ENDPROC
a()
```

Los números son los del pad; **los valores** de cada herramienta y cada sistema (el TCP, el origen) se cargan en el pad, no en el programa. El simulador los toma de la pestaña Herramientas / Coordenadas de la celda.

## 8. Esperas

```text
WAIT <n>s      espera n segundos (de 0,001 a 3600)
```

El sufijo `s` es obligatorio.

```krl
PROC dar_tiempo()
    SET_OUT(Y010, ON)
    WAIT 0.5s
    SET_OUT(Y010, OFF)
ENDPROC

dar_tiempo()
```

```krl-error
; ERROR: Se esperaba: «s»
WAIT 2
```

> ⚠️ **Hipótesis.** Que la acción `100` del pad sea la espera temporizada se dedujo de pocos casos. Confirmar con un `WAIT` de 2 s.

Lo que **no** se puede en el pad:

- `WAIT_IN(X010, 5)`, esperar una entrada bloqueando: no hay ningún ejemplo en el respaldo real. Para reaccionar a un sensor usá `IF` (sección 9).
- `WAIT UNTIL MOVE_DONE` compila pero **no se exporta** (con aviso): se supone que el pad termina cada movimiento antes de seguir, y los movimientos ya son bloqueantes. Con suavizado podría no ser así: sin confirmar.

```krl-error
; ERROR: WAIT_IN(X010) todavía no se puede exportar al pad
WAIT_IN(X010, 5)
```

## 9. Control de flujo

### `IF` sobre una entrada

```krl
PROC revisar()
    IF X010 == 0 THEN
        SET_OUT(Y020, ON)
    ENDIF
ENDPROC

revisar()
```

- La condición es **`Xnnn == 0`**: el cuerpo se ejecuta cuando la entrada está **apagada**. En el pad es «si X está encendida, saltar al final del `IF`», y es lo único que se vio en el respaldo real (44 veces).
- **`Xnnn == 1`** («si está encendida») necesitaría el salto contrario, que **nunca se vio**: solo se exporta si se permiten instrucciones sin confirmar (sección 12). En el simulador siempre se permite.
- Nada más: ni `ON`/`OFF` en la condición, ni `!=`, ni condiciones sobre variables, ni salidas (`Y`) como entrada.
- Se pueden anidar, y el cuerpo puede llevar movimientos, `SET_OUT` y `WAIT`:

```krl
POINT p_a = JOINT(0, 0, 0, 0, -90, 0)

PROC si_no_hay_pieza()
    IF X010 == 0 THEN
        WAIT 1s
        MOVEJ p_a SPEED 20
        IF X011 == 0 THEN
            SET_OUT(Y010, ON)
        ENDIF
    ENDIF
ENDPROC

si_no_hay_pieza()
```

```krl-unverified
; permitido solo con «instrucciones sin confirmar»
PROC revisar()
    IF X010 == 1 THEN
        SET_OUT(Y020, ON)
    ENDIF
ENDPROC

revisar()
```

```krl-error
; ERROR: En el pad, IF solo puede preguntar por una entrada
PROC revisar()
    IF X010 == ON THEN
        SET_OUT(Y020, ON)
    ENDIF
ENDPROC
revisar()
```

```krl-error
; ERROR: En el pad, IF solo puede preguntar por una entrada
PROC revisar()
    IF X010 != 0 THEN
        SET_OUT(Y020, ON)
    ENDIF
ENDPROC
revisar()
```

```krl-error
; ERROR: falta cerrar un paréntesis «)»
PROC revisar()
    IF INPUT(X010) == ON THEN
        SET_OUT(Y020, ON)
    ENDIF
ENDPROC
revisar()
```

(La última es la sintaxis de una versión vieja de esta guía: `INPUT(...)` no existe.)

### ⚠️ No hay `ELSE`

```krl-error
; ERROR: ELSE todavía no se puede exportar al pad
PROC revisar()
    IF X010 == 0 THEN
        SET_OUT(Y020, ON)
    ELSE
        SET_OUT(Y021, ON)
    ENDIF
ENDPROC
revisar()
```

**Por qué:** el `IF` del pad solo sabe saltar hacia adelante cuando la condición se cumple. Para saltear una segunda rama haría falta un salto incondicional, y en el respaldo real no hay ninguno. Generar algo aproximado sería inventar comportamiento del robot.

**Cómo se escribe entonces:** dos `IF` con condiciones opuestas (el segundo necesita `== 1`, que está sin confirmar; ver arriba). Ojo: no es exactamente equivalente si el cuerpo del primero cambia la entrada.

### No hay bucles

Ni `WHILE` ni `FOR`, en ningún destino. Para repetir se repite el código o se llama varias veces a un `PROC`.

### Variables, solo en la VM de la PC

```krl-vm
VAR pieza : INT = 1
VAR contador : INT = 0
contador = contador + 1
IF pieza == 1 THEN
    SET_OUT(Y010, ON)
ELSE
    SET_OUT(Y011, ON)
ENDIF
WAIT_IN(X010, 5)
```

La VM soporta `VAR` (`INT`, `REAL`, `BOOL`), asignaciones (constante, otra variable, `x = x + k`), `IF` sobre variables con `ELSE`, y `WAIT_IN`. **Nada de eso se exporta al pad**: da error de compilación (`todavía no se pueden exportar al pad`). Sirve para probar la lógica en la PC.

## 10. Subprogramas (`PROC`)

En el pad cada `PROC` es un **módulo**, y el programa principal solo los **llama**.

```krl
POINT p_home = JOINT(0, 0, 0, 0, -90, 0)

PROC home()
    MOVEJ p_home SPEED 100
ENDPROC

home()
home()
```

- Sin parámetros: `PROC home()`. No hay `id`: `PROC home(id=1)` es error de sintaxis.
- Se puede llamar **antes** de declarar el `PROC` (el que tiene que estar antes es el `POINT`).
- Se declaran en el nivel superior: no dentro de un `IF` ni de otro `PROC`. Un nombre no se repite.
- **Solo el programa principal llama módulos.** Un `PROC` que llama a otro `PROC` (y la recursión) no se exporta salvo que se permitan instrucciones sin confirmar (sección 12), porque nunca se vio en un respaldo real.
- El principal del respaldo real **solo llama módulos**: movimientos o salidas sueltas en el principal compilan, pero con un aviso. Si el pad los rechazara, ponerlos dentro de un `PROC`.

```krl-error
; ERROR: los módulos del pad no reciben parámetros
PROC apilar(altura)
    SET_OUT(Y010, ON)
ENDPROC
apilar(5)
```

```krl-error
; ERROR: falta cerrar un paréntesis «)» en «PROC home(id=1)»
PROC home(id=1)
    SET_OUT(Y010, ON)
ENDPROC
home()
```

```krl-error
; ERROR: solo el programa principal llama módulos
PROC b()
    SET_OUT(Y010, ON)
ENDPROC
PROC a()
    b()
ENDPROC
a()
```

```krl-unverified
; un módulo que llama a otro: solo con «instrucciones sin confirmar»
PROC b()
    SET_OUT(Y010, ON)
ENDPROC
PROC a()
    b()
ENDPROC
a()
```

```krl-error
; ERROR: declarado más de una vez
PROC a()
    SET_OUT(Y010, ON)
ENDPROC
PROC a()
    SET_OUT(Y011, ON)
ENDPROC
a()
```

Para pasar información entre módulos se usan las entradas y salidas.

## 11. Tabla de compatibilidad

Verificada ejecutando el compilador sobre cada construcción.

| Construcción | Pad (respaldo) | VM de la PC |
|---|:---:|:---:|
| `POINT` `JOINT` + `MOVEJ` | ✅ | ✅ |
| `POINT` `WORLD` + `MOVEL` | ✅ | ✅ |
| `MOVEJ` a un `WORLD` / `MOVEL` a un `JOINT` | ❌ error | ✅ (la VM no distingue) |
| `OFFSET` sobre `WORLD` | ✅ | ✅ |
| `OFFSET` sobre `JOINT` | ❌ error | ✅ |
| `SET_OUT` | ✅ | ✅ |
| `TOOL` / `COORD` | ✅ | ✅ |
| `BASE` | ❌ no existe | ❌ no existe |
| `WAIT <n>s` | ✅ (hipótesis) | ✅ |
| `WAIT UNTIL MOVE_DONE` | ⚠️ compila, no se exporta | ✅ |
| `WAIT_IN` | ❌ error | ✅ |
| `IF Xnnn == 0` | ✅ | ✅ |
| `IF Xnnn == 1` | ⚠️ solo «sin confirmar» | ✅ |
| `IF` anidado | ✅ | ✅ |
| `IF … ELSE` | ❌ error | ✅ |
| `VAR`, asignación, `IF` sobre variables | ❌ error | ✅ |
| `TIMER` | ⚠️ compila, no hace nada | ✅ (se parsea, no emite nada) |
| `PROC()` sin parámetros + llamada del principal | ✅ | ✅ |
| `PROC` con parámetros | ❌ error | ✅ |
| `PROC` que llama a otro `PROC` | ⚠️ solo «sin confirmar» | ✅ |
| `.src` + `.dat` + `config.dat` | ✅ | ✅ |

Todo lo marcado ❌ falla al compilar con un mensaje que explica el motivo. Nunca genera algo aproximado.

## 12. Compilar, exportar y simular

### Desde la interfaz

```text
python -m gui.app
```

- **F7** compila (VM de la PC): bytecode, tabla de puntos y árbol del proyecto.
- **F5** ejecuta en la VM de la PC (contra el simulador de E/S o contra el robot real conectado).
- **Ctrl+E** — *Exportar para el pad…*: escribe el respaldo `HCBackupRobot_<fecha>.zip` en la carpeta que elijas. El nombre del programa en el pad sale del nombre del archivo (hasta 40 letras sin acentos, números, `_`, `-` y `.`). Se copia a la raíz de un pendrive y se importa desde el pad.
- **Programa → Permitir instrucciones sin confirmar en el pad**: apagado por defecto. Habilita lo que sabemos expresar pero el respaldo real nunca mostró (`IF Xnnn == 1`, un módulo que llama a otro). Probar a velocidad baja.
- **F9** simula (pestaña «Simulación 3D»). El simulador siempre permite lo «sin confirmar»: simular no tiene riesgo.
- **F8** digitaliza la posición actual del robot (conectado) como punto `WORLD`; el punto se agrega al final del `.dat`.

### El simulador (`docs/SIMULATOR.md`)

Ejecuta el respaldo del pad sobre el modelo cinemático del robot (el BRTIRUS1510A de la celda, y los de la biblioteca de Drive). Avisa ejes fuera de rango, puntos inalcanzables y singularidades, estima el tiempo de ciclo, busca **choques** del brazo y la herramienta contra las piezas de la celda, el piso y el propio brazo, y dibuja gráficas de ejes y de la punta. El robot se puede correr y girar dentro de la celda (pedestal, pared, techo); los puntos del programa siguen siendo respecto de su base.

> ⚠️ Es un modelo: el rango y el sentido de giro de cada eje, el cero de la muñeca y la convención de los ángulos `U, V, W` (se supone R = Rz(W)·Ry(V)·Rx(U)) son **hipótesis** hasta contrastarlas con el robot real. El perfil de aceleración también es un supuesto: el respaldo no trae la del controlador.

### Desde Python

```python
from compiler.pad_codegen import PadOptions, compile_to_pad_report

texto = open("programa.krlb", encoding="utf-8").read()
backup, avisos = compile_to_pad_report(texto, PadOptions(program_name="MiPrograma"))
for aviso in avisos:
    print(aviso)
backup.write("carpeta_de_salida")          # escribe HCBackupRobot_<fecha>.zip
```

Con los tres archivos:

```python
from compiler import program_files

fuentes = program_files.load("paletizado.src")      # trae su .dat y el config.dat
backup, avisos = program_files.compile_pad(fuentes)  # errores con archivo y línea
```

Para la VM de la PC: `compiler.codegen.compile_source(texto)` (o `program_files.compile_vm`).

### Desde la consola

```text
python -m sim.check paletizado.src                       # simula un .src, un .krlb o un respaldo .zip
python -m sim.check HCBackupRobot_*.zip --model BRTIRUS1510A --input X012=1
python -m pad.listing HCBackupRobot_*.zip                # lista legible de un respaldo
```

## 13. Programa completo de ejemplo

Usa todo lo que el pad acepta. Los puntos van arriba, la lógica en `PROC` y el principal solo los llama.

```krl
; ---------------------------------------------------------------
; Celda de soldadura — cordón de 300 mm, con sensor de pieza
; ---------------------------------------------------------------
POINT p_home  = JOINT(0.347, 45.894, -44.865, -0.792, -75.952, -0.859)
POINT p_pieza = WORLD(1097.1, -150.0, 721.9, 180.0, -10.0, 180.0)
POINT p_sobre = p_pieza + OFFSET(0, 0, 100, 0, 0, 0)
POINT p_fin   = p_pieza + OFFSET(0, 300, 0, 0, 0, 0)

PROC ir_a_home()
    MOVEJ p_home SPEED 80
ENDPROC

PROC soldar()
    COORD 1
    TOOL 2
    MOVEL p_sobre SPEED 50          ; aproximación
    MOVEL p_pieza SPEED 10          ; bajada lenta y recta
    SET_OUT(Y010, ON)               ; arco
    MOVEL p_fin SPEED 5             ; el cordón
    SET_OUT(Y010, OFF)
    MOVEL p_fin + OFFSET(0, 0, 100, 0, 0, 0) SPEED 50
ENDPROC

PROC avisar()
    SET_OUT(Y011, ON)
    WAIT 1s
    SET_OUT(Y011, OFF)
ENDPROC

; --- programa principal: solo llama módulos ---
ir_a_home()
soldar()
ir_a_home()
```

## 14. Errores comunes y qué significan

| Mensaje (resumido) | Qué pasó |
|---|---|
| `MOVEJ necesita un punto JOINT(...)` / `MOVEL necesita un punto WORLD(...)` | Moviste a un punto del otro tipo (sección 4) |
| `Punto no definido: 'p'` | Falta el `POINT p = …`, hay un typo, o el `POINT` está **después** de donde se usa |
| `POINT 'p' ya estaba declarado` | Nombre repetido (también entre `config.dat` y el `.dat`) |
| `OFFSET sobre un punto JOINT…` | `OFFSET` solo suma sobre `WORLD` |
| `SPEED n: en el pad es un % entre 0.1 y 100` | Velocidad fuera de rango |
| `'Y20' no es una salida (Y) del pad` | Las E/S se escriben `Y` + tres dígitos octales: `Y020` |
| `las palabras clave van en mayúscula` | Escribiste `movej` en vez de `MOVEJ` |
| `no se esperaba «1» en «BASE 1»` | `BASE` no existe: es `COORD` |
| `En el pad, IF solo puede preguntar por una entrada` | Condición distinta de `Xnnn == 0` (`ON`/`OFF`, `!=`, variables) |
| `ELSE todavía no se puede exportar al pad` | Sección 9 |
| `Las variables (VAR y asignaciones) todavía no se pueden exportar al pad` | `VAR` solo sirve en la VM de la PC |
| `WAIT_IN(X…) todavía no se puede exportar al pad` | Usá `IF Xnnn == 0` para reaccionar a un sensor |
| `los módulos del pad no reciben parámetros` | `PROC` sin parámetros; pasá información por E/S |
| `falta cerrar un paréntesis «)» en «PROC x(id=1)»` | `PROC` no lleva `id` |
| `solo el programa principal llama módulos` | Un `PROC` llama a otro: sección 10 |
| `TOOL dentro de un IF no se puede exportar` | Sacalo del `IF` |
| `un .dat solo tiene puntos (POINT)` | Pusiste otra cosa en un `.dat` |
| `el punto p se declara en p.dat…` | Un `POINT` en el `.src` |
| `Unexpected token …` / `no se esperaba …` | Error de sintaxis. Lo más común: palabra clave en minúscula, falta `ENDIF` o `ENDPROC`, o una coma decimal |

## 15. Ficha resumida

Para pegar en otro chat como contexto mínimo:

```text
LENGUAJE DSL TIPO KRL — ROBOT BORUNTE (destino: respaldo del pad)

Archivos: <nombre>.src (lógica, sin POINT) + <nombre>.dat (solo POINT)
          + config.dat (puntos comunes de la carpeta). O un .krlb todo junto.
Sintaxis: una sentencia por línea, palabras clave en MAYÚSCULAS,
comentarios con ';', decimales con punto. E/S: X o Y + 3 dígitos octales
(X010..X107, Y010..Y107).

PUNTOS (se declaran ANTES de usarse, también antes del PROC que los usa)
  POINT <n> = JOINT(j1, j2, j3, j4, j5, j6)    grados; para MOVEJ
  POINT <n> = WORLD(x, y, z, u, v, w)          mm y grados; para MOVEL
  POINT <n> = <WORLD> + OFFSET(6 valores)      se resuelve al compilar

SENTENCIAS
  MOVEJ <JOINT> SPEED <0.1..100>     articular, bloqueante
  MOVEL <WORLD> SPEED <0.1..100>     lineal, bloqueante
  SET_OUT(Y<nnn>, ON|OFF)
  WAIT <n>s                          0.001 .. 3600 s
  COORD <0..15>   TOOL <0..15>       afectan los movimientos siguientes
                                     (no van dentro de un IF)
  <nombre>()                         llamar un PROC (solo desde el principal)
  IF X<nnn> == 0 THEN ... ENDIF      cuerpo si la entrada está APAGADA
  PROC <nombre>() ... ENDPROC        sin parámetros, sin id, en el nivel superior

NO EXISTE EN EL PAD (falla al compilar, a propósito)
  ELSE            -> dos IF (el segundo con == 1, sin confirmar)
  WHILE / FOR     -> repetir código o llamar varias veces
  VAR / asignar   -> solo en la VM de la PC
  WAIT_IN         -> IF Xnnn == 0
  BASE            -> es COORD
  PROC con parámetros, PROC con id, PROC que llama a otro PROC (sin confirmar)

SIN CONFIRMAR CONTRA EL PAD
  que el pad acepte un respaldo generado por este compilador; cuál acción
  es MOVEJ y cuál MOVEL; que WAIT sea la espera temporizada; la numeración
  octal de E/S; IF Xnnn == 1; un módulo que llama a otro.
```

## 16. Qué está confirmado y qué no

**Nada de lo que genera este compilador se probó importándolo a un pad real.** El formato del respaldo se dedujo de un único export de producción (programa de soldadura de 9 módulos, ~1100 acciones, agosto 2026). Que `pad/backup.py` lea y reescriba ese respaldo **byte a byte** prueba que entendimos el archivo, no que el pad acepte lo que escribimos.

| Cosa | Grado | Nota |
|---|---|---|
| Contenedor del zip, `.act` en una línea JSON por bloque | Verificado | Round-trip byte a byte |
| `MOVEJ` = acción 4 (ejes), `MOVEL` = acción 10 (cartesiano) | Deducido | Encaja con todos los casos del respaldo |
| `SET_OUT`, `TOOL`, `COORD`, `CALL`, fin de módulo | Deducido | |
| `IF Xnnn == 0` | Deducido | 44 casos |
| `IF Xnnn == 1` | **Sin confirmar** | Nunca visto; solo con «sin confirmar» |
| Un módulo que llama a otro | **Sin confirmar** | Nunca visto; solo con «sin confirmar» |
| `WAIT n s` = acción 100 | **Hipótesis** | Pocos casos |
| Numeración octal de E/S | **Hipótesis** | 3 casos |
| `smooth`, `bindIOInfo`, `distance` | Desconocido | Se copian tal cual |
| Valores de herramientas y sistemas | Se cargan en el pad | El programa solo guarda el número |

La lista ordenada de qué probar primero en el pad está en `docs/PAD_FORMAT.md` («Para confirmar con el pad»). Lo primero: importar un respaldo **reescrito sin cambios** por `pad/backup.py`; después uno generado, **a velocidad baja**.

<!-- limites: speed=0.1-100 wait=0.001-3600 tool=0-15 io=010-107 pos=10000 -->
