# Arquitectura — lenguaje tipo KRL para Borunte, ejecutado por el pad

## Resumen

El programa se escribe en la PC en un lenguaje tipo KRL (`.src` con la
lógica + `.dat` con sus puntos + `config.dat` con los puntos comunes de la
carpeta; o todo en un `.krlb`, como antes), se compila a un **respaldo del pad** (`HCBackupRobot_<fecha>.zip`) y se importa
en el teach pendant del Borunte por pendrive. A partir de ahí **lo ejecuta el
controlador del robot solo**, con sus propias E/S: no hay PLC ni PC en el
lazo de ejecución.

```
┌───────────────────────── PC ──────────────────────────┐
│                                                        │
│  DSL (.src + .dat + config.dat, o un .krlb)            │
│    │  compiler/program_files.py: los junta y revisa    │
│    │  parser (Lark) — compiler/grammar.lark            │
│    ▼                                                   │
│  AST — compiler/ast_nodes.py                           │
│    │                                                   │
│    ├──► compiler/pad_codegen.py ──► HCBackupRobot_*.zip ──── pendrive ───┐
│    │      (salida de producción)                       │                 │
│    │                                                   │                 ▼
│    └──► compiler/codegen.py ──► bytecode               │     ┌──────────────────────┐
│           (solo simulación)       │                    │     │ Pad + controlador     │
│                                   ▼                    │     │ Borunte               │
│                  runtime/vm.py ─► robot_client.py ─────┼───► │ - ejecuta el programa │
│                  (VM de referencia)  (Modbus TCP)      │     │ - E/S propias (X/Y)   │
│                                                        │     └──────────────────────┘
└────────────────────────────────────────────────────────┘
```

## Las dos salidas del compilador

Comparten gramática y AST. Difieren en lo que soportan, porque el formato del
pad todavía está a medio decodificar:

| | Pad (`pad_codegen.py`) | VM de referencia (`codegen.py`) |
|---|---|---|
| Para qué | Producción: lo ejecuta el robot | Probar la lógica en la PC |
| `MOVEJ` / `MOVEL` | Sí. `MOVEJ` exige punto `JOINT`, `MOVEL` exige `WORLD` | Sí, sin distinguir |
| `SET_OUT`, `WAIT n s`, `PROC`/llamadas | Sí | Sí |
| `IF` | Solo sobre una entrada (`IF X010 == 1`), sin `ELSE` | `VAR == CONST`, `VAR == VAR`, con `ELSE` |
| `VAR`, asignaciones, `WAIT_IN`, `PROC` con parámetros | No (error de compilación) | Sí |
| Formato | Deducido de un respaldo real — `docs/PAD_FORMAT.md` | `docs/INSTRUCTION_SET.md` |

Regla del generador del pad: **lo que no sabemos expresar es un error de
compilación, nunca una adivinanza**. El robot ejecuta lo que se le carga sin
nadie mirando.

## Qué se hace por Modbus y qué no

`comms/robot_client.py` (Modbus TCP, mapa en `docs/MODBUS_REGISTER_MAP.md`)
sigue sirviendo para:

- **Digitalizar puntos**: llevar el robot a una posición y leerla.
- **Simular contra el robot real** con la VM de referencia (un movimiento a la
  vez, sin confirmar contra hardware).

No se usa para ejecutar en producción. Por Modbus no se puede leer ni escribir
el programa del pad en bloque; por eso la vía es el respaldo por pendrive.

## Decisiones de diseño

1. **Ejecuta el pad, no un PLC.** Se descartó la VM en ladder sobre un Coolmay
   CX3G: el pad ya tiene movimientos, E/S, saltos y subrutinas, y el programa
   no depende de que una PC o un PLC estén vivos.
2. **El respaldo se reescribe sin perder nada.** `pad/backup.py` guarda las
   acciones como `dict` crudos y conserva lo que no entiende. Leer y reescribir
   un respaldo real da el mismo contenido byte a byte.
3. **Lo desconocido se copia de un respaldo real.** El `.fnc` y las líneas del
   `.act` que no entendemos salen de `pad/template.fnc` o de un respaldo del
   mismo robot (`PadOptions.template`).
4. **Persistencia de puntos: en la PC** (pendiente: hoy viven solo en memoria
   de la GUI).

## Pendiente de validar con el pad

- [ ] Que el pad importe un respaldo reescrito sin cambios por `pad/backup.py`.
- [ ] Que importe y ejecute uno generado por `pad_codegen.py` (a velocidad baja).
- [ ] Numeración octal de las E/S (`point` 2 = `X012`): solo 3 casos vistos.
- [ ] `WAIT` = acción `100` con `type: 100`, e `IF` con `pointStatus: 0` = "si
      la entrada está en OFF".
- [ ] Cómo se expresan en el pad: espera de entrada, salto incondicional,
      variables.


## Archivos del programa (`compiler/program_files.py`)

Como en KUKA, un programa son dos archivos con el mismo nombre, más uno común
a la carpeta:

| Archivo | Qué tiene |
|---|---|
| `paletizado.src` | la lógica: movimientos, velocidades, esperas, salidas, IF, PROC. **Sin puntos** |
| `paletizado.dat` | los puntos de ese programa: **solo** `POINT` (y comentarios) |
| `config.dat` | puntos comunes a todos los programas de la carpeta (HOME, poses de traslado…). Opcional |

- Se compilan juntos en ese orden (config.dat, .dat, .src); los errores y
  avisos dicen archivo y línea («paletizado.src, línea 12: …»).
- Errores: algo que no sea `POINT` en un `.dat`; un `POINT` en el `.src`
  (también dentro de un PROC o un IF); el mismo nombre de punto en
  config.dat y en el `.dat`.
- Aviso: un punto del `.dat` que el `.src` no usa (los de config.dat no
  avisan: son para varios programas).
- El respaldo del pad **no cambia**: ahí cada movimiento lleva su punto
  adentro (ver `docs/PAD_FORMAT.md`). La separación es solo de la PC.
- Un `.krlb` de antes (todo junto) se abre y compila igual; *Archivo →
  Separar en .src y .dat* lo convierte, y guardarlo como `.src` también.
- GUI: la pestaña Programa tiene una sub-pestaña por archivo. Guardar como
  `.src` escribe el `.src` y su `.dat`; `config.dat` se escribe solo si se
  editó o si la carpeta todavía no tiene uno (si la carpeta tiene uno
  distinto y no se tocó, manda el de la carpeta: lo puede haber cambiado otro
  programa). Los puntos digitalizados (F8) se agregan al final del `.dat`.
- `python -m sim.check paletizado.src` simula el programa con sus `.dat`.
