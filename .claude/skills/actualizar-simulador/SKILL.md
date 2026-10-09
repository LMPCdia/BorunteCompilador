---
name: actualizar-simulador
description: Mantener al día la biblioteca de modelos 3D (carpeta RobotsBoruntesSimulador de Google Drive) y el simulador de BorunteDSL - revisar y acomodar la carpeta, importar un robot nuevo desde el STEP del fabricante, agregar herramientas/piezas, y publicar un .exe nuevo con su link. Usar cuando el usuario dice que subió modelos al Drive, pide agregar un robot, actualizar la biblioteca, "actualizá el simulador", o pide el link del .exe nuevo.
---

# Actualizar la biblioteca y el simulador

La app lee la biblioteca EN LÍNEA (`sim/library.py`): para piezas y herramientas
no hace falta tocar código, alcanza con que estén bien ubicadas en Drive. El
código cambia solo para **robots que vienen con la app** y para **arreglos**.
Leé `CLAUDE.md` (reglas del proyecto) y `docs/SIMULATOR.md` antes de empezar.

Datos fijos:

- Carpeta: **RobotsBoruntesSimulador**, ID `1BnYko-GDci726XcNMSW4Adn8om8zg6oO`
  (unidad compartida INGENIERIA de defymotion). Subcarpetas: `Robots`,
  `Herramientas`, `Bases`, `Mesas`, `Piezas`, `Seguridad`, y el doc
  `LEEME - cómo cargar modelos` con las reglas para quien sube archivos.
- Tiene que estar compartida como "Cualquier persona con el enlace (lector)".
- Repo `LMPCdia/BorunteCompilador`; el `.exe` lo arma GitHub Actions
  (`Build .exe`) en cada push.

## 1. Revisar la carpeta

```bash
python -m sim.library            # árbol por categoría + "cosas para acomodar"
```

- Cada problema que lista tiene su arreglo (suelto en la raíz, robot fuera de
  `Robots/`, CAD nativo sin STEP al lado).
- Si falla con "¿Está compartida…?", la carpeta perdió el permiso público:
  pedírselo al usuario (no se arregla desde acá).
- Si el entorno no llega a `drive.google.com` (403 del proxy), el usuario
  tiene que agregar `drive.google.com` y `drive.usercontent.google.com` a los
  dominios permitidos del entorno (leer la doc con `read_documentation`
  topic `environment.network`).

**Acomodar.** Con el conector de Google Drive se pueden crear carpetas
(`create_file` con `application/vnd.google-apps.folder`) y renombrar
(`update_file` con `title`). **Mover NO anda** en la unidad compartida
("The caller does not have permission", aunque el usuario sea organizador de
archivos): pedile al usuario que arrastre, diciéndole exactamente qué va
adónde. Nunca mandes nada a la papelera sin que lo pida.

Clasificación (por carpeta, no por nombre; ver `classify` en `sim/library.py`):
`Robots/…` o un STEP con `BRTIRUSxxxx` en el nombre → robot; `Herramientas`,
`Grippers`, `Pinzas`, `Antorchas`, `Ventosas`… → herramienta; el resto →
pieza. Si el usuario inventa una categoría nueva de herramientas, sumá la
palabra a `TOOL_WORDS` (y un test).

## 2. Robot nuevo

Un robot es su carpeta `Robots/<MODELO>/` con tres cosas (así lo pide el
LEEME de la carpeta):

1. el **STEP del ensamble** del fabricante (obligatorio);
2. el **datasheet en PDF** (fuente de los datos; la app no lo lee, solo lo
   abre con *Ver datasheet*);
3. la **planilla «Parámetros <MODELO>»** (Google Sheet; formato en
   `sim/robot_params.py`), que es lo que la app usa para los ejes.

En la app, *Usar este robot* lee la planilla, importa el STEP si hace falta,
compara cotas y alcance con el CAD y verifica la cinemática inversa
(`prepare_from_library`).

### Armar la planilla desde el PDF (la rutina lo hace sola)

`python -m sim.library --pendientes` lista en JSON qué hacer:

- `"action": "crear"`: carpeta de robot con PDF y sin planilla;
- `"action": "revisar"`: hay planilla y PDF, pero la planilla no se revisó
  contra ESE PDF (la fila «Datasheet revisado» no tiene el `pdf_id`).

Por cada uno:

1. Bajar el PDF al scratchpad:
   `curl -L -o ds.pdf "https://drive.usercontent.google.com/download?id=<pdf_id>&export=download&confirm=t"`
   y leerlo con la herramienta Read (de a 20 páginas). Buscar la tabla de
   parámetros ("Basic Parameters", "Specifications", 基本参数, 动作范围,
   最大速度): rango y velocidad máxima por eje, alcance, carga,
   repetibilidad, y si aparecen, aceleraciones y velocidad lineal máxima.
2. **crear**: copiar la plantilla formateada con `copy_file`
   (`fileId` `10p1u009Q3EEa8RKT2w4vAlHO6KkgjUvT9q4enW9e-ss`, `parentId` =
   `folder_id`, `title` = `Parámetros <model>`) y escribir el modelo en B1.
   **revisar**: trabajar sobre `params_id`.
3. Escribir con `update_values` (Google Sheets, pestaña `Parámetros`) solo
   las celdas de datos, sin tocar el formato:
   - ejes en B4:H9 (Mínimo, Máximo, Velocidad, Aceleración, Sentido,
     Confirmado, Notas); generales en B12:E21; `Datasheet revisado` en B22
     con el `pdf_id`.
   - **No adivines.** Cada valor que escribas tiene que leerse en el PDF;
     en Notas va "pág. N" de donde salió. Lo que no está o no se lee con
     certeza queda vacío. Un rango simétrico se escribe "±165".
   - **Confirmado = "no"** en todo lo que escribas (lo confirma el usuario).
     En "revisar", no pises un valor ya confirmado ("sí"): si el PDF dice
     otra cosa, dejalo y anotá la diferencia en Notas.
   - **Sentido**: el datasheet casi nunca lo dice; no lo cambies.
   - Las aceleraciones casi nunca están: vacías salvo que el PDF las dé.
4. Verificar: bajar la planilla como CSV
   (`https://docs.google.com/spreadsheets/d/<id>/export?format=csv`) y
   pasarla por `sim.robot_params.parse_params`; tiene que leerse sin errores.
   `python -m sim.library --pendientes` ya no tiene que listarla.
5. Avisar al usuario: qué planilla, la tabla transcripta con la página de
   cada dato, qué quedó vacío y por qué, y que revise y ponga "sí" en
   Confirmado.

Un robot que el usuario va a usar seguido conviene **traerlo a la app**
(`sim/models/`), como el BRTIRUS1510A: así funciona sin red y queda probado.
Desde la pestaña Biblioteca también se importa solo, pero a la PC de cada uno
y sin verificación.

1. Bajar el STEP (en el scratchpad, no en el repo: es del fabricante y pesa):
   ```bash
   curl -L -o robot.step "https://drive.usercontent.google.com/download?id=<ID>&export=download&confirm=t"
   ```
   El ID sale de `python -m sim.library` o de `mcp__Google_Drive__search_files`.
2. Importar (unos minutos; corré en segundo plano si pasa de 10 min):
   ```bash
   python -m sim.robot_import robot.step --name BRTIRUSxxxxA --joints-from BRTIRUS1510A --out sim/models
   ```
3. **Verificar antes de confiar** (el importador está calibrado con el 1510A;
   `BORUNTE_RECIPE` dice en qué cilindro de cada parte está cada eje):
   - El **alcance** que imprime tiene que coincidir con el del modelo (el
     1510A dio 1511 mm). Si no coincide, la receta no sirve para este robot:
     buscá los ejes con los cilindros grandes de cada parte (como se hizo en
     el 1510A: agrupar `CYLINDRICAL_SURFACE` por eje y radio) y ajustá una
     receta propia. No publiques un robot con el alcance mal.
   - Las partes tienen que llamarse `PBR…A000`..`F000` + la brida sin código.
     Si `import_robot` dice que falta una, mirá los nombres con
     `StepFile(path).parts()`.
   - Mirá la nota del antebrazo corrido: si lo centró, el antebrazo tiene que
     quedar simétrico en Y (`robot_link_meshes(model)[4].bounds()`).
   - Dibujalo en 3 poses (cero, HOME, una girada) con `xvfb-run` +
     `QT_QPA_PLATFORM=xcb` y mirá la captura: los eslabones no se separan.
   - `CollisionChecker(...).distances_at` en HOME y en cero: sin choques
     consigo mismo.
4. **Rangos, velocidades y sentidos de giro**: el importador los copia de
   `--joints-from` y los marca como HIPÓTESIS en `notes`. Pedile al usuario la
   tabla "Basic Parameters" del robot. Solo lo que el usuario confirme se
   escribe como confirmado (regla 1 de `CLAUDE.md`).
5. Tests como los del 1510A en `tests/test_robot_import.py` (alcance, base en
   z=0, brida donde dice la cinemática, sin choques en HOME). Si es el robot
   por defecto, cambiá `DEFAULT_MODEL` en `gui/sim_view.py` y `sim/check.py`.
6. Docs: tabla de modelos en `docs/SIMULATOR.md`, estado en `CLAUDE.md` y
   `README.md`.

## 3. Herramientas y piezas

No requieren código: la app las baja de la carpeta. Lo que sí conviene revisar:

- Herramienta: el STEP en coordenadas de la brida (origen en el centro de la
  brida, Z saliendo). Si no viene así, se corrige con **Montaje** en la
  pestaña Choques; anotá en el LEEME o en el nombre qué montaje lleva.
- Un STEP que gmsh no puede mallar: `load_step` saca las caras que fallan
  (`Mesh.skipped_faces`) y sigue; si saca muchas, pedí otra exportación.

## 4. Publicar

1. `QT_QPA_PLATFORM=offscreen pytest tests/ -q` en verde y
   `QT_QPA_PLATFORM=offscreen python -m gui.app --self-test` → `SELF-TEST OK`.
2. Commit (con el trailer de la sesión) y `git push -u origin <rama>`.
3. Esperar el build: `mcp__github__actions_list` (`list_workflow_runs`, rama)
   → cuando termine, `list_workflow_run_artifacts` del run. Un build tarda
   ~4 min; esperar con `Monitor` (`sleep 240`), no con `sleep` suelto.
4. Darle al usuario el link
   `https://github.com/LMPCdia/BorunteCompilador/actions/runs/<run>/artifacts/<artifact>`
   (pide estar logueado en GitHub; vence a los 90 días). Si el build falla,
   `get_job_logs` con `failed_only`, arreglar y volver a empujar: nunca dar el
   link de un build rojo.

## Cosas que ya pasaron (no repetir)

- Los nombres de las partes de Borunte vienen en **GBK**: OpenCascade los
  pierde. Por eso `sim/step_assembly.py` separa el ensamble leyendo el STEP a
  mano (padre/hijo sale del `NEXT_ASSEMBLY_USAGE_OCCURRENCE`).
- El ensamble del fabricante no está en la posición cero (J4 a 72° en el
  1510A) y una unión del CAD puede dejar una parte corrida a lo largo de un
  eje: siempre verificar con el alcance y la simetría.
- Una carpeta de Drive vacía devuelve `flip-entries` sin entradas: no es un
  error de permisos.
- Los archivos de la caché de la biblioteca viven en `~/BorunteDSL/biblioteca`;
  las celdas guardan el ID de Drive (`drive_id`) para volver a bajarlos en
  otra PC.
