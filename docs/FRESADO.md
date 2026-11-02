# Fresado: usar el robot como fresa CNC 3D

Pestaña **Fresado** del simulador (`gui/cam_panel.py`), cálculo en `sim/cam.py`. De una pieza
CAD y un área dibujada en 2D sale un programa `.src` + `.dat` común: se compila, se simula y se
exporta al pad por el camino de siempre. El formato del pad no cambió.

> ⚠️ **No probado en un robot real.** La primera vez: sin fresa, en el aire y a velocidad baja.
> Un brazo de 6 ejes es mucho menos rígido que una CNC: sirve para materiales blandos (madera,
> espuma, plásticos) y pasadas livianas. La exactitud real depende de la calibración de la
> herramienta y de la pieza respecto del robot; acá se calcula con el modelo ideal.

## Cómo se usa

1. Elegí el robot e importá la pieza en **Celda** (STEP, STL u OBJ, en mm, con los polígonos que
   tenga) y ubicala donde va a estar de verdad.
2. Cargá la fresa en **Herramientas**: X, Y, Z hasta la punta y **+Z por el eje de la fresa**. Con
   la herramienta 0 el programa pone la *brida* sobre la pieza, y la pestaña avisa.
3. En **Fresado** elegí la pieza y la vista. La pieza aparece en planta (claro = alto).
4. Delimitá el área: clic agrega un punto, clic derecho o doble clic la cierra. Se pueden dibujar
   varias áreas. **Toda la pieza** toma el rectángulo que la encierra.
5. **Calcular trayectoria**: dibuja las pasadas en el plano y en la vista 3D, y verifica con la
   cinemática inversa que el robot alcance todos los puntos.
6. **Generar código**: carga el programa en el editor, sin guardar. Guardalo y simulalo.

## Qué calcula

**Área → superficie.** La pieza se convierte en un mapa de alturas visto desde la vista elegida.
Las pasadas se arman en el plano y cada punto toma la altura de la superficie abajo. No se usa la
altura del punto sino la de la **fresa apoyada** (*drop cutter*): se pasa la forma de la
herramienta por el mapa, así no se clava en pendientes ni escalones.

| Opción | Qué hace |
|---|---|
| Zigzag | Renglones separados por *Entre pasadas*, en la *Dirección* dada. El centro queda a un radio del borde del área. Une renglones sin levantar si el paso queda dentro |
| Contorno | Sigue el borde: por dentro, por fuera (corrido un radio) o sobre la línea |
| Profundidad / Por nivel | Bajadas sucesivas debajo de la superficie, que copian su forma. 0 = una pasada apoyada |
| Eje fijo | La fresa siempre paralela a la vista, como una CNC de 3 ejes. Vale para bola y plana |
| Eje normal | La fresa se inclina con la superficie, hasta *Inclinación máx.* Solo bola: su centro no depende de la inclinación, el canto de una plana sí, y ahí no se garantiza que no se clave |
| Diámetro porta | Limita la profundidad para que la porta no toque la pieza, y avisa. Solo con eje fijo |

Entre pasadas la fresa sube por su eje hasta *Altura segura* por encima de lo más alto de los
puntos de corte, viaja a *Aire* y entra a *Entrada*. Donde el área queda fuera de la pieza no hay
pasadas. Las rectas se reducen a sus extremos (tolerancia 0,05 mm).

**Robot.** Como la fresa gira, el giro alrededor de su eje es libre: se prueban 12 valores y se
elige el que alcanza todo con J5 más lejos de 0 (la singularidad de muñeca). Si un punto no se
alcanza no hay programa: es un error con las coordenadas, no un punto "cercano".

## Limitaciones conocidas

- **Una vista por trabajo.** El mapa ve la cara de arriba: lo que queda debajo de un voladizo no
  se mecaniza desde esa vista. Para otra cara, otra vista y otro programa.
- **`SPEED` es un %** (`docs/LENGUAJE.md`), no mm/s: los modelos no traen la velocidad lineal a
  100 %. El avance real hay que medirlo. Cuánto se acerca el pad a la recta con muchos puntos
  seguidos tampoco está confirmado.
- **Cantidad de puntos.** No se conoce un máximo del pad; un programa de miles de puntos no está
  probado. Sale un `POINT` por punto.
- **Sin simulación de material**: no se ve la pieza cortada.
- **Áreas y parámetros no se guardan** con la celda: si cerrás, se vuelven a dibujar.
- Las áreas quedan en coordenadas de la celda: si movés la pieza, hay que recalcular (y
  normalmente redibujar).

## Choques

Al generar el programa la pieza queda marcada `machined` (`sim/scene.py`, se guarda con la celda):
la herramienta entra a propósito, así que no se buscan choques **herramienta contra esa pieza**.
El brazo sí se revisa, y la herramienta contra todo lo demás. Por eso el cuerpo del husillo contra
la pieza solo lo cubre *Diámetro porta*. Para volver a revisarlo hay que poner `machined: false`
en el JSON de la celda.
