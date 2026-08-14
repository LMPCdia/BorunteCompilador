# Mapa de registros Modbus del Borunte — consolidado

Fuente: `伯朗特Modbus_TCP通信协议说明书V1_0`, `伯朗特驱控一体MODBUS_通讯协议解析`.
Todas las direcciones en formato `decimal (hex)`. Valores de posición: enteros
escalados x1000 (3 decimales). Registros de 32 bits ocupan 2 words (16 bits c/u).

> ⚠️ Todo lo marcado como "no confirmado" debe validarse antes de depender de
> ello en el diseño final.

## Lectura — telemetría en vivo

| Dato | Dirección inicio | Registros | Notas |
|---|---|---|---|
| Estado de movimiento | `2470 (0x09A6)` | 1 | 0=parado, 1=en movimiento. **Poll obligatorio tras cada comando de movimiento.** |
| Posición de eje (joint) | `2267 (0x08DC)` | 2 por eje | Grados, x1000. Base + eje×2 |
| Cantidad de ejes | `2267 (0x08DB)` | 1 | Típico 6 |
| Posición mundo (cartesiano) | `2347 (0x091C)` | 2 por eje | mm, x1000. Orden: X,Y,Z,U,V,W,M7,M8 |
| Código de alarma actual | `(0x095C)` | 1 | |
| Estado E/S (entrada) | `(0x088B)` aprox | 2 por bloque IO | hasta 5 tarjetas |
| Estado E/S (salida) | `(0x08B3)` | 2 por bloque IO | hasta 5 tarjetas |

## Escritura — comandos tipo "botón remoto" (función 0x06, un valor = ejecuta)

| Comando | Dirección | Notas |
|---|---|---|
| Detener acción actual | `20000 (0x4E20)` | Reinicia desde el principio |
| Pausar acción actual | `20001 (0x4E21)` | Continúa desde el paso actual |
| Modo single-loop | `20002 (0x4E22)` | |
| Activar botón Start | `20003 (0x4E23)` | ⚠️ No confirmado si dispara el programa cargado en el pad o una posición mandada manualmente — validar con hardware |
| Botón Stop | `20004 (0x4E24)` | |
| Limpiar alarma + siguiente instrucción | `20005 (0x4E25)` | |
| Limpiar alarma + continuar automático | `20006 (0x4E26)` | Solo en modo automático |
| Velocidad global | `20200 (0x4EE8)` | 0–1000 = 0.0%–100.0%, lectura/escritura |

## Escritura — E/S forzadas

| Dato | Dirección | Función Modbus |
|---|---|---|
| Forzar bit Y/M/EUY | variable | 0x05 (single) / 0x0F (bulk) |
| Bloque salida IO board completo | `2227 (0x08B3)` en adelante | 0x10, múltiplo de 2 registros |
| Control de punto de salida (ID+estado) | `20057-20058 (0x4E59-0x4E5A)` | punto 0-31, estado 0/1 |

## Bloque de variables abiertas (el más importante para "mandar un punto")

| Dato | Dirección | Notas |
|---|---|---|
| Tabla interna `allpara[]` | `20300 (0x4F4C)` en adelante | 1000 parámetros, 32 bits c/u (2 registros) |
| **Zona abierta a host externo** | `21900 (0x558C)` = `allpara[800]` | **Rango usable: 800–900 (aprox 100 slots libres)** |

Ejemplo confirmado (enviar 6 posiciones articulares, escaladas x1000):
```
TCP: 00 00 00 00 00 1F 01 10 55 8C 00 0C 18 <6 x 4 bytes de posición>
```
Después de escribir esto, según el manual: "seleccionar la referencia de
trayectoria (free path / posture line) en el controlador de mano para
ejecutar" — **este paso es el que hay que confirmar si es 100% remoto**.

## Tabla de "stack" (paletizado nativo) — ⚠️ confirmado volátil, no usar para persistencia

| Dato | Dirección |
|---|---|
| ID de stack a modificar | `20059 (0x4E5B)` |
| Intervalo X/Y/Z | `20060-20065 (0x4E5C-0x4E61)` |
| Cantidad X/Y/Z | `20066-20071 (0x4E62-0x4E67)` |

Útil solo para patrones regulares de ejecución en vivo (grilla), nunca para
guardar puntos permanentemente.

## Tabla de "data source" / puntos tipo visión — ⚠️ confirmado volátil

| Dato | Dirección |
|---|---|
| ID de fuente a consultar | `30000 (0x7530)` |
| Longitud de buffer | `30001 (0x7531)` |
| ID de fuente a modificar | `30002 (0x7532)` |
| Formato (4 o 6 datos/punto) | `30003 (0x7533)` |
| Máscara de ejes válidos | `30004 (0x7534)` |
| Limpiar datos pendientes | `30006 (0x7536)` (escribir 1) |
| Datos de posición (por eje) | `30007 (0x7537)` en adelante | 2 registros por eje, alta/baja |

## No confirmado / pendiente

- Selección de tool frame / user frame por Modbus.
- Blending / suavizado multipunto controlado externamente.
- Lectura o escritura del programa/módulo del pad en bloque.
- Comportamiento exacto de "Activar botón Start" (0x4E23) sobre posiciones
  mandadas manualmente vs. programa cargado.
