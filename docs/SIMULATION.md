# Desarrollo sin hardware — estrategia de simulación

No tener el robot a mano no bloquea el desarrollo del `compiler/`
ni del `runtime/` (VM de referencia) ni de la `gui/`. Lo que sí bloquea es
la validación final de los supuestos sobre el comportamiento real del
Borunte — eso se pospone, no se evita.

## Las dos piezas de simulación

### `comms/fake_modbus.py` — `FakeModbusClient`
Reemplazo en memoria de `pymodbus.client.ModbusTcpClient`. Implementa el
mismo subconjunto de métodos que usa `BorunteRobotClient`
(`read_holding_registers`, `write_register`, `write_registers`,
`connect`, `close`), guardando todo en un dict. No abre sockets.

### `comms/robot_simulator.py` — `SimulatedBorunteRobot`
Hereda de `FakeModbusClient` y le agrega comportamiento: cuando se escribe
el comando de "activar botón start" (`0x4E23`), lee la pose pendiente del
bloque 800-890 y simula el movimiento (interpola la posición durante
`move_duration_s`, manteniendo `0x09A6=1` mientras tanto).

**Esto es una hipótesis, no el comportamiento confirmado.** Está declarado
así explícitamente en el docstring del archivo. Los tres supuestos que
codifica:

1. Pose en 800-890 + comando `0x4E23` → dispara movimiento hacia esa pose
2. `0x09A6` refleja el estado del movimiento en curso
3. `0x4E20` frena en seco

Si al probar contra hardware real alguno de estos supuestos resulta
incorrecto, el arreglo es **local a este archivo** — el compiler, la VM de
referencia y la GUI no deberían necesitar cambios, porque todos hablan con
el robot exclusivamente a través de la interfaz de `BorunteRobotClient`
(nunca directamente contra registros Modbus).

## Cómo usar el simulador en tus propios tests o scripts

```python
from comms.robot_client import BorunteRobotClient
from comms.robot_simulator import SimulatedBorunteRobot

sim = SimulatedBorunteRobot(move_duration_s=0.3)
robot = BorunteRobotClient(host="fake", client=sim)
robot.connect()

robot.send_target_pose(...)
robot.cmd_start_button()
robot.wait_until_stopped()
print(robot.read_world_position())
```

Cuando llegue el hardware, el único cambio para pasar a producción es:

```python
robot = BorunteRobotClient(host="192.168.1.10")  # sin `client=`, usa Modbus TCP real
```

Todo el resto del código (VM, compiler, GUI) queda igual.

## Qué NO cubre esta simulación

- Timing real de red / latencia de transacciones Modbus (el simulador es
  instantáneo salvo por el `move_duration_s` artificial).
- Comportamiento de alarmas reales del robot.
- Si `0x4E23` en verdad requiere alguna selección previa en el pad (la
  duda abierta de siempre — ver `docs/MODBUS_REGISTER_MAP.md`).
- El controlador del Borunte ejecutando el respaldo del pad. `runtime/vm.py`
  simula la *lógica* del programa, no cómo la interpreta el pad: por ejemplo,
  en la VM un `IF X010 == 1` lee una variable, no la entrada, y las E/S se
  numeran en decimal y no en octal como en el pad.

## Próximo hito cuando llegue el hardware

1. **Importar en el pad un respaldo reescrito sin cambios** por
   `pad/backup.py` (prueba que el pad acepta lo que generamos), y después uno
   generado por `compiler/pad_codegen.py`, a velocidad baja. Ver
   `docs/PAD_FORMAT.md`, "Para confirmar".
2. Correr `comms/robot_client.py` manualmente contra el robot real, un
   método a la vez, confirmando o corrigiendo cada supuesto (lo usan la
   digitalización de puntos y la simulación contra el robot real).
3. Actualizar `comms/robot_simulator.py` para que coincida con la realidad.
