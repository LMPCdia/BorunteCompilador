# Arquitectura — Lenguaje tipo KRL para Borunte vía Coolmay CX3G

## Resumen

Este proyecto NO reemplaza el programa nativo del robot (el que se enseña en el
pad). Construye una **capa externa de orquestación** que corre en el PLC,
controlando al Borunte como un ejecutor de posiciones vía Modbus.

```
┌─────────────────────────────┐
│   PC — Host                  │
│                              │
│  DSL (.krlb)                 │
│    │  parser (Lark)          │
│    ▼                         │
│  AST                         │
│    │  codegen                │
│    ▼                         │
│  Bytecode + tabla de puntos  │──── Modbus TCP (escritura D/R) ────┐
└─────────────────────────────┘                                     │
                                                                       ▼
                                                         ┌─────────────────────┐
                                                         │  Coolmay CX3G        │
                                                         │  (VM en ladder/IL)   │
                                                         │  - Program Counter   │
                                                         │  - Dispatch opcode   │
                                                         │  - IF/WHILE/timers   │
                                                         │    nativos del PLC   │
                                                         └─────────────────────┘
                                                                       │
                                                          Modbus TCP/RTU (maestro)
                                                                       ▼
                                                         ┌─────────────────────┐
                                                         │  Borunte (esclavo)   │
                                                         │  - mover a posición  │
                                                         │  - forzar E/S        │
                                                         │  - reportar estado   │
                                                         └─────────────────────┘
```

## Los dos niveles (no confundir)

### Nivel Host (esto es lo que construimos)
- Vive en el PLC (ejecución) + PC (edición/compilación).
- Tiene: variables tipadas, IF/WHILE/FOR, timers, subrutinas, puntos como datos,
  offsets calculados.
- Es Turing-completo dentro de los límites del PLC (sin recursión, sin memoria
  dinámica, sin objetos — más cerca de KRL/Structured Text que de Python real).

### Nivel Pad (nativo, cerrado)
- Programa enseñado a mano en el teach pendant, vive en la EEPROM del Borunte.
- Por Modbus se puede: iniciar/pausar/detener, limpiar alarma, leer posición
  actual en vivo. NO se puede leer ni escribir el programa en bloque.
- Los puntos se "digitalizan" uno por uno: se lleva el robot a la posición
  (jog o stepping del programa) y se lee `0x091C` (mundo) o `0x08DC` (ejes).

## Decisiones de diseño ya tomadas

1. El bytecode vive en registros D/R del PLC, en registros consecutivos de
   ancho fijo (ver `INSTRUCTION_SET.md`).
2. Los puntos son una tabla separada del bytecode, referenciados por índice.
3. Persistencia de puntos: SOLO en la PC (SQLite/JSON). La tabla "stack /
   data source" del Borunte es volátil — confirmado — no se usa para guardar
   nada a largo plazo, solo como buffer de ejecución si hiciera falta.
4. Todo movimiento se ejecuta de a uno (comando → poll de `0x09A6` hasta que
   vuelva a 0). No hay blending nativo multipunto controlado desde afuera —
   pendiente de confirmar con Borunte si existe alguna vía adicional.

## Pendiente de validar con hardware real / con Borunte

- [ ] Latencia real por transacción Modbus (define el tiempo de ciclo mínimo
      por punto).
- [ ] Si existe selección de tool/frame por Modbus (no confirmado en manuales
      indexados).
- [ ] Si Borunte tiene software de programación offline / formato de archivo
      de programa (preguntar directamente — cambiaría el alcance).
- [ ] Comportamiento exacto del disparador de "free path" / "posture line"
      cuando se manda una posición manual (¿requiere selección previa en el
      pad o es 100% remoto?).

## Roadmap

1. **Congelar el contrato** (`INSTRUCTION_SET.md` + `MODBUS_REGISTER_MAP.md`) — HOY
2. **PoC de la VM**: 3-4 opcodes a mano en el CX3G, probado contra el robot real
3. **`comms/`**: cliente Modbus robusto para robot + PLC
4. **`compiler/`**: gramática, parser, codegen contra el bytecode congelado
5. **`gui/`**: editor + tabla de puntos + panel de conexión + botón de deploy
6. Iterar
