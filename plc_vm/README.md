# plc_vm — NO es código Python

Esta carpeta documenta el diseño de la VM que corre en el CX3G (ladder/IL),
pero la implementación real se hace en GX Developer / Works2, fuera de este
repo, porque ese software no tiene un formato de proyecto abierto/versionable
de forma simple con git en texto plano.

Lo que sí puede vivir acá:
- Pseudocódigo del bucle principal (fetch-decode-execute)
- Mapeo de opcodes de docs/INSTRUCTION_SET.md a instrucciones FNC del CX3G
- Notas de cada iteración probada contra el robot real

Ver docs/ARCHITECTURE.md → roadmap → fase 2 (PoC) antes de tocar esto.
