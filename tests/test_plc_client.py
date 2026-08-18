"""
Tests del cliente Modbus del PLC Coolmay CX3G (comms/plc_client.py).

No hay hardware ni red: se inyecta un PLC falso construido sobre
comms/fake_modbus.py, igual que el simulador del robot. Lo que se verifica es
la lógica del lado host — codificación de la tabla de 8 words, orden de los
words de 32 bits, handshake, troceado de transacciones y límites de las
tablas.

Ojo con lo que estos tests NO prueban: que el mapa de registros y la
convención de bytes sean los correctos contra un CX3G real. Eso es hipótesis
sin confirmar (ver el docstring de comms/plc_client.py). Un test verde acá
significa "el host hace lo que dijimos que iba a hacer", no "el PLC lo
entiende".
"""

import pytest

from comms.fake_modbus import FakeModbusClient
from comms.plc_client import (
    CONTRACT_VERSION,
    D_BYTECODE_BASE,
    D_COMMAND,
    D_CONTRACT_VERSION,
    D_INSTRUCTION_COUNT,
    D_POINT_COUNT,
    D_POINTS_BASE,
    D_PC,
    D_STATE,
    D_VAR_BASE,
    MAX_INSTRUCTIONS,
    MAX_POINTS,
    VAR_BANK_SIZE,
    WORDS_PER_INSTRUCTION,
    WORDS_PER_POINT,
    CoolmayModbusError,
    CoolmayPlcClient,
    PlcCommand,
    PlcVmState,
)
from comms.robot_client import Pose
from compiler.codegen import compile_source
from runtime.bytecode import Instruction, Program


class FakePlc(FakeModbusClient):
    """PLC falso.

    `auto_ack=True` imita a la VM acusando los comandos: devuelve D3 a 0 en
    cuanto el host escribe ahí. Con `auto_ack=False` se simula el caso más
    importante de todos — que del otro lado no haya nadie que entienda el
    contrato.
    """

    def __init__(self, auto_ack: bool = True) -> None:
        super().__init__()
        self.auto_ack = auto_ack
        self.writes: list[tuple[int, list[int]]] = []

    def _on_write(self, address: int, values: list[int]) -> None:
        self.writes.append((address, list(values)))
        if self.auto_ack and address == D_COMMAND:
            self.registers[D_COMMAND] = 0


def _plc(auto_ack: bool = True, **kw) -> tuple[CoolmayPlcClient, FakePlc]:
    fake = FakePlc(auto_ack=auto_ack)
    client = CoolmayPlcClient(host="fake", client=fake, **kw)
    client.connect()
    return client, fake


# --- estado -----------------------------------------------------------------


def test_read_status_decodes_the_control_block():
    client, fake = _plc()
    fake.registers.update({
        D_PC: 42,
        D_STATE: int(PlcVmState.RUNNING),
        2: 0,
        D_COMMAND: 0,
        D_INSTRUCTION_COUNT: 12,
        D_POINT_COUNT: 3,
        D_CONTRACT_VERSION: CONTRACT_VERSION,
    })
    status = client.read_status()
    assert status.pc == 42
    assert status.state == PlcVmState.RUNNING
    assert status.is_running is True
    assert status.instruction_count == 12
    assert status.point_count == 3
    assert status.contract_version == CONTRACT_VERSION


def test_read_status_is_a_single_transaction():
    """Si se leyera registro por registro, el PC y el estado podrían venir de
    momentos distintos y mostrar combinaciones imposibles."""
    client, fake = _plc()
    reads: list[tuple[int, int]] = []
    original = fake.read_holding_registers

    def spy(address, count=1, slave=1):
        reads.append((address, count))
        return original(address, count=count, slave=slave)

    fake.read_holding_registers = spy
    client.read_status()
    assert len(reads) == 1
    assert reads[0] == (D_PC, 8)


def test_read_status_rejects_unknown_state():
    client, fake = _plc()
    fake.registers[D_STATE] = 99
    with pytest.raises(CoolmayModbusError, match="Estado de VM desconocido"):
        client.read_status()


def test_error_name_for_known_and_unknown_codes():
    client, fake = _plc()
    fake.registers[2] = 4
    assert client.read_status().error_name == "WAIT_IN_TIMEOUT"
    fake.registers[2] = 77
    assert "DESCONOCIDO(77)" in client.read_status().error_name


def test_read_pc():
    client, fake = _plc()
    fake.registers[D_PC] = 7
    assert client.read_pc() == 7


# --- handshake de comandos ---------------------------------------------------


def test_send_command_writes_the_code_and_waits_for_ack():
    client, fake = _plc(auto_ack=True)
    client.start()
    assert (D_COMMAND, [int(PlcCommand.START)]) in fake.writes
    assert fake.registers[D_COMMAND] == 0  # la VM acusó


def test_all_commands_write_their_code():
    for method, code in [
        ("start", PlcCommand.START),
        ("stop", PlcCommand.STOP),
        ("pause", PlcCommand.PAUSE),
        ("resume", PlcCommand.RESUME),
        ("reset", PlcCommand.RESET),
    ]:
        client, fake = _plc(auto_ack=True)
        getattr(client, method)()
        assert (D_COMMAND, [int(code)]) in fake.writes


def test_send_command_fails_if_the_vm_never_acknowledges():
    """El caso que importa: no hay ladder del otro lado, o no implementa el
    handshake. Mejor fallar que seguir a ciegas."""
    client, _ = _plc(auto_ack=False)
    with pytest.raises(CoolmayModbusError, match="no acusó el comando START"):
        client.start(timeout_s=0.05, poll_interval_s=0.01)


def test_send_command_without_waiting_does_not_raise():
    client, fake = _plc(auto_ack=False)
    client.start(wait_ack=False)
    assert fake.registers[D_COMMAND] == int(PlcCommand.START)


# --- banco de variables ------------------------------------------------------


def test_write_and_read_variable():
    client, _ = _plc()
    client.write_variable(3, 1234)
    assert client.read_variable(3) == 1234


def test_variable_round_trip_with_negative_value():
    client, _ = _plc()
    client.write_variable(0, -5)
    assert client.read_variable(0) == -5


def test_variable_lands_in_the_right_register():
    client, fake = _plc()
    client.write_variable(7, 99)
    assert fake.registers[D_VAR_BASE + 7] == 99


def test_read_variables_reads_the_whole_bank():
    client, _ = _plc()
    client.write_variable(0, 10)
    client.write_variable(VAR_BANK_SIZE - 1, 20)
    values = client.read_variables()
    assert len(values) == VAR_BANK_SIZE
    assert values[0] == 10
    assert values[-1] == 20


def test_variable_index_out_of_bank_is_rejected():
    client, _ = _plc()
    with pytest.raises(ValueError):
        client.write_variable(VAR_BANK_SIZE, 1)
    with pytest.raises(ValueError):
        client.read_variable(-1)


# --- codificación de la tabla de 8 words -------------------------------------


def test_instruction_encodes_to_exactly_eight_words():
    words = CoolmayPlcClient.encode_instruction(Instruction("MOVEJ", a=1, b=2, c=3, d=4))
    assert len(words) == WORDS_PER_INSTRUCTION


def test_instruction_word_layout_matches_the_contract():
    """word 0 = opcode, word 1 = A, words 2-3 = B, words 4-5 = C, word 6 = D."""
    words = CoolmayPlcClient.encode_instruction(
        Instruction("SET_VAR", a=5, b=0x00010002, c=0x00030004, d=6)
    )
    assert words[0] == 0x0A                 # SET_VAR
    assert words[1] == 5                    # A
    assert words[2:4] == [0x0002, 0x0001]   # B, word bajo primero
    assert words[4:6] == [0x0004, 0x0003]   # C, word bajo primero
    assert words[6] == 6                    # D
    assert words[7] == 0                    # reservado


def test_int32_uses_low_word_first():
    """La convención FX del PLC, al revés que el robot Borunte. Es la hipótesis
    que más conviene validar primero contra hardware."""
    words = CoolmayPlcClient.encode_instruction(Instruction("JUMP", b=0x12345678))
    assert words[2] == 0x5678  # word bajo
    assert words[3] == 0x1234  # word alto


def test_instruction_round_trip():
    original = Instruction("JUMP_IF_VAR_NEQ_VAR", a=1, b=2, c=300, d=0)
    words = CoolmayPlcClient.encode_instruction(original)
    back = CoolmayPlcClient.decode_instruction(words)
    assert (back.opcode, back.a, back.b, back.c, back.d) == ("JUMP_IF_VAR_NEQ_VAR", 1, 2, 300, 0)


def test_instruction_round_trip_with_negative_operands():
    original = Instruction("SET_VAR", a=2, b=-1234, c=-1)
    back = CoolmayPlcClient.decode_instruction(
        CoolmayPlcClient.encode_instruction(original)
    )
    assert back.b == -1234
    assert back.c == -1


def test_all_contract_opcodes_round_trip():
    from runtime.bytecode import OPCODE_NUMBERS

    for name in OPCODE_NUMBERS:
        back = CoolmayPlcClient.decode_instruction(
            CoolmayPlcClient.encode_instruction(Instruction(name))
        )
        assert back.opcode == name


def test_encode_rejects_opcode_without_number():
    with pytest.raises(CoolmayModbusError, match="sin número asignado"):
        CoolmayPlcClient.encode_instruction(Instruction("OPCODE_INVENTADO"))


def test_decode_rejects_unknown_opcode_number():
    words = [0x77] + [0] * 7
    with pytest.raises(CoolmayModbusError, match="Opcode desconocido"):
        CoolmayPlcClient.decode_instruction(words)


def test_decode_rejects_wrong_word_count():
    with pytest.raises(CoolmayModbusError):
        CoolmayPlcClient.decode_instruction([0, 0, 0])


# --- tabla de puntos ---------------------------------------------------------


def test_point_encodes_to_twelve_words():
    words = CoolmayPlcClient.encode_point(Pose(1.0, 2.0, 3.0, 4.0, 5.0, 6.0))
    assert len(words) == WORDS_PER_POINT


def test_point_round_trip_keeps_three_decimals():
    pose = Pose(123.456, -78.9, 0.001, 0.0, 360.0, -180.0)
    back = CoolmayPlcClient.decode_point(CoolmayPlcClient.encode_point(pose))
    assert back.to_scaled_ints() == pose.to_scaled_ints()


# --- carga del programa ------------------------------------------------------


def _programa_ejemplo() -> Program:
    return compile_source(
        "POINT p = WORLD(10.0, 20.0, 30.0, 0.0, 0.0, 0.0)\n"
        "VAR n : INT = 2\n"
        "MOVEJ p SPEED 50\n"
        "IF n == 2 THEN\n"
        "SET_OUT(Y10, ON)\n"
        "ENDIF\n"
    )


def test_upload_then_read_back_returns_the_same_program():
    client, _ = _plc()
    program = _programa_ejemplo()
    client.upload_program(program)
    instructions, points = client.read_back_program()
    assert [i.opcode for i in instructions] == [i.opcode for i in program.instructions]
    assert [p.to_scaled_ints() for p in points] == [
        p.to_scaled_ints() for p in program.points
    ]


def test_verify_program_passes_after_a_clean_upload():
    client, _ = _plc()
    program = _programa_ejemplo()
    client.upload_program(program)
    client.verify_program(program)  # no levanta


def test_verify_program_detects_a_corrupted_word():
    client, fake = _plc()
    program = _programa_ejemplo()
    client.upload_program(program)
    fake.registers[D_BYTECODE_BASE + 1] = 0x4242  # ensucia el operando A de la instrucción 0
    with pytest.raises(CoolmayModbusError, match="volvió distinta"):
        client.verify_program(program)


def test_upload_writes_counters_after_the_tables():
    """Si el contador se escribiera primero, una carga interrumpida dejaría a
    la VM creyendo que hay más instrucciones válidas de las que se
    escribieron."""
    client, fake = _plc()
    client.upload_program(_programa_ejemplo())
    direcciones = [addr for addr, _ in fake.writes]
    ultimo_dato = max(
        i for i, addr in enumerate(direcciones)
        if addr >= D_BYTECODE_BASE
    )
    contadores = [
        i for i, addr in enumerate(direcciones)
        if addr in (D_INSTRUCTION_COUNT, D_POINT_COUNT)
    ]
    assert contadores, "no se escribieron los contadores"
    assert min(contadores) > ultimo_dato


def test_upload_reports_the_right_counts():
    client, fake = _plc()
    program = _programa_ejemplo()
    client.upload_program(program)
    assert fake.registers[D_INSTRUCTION_COUNT] == len(program.instructions)
    assert fake.registers[D_POINT_COUNT] == len(program.points)


def test_bytecode_and_points_land_at_their_bases():
    client, fake = _plc()
    program = _programa_ejemplo()
    client.upload_program(program)
    esperado = CoolmayPlcClient.encode_instruction(program.instructions[0])
    leido = [fake.registers.get(D_BYTECODE_BASE + i, 0) for i in range(WORDS_PER_INSTRUCTION)]
    assert leido == esperado
    esperado_punto = CoolmayPlcClient.encode_point(program.points[0])
    leido_punto = [fake.registers.get(D_POINTS_BASE + i, 0) for i in range(WORDS_PER_POINT)]
    assert leido_punto == esperado_punto


# --- troceado de transacciones ------------------------------------------------


def test_large_upload_is_split_into_chunks():
    """Modbus limita una escritura a 123 registros por trama; un programa
    entero son miles de words."""
    client, fake = _plc(max_registers_per_transaction=10)
    program = Program(instructions=[Instruction("NOP") for _ in range(20)])
    client.upload_program(program)
    escrituras_de_bytecode = [
        values for addr, values in fake.writes if addr >= D_BYTECODE_BASE
    ]
    assert all(len(v) <= 10 for v in escrituras_de_bytecode)
    assert len(escrituras_de_bytecode) == 16  # 20 instr * 8 words / 10


def test_chunked_upload_still_round_trips():
    client, _ = _plc(max_registers_per_transaction=7)
    program = Program(
        instructions=[Instruction("SET_VAR", a=i, b=i * 100) for i in range(25)],
        points=[Pose(float(i), 0.0, 0.0, 0.0, 0.0, 0.0) for i in range(9)],
    )
    client.upload_program(program)
    client.verify_program(program)  # el troceado no perdió ni corrió nada


def test_chunked_read_reassembles_in_order():
    client, _ = _plc(max_registers_per_transaction=3)
    program = Program(instructions=[Instruction("SET_VAR", a=i) for i in range(10)])
    client.upload_program(program)
    instructions, _ = client.read_back_program()
    assert [i.a for i in instructions] == list(range(10))


# --- límites de las tablas ---------------------------------------------------


def test_exactly_max_instructions_fits():
    client, _ = _plc()
    program = Program(instructions=[Instruction("NOP") for _ in range(MAX_INSTRUCTIONS)])
    client.upload_program(program)  # no levanta


def test_one_instruction_over_the_limit_is_rejected():
    client, _ = _plc()
    program = Program(instructions=[Instruction("NOP") for _ in range(MAX_INSTRUCTIONS + 1)])
    with pytest.raises(CoolmayModbusError, match="instrucciones"):
        client.upload_program(program)


def test_exactly_max_points_fits():
    client, _ = _plc()
    program = Program(points=[Pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0) for _ in range(MAX_POINTS)])
    client.upload_program(program)  # no levanta


def test_one_point_over_the_limit_is_rejected():
    client, _ = _plc()
    program = Program(points=[Pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0) for _ in range(MAX_POINTS + 1)])
    with pytest.raises(CoolmayModbusError, match="puntos"):
        client.upload_program(program)


def test_tables_do_not_overlap_at_the_limit():
    """375 instrucciones * 8 words tienen que terminar justo antes de D4000,
    donde arranca la tabla de puntos."""
    assert D_BYTECODE_BASE + MAX_INSTRUCTIONS * WORDS_PER_INSTRUCTION == D_POINTS_BASE
    assert D_POINTS_BASE + MAX_POINTS * WORDS_PER_POINT <= 8000


# --- offset de los registros D ------------------------------------------------


def test_d_register_base_offsets_every_address():
    """Si el CX3G expone los D corridos, se corrige en un solo lugar."""
    fake = FakePlc()
    client = CoolmayPlcClient(host="fake", client=fake, d_register_base=5000)
    client.connect()
    client.write_variable(0, 77)
    assert fake.registers[5000 + D_VAR_BASE] == 77
    assert (5000 + D_VAR_BASE) in fake.registers
    assert D_VAR_BASE not in fake.registers


def test_d_register_base_does_not_break_upload_round_trip():
    fake = FakePlc()
    client = CoolmayPlcClient(host="fake", client=fake, d_register_base=3)
    client.connect()
    program = _programa_ejemplo()
    client.upload_program(program)
    client.verify_program(program)


# --- el contrato y el codigo no pueden divergir ------------------------------


def test_opcode_numbers_match_the_contract_document():
    """OPCODE_NUMBERS existe en codigo Y en docs/INSTRUCTION_SET.md. Si alguien
    agrega un opcode en uno solo de los dos lados, plc_vm/ queda implementando
    otra cosa que el compiler."""
    import re
    from pathlib import Path

    from runtime.bytecode import OPCODE_NUMBERS

    doc = Path(__file__).resolve().parents[1] / "docs" / "INSTRUCTION_SET.md"
    texto = doc.read_text(encoding="utf-8")

    # filas de la tabla de opcodes: | `0x0A` | `SET_VAR` | ...
    del_doc = {
        nombre: int(num, 16)
        for num, nombre in re.findall(
            r"^\|\s*`(0x[0-9A-Fa-f]{2})`\s*\|\s*`([A-Z_]+)`\s*\|", texto, re.MULTILINE
        )
    }

    assert del_doc, "no se pudo parsear la tabla de opcodes del contrato"
    assert del_doc == OPCODE_NUMBERS, (
        "la tabla de docs/INSTRUCTION_SET.md y OPCODE_NUMBERS de "
        "runtime/bytecode.py no coinciden.\n"
        f"solo en el documento: {set(del_doc) - set(OPCODE_NUMBERS)}\n"
        f"solo en el codigo:    {set(OPCODE_NUMBERS) - set(del_doc)}\n"
        f"numeros distintos:    "
        f"{ {k: (del_doc[k], OPCODE_NUMBERS[k]) for k in set(del_doc) & set(OPCODE_NUMBERS) if del_doc[k] != OPCODE_NUMBERS[k]} }"
    )
