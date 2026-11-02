#!/usr/bin/env python3
"""파이썬으로 적은 레더 목록을 XG5000 XGI 프로젝트(.xgwx)로 만든다.

CPU, 모듈 구성, 프로젝트 설정은 템플릿 파일의 것을 그대로 쓰고 레더와 로컬 변수표만 교체한다.
템플릿과 입력은 수정하지 않으며, 이미 있는 출력 파일은 덮어쓰지 않는다.

사용법: python3 write_xgwx.py <프로그램.py> --template <템플릿.xgwx> -o <출력.xgwx> [--raw 추출폴더]

프로그램 파일에는 PROGRAM, INPUT_ADDRESSES, OUTPUT_ADDRESSES가 있어야 한다.
형식은 examples/motor_start_stop.py를 본다.
"""
import argparse
import base64
import bz2
import importlib.util
import os
import re
import struct
import sys
import zlib

TOKEN_PATTERN = re.compile(r"\s*([()!+*]|[^\s()!+*]+)")


def parse_expression(text):
    tokens = TOKEN_PATTERN.findall(text)
    index = [0]

    def peek():
        return tokens[index[0]] if index[0] < len(tokens) else None

    def take():
        index[0] += 1
        return tokens[index[0] - 1]

    def parse_parallel():
        branches = [parse_series()]
        while peek() == "+":
            take()
            branches.append(parse_series())
        return branches[0] if len(branches) == 1 else ("or", branches)

    def parse_series():
        items = [parse_contact()]
        while peek() == "*":
            take()
            items.append(parse_contact())
        return items[0] if len(items) == 1 else ("and", items)

    def parse_contact():
        token = take()
        if token == "(":
            inner = parse_parallel()
            assert take() == ")"
            return inner
        if token == "!":
            return ("nc", take())
        return ("no", token)

    tree = parse_parallel()
    assert index[0] == len(tokens), text
    return tree

COIL_COLUMN = 94
LAST_WIRE_COLUMN = 91
TON_FUNCTION_ID = 0x5121
NORMAL_ROW_HEIGHT, TALL_ROW_HEIGHT, TIMER_ROW_HEIGHT = 0x27, 0x32, 0x34
LONG_NAME_LENGTH = 7


def u8(value):
    return struct.pack("<B", value)


def u16(value):
    return struct.pack("<H", value)


def u32(value):
    return struct.pack("<I", value)


def text(value):
    encoded = value.encode("utf-16le")
    return b"\xff\xfe\xff" + u8(len(value)) + encoded


def position(row, column):
    return u32((row << 10) | column)


def cell_column(cell_index):
    return 1 + 3 * cell_index


def boundary_column(cell_index):
    return 3 * cell_index


def tree_size(node):
    kind = node[0]
    if kind in ("no", "nc"):
        return 1, 1
    sizes = [tree_size(child) for child in node[1]]
    if kind == "and":
        return sum(w for w, _ in sizes), max(h for _, h in sizes)
    return max(w for w, _ in sizes), sum(h for _, h in sizes)


class RungGrid:
    def __init__(self):
        self.contacts = {}          # (행, 셀) -> (no|nc, 이름)
        self.wires = set()          # (행, 셀)
        self.verticals = set()      # (행, 경계셀): 이 행에서 다음 행으로 내려가는 수직선

    def place(self, node, row, cell):
        kind = node[0]
        if kind in ("no", "nc"):
            self.contacts[(row, cell)] = (kind, node[1])
        elif kind == "and":
            for child in node[1]:
                self.place(child, row, cell)
                cell += tree_size(child)[0]
        else:
            width = tree_size(node)[0]
            branch_row = row
            first_rows = []
            for child in node[1]:
                child_width, child_height = tree_size(child)
                self.place(child, branch_row, cell)
                for filler in range(cell + child_width, cell + width):
                    self.wires.add((branch_row, filler))
                first_rows.append(branch_row)
                branch_row += child_height
            for vertical_row in range(first_rows[0], first_rows[-1]):
                if cell > 0:
                    self.verticals.add((vertical_row, cell))
                self.verticals.add((vertical_row, cell + width))


def wire_runs(cells, branch_boundaries):
    # 수평선은 수직 분기점에서 끊어서 저장해야 XG5000이 연결로 인식한다.
    runs = []
    for cell in sorted(cells):
        if runs and runs[-1][1] == cell - 1 and cell not in branch_boundaries:
            runs[-1][1] = cell
        else:
            runs.append([cell, cell])
    return runs


def encode_row(row, elements, is_comment, height, previous_row_end):
    row_end = (row << 10) | COIL_COLUMN
    elements.sort(key=lambda item: item[0])
    last_position = (row << 10) | (elements[-1][0] // 2 if elements else 0)
    header = b"\xff" + u32(0x43) + u32(0) + u32(1 if is_comment else 0) + u32(height)
    header += u32(row_end) + u32(previous_row_end) + u32(last_position) + u16(len(elements))
    return header + b"".join(item[1] for item in elements), row_end


def contact_element(kind, row, column, name):
    record_type = 6 if kind == "no" else 7
    return column * 2, b"\xff" + u32(record_type) + position(row, column) + u16(1) + u32(0) + text(name)


def wire_element(row, first_column, last_column):
    return first_column * 2, b"\xff" + u32(2) + position(row, first_column) + u16(0) + u32(0) + position(row, last_column)


def vertical_from_above(row_above, column):
    return column * 2 - 1, b"\x01" + u32(0) + position(row_above, column)


def vertical_down(row, column):
    return column * 2, (b"\x00" + u32(0) + u16(2) + position(row, column) + u16(0) + u32(0)
                        + position(row + 1, column - 1) + u16(0) + u32(0))


def coil_element(row, name):
    return COIL_COLUMN * 2, b"\xff" + u32(0xE) + position(row, COIL_COLUMN) + u16(1) + u32(0x20) + text(name)


def timer_preset_text(milliseconds):
    return f"t#{milliseconds // 1000}s" if milliseconds % 1000 == 0 else f"t#{milliseconds}ms"


def timer_element(row, column, instance):
    pin_in = text("") + u32(0x200001) + text("IN") + u32(1) + u8(0)
    pin_q = text("") + u32(1) + text("Q") + u32(3) + u8(0)
    pin_pt = text("") + u32(0x208000) + text("PT") + u32(1) + u8(0)
    pin_et = text("") + u32(0x8000) + text("ET") + u32(3) + u8(0)
    data = b"\x00" + u32(0x67) + position(row, column) + u16(1) + u32(0x200) + u32(TON_FUNCTION_ID) + u32(0x200) + u8(0) + u16(3)
    data += position(row, column) + u16(1) + u32(0) + u16(2) + pin_in + pin_q + text("TON") + text(instance)
    data += position(row + 1, column) + u16(0) + u32(0) + u16(2) + pin_pt + pin_et
    data += position(row + 2, column) + u16(0) + u32(0) + u16(0)
    return column * 2, data


def encode_grid_rows(grid, first_row, row_count, extra_elements, height_of_row):
    encoded = []
    previous_end = (first_row << 10) | COIL_COLUMN
    for row in range(first_row, first_row + row_count):
        elements = list(extra_elements.get(row, []))
        for (contact_row, cell), (kind, name) in grid.contacts.items():
            if contact_row == row:
                elements.append(contact_element(kind, row, cell_column(cell), name))
        branch_boundaries = {c for r, c in grid.verticals if r in (row, row - 1)}
        for first_cell, last_cell in wire_runs([c for r, c in grid.wires if r == row], branch_boundaries):
            elements.append(wire_element(row, cell_column(first_cell), cell_column(last_cell)))
        for vertical_row, cell in grid.verticals:
            if vertical_row == row:
                elements.append(vertical_down(row, boundary_column(cell)))
            if vertical_row == row - 1:
                elements.append(vertical_from_above(row - 1, boundary_column(cell)))
        data, previous_end = encode_row(row, elements, False, height_of_row(row, grid), previous_end)
        encoded.append(data)
    return encoded


def build_program_data(ladder_program):
    networks = []
    row = 0

    def row_height(current_row, grid):
        names = [name for (r, _), (_, name) in grid.contacts.items() if r == current_row]
        return TALL_ROW_HEIGHT if any(len(name) >= LONG_NAME_LENGTH for name in names) else NORMAL_ROW_HEIGHT

    for title, rungs in ladder_program.PROGRAM:
        comment = (b"\xff" + u32(0x3F) + position(row, 1) + u16(0) + u32(0x20) + text(title) + u32(13) + u32(13))
        data, _ = encode_row(row, [(2, comment)], True, NORMAL_ROW_HEIGHT, (row << 10) | COIL_COLUMN)
        networks.append((row, [data]))
        row += 1
        for rung in rungs:
            tree = parse_expression(rung[2])
            width, height = tree_size(tree)
            grid = RungGrid()
            grid.place(tree, row, 0)
            extra = {}
            if rung[0] == "coil":
                for filler in range(width, (LAST_WIRE_COLUMN - 1) // 3 + 1):
                    grid.wires.add((row, filler))
                extra.setdefault(row, []).append(coil_element(row, rung[1]))
                row_count = height
                height_of = row_height
            else:
                assert height == 1, f"타이머 입력은 직렬 접점만: {rung[1]}"
                column = cell_column(width)
                row_count = 3
                extra[row] = [timer_element(row, column, rung[1])]
                preset = b"\xff" + u32(0x46) + position(row + 1, column - 3) + u16(0) + u32(0) + text(timer_preset_text(rung[3]))
                extra[row + 1] = [((column - 3) * 2, preset), (column * 2, b"\x01" + u32(0x68) + position(row, column))]
                extra[row + 2] = [(column * 2, b"\x02" + u32(0x69) + position(row, column))]

                def height_of(current_row, current_grid, timer_row=row):
                    return TIMER_ROW_HEIGHT if current_row == timer_row else NORMAL_ROW_HEIGHT
            networks.append((row, encode_grid_rows(grid, row, row_count, extra, height_of)))
            row += row_count

    blob = u32(0) + u32((len(networks) << 16) | row)
    for index, (first_row, rows) in enumerate(networks):
        blob += u32(index) + u32(0) + u16(len(rows))
        for offset, data in enumerate(rows):
            blob += u32(first_row + offset) + data
    return blob


def address_bit_index(address):
    match = re.fullmatch(r"%([IQ])X(\d+)\.(\d+)\.(\d+)", address)
    if match:
        return int(match[2]) * 64 * 12 + int(match[3]) * 64 + int(match[4])
    return int(re.fullmatch(r"%MX(\d+)", address)[1])


SYMBOL_SCOPE = text("") + text("[0:0];[1:0]")


def bool_symbol(name, address=None):
    head = text("PB50") + text(name) + u32(1) + u32(1) + u32(0)
    if address is None:
        middle = text("") + text("") + text("") + text("") + u32(0xFFFFFFFF) + u32(0) + u32(0xFFFFFFFF)
    else:
        middle = text(address) + text("") + text("") + text(address[1]) + u32(address_bit_index(address)) + u32(1) + u32(0xFFFFFFFF)
    return head + middle + b"\x00" * 24 + SYMBOL_SCOPE


def timer_symbol(name):
    return (text("PB50") + text(name) + u32(1) + u32(0x18) + text("TON") + u32(0) + text("") + text("") + text("") + text("")
            + u32(0xFFFFFFFF) + u32(0) + u32(0xFFFFFFFF) + b"\x00" * 24 + SYMBOL_SCOPE)


def build_symbols(ladder_program):
    addresses = {**ladder_program.INPUT_ADDRESSES, **ladder_program.OUTPUT_ADDRESSES}
    records = {name: bool_symbol(name, address) for name, address in addresses.items()}
    for _, rungs in ladder_program.PROGRAM:
        for rung in rungs:
            if rung[0] == "ton":
                records[rung[1]] = timer_symbol(rung[1])
            elif rung[1] not in records:
                records[rung[1]] = bool_symbol(rung[1])
    names = sorted(records)
    return b"".join(records[name] for name in names), len(names)


def compress_field(data):
    encoded = base64.b64encode(bz2.compress(data)).decode()
    return "\n".join(encoded[i:i + 76] for i in range(0, len(encoded), 76))


def replace_field(xml, opening_pattern, data, count=None):
    match = re.search(opening_pattern + r"[^<]*</", xml)
    assert match, opening_pattern
    tag_end = xml.index(">", match.start()) + 1
    opening = xml[match.start():tag_end]
    if count is not None:
        opening = re.sub(r'Count="\d+"', f'Count="{count}"', opening)
    closing = xml.index("</", tag_end)
    return xml[:match.start()] + opening + compress_field(data) + xml[closing:]


def build_container(xml_text, template):
    gzip_start = template.find(b"\x1f\x8b")
    old_length = struct.unpack("<I", template[gzip_start - 4:gzip_start])[0]
    tail = template[gzip_start + old_length:]
    compressor = zlib.compressobj(9, zlib.DEFLATED, 31)
    packed = compressor.compress(xml_text.encode("utf-8")) + compressor.flush()
    packed += b"\x00" * (-len(packed) % 4)
    header = bytearray(template[:gzip_start])
    checksum_offset = template.find(b"\xff\xfe\xff\x00\xff\xfe\xff\x00") + 8
    header[checksum_offset:checksum_offset + 4] = u32(sum(packed) + len(packed))
    header[-4:] = u32(len(packed))
    return bytes(header) + packed + tail


def load_program(path):
    spec = importlib.util.spec_from_file_location("ladder_program", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def main():
    options = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    options.add_argument("program", help="레더 목록을 적은 파이썬 파일")
    options.add_argument("--template", required=True, help="틀로 쓸 기존 .xgwx 파일")
    options.add_argument("-o", "--output", required=True, help="만들 .xgwx 파일")
    options.add_argument("--raw", help="ProgramData와 Symbols 원본을 저장할 폴더")
    args = options.parse_args()
    if os.path.exists(args.output):
        sys.exit(f"출력 파일이 이미 있음: {args.output}")
    ladder_program = load_program(args.program)
    template = open(args.template, "rb").read()
    gzip_start = template.find(b"\x1f\x8b")
    xml = zlib.decompressobj(31).decompress(template[gzip_start:]).decode("utf-8")
    program_data = build_program_data(ladder_program)
    symbols, symbol_count = build_symbols(ladder_program)
    xml = replace_field(xml, r"<ProgramData\b", program_data)
    local_var = xml.index("<LocalVar")
    xml = xml[:local_var] + replace_field(xml[local_var:], r"<Symbols\b", symbols, symbol_count)
    output = build_container(xml, template)
    open(args.output, "wb").write(output)
    if args.raw:
        open(f"{args.raw}/ProgramData.bin", "wb").write(program_data)
        open(f"{args.raw}/Symbols.bin", "wb").write(symbols)
    print(args.output, len(output), "bytes, 변수", symbol_count, "개, 레더", len(program_data), "bytes")


if __name__ == "__main__":
    main()
