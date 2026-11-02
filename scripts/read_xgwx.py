#!/usr/bin/env python3
"""XG5000 XGI 프로젝트(.xgwx)를 XG5000 없이 읽어서 레더와 변수표를 출력한다. 파일은 수정하지 않는다.

사용법: python3 read_xgwx.py <파일.xgwx> [--raw 추출폴더]
"""
import argparse
import base64
import bz2
import re
import struct
import sys
import zlib

STRING_MARK = b"\xff\xfe\xff"
ELEMENT_NAMES = {1: "wire", 2: "wire", 6: "NO", 7: "NC", 0xE: "COIL", 0x3F: "COMMENT", 0x46: "CONST"}


class Reader:
    def __init__(self, data):
        self.data, self.pos = data, 0

    def u8(self):
        value = self.data[self.pos]
        self.pos += 1
        return value

    def u16(self):
        value = struct.unpack_from("<H", self.data, self.pos)[0]
        self.pos += 2
        return value

    def u32(self):
        value = struct.unpack_from("<I", self.data, self.pos)[0]
        self.pos += 4
        return value

    def is_string(self):
        return self.data[self.pos:self.pos + 3] == STRING_MARK

    def string(self):
        assert self.is_string(), f"문자열 위치가 아님 @{self.pos:#x}"
        self.pos += 3
        length = self.u8()
        if length == 0xFF:
            length = self.u16()
        value = self.data[self.pos:self.pos + 2 * length].decode("utf-16le")
        self.pos += 2 * length
        return value


def split_position(value):
    return value >> 10, value & 1023


def read_function_block(reader, element):
    element["pos"] = split_position(reader.u32())
    reader.u16()
    reader.u32()
    element["function_id"] = reader.u32()
    reader.u32()
    reader.u8()
    element["rows"] = []
    for _ in range(reader.u16()):
        row_pos = split_position(reader.u32())
        has_names = reader.u16()
        reader.u32()
        pins = []
        for _ in range(reader.u16()):
            argument = reader.string()
            reader.u32()
            pin_name = reader.string()
            reader.u32()
            reader.u8()
            pins.append((pin_name, argument))
        names = [reader.string(), reader.string()] if has_names else []
        element["rows"].append({"pos": row_pos, "pins": pins, "names": names})


def read_element(reader):
    element = {"cls": reader.u8(), "type": reader.u32()}
    cls, kind = element["cls"], element["type"]
    if cls == 0 and kind == 0:
        element["vertical"] = []
        for _ in range(reader.u16()):
            element["vertical"].append(split_position(reader.u32()))
            reader.u16()
            reader.u32()
    elif cls in (1, 2):
        element["pos"] = split_position(reader.u32())
    elif kind == 0x67:
        read_function_block(reader, element)
    elif cls in (0, 0xFF):
        element["pos"] = split_position(reader.u32())
        reader.u16()
        reader.u32()
        element["text"] = []
        while reader.is_string():
            element["text"].append(reader.string())
        if kind == 2:
            element["end"] = split_position(reader.u32())
        if kind == 0x3F:
            reader.u32()
            reader.u32()
    else:
        raise ValueError(f"미확인 요소 cls={cls} type={kind:#x} @{reader.pos:#x}")
    return element


def parse_program(data):
    reader = Reader(data)
    reader.u32()
    header = reader.u32()
    networks = []
    while reader.pos < len(data):
        number, flag, row_count = reader.u32(), reader.u32(), reader.u16()
        network = {"number": number, "flag": flag, "rows": []}
        for _ in range(row_count):
            row = reader.u32()
            assert reader.u8() == 0xFF and reader.u32() == 0x43, f"행 표시 오류 @{reader.pos:#x}"
            for _ in range(6):
                reader.u32()
            elements = [read_element(reader) for _ in range(reader.u16())]
            network["rows"].append({"row": row, "elements": elements})
        networks.append(network)
    assert reader.pos == len(data), "남은 바이트가 있음"
    assert header >> 16 == len(networks), "네트워크 수 불일치"
    return networks


def list_symbol_strings(data):
    """변수표는 XG5000이 저장할 때 형식이 달라질 수 있어, 문자열 단위로 읽어 이름과 주소를 묶는다."""
    strings, position = [], 0
    while True:
        found = data.find(STRING_MARK, position)
        if found < 0:
            break
        length = data[found + 3]
        strings.append(data[found + 4:found + 4 + 2 * length].decode("utf-16le", "replace"))
        position = found + 4 + 2 * length
    records, current = [], None
    for index, text in enumerate(strings):
        if text == "PB50":
            current = {"name": strings[index + 1] if index + 1 < len(strings) else "", "address": "", "type": "BOOL"}
            records.append(current)
        elif current is not None and text.startswith("%"):
            current["address"] = text
        elif current is not None and text in ("TON", "TOF", "TP", "CTU", "CTD", "CTUD"):
            current["type"] = text
    return records


def decode_field(match):
    raw = base64.b64decode(re.sub(r"\s|&#xA;", "", match.group(2)))
    return bz2.decompress(raw) if 'Compressed="1"' in match.group(1) else raw


def describe_network(network):
    lines = []
    for row in network["rows"]:
        parts = []
        for element in row["elements"]:
            kind = element["type"]
            if "vertical" in element:
                parts.append(f"│down@{element['vertical'][0][1]}")
            elif element["cls"] == 1 and kind == 0:
                parts.append(f"│from@{element['pos'][1]}")
            elif kind == 0x67:
                first = element["rows"][0]
                parts.append(f"[{first['names'][0]} {first['names'][1]}]")
            elif kind in (6, 7, 0xE, 0x3F, 0x46):
                parts.append(f"{ELEMENT_NAMES[kind]}:{element['text'][0]}@{element['pos'][1]}")
        lines.append(f"  행{row['row']:>4}: " + " ".join(parts))
    return lines


def main():
    options = argparse.ArgumentParser(description=__doc__)
    options.add_argument("path")
    options.add_argument("--raw", help="ProgramData와 Symbols 원본을 저장할 폴더")
    args = options.parse_args()
    data = open(args.path, "rb").read()
    xml = zlib.decompressobj(31).decompress(data[data.find(b"\x1f\x8b"):]).decode("utf-8")
    for base, slot, name in re.findall(r'<Module Base="(\d+)" Slot="(\d+)"[^>]*?Name="([^"]+)"', xml):
        print(f"베이스{base} 슬롯{slot}: {name}")
    program = re.search(r"<ProgramData\b([^>]*)>([^<]*)<", xml)
    local = xml[xml.index("<LocalVar"):]
    symbols_match = re.search(r"<Symbols\b([^>]*)>([^<]*)<", local)
    program_data, symbol_data = decode_field(program), decode_field(symbols_match)
    if args.raw:
        open(f"{args.raw}/ProgramData.bin", "wb").write(program_data)
        open(f"{args.raw}/Symbols.bin", "wb").write(symbol_data)
    networks = parse_program(program_data)
    print(f"네트워크 {len(networks)}개, 레더 {len(program_data)} bytes 전부 해석")
    for network in networks:
        print(f"네트워크 {network['number']}" + (f" (플래그 {network['flag']})" if network["flag"] else ""))
        print("\n".join(describe_network(network)))
    print("변수:")
    for record in list_symbol_strings(symbol_data):
        print(f"  {record['name']}\t{record['type']}\t{record['address']}")


if __name__ == "__main__":
    sys.exit(main())
