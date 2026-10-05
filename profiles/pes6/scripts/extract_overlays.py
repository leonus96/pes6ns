#!/usr/bin/env python3
"""Extracts the code overlays PES6 loads at runtime and recompiles each one.

PES6 keeps ~36 overlays (title, game, edit, master league...) in
PSP_GAME/USRDIR/over.afs. Every overlay starts on a 0x800-aligned offset with a
64-byte "MWo3" header:

    +0x00 'MWo3'        +0x04 overlay id
    +0x08 load address  +0x0C text size (from +0x40)
    +0x10 data size     +0x14 bss size
    +0x18 image end     +0x1C image end (again)
    +0x20 name[32]

The image (header + text + data) is loaded verbatim at the load address, so
code addresses in the file map 1:1 to guest addresses.

For each overlay with code this script writes a minimal ELF (.text/.data
sections at the load address) to analysis/overlays/, runs
`psp_recomp --auto --tag <name> --unit-base <EBOOT text base>` into
generated/overlays/<name>/, and writes generated/overlays/overlay_table.cpp
with the identity (load address, size, FNV-1a hash) of every image so the host
only activates translations that match the bytes in guest memory.

Overlays are mostly entered from the EBOOT (`jal` to fixed overlay addresses)
or from other overlays, so every jal/j target, every lui+addiu/ori address
constant and every pointer-sized data word of the EBOOT and of all overlays
that lands in an overlay's text is passed to psp_recomp as an external seed.

    extract_overlays.py <over.afs> <psp_recomp> <profile_dir> <decrypted EBOOT>
"""
import pathlib
import re
import struct
import subprocess
import sys

HEADER_SIZE = 0x40
EBOOT_UNIT_BASE = 0x08804000  # .text start of the ULES-00476 EBOOT
UNIT_SPAN = 16384


def fnv1a64(data: bytes) -> int:
    value = 0xCBF29CE484222325
    for byte in data:
        value = ((value ^ byte) * 0x100000001B3) & 0xFFFFFFFFFFFFFFFF
    return value


def find_overlays(blob: bytes):
    overlays = []
    for offset in range(0, len(blob) - HEADER_SIZE, 0x800):
        if blob[offset:offset + 4] != b'MWo3':
            continue
        ident, load, text, data, bss, end, _ = struct.unpack_from('<7I', blob, offset + 4)
        name = blob[offset + 0x20:offset + 0x40].split(b'\0')[0].decode('ascii')
        if end != load + HEADER_SIZE + text + data:
            raise SystemExit(f'{name}: inconsistent MWo3 header at {offset:#x}')
        overlays.append(dict(name=name, id=ident, offset=offset, load=load, text=text, data=data,
                             bss=bss, image=blob[offset:offset + HEADER_SIZE + text + data]))
    return overlays


def elf_sections(blob: bytes):
    """(name, flags, addr, bytes) of every allocated PROGBITS section."""
    shoff, = struct.unpack_from('<I', blob, 0x20)
    shnum, shstrndx = struct.unpack_from('<HH', blob, 0x30)
    headers = [struct.unpack_from('<10I', blob, shoff + i * 40) for i in range(shnum)]
    names = headers[shstrndx]
    out = []
    for h in headers:
        if h[1] != 1 or (h[2] & 0x2) == 0 or h[5] == 0:
            continue
        name = blob[names[4] + h[0]:blob.index(b'\0', names[4] + h[0])].decode()
        out.append((name, h[2], h[3], blob[h[4]:h[4] + h[5]]))
    return out


LUI_WINDOW = 32  # instructions after a lui searched for its addiu/ori


def lui_pairs(code):
    """Addresses built with `lui rt, hi` + `addiu/ori rx, rt, lo` (function pointers
    passed as arguments, e.g. callbacks the EBOOT registers inside game.ovl)."""
    words = struct.unpack_from(f'<{len(code) // 4}I', code)
    for i, word in enumerate(words):
        if word >> 26 != 0x0F:
            continue
        rt, hi = (word >> 16) & 0x1F, word & 0xFFFF
        for next_word in words[i + 1:i + 1 + LUI_WINDOW]:
            op, rs, dest = next_word >> 26, (next_word >> 21) & 0x1F, (next_word >> 16) & 0x1F
            if op in (0x09, 0x0D) and rs == rt:
                lo = next_word & 0xFFFF
                if op == 0x09 and lo & 0x8000:
                    lo -= 0x10000
                yield ((hi << 16) + lo) & 0xFFFFFFFF if op == 0x09 else (hi << 16) | lo
            # Stop once rt is overwritten (I-type rt or R-type rd destinations).
            if (op not in (0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x14, 0x15, 0x16, 0x17)
                    and op < 0x28 and dest == rt) or (op == 0x00 and (next_word >> 11) & 0x1F == rt):
                break


def references(code_regions, data_regions):
    """Targets of j/jal and lui/addiu address pairs in code plus every word stored in data."""
    targets = set()
    for addr, code in code_regions:
        for i in range(0, len(code) - 3, 4):
            word, = struct.unpack_from('<I', code, i)
            if word >> 26 in (2, 3):
                targets.add(((addr + i + 4) & 0xF0000000) | ((word & 0x03FFFFFF) << 2))
        targets.update(lui_pairs(code[:len(code) & ~3]))
    for addr, data in data_regions:
        for i in range(0, len(data) - 3, 4):
            targets.add(struct.unpack_from('<I', data, i)[0])
    return targets


def build_elf(overlay) -> bytes:
    """ET_EXEC MIPS ELF: one RWX PT_LOAD plus .text/.data section headers."""
    image = overlay['image']
    load = overlay['load']
    shstrtab = b'\0.text\0.data\0.shstrtab\0'
    ehsize, phentsize, shentsize = 52, 32, 40
    image_offset = 0x100
    shstr_offset = image_offset + len(image)
    shoff = (shstr_offset + len(shstrtab) + 3) & ~3
    header = struct.pack('<16sHHIIIIIHHHHHH',
                         b'\x7fELF\x01\x01\x01' + b'\0' * 9,
                         2, 8, 1,                       # ET_EXEC, EM_MIPS, EV_CURRENT
                         load + HEADER_SIZE, ehsize, shoff, 0,
                         ehsize, phentsize, 1, shentsize, 4, 3)
    program = struct.pack('<8I', 1, image_offset, load, load, len(image),
                          len(image) + overlay['bss'], 7, 16)
    text_addr = load + HEADER_SIZE
    data_addr = text_addr + overlay['text']
    sections = b''.join([
        b'\0' * shentsize,
        struct.pack('<10I', 1, 1, 0x6, text_addr, image_offset + HEADER_SIZE, overlay['text'], 0, 0, 16, 0),
        struct.pack('<10I', 7, 1, 0x3, data_addr, image_offset + HEADER_SIZE + overlay['text'],
                    overlay['data'], 0, 0, 16, 0),
        struct.pack('<10I', 13, 3, 0, 0, shstr_offset, len(shstrtab), 0, 0, 1, 0),
    ])
    out = bytearray(header + program)
    out += b'\0' * (image_offset - len(out))
    out += image + shstrtab
    out += b'\0' * (shoff - len(out))
    out += sections
    return bytes(out)


def main():
    if len(sys.argv) != 5:
        raise SystemExit(__doc__)
    afs, recomp, profile = pathlib.Path(sys.argv[1]), sys.argv[2], pathlib.Path(sys.argv[3])
    eboot = pathlib.Path(sys.argv[4]).read_bytes()
    overlays = [o for o in find_overlays(afs.read_bytes()) if o['text'] > HEADER_SIZE]

    code_regions = [(a, b) for _, f, a, b in elf_sections(eboot) if f & 0x4]
    data_regions = [(a, b) for _, f, a, b in elf_sections(eboot) if not f & 0x4]
    for o in overlays:
        text_start = o['load'] + HEADER_SIZE
        code_regions.append((text_start, o['image'][HEADER_SIZE:HEADER_SIZE + o['text']]))
        data_regions.append((text_start + o['text'], o['image'][HEADER_SIZE + o['text']:]))
    targets = references(code_regions, data_regions)
    elf_dir = profile / 'analysis' / 'overlays'
    out_dir = profile / 'generated' / 'overlays'
    elf_dir.mkdir(parents=True, exist_ok=True)
    out_dir.mkdir(parents=True, exist_ok=True)

    rows = []
    for overlay in overlays:
        tag = re.sub(r'[^A-Za-z0-9_]', '_', overlay['name'].removesuffix('.ovl'))
        elf = elf_dir / f"{overlay['name']}.elf"
        elf.write_bytes(build_elf(overlay))
        text_start = overlay['load'] + HEADER_SIZE
        seeds = sorted(t for t in targets if text_start <= t < text_start + overlay['text'] and t % 4 == 0)
        seeds_file = elf_dir / f"{overlay['name']}.seeds"
        seeds_file.write_text(''.join(f'{t:#010x}\n' for t in seeds))
        subprocess.run([recomp, str(elf), '--auto', str(out_dir / tag), hex(overlay['load']), str(UNIT_SPAN),
                        '--tag', tag, '--unit-base', hex(EBOOT_UNIT_BASE), '--seeds', str(seeds_file)],
                       check=True, stdout=subprocess.DEVNULL)
        rows.append((tag, overlay))
        print(f"{overlay['name']:24} load={overlay['load']:#010x} size={len(overlay['image']):#09x} "
              f"external_seeds={len(seeds)}")

    lines = ['// Generated by profiles/pes6/scripts/extract_overlays.py. Do not edit.',
             '#include "pes6_overlays.hpp"', '', 'namespace psprecomp {']
    lines += [f'void register_generated_functions_{tag}(Runtime &runtime);' for tag, _ in rows]
    lines += ['} // namespace psprecomp', '', 'namespace pes6 {', 'const OverlayDescriptor kOverlays[] = {']
    for tag, o in rows:
        lines.append(f'    {{"{o["name"]}", {o["id"]}u, 0x{o["load"]:08X}u, 0x{len(o["image"]):X}u, '
                     f'0x{o["bss"]:X}u, 0x{fnv1a64(o["image"]):016X}ull, '
                     f'&psprecomp::register_generated_functions_{tag}}},')
    lines += ['};', 'const std::size_t kOverlayCount = sizeof(kOverlays) / sizeof(kOverlays[0]);',
              '} // namespace pes6', '']
    (out_dir / 'overlay_table.cpp').write_text('\n'.join(lines))
    print(f'{len(rows)} overlays recompiled into {out_dir}')


if __name__ == '__main__':
    main()
