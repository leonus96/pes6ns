#!/usr/bin/env python3
"""Converts the binary PPM frames written by PSPRECOMP_FRAME_DUMP_DIR to PNG."""
import struct
import sys
import zlib


def read_ppm(path):
    data = open(path, 'rb').read()
    tokens, pos = [], 0
    while len(tokens) < 4:
        while data[pos:pos + 1].isspace():
            pos += 1
        if data[pos:pos + 1] == b'#':
            pos = data.index(b'\n', pos) + 1
            continue
        end = pos
        while not data[end:end + 1].isspace():
            end += 1
        tokens.append(data[pos:end])
        pos = end
    assert tokens[0] == b'P6', 'only binary P6 PPM is supported'
    width, height = int(tokens[1]), int(tokens[2])
    return width, height, data[pos + 1:pos + 1 + width * height * 3]


def write_png(path, width, height, rgb):
    raw = b''.join(b'\0' + rgb[y * width * 3:(y + 1) * width * 3] for y in range(height))

    def chunk(kind, body):
        return struct.pack('>I', len(body)) + kind + body + struct.pack('>I', zlib.crc32(kind + body))

    with open(path, 'wb') as out:
        out.write(b'\x89PNG\r\n\x1a\n')
        out.write(chunk(b'IHDR', struct.pack('>IIBBBBB', width, height, 8, 2, 0, 0, 0)))
        out.write(chunk(b'IDAT', zlib.compress(raw, 9)))
        out.write(chunk(b'IEND', b''))


for source in sys.argv[1:]:
    w, h, pixels = read_ppm(source)
    write_png(source.rsplit('.', 1)[0] + '.png', w, h, pixels)
    print(source, f'{w}x{h}', 'distinct colors:', len(set(pixels[i:i + 3] for i in range(0, len(pixels), 3))))
