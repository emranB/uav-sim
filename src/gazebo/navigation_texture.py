"""Deterministic visual texture preparation; not a perception or planning input."""

import random
import struct
import zlib
from pathlib import Path


def write_navigation_texture(destination: Path, seed: int) -> Path:
    """Write a PNG using the declared visual seed, without runtime image dependencies."""
    if type(seed) is not int or not 0 <= seed < 2**31:
        raise ValueError("Texture seed must be a nonnegative 31-bit integer")
    if destination.is_file():
        return destination
    rng = random.Random(seed)
    size = 512
    rows = []
    tile = [[rng.randrange(50, 220) for _ in range(size // 4)] for _ in range(size // 4)]
    for y in range(size):
        row = bytearray([0])
        for x in range(size):
            value = max(0, min(255, tile[y // 4][x // 4] + rng.randrange(-20, 21)))
            row.extend([value, value, value])
        rows.append(bytes(row))

    def chunk(kind: bytes, contents: bytes) -> bytes:
        return (
            struct.pack(">I", len(contents))
            + kind
            + contents
            + struct.pack(">I", zlib.crc32(kind + contents))
        )

    image = (
        b"\x89PNG\r\n\x1a\n"
        + chunk(b"IHDR", struct.pack(">IIBBBBB", size, size, 8, 2, 0, 0, 0))
        + chunk(b"IDAT", zlib.compress(b"".join(rows)))
        + chunk(b"IEND", b"")
    )
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_bytes(image)
    return destination
