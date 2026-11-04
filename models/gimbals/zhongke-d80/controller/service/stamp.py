"""帧序号二进制条：服务端烧录、客户端读取。

左下角 16px 高的条带，共 32 个 16px 宽的方块：
  高 8bit 固定魔数 0xA5，低 24bit 为帧序号（mod 2^24）。
方块白色=1，黑色=0。
"""
from __future__ import annotations

STAMP_BLOCK_W = 16
STAMP_BLOCK_H = 16
STAMP_BITS = 32
STAMP_MAGIC = 0xA5


def seq_to_bits(seq: int) -> list[int]:
    value = ((STAMP_MAGIC & 0xFF) << 24) | (seq & 0xFFFFFF)
    return [(value >> (31 - i)) & 1 for i in range(STAMP_BITS)]


def bits_to_seq(bits: list[int]) -> int | None:
    if len(bits) != STAMP_BITS:
        return None
    value = 0
    for b in bits:
        value = (value << 1) | (1 if b else 0)
    if (value >> 24) & 0xFF != STAMP_MAGIC:
        return None
    return value & 0xFFFFFF


def decode_stamp_from_bgr(buf, width: int, height: int) -> int | None:
    """从 BGR 图像左下角的二进制条读出序号（buf: numpy uint8 (h,w,3)）。"""
    if buf.shape[0] < height or buf.shape[1] < width:
        return None
    y = height - STAMP_BLOCK_H // 2
    bits = []
    row = buf[y]
    for i in range(STAMP_BITS):
        x = i * STAMP_BLOCK_W + STAMP_BLOCK_W // 2
        px = row[x]
        bits.append(1 if int(px[1]) > 127 else 0)
    return bits_to_seq(bits)
