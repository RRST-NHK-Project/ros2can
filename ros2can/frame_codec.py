"""serial_bridge と互換のバイナリフレームプロトコル (エンコード/デコード)。

[serial_bridge/src/bridge_node.cpp, port_scanner.cpp より移植]

  [0]     START_BYTE  : 0xAA
  [1]     DEVICE_ID   : uint8
  [2]     LENGTH      : ペイロードのバイト数 (= int16 個数 x 2)
  [3..N]  DATA        : int16 配列、ビッグエンディアン (高位バイト先行)
  [N+1]   CHECKSUM    : バイト [1]..[N] の XOR

xiao_esp32_s3_smd_serial_bridge (CANホスト) ファームウェアが使う既定値と
そのままワイヤ互換になるよう、定数はすべて元の serial_bridge と同じにしてある。
"""

from __future__ import annotations

from typing import List, Optional, Tuple

START_BYTE = 0xAA
SLOT_COUNT = 24  # serial_bridge::config::kTx16Num / kRx16Num
MAX_FRAME_SIZE = 256


def encode_frame(device_id: int, data: List[int]) -> bytes:
    """24 x int16 のスロット配列を1フレームにエンコードする。"""
    length = SLOT_COUNT * 2
    body = bytearray()
    body.append(device_id & 0xFF)
    body.append(length & 0xFF)
    values = list(data[:SLOT_COUNT]) + [0] * max(0, SLOT_COUNT - len(data))
    for v in values:
        iv = int(v) & 0xFFFF  # 2の補数として符号なし16bit化
        body.append((iv >> 8) & 0xFF)
        body.append(iv & 0xFF)

    checksum = 0
    for b in body:
        checksum ^= b

    return bytes(bytearray([START_BYTE]) + body + bytearray([checksum]))


class FrameParser:
    """ストリーミング受信バッファに対して START_BYTE 同期 + CHECKSUM 検証を行うパーサ。

    不正なバイトは自動的に1バイトずつ捨てて再同期する
    (serial_bridge::SerialBridgeNode::update() と同じ挙動)。
    """

    def __init__(self):
        self._buf = bytearray()
        self.dropped_bytes = 0
        self.checksum_errors = 0

    def feed(self, data: bytes) -> None:
        self._buf.extend(data)

    def __len__(self) -> int:
        return len(self._buf)

    def pop_frame(self) -> Optional[Tuple[int, List[int]]]:
        """バッファから1フレーム取り出す。(device_id, values) か、未完/データ無しなら None。"""
        buf = self._buf
        while True:
            if len(buf) < 4:
                return None

            if buf[0] != START_BYTE:
                del buf[0]
                self.dropped_bytes += 1
                continue

            length = buf[2]
            if length % 2 != 0 or length > SLOT_COUNT * 2 * 4:
                # あり得ない LENGTH: 同期ずれとみなして1バイト捨てる
                del buf[0]
                self.dropped_bytes += 1
                continue

            frame_size = 1 + 1 + 1 + length + 1
            if frame_size > MAX_FRAME_SIZE:
                del buf[0]
                self.dropped_bytes += 1
                continue
            if len(buf) < frame_size:
                return None  # フレーム未完、続きを待つ

            checksum = 0
            for i in range(1, 3 + length):
                checksum ^= buf[i]
            if checksum != buf[3 + length]:
                del buf[0]
                self.dropped_bytes += 1
                self.checksum_errors += 1
                continue

            device_id = buf[1]
            raw = bytes(buf[3:3 + length])
            values = [
                _to_signed16((raw[i] << 8) | raw[i + 1])
                for i in range(0, length, 2)
            ]
            del buf[0:frame_size]
            return device_id, values


def _to_signed16(v: int) -> int:
    return v - 0x10000 if v >= 0x8000 else v
