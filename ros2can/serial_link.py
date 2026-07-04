"""シリアルポートの列挙・排他オープン・serial_bridge フレームの送受信。

[serial_bridge/src/port_scanner.cpp, bridge_node.cpp からの移植 + 排他制御の追加]

serial_bridge (C++版) は複数プロセスからの同時オープンに対する保護
(`O_EXCL` / `TIOCEXCL` / `flock`) を一切行っていない。ros2can と serial_bridge を
同一マシンで併用した場合、両者が同じ `/dev/ttyUSBx` を同時に掴んでしまうと
read/write が競合しフレームが破損しうる。

これを防ぐため、ros2can 側ではポートを開いた直後に `ioctl(fd, TIOCEXCL)` を
発行する。これは Linux のトポート層で強制される排他制御で、既に TIOCEXCL 済みの
tty に対する**他プロセスからの新規 open() は EBUSY で失敗する**。
serial_bridge の port_scanner / bridge_node は open 失敗を「使用中/失敗」として
静かにスキップ・リトライする作りになっているため、ros2can が先に掴んだポートに
serial_bridge が誤って重複接続することは防止できる。

逆方向 (serial_bridge が先にポートを掴んだ場合に ros2can から守る) は
serial_bridge 側にも同様の TIOCEXCL を入れない限り完全ではない。これは
serial_bridge 本体 (別リポジトリのサブモジュール) 側の改修が必要なため、
本ツールでは「ros2can が先に掴んだ場合の保護」のみを提供する。
"""

from __future__ import annotations

import fcntl
import glob
import time
from typing import List, Optional, Tuple

import serial

from .frame_codec import FrameParser, encode_frame

BAUD_RATE = 115200

# Linux 固有の ioctl 番号 (asm-generic/ioctls.h の TIOCEXCL / TIOCNXCL)。
# 対象OSは serial_bridge と同じく Ubuntu (Linux) のみを想定する。
TIOCEXCL = 0x540C
TIOCNXCL = 0x540D


def list_serial_ports() -> List[str]:
    """/dev/ttyUSB* (ESP32等) と /dev/ttyACM* (Arduino/STM32等) を列挙する。"""
    ports: List[str] = []
    for pattern in ("/dev/ttyUSB*", "/dev/ttyACM*"):
        ports.extend(glob.glob(pattern))
    return sorted(set(ports))


class SerialLinkError(Exception):
    pass


def _open_raw_serial(port: str, timeout: float = 0.0) -> serial.Serial:
    ser = serial.Serial()
    ser.port = port
    ser.baudrate = BAUD_RATE
    ser.bytesize = serial.EIGHTBITS
    ser.parity = serial.PARITY_NONE
    ser.stopbits = serial.STOPBITS_ONE
    ser.timeout = timeout
    ser.write_timeout = 1.0
    # pyserial 独自の flock ベース排他。ros2can 同士の多重起動対策として保険的に有効化。
    # (serial_bridge 側はこれを見ないため、対 serial_bridge の保護には TIOCEXCL が必要)
    ser.exclusive = True
    ser.open()
    return ser


def _claim_tiocexcl(ser: serial.Serial) -> bool:
    try:
        fcntl.ioctl(ser.fileno(), TIOCEXCL)
        return True
    except OSError:
        return False


def _release_tiocexcl(ser: serial.Serial) -> None:
    # 参照カウントが残っている(他にもそのttyを開いているfdがある)場合、
    # close() だけでは EXCL フラグが tty に残り続けることがあるため明示的に解除する。
    try:
        fcntl.ioctl(ser.fileno(), TIOCNXCL)
    except OSError:
        pass


def probe_port(port: str, timeout_sec: float = 2.0,
               settle_sec: float = 0.5) -> Optional[int]:
    """ポートを一時的に開き、有効な serial_bridge フレームを受信できるか確認する。

    成功したら受信フレームの DEVICE_ID を返す。見つからなければ None。
    [port_scanner.cpp: open_serial_port() + read_frame() を移植]
    """
    try:
        ser = _open_raw_serial(port, timeout=0.05)
    except (OSError, serial.SerialException):
        return None

    try:
        _claim_tiocexcl(ser)
        # USB CDC の安定待ち (port_scanner.cpp と同じく 500ms)
        # 元のC++実装同様、待ち後にバッファをフラッシュせずそのまま読み始める
        time.sleep(settle_sec)

        parser = FrameParser()
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            waiting = ser.in_waiting
            data = ser.read(waiting if waiting else 1)
            if data:
                parser.feed(data)
                frame = parser.pop_frame()
                if frame is not None:
                    device_id, _values = frame
                    return device_id
        return None
    except (OSError, serial.SerialException):
        return None
    finally:
        _release_tiocexcl(ser)
        try:
            ser.close()
        except OSError:
            pass


class SerialLink:
    """1つのシリアルポートを排他的に専有し、serial_bridge フレームを継続的に送受信する。

    [serial_bridge::SerialBridgeNode (bridge_node.cpp) を移植]
    """

    def __init__(self, port: str, device_id: int):
        self.port = port
        self.device_id = device_id
        self.parser = FrameParser()
        self.exclusive_claimed = False
        self._ser: Optional[serial.Serial] = None

    @property
    def is_open(self) -> bool:
        return self._ser is not None and self._ser.is_open

    def open(self) -> None:
        self.close()
        ser = _open_raw_serial(self.port, timeout=0.0)
        self.exclusive_claimed = _claim_tiocexcl(ser)
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        self._ser = ser
        self.parser = FrameParser()

    def close(self) -> None:
        if self._ser is not None:
            if self.exclusive_claimed:
                _release_tiocexcl(self._ser)
            try:
                self._ser.close()
            except OSError:
                pass
            self._ser = None
            self.exclusive_claimed = False

    def read_frames(self) -> List[Tuple[int, List[int]]]:
        """今読める分をすべて読み、完成したフレームをすべて返す(0個のこともある)。"""
        if self._ser is None:
            return []
        try:
            waiting = self._ser.in_waiting
            if waiting:
                data = self._ser.read(waiting)
                if data:
                    self.parser.feed(data)
        except (OSError, serial.SerialException) as exc:
            raise SerialLinkError(str(exc)) from exc

        frames: List[Tuple[int, List[int]]] = []
        while True:
            frame = self.parser.pop_frame()
            if frame is None:
                break
            frames.append(frame)
        return frames

    def write_data(self, data: List[int]) -> None:
        if self._ser is None:
            return
        frame = encode_frame(self.device_id, data)
        try:
            self._ser.write(frame)
        except (OSError, serial.SerialException) as exc:
            raise SerialLinkError(str(exc)) from exc
