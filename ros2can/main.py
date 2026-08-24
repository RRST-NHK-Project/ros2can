"""ros2can エントリポイント。

Qt のイベントループをメインループとし、QTimer から高頻度に
`rclpy.spin_once(timeout_sec=0)` を呼び出すことで ROS のコールバックを
GUI スレッド上でシリアルに処理する。これによりスレッド間排他を持ち込まずに
Qt シグナルを安全に発行できる。

serial_bridge (config/serial_bridge.yaml) と同名のパラメータで
ハードウェア直結スキャンの挙動を設定できる (excluded_ports 等、
RosBackend._load_hardware_config_from_params 参照)。

--nogui: PyQt5 のウィンドウを開かず、serial_bridge (graphical_ui.hpp) 風の
ターミナルダッシュボードで動作する。この場合ハードウェア直結デバイスは
serial_bridge 互換で常時ダイレクト送信 ON (GUIの手動ゲート無しでそのまま
TX/RXを中継するブリッジとして動く)。
"""

from __future__ import annotations

import argparse
import signal
import sys
from typing import List, Optional

import rclpy
from rclpy.utilities import remove_ros_args
from PyQt5.QtCore import QCoreApplication, QTimer
from PyQt5.QtWidgets import QApplication

from .ros_backend import RosBackend
from .main_window import MainWindow
from .console_ui import ConsoleUi

SPIN_INTERVAL_MS = 10
PUBLISH_INTERVAL_MS = 50   # ダイレクト送信が有効なデバイスへの周期送信 (20Hz)
HARDWARE_SERVICE_MS = 10   # シリアルリンクの読み書きサービス周期
SIMULATOR_SERVICE_MS = 50  # デバッグ用仮想デバイスのTX->RXループバック周期 (20Hz、実機のRXを模す)
TOPIC_RESCAN_MS = 1000     # --nogui時のトピック相乗り自動検出周期 (main_window.pyと同一)
CONSOLE_RENDER_MS = 100    # --nogui時のダッシュボード再描画周期 (serial_bridgeのkGraphicalUiFrameMsと同一)


def _parse_args(argv: List[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog="ros2can")
    parser.add_argument(
        "--nogui", action="store_true",
        help="PyQt5 GUIの代わりに、serial_bridge風のターミナルダッシュボード(CUI)で動作する")
    # rclpy用の --ros-args 以降やその他ROS予約引数は自前のパーサーに渡さない。
    stripped = remove_ros_args(args=argv)
    return parser.parse_args(stripped[1:])


def main(argv: Optional[List[str]] = None) -> int:
    raw_argv = argv if argv is not None else sys.argv
    args = _parse_args(raw_argv)

    rclpy.init(args=raw_argv)

    app = QCoreApplication(sys.argv) if args.nogui else QApplication(sys.argv)
    app.setApplicationName("ros2can")

    backend = RosBackend()

    console: Optional[ConsoleUi] = None

    if args.nogui:
        def _force_direct_tx(device_id: int, _port: str) -> None:
            ch = backend.devices.get(device_id)
            if ch is not None:
                ch.direct_tx = True

        backend.hardware.deviceClaimed.connect(_force_direct_tx)

        console = ConsoleUi(backend)
        console.start()
    else:
        window = MainWindow(backend)
        window.show()

    backend.start_hardware_scanning()

    spin_timer = QTimer()
    spin_timer.timeout.connect(backend.spin_once)
    spin_timer.start(SPIN_INTERVAL_MS)

    hardware_timer = QTimer()
    hardware_timer.timeout.connect(backend.service_hardware)
    hardware_timer.start(HARDWARE_SERVICE_MS)

    simulator_timer = QTimer()
    simulator_timer.timeout.connect(backend.service_simulators)
    simulator_timer.start(SIMULATOR_SERVICE_MS)

    publish_timer = QTimer()
    publish_timer.timeout.connect(backend.publish_all_direct)
    publish_timer.start(PUBLISH_INTERVAL_MS)

    if console is not None:
        rescan_timer = QTimer()
        rescan_timer.timeout.connect(backend.rescan_topics)
        rescan_timer.start(TOPIC_RESCAN_MS)
        backend.rescan_topics()

        render_timer = QTimer()
        render_timer.timeout.connect(console.render)
        render_timer.start(CONSOLE_RENDER_MS)

        # QCoreApplicationのイベントループがPythonのSIGINTハンドラ処理をブロックしない
        # よう、短周期タイマーで定期的にPythonインタプリタへ制御を戻す (定番の回避策)。
        signal.signal(signal.SIGINT, lambda *_: app.quit())
        sigint_pump = QTimer()
        sigint_pump.timeout.connect(lambda: None)
        sigint_pump.start(200)

    exit_code = app.exec_()

    if console is not None:
        console.stop()

    backend.emergency_stop_all()
    backend.shutdown()
    rclpy.shutdown()
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
