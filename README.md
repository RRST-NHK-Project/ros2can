# **ros2can**

> 📖 マニュアル(GitHub Pages): https://rrst-nhk-project.github.io/ros2can/
> （初めての方は [クイックスタート](https://rrst-nhk-project.github.io/ros2can/quickstart) がおすすめです）
> 内部の処理の流れ（スレッド/タスク構成、シリアル/CANプロトコル詳細等）は
> [技術マニュアル（内部実装編）](https://rrst-nhk-project.github.io/ros2can/internals) を参照してください。

> 本パッケージは [serial_bridge](https://github.com/RRST-NHK-Project/serial_bridge) の後継です。
> serial_bridgeとの比較（配線・GUI・対応アクチュエータ等）は
> マニュアルの[1.1 serial_bridgeとの比較](https://rrst-nhk-project.github.io/ros2can/#11-serial_bridgeとの比較)を参照してください。

## 概要

`ros2can` は、CANバス経由でマイコン（ロボマス・CubeMars AKシリーズ等）を直接操作できる
ROS 2 GUIパッケージです。機能・使い方の詳細は上記マニュアルを参照してください。

## インストールと起動

```bash
sudo apt install python3-pyqt5 python3-serial

cd ~/ros2_ws
colcon build --packages-select ros2can
source install/setup.bash

ros2 run ros2can ros2can
# または（config/ros2can.yaml のパラメータを読み込む場合）
ros2 launch ros2can ros2can.launch.py
```

Ubuntu 24.04 LTS + ROS 2 Jazzy を前提としています。`/dev/ttyUSB*` /
`/dev/ttyACM*` を使うには `dialout` グループへの追加が必要です（初回のみ、
反映には再ログインが必要）。

```bash
sudo usermod -aG dialout $USER
```

同一マシンで `serial_bridge` を併用する場合の排他制御・設定については、
マニュアルの[3.1 serial_bridgeとの併用について](https://rrst-nhk-project.github.io/ros2can/#31-serial_bridge-との併用について)
を参照してください。

## ディレクトリ構成

| パス | 説明 |
|:---|:---|
| `ros2can/` | GUI本体のソース（`main`, `main_window`, `device_panel`, `hardware_manager`, `ros_backend`, `frame_codec` 等） |
| `config/` | ROS 2 パラメータファイル（`ros2can.yaml`） |
| `launch/` | ランチファイル（`ros2can.launch.py`） |
| `resources/` | ロゴ・スタイルシート・バージョン情報（`logo.png`, `soki_logo.png`, `style.qss`, `git_version.txt`） |
| `firmware/xiao-esp32-s3_can2io/` | CANノード側マイコンファームウェア（PlatformIO） |
| `firmware/b-g431-esc1_can2io/` | FOCモータ用CANノードファームウェア（PlatformIO） |
| `test/` | ユニットテスト（`test_counter_unwrapper.py` 等） |

---

## About
<img src="https://www.rrst.jp/img/logo.png" alt="Logo" height="60"><br>
立命館大学 ロボット技術研究会, RRST, NHKプロジェクト（2024–2026）<br><br>

<img src="resources/soki_logo.png" alt="soki Logo" height="60"><br>
キャチロボ2026, 創機立動<br>

