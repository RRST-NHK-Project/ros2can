import os
import subprocess

from setuptools import find_packages, setup

package_name = 'ros2can'
_here = os.path.dirname(os.path.abspath(__file__))


def _write_git_version() -> str:
    """ビルド時点の git short hash を resources/git_version.txt に焼き込む。
    GUI側 (main_window.py) はこれを読んでバージョン表示に使う。colcon build
    のたびに書き直されるので、コミットが進めば表示も自動で追従する。"""
    try:
        git_hash = subprocess.check_output(
            ['git', 'rev-parse', '--short', 'HEAD'],
            cwd=_here, stderr=subprocess.DEVNULL,
        ).decode().strip()
        dirty = subprocess.call(
            ['git', 'diff', '--quiet', '--exit-code'],
            cwd=_here, stderr=subprocess.DEVNULL,
        ) != 0
        if dirty:
            git_hash += '-dirty'
    except Exception:
        git_hash = ''
    os.makedirs(os.path.join(_here, 'resources'), exist_ok=True)
    with open(os.path.join(_here, 'resources', 'git_version.txt'), 'w') as f:
        f.write(git_hash)
    return git_hash


_write_git_version()

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/ros2can.launch.py']),
        ('share/' + package_name + '/config', ['config/ros2can.yaml']),
        ('share/' + package_name + '/resources',
         ['resources/logo.png', 'resources/soki_logo.png', 'resources/git_version.txt',
          'resources/style.qss']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dev',
    maintainer_email='tashikou1682@gmail.com',
    description=(
        'xiao_esp32_s3_smd_serial_bridge (CANバス複数マイコン対応) 専用のスタンドアローンGUI。'
        'serial_bridge の後継として自前でシリアル通信を行い、'
        'CANノードを選択してアクチュエータへの指令値を直接送信し、センサ値を表示する。'
    ),
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ros2can = ros2can.main:main',
        ],
    },
)
