# 実行環境の準備

Ubuntu 24.04とROS 2 Jazzyが前提です。WindowsはWSL2とGUI表示を使用します。
ROS自体の導入は [ROS公式手順](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html) を参照してください。
JazzyとHarmonicの組み合わせは [Gazebo公式資料](https://gazebosim.org/docs/harmonic/ros_installation/) に記載されています。

```bash
# ROSのaptリポジトリを設定した後、Ubuntu/WSLで実行する。
sudo apt update
sudo apt install ros-jazzy-ros-gz ros-jazzy-tf2-ros python3-venv python3-pip
# リポジトリのルートで実行する。pyenvの別バージョンを使わない。
/usr/bin/python3 -m venv --system-site-packages .venv
source .venv/bin/activate
python3 -m pip install --index-url https://download.pytorch.org/whl/cpu 'torch==2.14.1+cpu'
python3 -m pip install 'numpy==1.26.4'
source scripts/env.sh
python3 scripts/check_submission.py --runtime
```

同梱モデルを評価した環境は Python 3.12.3、NumPy 1.26.4、PyTorch 2.14.1+cpu、rclpy 7.1.12、Gazebo Sim 8.15.0です。
上記は検証済みのバージョンです。将来の配布状況や別バージョンでの動作を保証しません。
`requirements.txt` にもPython依存を記録しました。rclpyなどはpipからではなくROSパッケージから読みます。

実行確認とAI走行は README の手順を使用します。ソースからの実行にcolconビルドは不要です。

## ROSパッケージとしてビルドする場合

```bash
source scripts/env.sh
bash scripts/build.sh
source .ros_install/local_setup.sh
ros2 pkg executables pai_ai_navigation
```

ビルド用ツールはプロジェクト内の `.build_tools/` に入り、生成物はGit管理しません。

## 追加の制御試験

次のコマンドはシミュレーターを起動し、意図的な姿勢・指令を設定して検証します。
AIの通常走行と同時に実行しないでください。既存の検証結果は上書きしません。

```bash
source scripts/env.sh
python3 -m pai_ai_navigation.verify_contacts
python3 -m pai_ai_navigation.verify_shield --output runtime/shield_check_new.json
```

`verify_contacts` は接触チャネルを確認するため、一時的に車体を障害物へ重ねます。
保存済みの `docs/results/contact_channel_test.json` は過去の記録で、新しい環境で実行した証拠にはしません。

## WindowsのネイティブGUI

WSL側に上記環境を用意し、Windows側にはPython 3とTkを用意します。Windows側にはPyTorchやROSは不要です。
GUIはWindows上で実行してください。WSL内のPythonで起動する構成ではありません。

```powershell
# リポジトリのルート。既定のWSLディストリビューションを使う場合。
py -3 scripts/native_monitor.py
# 使用するWSLを明示する場合は、実際のディストリビューション名を設定する。
$env:PAI_WSL_DISTRO = 'Ubuntu'
py -3 scripts/native_monitor.py
```

WindowsとWSLから同じチェックアウトを参照できる場所に配置します（例：Windows側のフォルダー）。
`pythonw.exe` がPATHにあれば `scripts/Open-Native-Monitor.vbs` でもコンソールを出さず起動できます。
必要なら環境変数 `PAI_MONITOR_PYTHON` にWindowsのPython実行ファイルの絶対パスを設定してください。
