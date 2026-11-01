# ライセンスと参照元

この提出版のコードは同梱のApache License 2.0で提供します。
ロボットと障害物はローカルのSDFプリミティブ、モデルはローカルの模倣学習で作成しました。
ROS、Gazebo、PyTorch、NumPyなど外部依存の権利とライセンスは各プロジェクトに帰属します。
講義資料や課題PDFはこのリポジトリに再配布していません。

実装・検証で参照した一次資料:

- [Gazebo Harmonic Sensors](https://gazebosim.org/docs/harmonic/sensors/)
- [GazeboとROSの対応](https://gazebosim.org/docs/harmonic/ros_installation/)
- [Gazebo Sim Contact system](https://github.com/gazebosim/gz-sim/blob/gz-sim8/src/systems/contact/Contact.cc)
- [Gazebo Reset API](https://gazebosim.org/api/sim/8/reset_simulation.html)
- [ros_gz Contacts](https://github.com/gazebosim/ros_gz/blob/jazzy/ros_gz_interfaces/msg/Contacts.msg)
- [Microsoft CreateFileの共有モード](https://learn.microsoft.com/en-us/windows/win32/api/fileapi/nf-fileapi-createfilea)

AI制御と監視の分離や模倣学習という発想自体の新規性は主張しません。
本課題の工夫は、CPUで動く構成、入力と教師データの対応、独立した監視、指令の可視化、保存ログによる検証です。
