# omni_robot_ros

ROS **Melodic / ROS 1 / catkin / C++11** 用の3輪オムニロボット上位制御。
Windowsでソースと純粋C++のテストを開発し、GitHub経由でRaspberry Pi 3B+へ配置します。

```text
joy_node → /joy → teleop_twist_joy → /cmd_vel → omni_can_bridge → cansend → CAN HAT → STM32
               └────────────────────────────→ 許可ボタンと入力更新の監視
```

## 前提

- ラズパイでROS Melodicが動作するLinux環境。Melodicの標準対象はUbuntu 18.04 Bionicです。
- CAN HATの設定とcan-utilsの動作確認は済んでいる前提。HAT固有のoverlayや発振器設定は変更しません。
- STM32は既存仕様の500 kbps、標準ID、機体座標系、200 ms通信断停止に対応した版を書き込み済み。
- ゲームパッド機種が未指定のため、軸・ボタン番号は仮設定です。
- Linux/ROS本体の導入をこのパッケージから自動実行することはありません。

## ラズパイへ配置・ビルド

GitHubにこのフォルダー全体を1リポジトリとして登録し、`YOUR_REPOSITORY_URL`を実URLに置き換えます。

```bash
source /opt/ros/melodic/setup.bash
sudo apt-get update
sudo apt-get install ros-melodic-joy ros-melodic-teleop-twist-joy can-utils
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone YOUR_REPOSITORY_URL omni_robot_ros
cd ~/catkin_ws
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

Melodicは古いディストリビューションのため、aptが失敗する場合は使用OSとapt取得元を確認してください。
すでに作動しているROS/CAN環境の再インストールは不要です。
`package.xml`のmaintainerメールは仮値なので、公開前に実際の連絡先へ変更してください。

更新はラズパイの`~/catkin_ws/src/omni_robot_ros`で`git pull`し、ワークスペースで再度`catkin_make`します。
YAMLだけの変更は再ビルド不要ですが、launchは再起動してください。

## ボタン・軸の設定

`config/joystick.yaml`を編集します。同じファイルをteleopとCANブリッジの両方が読み込みます。

| キー | 意味 | 初期値 |
|---|---|---|
| `enable_button` | 押している間だけ駆動するボタン（0始まり） | 4 |
| `enable_turbo_button` | 高速モードの許可ボタン、無効は-1 | -1 |
| `axis_linear.x` | X方向用の軸番号 | 1 |
| `axis_linear.y` | Y方向用の軸番号 | 0 |
| `axis_angular.yaw` | 旋回用の軸番号 | 3 |
| `scale_linear.x/y` | 最大速度、m/s | 0.2 |
| `scale_angular.yaw` | 最大旋回速度、rad/s | 0.5 |

方向が反対なら該当する`scale`を負にします。軸番号を負にして反転する方式ではありません。
ROS上はX前方・Y左方・正のyaw反時計回りを想定します。実機の取付角度と方向は初回に確認してください。

次のコマンドでCANを送らずに起動し、`rostopic echo /joy`でaxes/buttonsの番号を確認します。

```bash
roslaunch omni_robot_ros teleop.launch dry_run:=true
# 別ターミナル（setup.bashをsource済み）
rostopic echo /joy
rostopic echo /cmd_vel
rostopic echo /omni_can_bridge/tx_frame
```

例：`200#0000640000000101`はY方向0.1 m/s、連番1、駆動許可です。
`tx_frame`は送信予定データです。実機が受理したことを示すACKではありません。
`dry_run:=true`ではcansend自体を実行しません。

設定ファイルやデバイスを外から指定することもできます。

```bash
roslaunch omni_robot_ros teleop.launch joy_dev:=/dev/input/js1 \
  joystick_config:=/absolute/path/gamepad.yaml \
  can_config:=/absolute/path/can.yaml dry_run:=true
```

## CAN接続と起動

`config/can.yaml`の`can_interface`を動作確認済みの名前（初期値can0）に合わせます。
ビットレートはOS側で設定し、ノードは変更しません。

```bash
ip -details -statistics link show can0
```

500000 bpsであることを確認してください。必要な場合だけ、他の送受信を終了して設定します。

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up
```

実通信時：

```bash
roslaunch omni_robot_ros teleop.launch
# 別ターミナル
candump can0,200:7FF
```

初回は車輪が自由に回る状態で、**許可ボタンを一度離してから押す**操作を行います。
起動時に押しっぱなしのボタンや、ROSの残った速度メッセージでは走り出さない設計です。
`rostopic hz /cmd_vel`も確認します。`joy.yaml`の`autorepeat_rate: 50.0`により、スティックを保持していても更新します。

## 送信・停止の仕様

- 50 Hz（20 ms）で最新の速度を送信。ROSの`/clock`ではなく単調増加時計で周期・入力期限を管理します。
- `linear.x/y`を1000倍してmm/s、`angular.z`を1000倍してmrad/sに変換し四捨五入。
- 各並進軸を±500 mm/s、旋回を±1000 mrad/s以内に制限。上限は`can.yaml`で下げられます。
- enableはJoyの指定ボタンで判定。ゼロ速度でも許可ボタンが押されていれば許可1になります。
- 起動直後、許可解除、Joy不正、NaN/Inf、Joyまたは速度の更新が150 ms途切れた場合、ゼロ速度・許可0。
- 通信エラーや入力期限切れの後は、許可ボタンを離して押し直すまで駆動しません。
- 正常終了時は停止フレームを3回送信。プロセス異常終了やCAN断線時はSTM32側の200 ms監視が最後の停止手段です。
- 停止はPWM停止指令です。物理的に瞬時停止する保証ではありません。
- 入力期限切れから次回送信まで最大1送信周期程度の遅れが加わります。Linuxはハードリアルタイムではありません。
- `cansend`を1フレームごとに起動します。子プロセスは通常10 msでタイムアウトし、終了回収します。タイムアウトは実機負荷を見て調整しますが、送信周期より短くしてください。
- 送信失敗時に過去の速度を蓄積・まとめ送りしません。カーネル側CAN送信キューは別なので、接続不良時は`ip -s -d link`の状態・エラーカウンターも確認します。

Joyの物理切断はJoyメッセージの途絶として検出します。無線ゲームパッドの受信器が古い入力を送信し続けるかどうかは機種依存です。実機で電源OFF・USB抜去・ノード停止の各操作を確認してください。

## パケット仕様

標準ID `0x200`、8バイト、リトルエンディアン：

| byte | 内容 |
|---|---|
| 0–1 | int16 X速度 [mm/s] |
| 2–3 | int16 Y速度 [mm/s] |
| 4–5 | int16 yaw速度 [0.001 rad/s] |
| 6 | uint8連番（255の次は0） |
| 7 | 駆動許可 0/1 |

GET_VERSIONもパケット変換ライブラリに用意しています。現段階ではROSサービス化せずcan-utilsで照会できます。

```bash
# 先に別ターミナルで受信を起動
candump can0,211:7FF
# 要求番号05
cansend can0 210#0105000000000000
# firmware 0.1.0の応答: 211 [8] 01 05 00 00 01 00 00 00
```

## 開発用テスト

ROSなしで、パケット変換・入力期限・許可状態のテストを実行できます。

```bash
cmake -S . -B build -DOMNI_BUILD_ROS=OFF -DBUILD_TESTING=ON
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
```

WindowsはVisual Studio C++とCMakeを使用します。Linuxではcansendの代役を使い、成功・失敗・タイムアウトと子プロセス回収もテストします。GitHub Actionsにも同じテストを登録しています。
Ubuntu 18.04標準の古いCMakeで`-S/-B`や`--test-dir`が使えない場合：

```bash
mkdir -p build && cd build
cmake .. -DOMNI_BUILD_ROS=OFF -DBUILD_TESTING=ON
cmake --build .
ctest --output-on-failure
```

STM32実機との結合確認：

1. dry_runで軸・ボタン・符号・送信形式を確認。
2. 実CANで50 Hzを確認し、STM32側の`CAN=RUN`とRPMを確認。
3. ボタン解除で`CAN=STOP`になることを確認。
4. Joyノード／teleopノード停止、USB抜去で停止することを確認。
5. ブリッジを強制終了しSTM32が約200 ms後に`TIMEOUT`になることを確認。
6. 再開時にボタンの離し直しが必要で、古い速度が再生されないことを確認。

## ファイルの役割

- `src/command_encoder.cpp`: 単位変換、制限、CANパケット生成。ROS非依存。
- `src/command_gate.cpp`: Joy許可・速度鮮度・再開条件。ROS非依存。
- `src/cansend_transport.cpp`: シェルを介さないcansend実行と待機期限。
- `src/omni_can_bridge.cpp`: ROS購読、周期送信、診断出力。
- `config/joystick.yaml`: ボタン・軸・速度スケール。
- `config/joy.yaml`: デッドゾーン・入力の自動再送。
- `config/can.yaml`: CANインターフェース・送信周期・タイムアウト。

## 参照

- [ROS 1 teleop_twist_joy](https://github.com/ros-teleop/teleop_twist_joy/tree/indigo-devel): 軸マッピングと速度スケール。
- [ROS Melodic joy_node](https://github.com/ros-drivers/joystick_drivers/blob/melodic-devel/joy/src/joy_node.cpp): autorepeatとLinux入力処理。
- [can-utils](https://github.com/linux-can/can-utils): cansend/candump。

GitHubへの送信先はプロジェクト所有者が指定してください。このソース生成だけではリモートへのpushは行いません。
