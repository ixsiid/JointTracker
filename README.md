# Joint Tracker

M5シリーズの開発キットを利用した、Steam VR上で動かせるハードウェアトラッカーです。
内蔵センサを利用するので、M5 StackとAtom Matrixだけで関節の動きをトラッキングすることが可能です。

パソコンへとの通信にはWiFiを使い、Unityなどで作成したアプリでトラッキングデータを利用する他に、
Virtual Motion Tracker（後述）を利用することで、Steam VR上にトラッカーとして認識させることも可能です。

## デモ

画像をクリックすると、Youtubeで動画が見れます

[![Joint Tracker simple demo](https://img.youtube.com/vi/OgLW1Vw-JYE/0.jpg)](https://www.youtube.com/watch?v=OgLW1Vw-JYE)


ToDo 写真を載せる

ToDo 動かしてる様子の動画
複雑な動作　　→　下半身 or 肘トラッカー　→　下半身用のものがあるので、VirtualCast様子を録画


# ハードウェア構成

必須
- [M5 Stack](https://www.switch-science.com/catalog/3953/)
- [M5 Atom Matrix](https://www.switch-science.com/catalog/6260/) × トラッキングしたい**関節**の数
- [Groveケーブル](https://www.switch-science.com/catalog/5215/)
- [Groveケーブルハブ](https://www.switch-science.com/catalog/796/) （用意したAtomが全て接続できる分だけ）

参考にリンクを貼っていますが、リンク先のモノを必ず購入する必要があるわけではありません

オプション
- 固定用のバンドなど
- （用途に合わせて分岐済みの）Grooveケーブル

# 必要環境

- Python 3.5以上 (Firmware書き込み用)
	- esptool
	- pyserial
- Steam VR
- [Virtual Motion Tracker](https://github.com/gpsnmeajp/VirtualMotionTracker)

推奨
- [Joint Tracker Manager](https://github.com/ixsiid/JointTrackerManager)を利用するとセットアップに便利です

# システム構成

![Joint Tracker System](./docs/images/JointTrackerSystem.png)

Parentデバイス(M5 Stack)は、複数のWorkerデバイス(M5 Atom)とI2Cにより接続されます

Workerデバイスは、IMUセンサーの値から姿勢を計算（Madgwick Filterを実装）して、Parentデバイスに送出します

Parentデバイスは、WiFiを利用しOSCプロトコルで、PC側にセットアップされたVirtual Motion Trackerと通信を行います

Virtual Motion Trackerは受信データを既存のトラッカーに**JOINT**させることで、Steam VR上でトラッカーが追加されたように認識します

# セットアップ

[Joint Tracker Manager](https://github.com/ixsiid/JointTrackerManager)を利用するとセットアップに便利です

以下は、手動でセットアップする場合の詳細な手順です

Parent側デバイス、Worker側デバイスにReleaseタグからFirmwareを取得して書き込む

Parent側の設定項目
- WiFi SSID/パスワード
- データ送信先ホストのIP Address:Port
- トラッキングさせる関節のボーン構造

Worker側の設定項目
- Parent側と通信識別用のアドレス

Parent側の設定はUSB UARTによるバイナリデータの通信によって行い
Worker側の設定は、同じくUSB UARTによるテキストデータ通信によって行います


## Parent側WiFiの設定用データ構造

```cpp:WiFi Configure Data
struct {
	size_t data_length = 72;
	uint32_t command = 0x128422fd;
	char ssid[32];
	char passphrase[32];
}
```

## Parent側データ送信先ホストの設定用データ構造

```cpp:WiFi Configure Data
struct {
	size_t data_length = 16;
	uint32_t command = 0x431fac89;
	uint8_t ip[4];
	uint16_t port;
	uint16_t reserved = 0x0000;
}
```

ipは、例えば*192.168.0.1*に送信したいときは、`ip = {192, 168, 0, 1};`となります

## Parent側ボーン構造設定

ボーン構造はデバイス側IMUの座標系で定義し、
M5 Atom Matrixの場合、Z軸が重力方向となる左手系になります

XY軸方向はWorkerデバイスの起動時に定まりますが、キャリブレーション（後述）によって任意の方向に変えることができます

### トラッカーの種類

ボーン構造として、固定トラッカーと可動トラッカーの2種類を定義することが可能で
全てのトラッカーは、動作中のトラッカーに**Joint**させる必要があります

Joint先は、最低でも1つ以上の動作済みのトラッカ（VIVEコントローラーやHMDなど）が必要で、Joint済みのJointTrackerにチェイン接続を行うこともできます

### ボーン構造定義データ

ボーン構造を送信するまえに、固定・可動ボーンそれぞれの構成数を設定する必要があります

```cpp:Bone Count Configure Data
struct {
	size_t data_length = 12;
	uint32_t command = 0x38771d81;
	uint8_t fix_bone_count;
	uint8_t movable_count;
	uint16_t reserved = 0x0000;
}
```

ボーンの個数を設定後、ボーン構造を一つずつ設定します

```cpp:Bone Structure Data
struct {
	size_t data_length = 60;
	uint32_t command = (固定トラッカー) 0xf729adc8 or (可動トラッカー)0xab8cf912;
	char root_serial[20];
	uint8_t reserved = 0x00;
	uint8_t bone_index;
	uint8_t address;
	uint8_t tracker_index;
	Vector3 bone; // float[3] x,y,z
	Quaternion rotation; // float[4] x,y,z,w
}
```

ここで、各プロパティは次のとおりです

|プロパティ名|意味|
|---|---|
|root_serial|ジョイント先トラッカーのシリアルナンバー|
|bone_index|0から順に重複しない番号|
|address|対応するWorker側のアドレス（固定トラッカーの場合無視される）|
|tracker_index|Virtual Motion Trackerに認識させるトラッカー番号|
|bone|ジョイント先から、IMU座標系で記したボーンの長さ（≒位置のオフセット）|
|rotation|ジョイント先から、IMU座標系で記した回転|


なお、Virtual Motion Trackerに登録したトラッカーは、`VMT_{トラッカー番号}`というシリアルナンバーが割り当てられる


### ボーン構造定義例

腰にVIVE Trackerを装着し、下半身（両膝、両足首）をJointTrackerで定義する例

<img src="./docs/images/BoneStructure.png" alt="ボーン構造の例">

まず、最初に注意するのは先述の通りボーンの定義はIMUの座標系で行います。
すなわちZ軸が重力方向とは逆方向の左手系です。
XYはキャリブレーションによって調整できるので任意の方向で構いません。

この例では、基準となる腰のVIVE Trackerから、最初の関節である左右の股関節までを**固定ボーン**として定義します

この時、トラッカーの位置は常に**関節**になるようにします

```cpp:股関節
right_hip = {
	data_length = 60;
	command = 0xf729adc8; // 固定トラッカー
	root_serial[20] = "LHR-12345678";
	reserved = 0x00;
	bone_index = 0;
	address = 0; // 固定トラッカーでは利用しない
	tracker_index = 10;
	bone = {0.2f, -0.3f, -0.15f};
	rotation = {0f, 0f, 1f, 0f}; // Z軸周りに180度回転
};

left_hip = {
	data_length = 60;
	command = 0xf729adc8; // 固定トラッカー
	root_serial[20] = "LHR-12345678";
	reserved = 0x00;
	bone_index = 1; // 固定トラッカー内で連番
	address = 0;
	tracker_index = 15;
	bone = {-0.2f, -0.3f, -0.15f};
	rotation = {0f, 0f, 1f, 0f}; // Z軸周りに180度回転
};
```

次に、股関節から次の関節になる膝までを**可動ボーン**として定義します

可動ボーン（太もものどこか）には、Workerデバイスを装着する必要があります

```cpp:膝
right_nee = {
	data_length = 60;
	command = 0xab8cf912; // 可動トラッカー
	root_serial[20] = "VMT_10"; // Virtual Motion Trackerによって振られる、右股関節（tracker_index = 10）のシリアルナンバー
	reserved = 0x00;
	bone_index = 0; // 可動トラッカー内で連番
	address = 11;
	tracker_index = 11; // 可動・固定含めて重複禁止
	bone = {0f, 0f, -0.4f};
	rotation = {0f, 0f, 0f, 1f};
};

leftt_nee = {
	data_length = 60;
	command = 0xab8cf912;
	root_serial[20] = "VMT_15";
	reserved = 0x00;
	bone_index = 1;
	address = 16;
	tracker_index = 16; // 可動・固定含めて重複禁止
	bone = {0f, 0f, -0.4f};
	rotation = {0f, 0f, 0f, 1f};
};
```

続いて、膝から足首までを同様に定義します

```cpp:膝
right_ankle = {
	data_length = 60;
	command = 0xab8cf912;
	root_serial[20] = "VMT_11";
	reserved = 0x00;
	bone_index = 0;
	address = 12;
	tracker_index = 12;
	bone = {0f, 0f, -0.35f};
	rotation = {0f, 0f, 0f, 1f};
};

leftt_nee = {
	data_length = 60;
	command = 0xab8cf912;
	root_serial[20] = "VMT_16";
	reserved = 0x00;
	bone_index = 1;
	address = 17;
	tracker_index = 17;
	bone = {0f, 0f, -0.35f};
	rotation = {0f, 0f, 0f, 1f};
};
```

ボーン構造の定義は以上で終了です

最後にボーンの個数についても一応記載します

```cpp:ボーンの個数
{
	data_length = 12;
	command = 0x38771d81;
	fix_bone_count = 2;
	movable_count = 4;
	reserved = 0x0000;
};
```

## Parent側セットアップ例

下記の内｛このように括弧で囲われた値｝はバイナリデータで送られていると考えてください

各設定コマンドは1つのコマンド毎にひとまとめに送り、続くコマンドの前に1秒程度のWaitをかけることで、安定して処理されます

```bash
# Firmware書き込み、実際の引数は各環境に合わせて変更してください
python -m esptool --chip esp32--port COM3 --baud 460800 --before default_reset --after hard_reset write_flash -z --flash_mode dio --flash_freq 40m --flash_size detect 0x1000 ./bootloader_dio_40m.bin 0x8000 ./partitions.bin 0xe000 ./boot_app0.bin 0x10000 ./firmware.bin

(中略)

  Writing at 0x00078000... (100 %)
  Wrote 783888 bytes (441436 compressed) at 0x00010000 in 11.3 seconds (effective 555.5 kbit/s)...
  Hash of data verified.

  Leaving...
  Hard resetting via RTS pin...
# Firmware書き込み完了

# UART接続（疑似クライアントソフトで記載しています）
serial console --port COM3 --baud_rate 115200
  >
# WiFi
{72, 0x128422fd, "ssid", "password"}
  >
# Host
{16, 0x431fac89, 192, 168, 0, 1, 39570, 0}
  >
# Bone count
{12, 0x38771d81, 2, 4, 0}
  >
# Bone structures
{60, 0xf729adc8, "LHB-12345678", 0, 0,  0, 10,  0.2f, -0.3f, -0.15f, 0.0f, 0.0f, 1.0f, 0.0f}
{60, 0xf729adc8, "LHB-12345678", 0, 1,  0, 15, -0.2f,  0.3f, -0.15f, 0.0f, 0.0f, 1.0f, 0.0f}
{60, 0xab8cf912, "VMT_10",       0, 0, 11, 11,  0.0f,  0.0f, -0.4f,  0.0f, 0.0f, 0.0f, 1.0f}
{60, 0xab8cf912, "VMT_11",       0, 1, 12, 12,  0.0f,  0.0f, -0.4f,  0.0f, 0.0f, 0.0f, 1.0f}
{60, 0xab8cf912, "VMT_15",       0, 2, 16, 16,  0.0f,  0.0f, -0.35f, 0.0f, 0.0f, 0.0f, 1.0f}
{60, 0xab8cf912, "VMT_16",       0, 3, 17, 17,  0.0f,  0.0f, -0.35f, 0.0f, 0.0f, 0.0f, 1.0f}
# 再起動
{8, 0xff8d91a8}
```

## Worker側の設定

Worker側はFirmwareの書き込み後、アドレスをASCII文字列で送信して完了です

```bash
# Firmware書き込み、実際の引数は各環境に合わせて変更してください
python -m esptool --chip esp32--port COM3 --baud 1500000 --before default_reset --after hard_reset write_flash -z --flash_mode dio --flash_freq 40m --flash_size detect 0x1000 ./bootloader.bin 0x8000 ./partitions.bin 0x10000 ./firmware.bin

(中略)

  Writing at 0x00078000... (100 %)
  Wrote 783888 bytes (441436 compressed) at 0x00010000 in 11.3 seconds (effective 555.5 kbit/s)...
  Hash of data verified.

  Leaving...
  Hard resetting via RTS pin...
# Firmware書き込み完了

# UART接続
serial console --port COM3 --baud_rate 115200
# Workerアドレス設定（自動で再起動されます）
13(\r)
```

Worker側はLED Matrixに現在のアドレス値が表示されます

<img src="./docs/images/WorkerZeroBiasMeasuring.png" alt="ゼロバイアス測定中" width="140pt" />
<img src="./docs/images/WorkerZeroBiasMeasured.png" alt="ゼロバイアス測定済み" width="120pt" />

# 使い方

M5 Stackの3つのボタンを使います

![M5 Stack ボタン配置](./docs/images/M5StackButton.png)

|ボタン|機能|
|---|---|
|Button A|現在のセンサー値をボーン構造の初期値として定義|
|Button B|現在のセンサー値をSteam VR空間で正面({0, 0, -1})として向きをキャリブレーション|
|Button C|トラッキングデータ送信の有効/無効切り替え（起動時は無効）|


## 利用時の流れ

- Worker側がジャイロのゼロバイアス測定が終了するのを待つ
  - ゼロバイアス測定が終了するとLEDマトリクスが緑の表示に変わります
- **Button C**を押し、トラッキングデータの送信を有効化する
- ボーン構造で定義された初期姿勢で、**Button A**を押す
  - この時点で、Virtual Motion Trackerが目的の位置にあるはずです
  - ただし、Workerを動かしても、向きが異なる方向に動きます
- 全てのWorkerを正面に向けて動かして、**Button B**を押す
  - この時点で、XY平面の向きが補正されて正常なトラッキングが開始されます

ToDo 動画

# ToDo

- BLE対応した独立動作型のWorkerを作成する
- トラッキングポジションのプルプル抑制でダンパー追加（Parent? Worker?） ToDo Issuesあげる

