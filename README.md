# LowerTracker

Vive Trackerから得られた腰のトラッキング情報に、膝・足首の姿勢を基にトラッキング情報を付加し
Virtual Motion TrackerによってSteamVR上にトラッキングポイントを追加するデバイス


# 開発メモ
基本的にはIMUの座標系で処理をする
X, Y→起動時の向き、Z→重力方向
※磁気センサーがついていれば、XYは北方向が基準になる

データ送信時に、SteamVR座標系（またはUnity座標系）に変換して送信する


# セットアップ
esptool

https://github.com/espressif/esptool/releases/



python3.7

python -m pip install --upgrade pip
pip install --upgrade esptool
python -m esptool --chip esp32 --port COM6 --baud 1500000 --before default_reset --after hard_reset write_flash -z --flash_mode dio --flash_freq 40m --flash_size detect 0x1000 C:\\Users\\tal\\Documents\\PlatformIO\\Projects\\LowerTracker\\.pio\\build\\slave\\bootloader.bin 0x8000 C:\\Users\\tal\\Documents\\PlatformIO\\Projects\\LowerTracker\\.pio\\build\\slave\\partitions.bin 0x10000 C:\\Users\\tal\\Documents\\PlatformIO\\Projects\\LowerTracker\\.pio\\build\\slave\\firmware.bin

COM調べる


// master
['C:\\Users\\tal\\.platformio\\packages\\tool-esptoolpy\\esptool.py', '--chip', 'esp32', '--port', 'COM3', '--baud', '460800', '--before', 'default_reset', '--after', 'hard_reset', 'write_flash', '-z', '--flash_mode', 'dio', '--flash_freq', '40m', '--flash_size', 'detect', '0x1000', 'C:\\Users\\tal\\.platformio\\packages\\framework-arduinoespressif32\\tools\\sdk\\bin\\bootloader_dio_40m.bin', '0x8000', 'C:\\Users\\tal\\Documents\\PlatformIO\\Projects\\LowerTracker\\.pio\\build\\master\\partitions.bin', '0xe000', 'C:\\Users\\tal\\.platformio\\packages\\framework-arduinoespressif32\\tools\\partitions\\boot_app0.bin', '0x10000', '.pio\\build\\master\\firmware.bin']