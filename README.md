# dynamixel_bilateral_contoroller


# 概要
本リポジトリは，cranex7を使ったバイラテラル制御を行うためのプログラムです．
Linux環境でのみ動作確認しています．

# インストール
## 環境の構築
環境を汚したくない場合は，以下のdocker containerを使用できます．
- https://github.com/HoneyMack/docker-deep-cuda-py3.git
    - ブランチ：develop_gl

## 依存関係のインストール

### dynamixelSDKのインストール

- https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/library_setup/c_linux/#c-linux

```bash
# 必要なパッケージのインストール
apt update
apt install gcc-multilib g++-multilib

# コンパイル
git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git
cd DynamixelSDK/c++/build/linux64/
make
make install
```

### boostのインストール

- https://www.baeldung.com/linux/boost-install-on-ubuntu

```bash
apt install libboost-all-dev
```

### opencvのインストール

- https://www.kkaneko.jp/tools/ubuntu/opencv.html

```bash
apt update
apt -y install libopencv-dev libopencv-core-dev libopencv-contrib-dev opencv-data
```

## dynamixel_bilateral_controllerのインストール

- https://github.com/HoneyMack/dynamixel_bilateral_controller

```bash
git clone https://github.com/HoneyMack/dynamixel_bilateral_controller.git controller
cd controller
cmake -B build # test用のプログラムをコンパイルしない場合は-DBUILD_TESTS=OFFをつける
# cmake -DCMAKE_BUILD_TYPE=Release -B build # リリースビルドしたい場合 
cmake --build build
```


# 実行手順
インストール&ビルドが適切に行えていれば，```controller/build/bin```以下に実行ファイルが生成されています．

## バイラテラル制御の実行
cranex7のバイラテラル制御を実行したい場合は，以下の実行ファイルを実行
```bash
./controller/build/bin/cranex7_bilateral
```