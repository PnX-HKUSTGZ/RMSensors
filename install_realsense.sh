#!/bin/bash

# 安装依赖
sudo apt-get update
# 选作
# sudo apt-get upgrade 
sudo apt-get install -y libssl-dev libusb-1.0-0-dev libudev-dev pkg-config libgtk-3-dev
sudo apt-get install -y git wget cmake build-essential
sudo apt-get install -y libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev at

# install librealsense2
REALSENSE_DIR="$HOME/librealsense"

cd ~
if [ -d "$REALSENSE_DIR" ]; then
	read -r -p "检测到已存在的 librealsense 目录，是否覆盖安装？[y/N] " overwrite_answer
	case "$overwrite_answer" in
		y|Y)
			echo "执行覆盖安装。"
			rm -rf "$REALSENSE_DIR"
			;;
		*)
			echo "已取消安装。"
			exit 0
			;;
	esac
fi

git clone https://github.com/realsenseai/librealsense.git "$REALSENSE_DIR"
cd "$REALSENSE_DIR"
./scripts/setup_udev_rules.sh
./scripts/patch-realsense-ubuntu-lts-hwe.sh

mkdir build && cd build

cmake ../ -DBUILD_EXAMPLES=true
sudo make uninstall || true
make clean
make -j 20
sudo make install