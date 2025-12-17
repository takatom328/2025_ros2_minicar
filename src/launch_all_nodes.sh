#!/bin/bash

# ROS2ノード一括起動スクリプト
# 使い方: ./launch_all_nodes.sh

echo "ROS2ワークスペースのセットアップ中..."
cd /home/ubuntu/data/ros2_ws

# ROS2環境をソース
source /opt/ros/humble/setup.bash

# 必要な依存関係をインストール
echo "必要な依存関係をインストール中..."
#sudo apt update -qq
sudo apt install -y ros-humble-ackermann-msgs 2>/dev/null || echo "ackermann-msgs は既にインストール済み"
pip install --user lgpio 2>/dev/null || echo "lgpio は既にインストール済み"
pip install --user adafruit-pca9685 2>/dev/null || echo "adafruit-pca9685 は既にインストール済み"
pip install --user adafruit-blinka 2>/dev/null || echo "adafruit-blinka は既にインストール済み"
pip install --user adafruit-circuitpython-vl53l0x 2>/dev/null || echo "adafruit-circuitpython-vl53l0x は既にインストール済み"

# ワークスペースをビルド
echo "ワークスペースをビルド中..."
colcon build --symlink-install

# ビルド結果をソース
source ./install/setup.bash

echo ""

echo "すべてのノードを起動します..."

# 各ノードをバックグラウンドで起動
echo "1. joy_to_servo ノードを起動中..."
ros2 run nullbot_teleop joy_to_servo &
PIDS[0]=$!

echo "2. joy_to_cmd ノードを起動中..."
ros2 run nullbot_teleop joy_to_cmd &
PIDS[1]=$!

echo "3. pca9685_steer_node ノードを起動中..."
ros2 run nullbot_actuators pca9685_steer_node &
PIDS[2]=$!

echo "5. range_kalman_filter ノードを起動中..."
ros2 run nullbot_sensors range_kalman_filter &
PIDS[3]=$!

echo "6. f710_cmd_node ノードを起動中..."
ros2 run nullbot_teleop_cmd f710_cmd_node &
PIDS[4]=$!

#echo "4. ultrasonic_node ノードを起動中..."
#ros2 run nullbot_sensors ultrasonic_node &
#PIDS[5]=$!

echo ""
echo "すべてのノードが起動しました!"
echo "終了するには Ctrl+C を押してください"
echo ""

# Ctrl+Cで全プロセスを終了
trap 'echo ""; echo "すべてのノードを終了中..."; kill ${PIDS[@]} 2>/dev/null; exit' INT

# すべてのプロセスが終了するまで待機
wait
