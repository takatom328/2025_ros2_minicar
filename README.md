
すべてのROS2ノードを一括起動するbashファイルを作成しました！ 

作成したファイル: 
 - launch_all_nodes.sh 起動されるノード:
 - joy_to_servo - ジョイスティック→サーボ制御
 - joy_to_cmd - ジョイスティック→コマンド変換
 - pca9685_steer_node - PCA9685ステアリング制御
 - ultrasonic_node - 超音波センサー
 - range_kalman_filter - 距離センサーカルマンフィルター
f710_cmd_node - F710コントローラー制御


使い方:
cd /home/ubuntu/data/ros2_ws/src
./launch_all_nodes.sh

終了するには Ctrl+C を押してください。すべてのノードが自動的に終了します。