#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import subprocess
import datetime
import signal
import os

class RosBagManagerNode(Node):
    def __init__(self):
        super().__init__('ros2_bag_manager_node')

        # パラメータ宣言・取得
        self.declare_parameter('output_dir', 'rosbag2_output')
        self.declare_parameter('all_topics', True)
        self.declare_parameter('topics', [])
        self.output_dir = self.get_parameter('output_dir').get_parameter_value().string_value
        self.all_topics = self.get_parameter('all_topics').get_parameter_value().bool_value
        self.topics = list(self.get_parameter('topics').get_parameter_value().string_array_value)

        # セッション用ディレクトリを作成（ノード起動時）
        now = datetime.datetime.now()
        session_ts = now.strftime('%Y%m%d_%H%M%S')
        self.session_dir = os.path.join(self.output_dir, session_ts)
        os.makedirs(self.session_dir, exist_ok=True)
        self.get_logger().info(f"セッションディレクトリ作成: {self.session_dir}")

        # トリガー購読
        self.trigger_sub = self.create_subscription(
            Bool,
            '/rosbag2_recorder/trigger',
            self.trigger_callback,
            10
        )

        self.recording_process = None

    def trigger_callback(self, msg: Bool):
        # true かつ未録画 => 録画開始
        if msg.data and self.recording_process is None:
            # 録画ごとのタイムスタンプ
            ts = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
            # <output_dir>/<session_ts>/<record_ts>/ に出力
            record_dir = os.path.join(self.session_dir, ts)
            os.makedirs(record_dir, exist_ok=True)

            cmd = ['ros2', 'bag', 'record']
            if self.all_topics:
                cmd.append('-a')
            elif self.topics:
                cmd.extend(self.topics)
            else:
                self.get_logger().warn("all_topics=false かつ topics=[] のため、何も録画しません")
            cmd.extend(['-o', record_dir])

            self.get_logger().info(f"録画開始: {' '.join(cmd)}")
            self.recording_process = subprocess.Popen(cmd, preexec_fn=os.setsid)

        # false かつ録画中 => 録画停止
        elif not msg.data and self.recording_process:
            self.get_logger().info("録画停止要求 — SIGINT を送信します")
            os.killpg(os.getpgid(self.recording_process.pid), signal.SIGINT)
            self.recording_process.wait()
            self.recording_process = None
            self.get_logger().info("録画を停止しました")

    def destroy_node(self):
        # ノード終了時に録画継続中なら停止
        if self.recording_process:
            self.get_logger().info("シャットダウン中に録画停止")
            os.killpg(os.getpgid(self.recording_process.pid), signal.SIGINT)
            self.recording_process.wait()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = RosBagManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
