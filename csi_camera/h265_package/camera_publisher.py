#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import ByteMultiArray
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst
import subprocess
import threading

# Define the gstreamer_pipeline1 function here

def gstreamer_pipeline1(sensor_id=0, capture_width=1920, capture_height=1080, framerate=30, bitrate=8000000, flip_method=0):
    return (
        f"nvarguscamerasrc sensor-id={sensor_id} ! "
        f"video/x-raw(memory:NVMM), width=(int){capture_width}, height=(int){capture_height}, format=(string)NV12, framerate=(fraction){framerate}/1 ! "
        f"nvvidconv flip-method={flip_method} ! "
        "video/x-raw(memory:NVMM), format=(string)NV12 ! "
        f"nvv4l2h265enc bitrate={bitrate} ! h265parse ! "
        "fdsink"
    )


class CameraNode(Node):
    def __init__(self, **kwargs):
        super().__init__("camera_node")
        self.image_topic_ = self.declare_parameter("image_topic", "/image/encoded").value
        self.image_publisher_ = self.create_publisher(ByteMultiArray, self.image_topic_, 1)

        # GStreamer setup for pipeline1
        Gst.init(None)
        self.pipeline1 = Gst.parse_launch(gstreamer_pipeline1())
        self.fdsink1 = self.pipeline1.get_by_name('fdsink')
        self.start_gstreamer_subprocess()

        self.total_data_size = 0
        self.message_count = 0

    def start_gstreamer_subprocess(self):
        cmd = ["gst-launch-1.0"] + gstreamer_pipeline1().split()
        self.get_logger().info("Starting GStreamer subprocess with command: " + ' '.join(cmd))
        self.gst_process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, bufsize=0)
        threading.Thread(target=self.read_gstreamer_output, daemon=True).start()

    def read_gstreamer_output(self):
        while self.gst_process.poll() is None:
            data = self.gst_process.stdout.read(1000000)
            if data:
                # Convert the data into a list of bytes
                msg = ByteMultiArray()
                msg.data = [bytes([b]) for b in data]
                self.image_publisher_.publish(msg)
                data_size = len(msg.data)
                self.get_logger().info(f"Published data of size {data_size} bytes")
                self.total_data_size += len(msg.data)
                self.message_count += 1
                print(f"\nTotal data size transmitted: {round(self.total_data_size / (1024 * 1024),2)} MB")
                print(f"Total number of messages transmitted: {self.message_count}")


    def close_gstreamer(self):
        if self.pipeline1:
            self.get_logger().info("Stopping GStreamer pipeline1")
            self.pipeline1.set_state(Gst.State.NULL)

def main(args=None):
    rclpy.init(args=args)
    camera_node = CameraNode()
    rclpy.spin(camera_node)
    camera_node.close_gstreamer()
    camera_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
