#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import ByteMultiArray
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst
import subprocess
import threading
import numpy as np
import ctypes
x11 = ctypes.cdll.LoadLibrary('libX11.so')
x11.XInitThreads()

def gstreamer_pipeline1(sensor_id=0, capture_width=3840, capture_height=2160, framerate=30, bitrate=8000000, flip_method=0):
    return (
        f"nvarguscamerasrc sensor-id={sensor_id} ! "
        f"video/x-raw(memory:NVMM), width=(int){capture_width}, height=(int){capture_height}, format=(string)NV12, framerate=(fraction){framerate}/1 ! "
        f"nvvidconv flip-method={flip_method} ! "
        "video/x-raw(memory:NVMM), format=(string)NV12 ! "
        f"nvv4l2h265enc bitrate={bitrate} ! h265parse ! "
        "fdsink"
    )

class CameraNode(Node):
    def __init__(self, camera_id, **kwargs):
        super().__init__(f"camera_node_{camera_id}")
        self.camera_id = camera_id
        if camera_id == 0:
            self.image_topic_ = self.declare_parameter("image_topic", f"/image/front/image_compressed").value
        else:
            self.image_topic_ = self.declare_parameter("image_topic", f"/image/back/image_compressed").value
        self.image_publisher_ = self.create_publisher(ByteMultiArray, self.image_topic_, 1)

        # GStreamer setup for pipeline1
        Gst.init(None)
        self.pipeline1 = Gst.parse_launch(gstreamer_pipeline1(sensor_id=camera_id))
        self.fdsink1 = self.pipeline1.get_by_name('fdsink')
        self.start_gstreamer_subprocess1()


    def start_gstreamer_subprocess1(self):
        cmd = ["gst-launch-1.0"] + gstreamer_pipeline1(sensor_id=self.camera_id).split()
        self.get_logger().info(f"Starting GStreamer subprocess for camera {self.camera_id} with command: " + ' '.join(cmd))
        self.gst_process1 = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, bufsize=0)
        threading.Thread(target=self.read_gstreamer_output1, daemon=True).start()

    def read_gstreamer_output1(self):
        while self.gst_process1.poll() is None:
            data = self.gst_process1.stdout.read(1000000)
            if data:
                # Convert the data into a list of bytes
                msg = ByteMultiArray()
                msg.data = [bytes([b]) for b in data]
                self.image_publisher_.publish(msg)
                self.get_logger().info(f"Image from Camera: {self.camera_id} published")

            else:
                err = self.gst_process1.stderr.readline()
                if err:
                    self.get_logger().error(f"GStreamer Error: {err.decode()}")


    def close_gstreamer(self):
        if self.pipeline1:
            self.get_logger().info(f"Stopping GStreamer pipeline1 for camera {self.camera_id}")
            self.pipeline1.set_state(Gst.State.NULL)


def main(args=None):
    rclpy.init(args=args)
    camera_node1 = CameraNode(camera_id=0)
    camera_node2 = CameraNode(camera_id=1)

    rclpy.spin(camera_node1)
    rclpy.spin(camera_node2)

    camera_node1.close_gstreamer()
    camera_node2.close_gstreamer()

    camera_node1.destroy_node()
    camera_node2.destroy_node()

    rclpy.shutdown()

if __name__ == "__main__":
    main()
