#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import ByteMultiArray
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst
import numpy as np
import cv2
import ctypes

# Initialize X11 threading for GUI compatibility
x11 = ctypes.cdll.LoadLibrary('libX11.so')
x11.XInitThreads()

def gstreamer_pipeline2(display_width=1920, display_height=1080):
    return (
        f"appsrc name=appsrc ! "
        "h265parse ! "
        "nvv4l2decoder ! "
        "nvvidconv ! "
        f"video/x-raw, width=(int){display_width}, height=(int){display_height}, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! "
        "appsink emit-signals=True name=appsink"
    )

class CameraSubscriberNode(Node):
    def __init__(self, cam_id, topic_name, window_name):
        super().__init__(f"camera_subscriber_node_{cam_id}")
        self.window_name = window_name
        self.subscription = self.create_subscription(ByteMultiArray, topic_name, self.listener_callback, 10)
        self.camera_id = cam_id

        # GStreamer setup
        Gst.init(None)
        self.pipeline = Gst.parse_launch(gstreamer_pipeline2())
        self.appsrc = self.pipeline.get_by_name('appsrc')
        self.appsink = self.pipeline.get_by_name('appsink')
        self.appsink.set_property('emit-signals', True)
        self.appsink.connect('new-sample', self.on_new_sample)
        self.pipeline.set_state(Gst.State.PLAYING)

    def listener_callback(self, msg):
        reconstructed_data = b''.join(msg.data)
        buffer = Gst.Buffer.new_wrapped(reconstructed_data)        
        self.appsrc.emit('push-buffer', buffer)

    def on_new_sample(self, appsink):
        sample = appsink.emit('pull-sample')
        self.get_logger().info(f"Image from Camera: {self.camera_id} received")

        if sample:
            buffer = sample.get_buffer()
            caps = sample.get_caps()
            structure = caps.get_structure(0)
            width = structure.get_value('width')
            height = structure.get_value('height')

            # Extract and display the frame
            buffer = buffer.extract_dup(0, buffer.get_size())
            frame = np.ndarray((height, width, 3), buffer=buffer, dtype=np.uint8)
            cv2.imshow(self.window_name, frame)
            cv2.waitKey(1)

            return Gst.FlowReturn.OK

    def close_pipeline(self):
        if self.pipeline:
            self.pipeline.set_state(Gst.State.NULL)

def main(args=None):
    rclpy.init(args=args)

    front_image_topic = "/image/front/image_compressed"
    back_image_topic = "/image/back/image_compressed"

    camera_subscriber_node_front = CameraSubscriberNode(cam_id=0, topic_name=front_image_topic, window_name="Front Camera")
    camera_subscriber_node_back = CameraSubscriberNode(cam_id=1, topic_name=back_image_topic, window_name="Back Camera")

    executor = MultiThreadedExecutor()
    executor.add_node(camera_subscriber_node_front)
    executor.add_node(camera_subscriber_node_back)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        camera_subscriber_node_front.close_pipeline()
        camera_subscriber_node_back.close_pipeline()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
