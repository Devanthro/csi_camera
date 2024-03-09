#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import ByteMultiArray
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst
import numpy as np
import cv2

def gstreamer_pipeline2(display_width=1920, display_height=1080):
    return (
        "appsrc name=appsrc ! "
        "h264parse ! "
        "nvv4l2decoder ! "
        "nvvidconv ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! "
        "appsink emit-signals=True name=appsink2"
        % (
            display_width,
            display_height,
        )
    )


class DisplayNode(Node):
    def __init__(self):
        super().__init__("display_node")
        self.image_topic_ = self.declare_parameter("image_topic", "/image/encoded_").value
        self.image_subscription_ = self.create_subscription(ByteMultiArray, self.image_topic_, self.image_callback, 1)

        # GStreamer setup for pipeline2
        Gst.init(None)
        self.pipeline2 = Gst.parse_launch(gstreamer_pipeline2())
        self.appsrc2 = self.pipeline2.get_by_name('appsrc')
        self.appsink2 = self.pipeline2.get_by_name('appsink2')
        self.appsink2.set_property('emit-signals', True)
        self.appsink2.connect('new-sample', self.on_new_sample2)
        self.pipeline2.set_state(Gst.State.PLAYING)

    def image_callback(self, msg):
        # Reconstruct the data from the list of bytes
        reconstructed_data = b''.join(msg.data)

        # Create a GStreamer buffer from the reconstructed data
        buffer = Gst.Buffer.new_wrapped(reconstructed_data)
        self.appsrc2.emit('push-buffer', buffer)

    def on_new_sample2(self, appsink):
        sample = appsink.emit('pull-sample')
        if sample:
            buffer = sample.get_buffer()
            caps = sample.get_caps()
            structure = caps.get_structure(0)
            width = structure.get_value('width')
            height = structure.get_value('height')

            # Extract the frame
            buffer = buffer.extract_dup(0, buffer.get_size())
            frame = np.ndarray((height, width, 3), buffer=buffer, dtype=np.uint8)

            # Display the frame
            cv2.imshow("Captured Frame", frame)
            cv2.waitKey(1)

            return Gst.FlowReturn.OK

def main(args=None):
    rclpy.init(args=args)
    display_node = DisplayNode()
    rclpy.spin(display_node)

    display_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
