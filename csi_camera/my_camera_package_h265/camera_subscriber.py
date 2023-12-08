#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import ByteMultiArray
import numpy as np
import cv2

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            ByteMultiArray,
            '/image/encoded',
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning
        self.decoder = cv2.createVideoWriter('appsrc ! video/x-h265, stream-format=byte-stream ! h265parse ! avdec_h265 ! videoconvert ! appsink', cv2.CAP_GSTREAMER, 0, 20.0, (640, 480), True)

    def listener_callback(self, msg):
        data = bytearray(msg.data)
        frame = np.array(data, dtype=np.uint8)
        if self.decoder.isOpened():
            ret, img = self.decoder.read(frame)
            if ret:
                cv2.imshow('Frame', img)
                cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)

    image_subscriber = ImageSubscriber()

    try:
        rclpy.spin(image_subscriber)
    except KeyboardInterrupt:
        pass

    image_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
