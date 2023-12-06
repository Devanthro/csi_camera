import rclpy
from rclpy.node import Node
from std_msgs.msg import ByteMultiArray
import gi
import time
import hashlib
import argparse
import sys
gi.require_version('Gst', '1.0')
from gi.repository import Gst

def gstreamer_pipeline(capture_width=4032, capture_height=3040, framerate=30, flip_method=0):
    return (
        "nvarguscamerasrc ! "
        "video/x-raw(memory:NVMM),width=%d,height=%d,format=NV12,framerate=%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw(memory:NVMM),format=I420 ! "
        "nvv4l2h265enc ! "
        "h265parse ! "
        "appsink emit-signals=True name=appsink"
        % (capture_width, capture_height, framerate, flip_method)
    )

class CameraNode(Node):
    def __init__(self, fps, **kwargs):
        super().__init__("camera_node")
        self.image_topic_ = self.declare_parameter("image_topic", "/image/encoded").value
        self.image_publisher_ = self.create_publisher(ByteMultiArray, self.image_topic_, 1)
        self.whole_size = 0

        # GStreamer setup
        Gst.init(None)
        self.pipeline = Gst.parse_launch(gstreamer_pipeline(framerate = fps))
        self.appsink = self.pipeline.get_by_name('appsink')
        self.appsink.connect('new-sample', self.on_new_sample)
        self.pipeline.set_state(Gst.State.PLAYING)

        # Timing variables
        self.start_time = None
        self.frame_count = 0
        self.total_bytes = 0
        self.start_time_1000 = time.time()
        self.end_time_1000 = None

    def on_new_sample(self, appsink):
       sample = appsink.emit('pull-sample')
       if sample:
           self.start_time = time.time()
           end_time = None
           buffer = sample.get_buffer()
           (result, map_info) = buffer.map(Gst.MapFlags.READ)
           if result:
               try:
                    data = map_info.data
                    import pdb; pdb.set_trace()
                    msg = ByteMultiArray()
                    msg.data = [bytes([byte]) for byte in data]
                    self.image_publisher_.publish(msg)
                    self.total_bytes += len(msg.data)
                    self.get_logger().info("Published frame successfully")

                    # Log data size and a part of its content
                    self.get_logger().info(f"Data size: {len(data)} bytes")
                    self.whole_size += len(data)
                    print(self.whole_size)
                    self.get_logger().info(f"Data sample: {data[:10]}")  # First 10 bytes

                    # Log checksum or hash of the data
                    data_hash = hashlib.md5(data).hexdigest()
                    self.get_logger().info(f"Data MD5 hash: {data_hash}")
                    print("Total Bytes: " + str(self.total_bytes))

                    end_time = time.time()
               except Exception as e:
                   self.get_logger().error(f"Error occurred during publishing: {e}")
               finally:
                   buffer.unmap(map_info)
                   elapsed_time = end_time - self.start_time
                   #self.get_logger().info(f"Frame {self.frame_count} process and publish time: {elapsed_time:.6f} seconds")
           if self.frame_count == 1000:
               self.end_time_1000 = time.time()
               delta = self.end_time_1000 - self.start_time_1000
               print("Time for 1000 Frames:" + str(round(delta,6)))
           self.frame_count += 1


    def close_gstreamer(self):
        if self.pipeline:
            self.get_logger().info("Stopping GStreamer pipeline")
            self.pipeline.set_state(Gst.State.NULL)

def main(args=None):
    parser = argparse.ArgumentParser(description="Camera node with adjustable FPS")
    parser.add_argument('--fps', type=int, default=21, help='Frames per second')
    args = parser.parse_args()

    # Verwenden von sys.argv statt args für rclpy.init
    rclpy.init(args=sys.argv)

    camera_node = CameraNode(fps=args.fps)
    rclpy.spin(camera_node)

    camera_node.close_videocapture()
    camera_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()