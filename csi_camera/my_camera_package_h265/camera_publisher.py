import rclpy
from rclpy.node import Node
from std_msgs.msg import ByteMultiArray
import subprocess
import threading
import csv
import time

def gstreamer_command(capture_width=3840, capture_height=2160, framerate=30, flip_method=0, bitrate = 80000):
    return [
        "gst-launch-1.0",
        "nvarguscamerasrc", 
        "!", "video/x-raw(memory:NVMM),width=%d,height=%d,format=NV12,framerate=%d/1" % (capture_width, capture_height, framerate),
        "!", "nvvidconv", "flip-method=%d" % flip_method,
        "!", "video/x-raw(memory:NVMM),format=I420",
        "!", "nvv4l2h265enc",
        "!", "h265parse",
        "!", "fdsink"
    ]


class CameraNode(Node):
    def __init__(self, **kwargs):
        super().__init__("camera_node")
        self.image_topic_ = self.declare_parameter("image_topic", "/image/encoded").value
        self.image_publisher_ = self.create_publisher(ByteMultiArray, self.image_topic_, 1)
        self.gst_process = None
        self.start_gstreamer()

        # CSV file setup
        self.csv_filename = 'publish_times.csv'
        self.csv_file = open(self.csv_filename, mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['Process and Publish Time (s)'])
        self.total_bytes = 0

        # Timing variables
        self.start_time = None
        self.frame_count = 0
        self.start_time_1000 = time.time()
        self.end_time_1000 = None

    def start_gstreamer(self):
        cmd = gstreamer_command()
        self.get_logger().info("Starting GStreamer with command: " + ' '.join(cmd))
        self.gst_process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, bufsize=0)
        threading.Thread(target=self.read_gstreamer_output, daemon=True).start()

    def read_gstreamer_output(self):
        while self.gst_process.poll() is None:
            self.start_time = time.time()
            data = self.gst_process.stdout.read(80000)
            if data:
                self.frame_count += 1
                msg = ByteMultiArray()
                msg.data = [bytes([b]) for b in data]
                self.image_publisher_.publish(msg)
                # End timing here
                self.total_bytes += len(msg.data)
                end_time = time.time()
                if self.frame_count == 1000:
                    self.end_time_1000 = time.time()
                    delta = self.end_time_1000 - self.start_time_1000
                    print("Time for 1000 Frames:" + str(round(delta,6)))

                if self.start_time is not None:
                    elapsed_time = end_time - self.start_time  # Calculate the combined time
                    self.csv_writer.writerow([elapsed_time])
                    self.get_logger().info(f"Frame {self.frame_count} process and publish time: {elapsed_time:.6f} seconds")
                else:
                    self.get_logger().error("Start time is None, elapsed time cannot be calculated.")
            else:
                err = self.gst_process.stderr.readline()
                if err:
                    self.get_logger().error(f"GStreamer Error: {err.decode()}")

    def close_gstreamer(self):
        if self.gst_process:
            self.get_logger().info("Terminating GStreamer process")
            self.gst_process.terminate()

def main(args=None):
    rclpy.init(args=args)
    camera_node = CameraNode()
    rclpy.spin(camera_node)

    camera_node.close_gstreamer()
    camera_node.csv_file.close()  # Close the CSV file
    camera_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
