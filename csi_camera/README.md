# CSI-Camera ROS2 nodes

Implementation of ROS2 nodes to publish CSI camera images from Nvidia Jetson as `sensor_msgs/CompressedImage` based on https://github.com/JetsonHacksNano/CSI-Camera.

## Usage

```ros2 run csi_camera simple_camera # to publish images from sensor_id=0
# or
ros2 run csi_camera dual_camera # to publish images from both cameras```

To see the data:
```ros2 topic echo /image/left/image_compressed```
