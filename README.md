# CSI-Camera ROS2 nodes

Implementation of ROS2 nodes to publish CSI camera images from Nvidia Jetson as `sensor_msgs/CompressedImage` based on https://github.com/JetsonHacksNano/CSI-Camera.

## Usage

```bash
ros2 run csi_camera simple_camera # to publish images from sensor_id=0
# or
ros2 run csi_camera dual_camera # to publish images from both cameras
```

To see the data:
```bash
ros2 topic echo /image/left/image_compressed
```

## Docker setup
To grant access to CSI cameras inside a docker container running on Jetson Xavier, run docker with the following arguments:
```bash
 docker run --gpus all -e NVIDIA_REQUIRE_JETPACK="csv-mounts=all" --runtime nvidia --name YOUR_NAME -ti --privileged -v /dev:/dev -v /tmp/argus_socket:/tmp/argus_socket --network host YOUR_IMAGE_ID /bin/bash
 ```
 
 Docker used is based on https://github.com/dusty-nv/jetson-containers, specifically `foxy-ros-base-l4t-r32.7.1`.
