# CSI-Camera ROS2 nodes

Implementation of ROS2 nodes to publish CSI camera images from Nvidia Jetson as `sensor_msgs/CompressedImage` based on https://github.com/JetsonHacksNano/CSI-Camera.

## Usage

```bash
ros2 run csi_camera simple_camera # to publish images from sensor_id=0
# or
ros2 run csi_camera dual_camera # to publish images from both cameras
```

I added the additional ros2 humble hawksbill packages h264_package, h265_package and h265_dual_package. These make use of the H264 / H65 videocodec for hardware encoding. When you extract them to you your packages folder on your ros humble hawksbill directory, you are able to use them by executing the following steps:

1. First go to the directory 
```bash
cd ros2_humble
```

2. If you’re package hasn’t been build yet, do so
```bash
~/ros2_humble$ source colcon build --packages-select my_camera_package_h265 (or h264)
```

3. Then source it
```bash
~/ros2_humble$ source /opt/ros/humble/local_setup.bash
```

4. Now you open a new terminal which you also source. head within both terminals to src folder with

```bash
cd src
```

5. Then type in one terminal

```bash
ros2 run my_camera_package_h265 camera_subscriber
```

6. And in the other one

```bash
ros2 run my_camera_package_h265 camera_publisher
```


To see the data:
```bash
ros2 topic echo /image/left/image_compressed # sensor_id=0
ros2 topic echo /image/right/image_compressed
```

## Docker setup
To grant access to CSI cameras inside a docker container running on Jetson Xavier, run docker with the following arguments:
```bash
 docker run --gpus all -e NVIDIA_REQUIRE_JETPACK="csv-mounts=all" --runtime nvidia --name YOUR_NAME -ti --privileged -v /dev:/dev -v /tmp/argus_socket:/tmp/argus_socket --network host YOUR_IMAGE_ID /bin/bash
 ```
 
 Docker used is based on https://github.com/dusty-nv/jetson-containers, specifically `foxy-ros-base-l4t-r32.7.1`.
