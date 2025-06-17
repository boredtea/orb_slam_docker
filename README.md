# ORB_SLAM3 Docker for RB5 (and RB3)
Credit: [https://github.com/ModernOctave/RB_Roboracer_Public.git](https://github.com/ModernOctave/RB_Roboracer_Public.git)

## Setup
Clone the repo
```bash
git clone https://github.com/boredtea/orb_slam_docker
```

Build the docker images
```bash
cd docker
make images
```

> [!IMPORTANT]
> If building on the RB3, you will need to have the `swap` partition enabled otherwise the build will fail due to insufficient memory.
> 
> To enable the `swap` partition, follow the instructions [here](https://www.digitalocean.com/community/tutorials/how-to-add-swap-space-on-ubuntu-22-04).
>
> We have confirmed that the build will succeed with a `swap` partition of 6GB but it may be possible to use a smaller size.

## Usage
### Running Monocular ORB_SLAM3
On host
```bash
docker run --ipc=host --pid=host --network host -it --rm --name orb_slam3 orb_slam3:humble 
```

In orb_slam3 container
```bash
ros2 run orb_slam3_rb5_ros2 Mono /root/ORB_SLAM3/Vocabulary/ORBvoc.txt /root/ORB_SLAM3/Examples_old/Monocular/EuRoC.yaml
```

### Running Laptop Camera Node
On host
```bash
docker run --network host --device /dev/video0 --device /dev/video1 --device /dev/media0 -v /run/udev:/run/udev:ro -it --rm --name ros2_humble ros2-base:humble
```
In ros2 container
```bash
sudo apt update && sudo apt install -y ros-humble-camera-ros
ros2 run camera_ros camera_node
```

## Notes
- Maintained by [Soumi Chakraborty](linkedin.com/in/soumi-chakraborty-a5365b1a7/).
- Refer to [https://github.com/boredtea/rf-data-collection](https://github.com/boredtea/rf-data-collection) for other utils to build a map using ORB SLAM3.