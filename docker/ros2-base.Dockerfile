FROM ubuntu:22.04

RUN apt-get update && DEBIAN_FRONTEND=noninteractive TZ=Etc/UTC apt-get -y install tzdata

RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    wget

RUN apt-get update && apt-get install -y \
    python3 \
    python3-setuptools \
    python3-wheel \
    libpython3-dev

WORKDIR /root

# ============ ROS ============
# Set locale
RUN apt-get update && apt-get install -y locales \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && export LANG=en_US.UTF-8

# Setup Sources
RUN apt-get update && apt-get install -y  software-properties-common \
    && add-apt-repository -y universe

RUN apt-get update && apt-get install -y curl \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS
RUN apt-get update && apt-get upgrade -y \
    && apt-get install -y ros-humble-ros-base ros-dev-tools
RUN rosdep init && rosdep update
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Setup workspace
RUN mkdir -p ~/ros2_ws/src
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && cd ~/ros2_ws && colcon build --symlink-install"
RUN echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc