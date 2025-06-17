FROM ros2-base:humble

# ============ ORB_SLAM3 ============
# Install Pangolin
# ADD https://github.com/stevenlovegrove/Pangolin.git#v0.9.2 Pangolin
RUN git clone --branch v0.9.2 https://github.com/stevenlovegrove/Pangolin.git
RUN apt-get update && apt-get install -y \
    libgl1-mesa-dev \
    libwayland-dev \
    libxkbcommon-dev \
    wayland-protocols \
    libegl1-mesa-dev \
    libc++-dev \
    libepoxy-dev \
    libglew-dev \
    libeigen3-dev \
    ninja-build \
    libjpeg-dev \
    libpng-dev \
    libavcodec-dev \
    libavutil-dev \
    libavformat-dev \
    libswscale-dev \
    libavdevice-dev
RUN cd Pangolin \
    && mkdir build \
    && cd build \
    && cmake .. \
    && make -j4 \
    && make install

# Install OpenCV & Eigen3
RUN apt-get update && apt-get install -y \
    libopencv-dev \
    libeigen3-dev

# Install ORB_SLAM3
# ADD https://github.com/UZ-SLAMLab/ORB_SLAM3.git#c++14_comp ORB_SLAM3
RUN git clone --branch c++14_comp https://github.com/UZ-SLAMLab/ORB_SLAM3.git ORB_SLAM3
RUN apt-get update && apt-get install -y \
    libboost-all-dev \
    libssl-dev
RUN cd ORB_SLAM3/Thirdparty/DBoW2 \
    && mkdir build \
    && cd build \
    && cmake .. -DCMAKE_BUILD_TYPE=Release \
    && make -j
RUN cd ORB_SLAM3/Thirdparty/g2o \
    && mkdir build \
    && cd build \
    && cmake .. -DCMAKE_BUILD_TYPE=Release \
    && make -j
RUN cd ORB_SLAM3/Thirdparty/Sophus \
    && mkdir build \
    && cd build \
    && cmake .. -DCMAKE_BUILD_TYPE=Release \
    && make -j \
    && make install
RUN cd ORB_SLAM3/Vocabulary \
    && tar -xf ORBvoc.txt.tar.gz
RUN cd ORB_SLAM3 \
    && mkdir build \
    && cd build \
    && cmake .. -DCMAKE_BUILD_TYPE=Release \
    && make -j2

# ============ src Packages ============
# Install ORB_SLAM3_RB5
COPY src ros2_ws/src

RUN apt-get update && apt-get install -y \
    ros-humble-image-transport \
    ros-humble-cv-bridge \
    ros-humble-message-filters

RUN cd ~/ros2_ws \
    && . /opt/ros/humble/setup.sh \
    && rosdep install -i --from-path src --rosdistro humble --skip-keys="image_transport;cv_bridge;message_filters" -y \
    && colcon build --symlink-install



# RUN cd ~/ros2_ws \
#     && . /opt/ros/humble/setup.sh \
#     && rosdep install -i --from-path src --rosdistro humble -y --skip-keys=Pangolin \
#     && colcon build --symlink-install