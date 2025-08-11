FROM ros:kilted

SHELL ["/bin/bash", "-c"]

WORKDIR /app

RUN apt-get update && apt-get install -y ros-kilted-demo-nodes-cpp git python3-pip git python3-jinja2 \
      libboost-dev \
      libgnutls28-dev openssl libtiff-dev pybind11-dev \
      meson cmake \
      python3-yaml python3-ply python3-serial \
      libglib2.0-dev libgstreamer-plugins-base1.0-dev \
      ros-kilted-cartographer ros-kilted-cartographer-ros

# Clone and build raspberrypi's libcamera fork
RUN git clone https://github.com/raspberrypi/libcamera.git \
  && cd libcamera \
  && meson setup build --buildtype=release -Dpipelines=rpi/vc4,rpi/pisp -Dipas=rpi/vc4,rpi/pisp -Dv4l2=true -Dgstreamer=enabled -Dtest=false -Dlc-compliance=disabled -Dcam=disabled -Dqcam=disabled -Ddocumentation=disabled -Dpycamera=enabled \
  && ninja -C build install

# Clone and build the camera_ros node
RUN mkdir -p /app/src \
  && cd /app/src \
  && git clone https://github.com/christianrauch/camera_ros.git \
  && source /opt/ros/$ROS_DISTRO/setup.bash \
  && cd /app \
  && rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO --skip-keys=libcamera \
  && colcon build --event-handlers=console_direct+

RUN cd /app/src \
  && git clone https://github.com/Slamtec/sllidar_ros2.git \
  && source /opt/ros/$ROS_DISTRO/setup.bash \
  && cd /app \
  && colcon build --symlink-install

COPY ./ros /app/src

RUN cd /app \
  && colcon build --symlink-install

COPY docker_entrypoint.sh /app/

ENTRYPOINT ["/app/docker_entrypoint.sh"]
CMD ["bash"]

