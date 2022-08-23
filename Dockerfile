ARG FROM_IMAGE=osrf/ros:humble-desktop
ARG OVERLAY_WS=/opt/ros/overlay_ws

# multi-stage for caching
FROM $FROM_IMAGE AS cacher

# clone overlay source
ARG OVERLAY_WS
WORKDIR $OVERLAY_WS/src
RUN echo "\
repositories: \n\
  gazebo_ros2_control: \n\
    type: git \n\
    url: https://github.com/lucaslins0035/gazebo_ros2_control.git \n\
    version: master \n\
  bir-black-mouth: \n\
    type: git \n\
    url: https://github.com/Brazilian-Institute-of-Robotics/bir-black-mouth.git \n\
    version: fix-docker \n\
" > ../overlay.repos
RUN vcs import ./ < ../overlay.repos
# RUN rm -rf gazebo_ros2_control/gazebo_ros2_control_demos

# copy manifests for caching
WORKDIR /opt
RUN mkdir -p /tmp/opt && \
    find ./ -name "package.xml" | \
      xargs cp --parents -t /tmp/opt && \
    find ./ -name "COLCON_IGNORE" | \
      xargs cp --parents -t /tmp/opt || true

# multi-stage for building
FROM $FROM_IMAGE AS builder

# install overlay dependencies
ARG OVERLAY_WS
WORKDIR $OVERLAY_WS
COPY --from=cacher /tmp/$OVERLAY_WS/src ./src
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    apt-get update && rosdep install -y \
      --from-paths src --ignore-src \
    && rm -rf /var/lib/apt/lists/*

# build overlay source
COPY --from=cacher $OVERLAY_WS/src ./src
ARG OVERLAY_MIXINS="release"
RUN apt install ros-humble-hardware-interface
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build

# source entrypoint setup
ENV OVERLAY_WS $OVERLAY_WS
RUN echo "if [ -e $OVERLAY_WS/install/setup.bash ] \n\
then \n\
  source $OVERLAY_WS/install/setup.bash \n\
fi" >> /ros_entrypoint.sh
RUN echo "PS1='${debian_chroot:+($debian_chroot)}\[\033[38;5;208m\]bm@humble:${CONTAINER_NAME}\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\$ '" >> /ros_entrypoint.sh
