ARG ROS_DISTRO=humble
FROM osrf/ros:$ROS_DISTRO-desktop

ARG PROJECT=black_mouth
ARG WS_PATH=/home/${PROJECT}_${ROS_DISTRO}

WORKDIR ${WS_PATH}/src
COPY ${PROJECT}_control/package.xml ./${PROJECT}_control/package.xml
COPY ${PROJECT}_description/package.xml ./${PROJECT}_description/package.xml
COPY ${PROJECT}_gazebo/package.xml ./${PROJECT}_gazebo/package.xml
COPY ${PROJECT}_kinematics/package.xml ./${PROJECT}_kinematics/package.xml

WORKDIR ${WS_PATH}
RUN . /opt/ros/humble/setup.sh && \
    rosdep update && apt-get update && \
    rosdep install -y \
    --from-paths src --ignore-src \
    && rm -rf /var/lib/apt/lists/*
# RUN apt-get install ros-humble-gazebo*

