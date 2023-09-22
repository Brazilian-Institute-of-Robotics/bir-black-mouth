ARG ROS_DISTRO=humble
FROM osrf/ros:$ROS_DISTRO-desktop

ARG PROJECT=caramel
ARG WS_PATH=/home/${PROJECT}_${ROS_DISTRO}

WORKDIR ${WS_PATH}/src
COPY ${PROJECT}_bringup/package.xml ./${PROJECT}_bringup/package.xml
COPY ${PROJECT}_control/package.xml ./${PROJECT}_control/package.xml
COPY ${PROJECT}_data_analysis/package.xml ./${PROJECT}_data_analysis/package.xml
COPY ${PROJECT}_description/package.xml ./${PROJECT}_description/package.xml
COPY ${PROJECT}_gait_planner/package.xml ./${PROJECT}_gait_planner/package.xml
COPY ${PROJECT}_gazebo/package.xml ./${PROJECT}_gazebo/package.xml
COPY ${PROJECT}_kinematics/package.xml ./${PROJECT}_kinematics/package.xml
COPY ${PROJECT}_teleop/package.xml ./${PROJECT}_teleop/package.xml

WORKDIR ${WS_PATH}
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    apt-get update && \
    rosdep install -y \
    --from-paths src --ignore-src \
    && rm -rf /var/lib/apt/lists/* && \
    apt-get install git -y

RUN touch /rsource.sh
RUN echo "#!/bin/bash \n\n\
source /opt/ros/$ROS_DISTRO/setup.bash \n\n\
if [ -e $WS_PATH/install/setup.bash ] \n\
then \n\
  source $WS_PATH/install/setup.bash \n\
fi" >> /rsource.sh
RUN echo "export RCUTILS_COLORIZED_OUTPUT=1" >> /rsource.sh
RUN echo "PS1='${debian_chroot:+($debian_chroot)}\[\033[38;5;208m\]bm@${ROS_DISTRO}\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\$ '" >> /rsource.sh
