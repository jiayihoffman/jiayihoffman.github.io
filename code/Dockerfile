FROM ros:humble-perception

ARG ROS_DISTRO=humble

# Install essential utilities
RUN apt-get update && apt-get install -y \
    nano \
    vim \
    python3-serial \
    python3-pip \
    libserial-dev \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-teleop-twist-keyboard \
    ros-${ROS_DISTRO}-twist-mux \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-slam-toolbox \
    ros-${ROS_DISTRO}-rosbridge-suite \
    && rm -rf /var/lib/apt/lists/*

# Create "ros" user and group
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=${USER_UID}

ENV BOT_HOME=/home/${USERNAME}

COPY start_robot.sh ${BOT_HOME}/start_robot.sh
RUN chmod +x ${BOT_HOME}/start_robot.sh

RUN groupadd --gid ${USER_GID} ${USERNAME} \
  && useradd -s /bin/bash --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} -d ${BOT_HOME} \
  && chown -R ${USER_UID}:${USER_GID} ${BOT_HOME}
    
RUN usermod -a -G dialout ${USERNAME}

# the rest of the command run as ros
USER ros

# Create the ROS 2 workspace
RUN mkdir -p ${BOT_HOME}/bot_ws/src 

# download the rplidar source code
RUN cd ${BOT_HOME}/bot_ws/src \
  && git clone -b ros2 https://github.com/Slamtec/rplidar_ros.git

# diffdrive hardware component
RUN cd ${BOT_HOME}/bot_ws/src \
  && git clone -b humble https://github.com/joshnewans/diffdrive_arduino.git

# Source ROS 2 env and build the rplidar nodes
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    cd ${BOT_HOME}/bot_ws && \
    colcon build --symlink-install"

# config ros2 environment
RUN echo "export ROS_DOMAIN_ID=1" >> ~/.bashrc \
  && echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc \
  && echo "source ${BOT_HOME}/bot_ws/install/setup.bash" >> ~/.bashrc

# install python dependencies
COPY requirements.txt ${BOT_HOME}/requirements.txt
RUN python3 -m pip install --user --no-cache-dir --no-compile -r ${BOT_HOME}/requirements.txt
    
# Set default working directory
WORKDIR ${BOT_HOME}/bot_ws

CMD ["~/start_robot.sh"]
