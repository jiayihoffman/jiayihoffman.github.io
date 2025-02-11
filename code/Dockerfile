FROM ros:humble-perception

# Install essential utilities
RUN apt-get update && apt-get install -y \
    nano \
    vim \
    python3-serial \
    && rm -rf /var/lib/apt/lists/*

# Create "ros" user and group
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

ENV BOT_HOME=/home/${USERNAME}

RUN groupadd --gid $USER_GID $USERNAME \
  && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME -d $BOT_HOME \
  && chown -R $USER_UID:$USER_GID ${BOT_HOME}
    
RUN usermod -a -G dialout ${USERNAME}

# the rest of the command run as ros
USER ros

# Create the ROS 2 workspace
RUN mkdir -p ${BOT_HOME}/bot_ws/src 

# download and build the rplidar node
RUN cd ${BOT_HOME}/bot_ws/src \
  && git clone -b ros2 https://github.com/Slamtec/rplidar_ros.git

# Source ROS 2 and build
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    cd ${BOT_HOME}/bot_ws && \
    colcon build --symlink-install --packages-select rplidar_ros"
    
RUN echo "export ROS_DOMAIN_ID=4" >> ~/.bashrc \
  && echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc \
  && echo "source ${BOT_HOME}/bot_ws/install/setup.bash" >> ~/.bashrc

# Set default working directory
WORKDIR ${BOT_HOME}/bot_ws

