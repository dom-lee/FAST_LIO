FROM nvidia/cudagl:11.0.3-devel-ubuntu20.04

ARG ROS_DISTRO=noetic
ARG USERNAME=domlee
ARG USER_UID=1000
ARG USER_GID=1000
ENV USER_HOME=/home/${USERNAME}
ENV DEBIAN_FRONTEND=noninteractive

# Add ROS sources
RUN apt-get update && apt-get install -y \
    curl gnupg lsb-release sudo \
    && echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list \
    && curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

# Install ROS and tools
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-desktop-full \
    python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential \
    ros-${ROS_DISTRO}-navigation \
    ros-${ROS_DISTRO}-robot-localization \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-pcl-ros \
    ros-${ROS_DISTRO}-tf2-geometry-msgs \
    ros-${ROS_DISTRO}-tf \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init 2>/dev/null || true && rosdep update

# Install Livox SDK
RUN apt-get update && apt-get install -y \
    libyaml-cpp-dev \
    libpcl-dev \
    libeigen3-dev \
    libboost-all-dev \
    && rm -rf /var/lib/apt/lists/*

RUN cd /tmp && \
    rm -rf Livox-SDK && \
    git clone https://github.com/Livox-SDK/Livox-SDK.git && \
    cd Livox-SDK && \
    mkdir -p build && cd build && \
    cmake .. && make -j$(nproc) && make install && \
    rm -rf /tmp/Livox-SDK

# GUI dependencies
RUN apt-get update && apt-get install -y \
    libgl1-mesa-glx libgl1-mesa-dri libglu1-mesa \
    libx11-6 libxext6 libxrender1 x11-xserver-utils \
    && rm -rf /var/lib/apt/lists/*

# Dev utilities
RUN apt-get update && apt-get install -y \
    tmux vim \
    && rm -rf /var/lib/apt/lists/*

# Create non-root user
RUN groupadd -g ${USER_GID} ${USERNAME} && \
    useradd -m -s /bin/bash -u ${USER_UID} -g ${USER_GID} ${USERNAME} && \
    usermod -aG sudo ${USERNAME} && \
    echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME}

# Switch to user
USER ${USERNAME}
ENV HOME=${USER_HOME}

# Create and configure Livox workspace (required by FAST-LIO)
WORKDIR ${USER_HOME}/ws_livox/src
RUN git clone https://github.com/Livox-SDK/livox_ros_driver.git

WORKDIR ${USER_HOME}/ws_livox
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && catkin_make -j$(nproc)"

# Source ws_livox in bashrc (must come before FAST-LIO)
RUN echo "source ~/ws_livox/devel/setup.bash" >> ~/.bashrc

# Create universal ROS entrypoint
USER root
RUN echo '#!/bin/bash' > /usr/local/bin/ros-entrypoint.sh && \
    echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /usr/local/bin/ros-entrypoint.sh && \
    echo "if [ -f \"$HOME/ws_livox/devel/setup.bash\" ]; then source \"$HOME/ws_livox/devel/setup.bash\"; fi" >> /usr/local/bin/ros-entrypoint.sh && \
    echo "if [ -f \"$HOME/catkin_ws/devel/setup.bash\" ]; then source \"$HOME/catkin_ws/devel/setup.bash\"; fi" >> /usr/local/bin/ros-entrypoint.sh && \
    echo 'exec "$@"' >> /usr/local/bin/ros-entrypoint.sh && \
    chmod +x /usr/local/bin/ros-entrypoint.sh

# Final setup
USER ${USERNAME}
WORKDIR ${USER_HOME}/catkin_ws
ENTRYPOINT ["/usr/local/bin/ros-entrypoint.sh"]
CMD ["bash"]