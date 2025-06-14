FROM ros2-humble-arm64

# Imposta variabili ambientali
ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# Aggiornamenti base e installazione dipendenze
RUN apt-get update && apt-get install -y \
    locales \
    curl \
    gnupg2 \
    lsb-release \
    tzdata \
    software-properties-common \
    build-essential \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LANG=en_US.UTF-8

# Imposta timezone (modificabile)
RUN ln -fs /usr/share/zoneinfo/UTC /etc/localtime && dpkg-reconfigure -f noninteractive tzdata


# Installa ROS 2 Humble Desktop Full + strumenti di sviluppo
RUN apt-get update && apt-get install -y \
    ros-humble-desktop-full \
    python3-colcon-common-extensions \
    python3-pip \
    git \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y \
    ros-humble-slam-toolbox \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-image-transport \
    ros-humble-image-transport-plugins \
    ros-humble-camera-info-manager \
    ros-humble-image-publisher \
    ros-humble-robot-localization \
    ros-humble-slam-toolbox \
    apt-transport-https\
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ros_ws
RUN git clone https://github.com/Slamtec/rplidar_ros.git
RUN git clone https://github.com/Adlink-ROS/rf2o_laser_odometry.git
RUN git clone https://github.com/orbbec/ros2_astra_camera.git
RUN colcon build
RUN echo "source install/setup.bash" >> /root/.bashrc
WORKDIR ../

# Setup ambiente ROS 2
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
COPY py_install /py_install

# Entra nella cartella e installa il pacchetto
WORKDIR /py_install
RUN python3 setup.py install
RUN pip install pyserial
WORKDIR ../
RUN apt-get update && apt-get install -y nano && rm -rf /var/lib/apt/lists/*

RUN pip install opencv-python
