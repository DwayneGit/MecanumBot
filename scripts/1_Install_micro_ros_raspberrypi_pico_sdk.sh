# These instructions are mostly copied from https://github.com/micro-ROS/micro_ros_raspberrypi_pico_sdk
# see the project repo for more details on setting up the enviorment
source /opt/ros/$ROS_DISTRO/setup.bash

# Install dependencies
apt update

# micro_ros_raspberrypi_pico_sdk deps
apt install -y gcc libstdc++-arm-none-eabi-newlib cmake g++ gcc-arm-none-eabi doxygen libnewlib-arm-none-eabi git python3 apt-utils

# raspberrypi pico sdk deps
apt install -y automake autoconf build-essential texinfo libtool libftdi-dev libusb-1.0-0-dev

# Install docker
is_installed() {
    [ -z "$(dpkg -l | awk "/^ii  $1/")" ]
}

if ! is_installed "docker"; then
    curl -fsSL https://get.docker.com -o get-docker.sh
    sh get-docker.sh
    groupadd docker
    usermod -aG docker $USER
    newgrp docker
fi

# Configure environment
export PICO_SDK_PATH=$HOME/micro_ros/src/pico-sdk
export MICRO_ROS_AGENT_PATH=$HOME/micro_ros/src/micro_ros_setup
export MICRO_ROS_PICO_SDK_DIR=$HOME/micro_ros/src/micro_ros_raspberrypi_pico_sdk

echo "export PICO_SDK_PATH=$HOME/micro_ros/src/pico-sdk" >> ~/.bashrc
echo "export MICRO_ROS_AGENT_PATH=$HOME/micro_ros/src/micro_ros_setup" >> ~/.bashrc
echo "export MICRO_ROS_PICO_SDK_DIR=$HOME/micro_ros/src/micro_ros_raspberrypi_pico_sdk" >> ~/.bashrc
echo "source $HOME/micro_ros/install/local_setup.bash" >> ~/.bashrc

mkdir -p $HOME/micro_ros/src
cd $HOME/micro_ros

echo "Cloning micro_ros_raspberrypi_pico_sdk repository"
if ! (git clone https://github.com/micro-ROS/micro_ros_raspberrypi_pico_sdk.git $MICRO_ROS_PICO_SDK_DIR)
then
    echo >&2 message
    exit 1
fi

echo "Cloning pico-sdk repository"
if ! (git clone --recurse-submodules https://github.com/raspberrypi/pico-sdk.git $PICO_SDK_PATH)
then
    echo >&2 message
    exit 1
fi

echo "Cloning micro_ros_setup repository"
if ! (git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git $MICRO_ROS_AGENT_PATH)
then
    echo >&2 message
    exit 1
fi

# Update dependencies using rosdep
apt update && rosdep update
rosdep install --from-paths src --ignore-src -y

apt-get install python3-pip

colcon build
source install/local_setup.bash
# Create the micro-ROS agent
# Download micro-ROS-Agent packages
ros2 run micro_ros_setup create_agent_ws.sh

# Build step
ros2 run micro_ros_setup build_agent.sh
source $HOME/micro_ros/install/local_setup.bash

cd $PICO_SDK_PATH
git submodule update --init

# recompile with custom message package
# cd $MICRO_ROS_PICO_SDK_DIR
# cp mecanumbot_msgs microros_static_library/library_generation/extra_packages/
# docker pull microros/micro_ros_static_library_builder:humble
# docker run -it --rm -v $(pwd):/project microros/micro_ros_static_library_builder:humble

# Compile the example
# cd micro_ros_raspberrypi_pico_sdk
# mkdir build
# cd build
# cmake ..
# make