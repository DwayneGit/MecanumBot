# These instructions are mostly copied from https://github.com/micro-ROS/micro_ros_raspberrypi_pico_sdk
# see the project repo for more details on setting up the enviorment

# Install dependencies
sudo apt update

# micro_ros_raspberrypi_pico_sdk deps
apt install -y gcc libstdc++-arm-none-eabi-newlib cmake g++ gcc-arm-none-eabi doxygen libnewlib-arm-none-eabi git python3

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

mkdir $HOME/micro_ros
git clone https://github.com/micro-ROS/micro_ros_raspberrypi_pico_sdk.git $HOME/micro_ros/micro_ros_raspberrypi_pico_sdk
git clone --recurse-submodules https://github.com/raspberrypi/pico-sdk.git $HOME/micro_ros/pico-sdk

# Configure environment
echo "export PICO_SDK_PATH=$HOME/micro_ros/pico-sdk" >> ~/.bashrc
echo "export MICRO_ROS_PICO_SDK_DIR=$HOME/micro_ros/micro_ros_raspberrypi_pico_sdk" >> ~/.bashrc
source ~/.bashrc

cd PICO_SDK_PATH
git submodule update --init

# recompile with custom message package

cd $MICRO_ROS_PICO_SDK_DIR
cp mecanumbot_msgs microros_static_library/library_generation/extra_packages/
docker pull microros/micro_ros_static_library_builder:humble
docker run -it --rm -v $(pwd):/project microros/micro_ros_static_library_builder:humble

# Compile the example
# cd micro_ros_raspberrypi_pico_sdk
# mkdir build
# cd build
# cmake ..
# make