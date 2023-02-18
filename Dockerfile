FROM ros:humble-ros-base
WORKDIR /

COPY . .

ENV ROS_DISTRO=humble

SHELL ["/bin/bash", "-c"]
RUN ./scripts/1_Install_micro_ros_raspberrypi_pico_sdk.sh

WORKDIR /mecanumbot_v1_ws

RUN colcon build

CMD ["ros2", "launch", "mecanumbot", "mecanumbot_launch.py" ]
# EXPOSE 3000