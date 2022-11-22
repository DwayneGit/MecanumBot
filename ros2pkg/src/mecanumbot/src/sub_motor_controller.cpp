#include <memory>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;

class SubMotorController : public rclcpp::Node
{
    public:
        SubMotorController()
        : Node("sub_motor_controller")
        {
            serial_port_ = open("/dev/ttyAMA0", O_RDWR);
            if(tcgetattr(serial_port_, &tty_) != 0) {
                RCLCPP_INFO(this->get_logger(), "Error %i from tcgetattr: %s\n", errno, strerror(errno));
                return;
            }
            
            tty_.c_cflag &= ~PARENB;
            tty_.c_cflag &= ~CSTOPB;
            tty_.c_cflag &= ~CSIZE;
            tty_.c_cflag |= CS8;
            tty_.c_cflag &= ~CRTSCTS;
            tty_.c_cflag |= CREAD | CLOCAL;
            tty_.c_lflag &= ~ICANON;
            tty_.c_lflag &= ~ECHO;
            tty_.c_lflag &= ~ECHOE;
            tty_.c_lflag &= ~ECHONL;
            tty_.c_lflag &= ~ISIG;
            tty_.c_iflag &= ~(IXON | IXOFF | IXANY);
            tty_.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);
            tty_.c_oflag &= ~OPOST;
            tty_.c_oflag &= ~ONLCR;
            tty_.c_cc[VTIME] = 10;
            tty_.c_cc[VMIN] = 0;

            cfsetispeed(&tty_, B9600);
            cfsetospeed(&tty_, B9600);

            if (tcsetattr(serial_port_, TCSANOW, &tty_) != 0) {
                RCLCPP_INFO(this->get_logger(), "Error %i from tcsetattr: %s\n", errno, strerror(errno));
                return;
            }

            subscription_ = this->create_subscription<std_msgs::msg::String>(
            "motor_control", 10, std::bind(&SubMotorController::motor_control_cb, this, _1));
        }

    private:
        void motor_control_cb(const std_msgs::msg::String & msg) const
        {
            RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
            write(this->serial_port_, msg, sizeof(msg));
        }
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
        int serial_port_;
        struct termios tty_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SubMotorController>());
  rclcpp::shutdown();
  return 0;
}