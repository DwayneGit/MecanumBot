#include <math.h>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <tf2_ros/transform_broadcaster.h>

#define PI 3.14159265

class OdometryPublisher : public rclcpp::Node
{
	public:
		OdometryPublisher();

	private:
		void encoderCallback(const std_msgs::msg::Float32MultiArray & encoders);
        double normalize_angle(double angle);

		rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr enc_sub_;
		rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
        rclcpp::Time last_enc_time;
		std::unique_ptr<tf2_ros::TransformBroadcaster> odom_broadcaster_;
		
        double TICKS_PER_METER;
        double BASE_WIDTH;
		double x, y, th;
		double actual_angle;
		double scale_x, scale_y, scale_th;
        int prev_enc1_ticks, prev_enc2_ticks, prev_enc3_ticks, prev_enc4_ticks;
};