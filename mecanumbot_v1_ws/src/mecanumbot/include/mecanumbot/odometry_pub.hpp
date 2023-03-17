#include <math.h>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp" 
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <tf2_ros/transform_broadcaster.h>

#define PI 3.14159265

class Velocities {
	public:
		Velocities(double x, double y, double th) : vx(x), 
			vy(y), 
			vth(th)
		{};
    	double vx, vy, vth;
};

class OdometryPublisher : public rclcpp::Node
{
	public:
		OdometryPublisher();

	private:
		void set_initial_2d(const geometry_msgs::msg::PoseStamped & rvizClick);
        double normalize_angle(double angle);

		Velocities update_odom(const std_msgs::msg::Int32MultiArray & encoders);
		void publish_odom(Velocities vels);
		void encoderCallback(const std_msgs::msg::Int32MultiArray & encoders);


		rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr enc_sub_;
		rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr initial_pose_sub_;
		rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
        rclcpp::Time last_enc_time_;
		std::unique_ptr<tf2_ros::TransformBroadcaster> odom_broadcaster_;
		
		nav_msgs::msg::Odometry odomNew;
		nav_msgs::msg::Odometry odomOld;
        double TICKS_PER_METER;
        double BASE_WIDTH;
		double x, y, th;
		double actual_angle;
		double scale_x, scale_y, scale_th;
        int prev_enc1_ticks, prev_enc2_ticks, prev_enc3_ticks, prev_enc4_ticks;
		bool initial_pose_recieved = false;
};