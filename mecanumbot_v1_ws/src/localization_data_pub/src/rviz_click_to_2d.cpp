#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_broadcaster.h>
#include <iostream>
#include <memory>
 
using namespace std;

class RvizClickTo2D : public rclcpp::Node
{
	public:
		RvizClickTo2D();

	private:
		void handle_goal(const geometry_msgs::msg::PoseStamped & encoders);
        void handle_initial_pose(const geometry_msgs::msg::PoseWithCovarianceStamped &pose);

		rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_;
		rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub2_;

		rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_;
		rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub2_;
};

RvizClickTo2D::RvizClickTo2D() : Node("rviz_click_to_2d")
{
    pub_  = create_publisher<geometry_msgs::msg::PoseStamped>("goal_2d", 0);
    pub2_ = create_publisher<geometry_msgs::msg::PoseStamped>("initial_2d", 0);

    sub_  = create_subscription<geometry_msgs::msg::PoseStamped>(
        "move_base_simple/goal", 0, 
		std::bind(&RvizClickTo2D::handle_goal, this, std::placeholders::_1)
    );
    sub2_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "initialpose", 0, 
		std::bind(&RvizClickTo2D::handle_initial_pose, this, std::placeholders::_1)
    );
}
 
// Take move_base_simple/goal as input and publish goal_2d
void RvizClickTo2D::handle_goal(const geometry_msgs::msg::PoseStamped & goal) {
    geometry_msgs::msg::PoseStamped rpyGoal;
    rpyGoal.header.frame_id = "map";
    rpyGoal.header.stamp = goal.header.stamp;
    rpyGoal.pose.position.x = goal.pose.position.x;
    rpyGoal.pose.position.y = goal.pose.position.y;
    rpyGoal.pose.position.z = 0;
    tf2::Quaternion q(0, 0, goal.pose.orientation.z, goal.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    rpyGoal.pose.orientation.x = 0;
    rpyGoal.pose.orientation.y = 0;
    rpyGoal.pose.orientation.z = yaw;
    rpyGoal.pose.orientation.w = 0;
    this->pub_->publish(rpyGoal);
}
 
// Take initialpose as input and publish initial_2d
void RvizClickTo2D::handle_initial_pose(const geometry_msgs::msg::PoseWithCovarianceStamped & pose) {
    geometry_msgs::msg::PoseStamped rpyPose;
    rpyPose.header.frame_id = "map";
    rpyPose.header.stamp = pose.header.stamp;
    rpyPose.pose.position.x = pose.pose.pose.position.x;
    rpyPose.pose.position.y = pose.pose.pose.position.y;
    rpyPose.pose.position.z = 0;
    tf2::Quaternion q(0, 0, pose.pose.pose.orientation.z, pose.pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    rpyPose.pose.orientation.x = 0;
    rpyPose.pose.orientation.y = 0;
    rpyPose.pose.orientation.z = yaw;
    rpyPose.pose.orientation.w = 0;
    this->pub2_->publish(rpyPose);
}
 
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::Rate loop_rate(10);
    while (rclcpp::ok()) {
        rclcpp::spin_some(std::make_shared<RvizClickTo2D>());
        loop_rate.sleep();
    }
  	rclcpp::shutdown();
	return 0;
}