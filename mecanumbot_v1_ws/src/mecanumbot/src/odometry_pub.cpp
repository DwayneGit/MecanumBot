#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "../include/mecanumbot/odometry_pub.hpp"

OdometryPublisher::OdometryPublisher() : Node("odometry_publisher")
{
	// intialize integrators
	x = y = th = 0;

	odom_broadcaster_ =
		std::make_unique<tf2_ros::TransformBroadcaster>(*this);
	
	// load parameters
	declare_parameter("scale_x"  , 0.6283185 / 360.0 / 2.15); // loosely base off of 360 counts per rev, 100mm wheels (which is 0.6283185 / 360.0)
	declare_parameter("scale_y"  , 0.6283185 / 360.0 / 2.7);
	declare_parameter("scale_th" , 0.0032);

	get_parameter("scale_x"  , scale_x);
	get_parameter("scale_y"  , scale_y);
	get_parameter("scale_th" , scale_th);
	
	// lets show em what we got
	RCLCPP_INFO(get_logger(), "param scale_x:  %f", scale_x);
	RCLCPP_INFO(get_logger(), "param scale_y:  %f", scale_y);
	RCLCPP_INFO(get_logger(), "param scale_th: %f", scale_th);

	double rev_per_meter = 1/(PI*0.1); // 100 mm diameter wheel
	double motor_rev_per_meter = rev_per_meter*2; //2:1 gear ratio
	double ticks_per_meter = motor_rev_per_meter*383.6; // 383.6 encoder counts per motor rev
	
	TICKS_PER_METER = ticks_per_meter;
	BASE_WIDTH = 0.152*2; // 15.2 cm from wheel to center of robot
	
	// connects subs and pubs
	enc_sub_  = create_subscription<std_msgs::msg::Int32MultiArray>(
		"encoders", 10, 
		std::bind(&OdometryPublisher::encoderCallback, this, std::placeholders::_1)
	);
	odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 50);
}

double OdometryPublisher::normalize_angle(double angle){
	while (angle > PI)
		angle -= 2.0 * PI;
	while (angle < -PI)
		angle += 2.0 * PI;
	return angle;
}

void OdometryPublisher::encoderCallback(const std_msgs::msg::Int32MultiArray & encoders)
{
	// unpack the encoder message in base_link frame

    int d_enc1_ticks = encoders.data[0] - this->prev_enc1_ticks;
    int d_enc2_ticks = encoders.data[1] - this->prev_enc2_ticks; 
    int d_enc3_ticks = encoders.data[2] - this->prev_enc3_ticks; 
    int d_enc4_ticks = encoders.data[3] - this->prev_enc4_ticks; 

    this->prev_enc1_ticks = encoders.data[0];
    this->prev_enc2_ticks = encoders.data[1];
    this->prev_enc3_ticks = encoders.data[2];
    this->prev_enc4_ticks = encoders.data[3];

    double dx = ( d_enc1_ticks + d_enc2_ticks + d_enc3_ticks + d_enc4_ticks ) / 4.0;
    double dy = ( 0 - d_enc1_ticks + d_enc2_ticks + d_enc3_ticks - d_enc4_ticks ) / 4.0;
    dx =   dx * this->scale_x / this->TICKS_PER_METER;
    dy =  -dy * this->scale_y / this->TICKS_PER_METER;
	
    double dr  = -( d_enc2_ticks + d_enc4_ticks ) / 2.0 / this->TICKS_PER_METER;
    double dl  =  ( d_enc1_ticks + d_enc3_ticks ) / 2.0 / this->TICKS_PER_METER;
    double dth =  ( dr - dl ) / this->BASE_WIDTH;

    // double dist = ( dr + dl ) / 4.0;

	rclcpp::Time current_time = this->get_clock()->now();
    rclcpp::Duration dt = current_time - this->last_enc_time;

    this->last_enc_time = current_time;

    this->x += dx * cos(this->actual_angle) - dy * sin(this->actual_angle);
    this->y += dx * sin(this->actual_angle) + dy * cos(this->actual_angle);
    this->th = this->normalize_angle(this->th + dth); // probably garbage value

    double vx, vy, vth;

    if (abs(dt.seconds()) < 0.000001) {
        vx  =  0.1;
        vy  = -0.1;
        vth =  0.1;
    } else {
        vx  = dx  / dt.seconds();
        vy  = dy  / dt.seconds();
        vth = dth / dt.seconds();
    }

	// since all odometry is 6DOF we'll need a quaternion created from yaw
	tf2::Quaternion q;
	q.setRPY(0, 0, th);
	geometry_msgs::msg::Quaternion odom_quat = tf2::toMsg(q);

	// first, we'll publish the transform over tf
	geometry_msgs::msg::TransformStamped odom_trans;
	odom_trans.header.stamp 	= current_time;
	odom_trans.header.frame_id 	= "odom";
	odom_trans.child_frame_id 	= "base_link";

	odom_trans.transform.translation.x 	= this->x;
	odom_trans.transform.translation.y 	= this->y;
	odom_trans.transform.translation.z 	= 0.0; // need some way to check this w/o the quaternion
	odom_trans.transform.rotation 		= odom_quat;

	// send the transform
	odom_broadcaster_->sendTransform(odom_trans);

	// next, we'll publish the odometry message over ROS
	auto odom 				= nav_msgs::msg::Odometry();
	odom.header.stamp 		= current_time;
	odom.header.frame_id 	= "odom";

	// set the position
	odom.pose.pose.position.x  = this->x;
	odom.pose.pose.position.y  = this->y;
	odom.pose.pose.position.z  = 0.0;
	odom.pose.pose.orientation = odom_quat;

	// set the velocity
	odom.child_frame_id        = "base_link";
	odom.twist.twist.linear.x  = vx;
	odom.twist.twist.linear.y  = vy;
	odom.twist.twist.angular.z = vth;

	// publish the message
	odom_pub_->publish(odom);
}