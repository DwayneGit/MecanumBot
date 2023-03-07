#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32MultiArray.h>
#include <math.h>

#define PI 3.14159265

class OdometryPublisher
{
	public:
		OdometryPublisher();

	private:
		void encoderCallback(const std_msgs::Float32MultiArray::ConstPtr& encoders);
        double OdometryPublisher::normalize_angle(double angle);

		ros::NodeHandle nh;
		ros::Subscriber enc_sub;
		ros::Publisher odom_pub;
        ros::Time last_enc_time;
		tf::TransformBroadcaster odom_broadcaster;
		
        double TICKS_PER_METER;
        double BASE_WIDTH;
		double x, y, th;
		double scale_x, scale_y, scale_th;
        int prev_enc1_ticks, prev_enc2_ticks, prev_enc3_ticks, prev_enc4_ticks;
		bool calibration_mode;
};


OdometryPublisher::OdometryPublisher()
{
	// intialize integrators
	x = y = th = 0;
	
	// load parameters
    ros::NodeHandle nh_priv("~");
	nh_priv.param("scale_x", scale_x, 0.6283185 / 360.0 / 2.15); // loosely base off of 360 counts per rev, 100mm wheels (which is 0.6283185 / 360.0)
	nh_priv.param("scale_y", scale_y, 0.6283185 / 360.0 / 2.7);
	nh_priv.param("scale_th", scale_th, 0.0032);
	nh_priv.param("calibration_mode", calibration_mode, false);
	
	// lets show em what we got
	ROS_INFO_STREAM("param scale_x: " << scale_x);
	ROS_INFO_STREAM("param scale_y: " << scale_y);
	ROS_INFO_STREAM("param scale_th: " << scale_th);
	ROS_INFO_STREAM("param calibration_mode: " << calibration_mode);

    double rev_per_meter = 1/(M_PI*0.1) // 100 mm diameter wheel
    double motor_rev_per_meter = rev_per_meter*2; //2:1 gear ratio
    double ticks_per_meter = motor_rev_per_meter*383.6; // 383.6 encoder counts per motor rev
    
    TICKS_PER_METER = ticks_per_meter;
    BASE_WIDTH = 0.152*2; // 15.2 cm from wheel to center of robot
	
    // connects subs and pubs
	enc_sub = nh.subscribe<std_msgs::Float32MultiArray>("encoders", 10, &OdometryPublisher::encoderCallback, this);
	odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);
}

double OdometryPublisher::normalize_angle(double angle){
    while (angle > M_PI)
        angle -= 2.0 * M_PI;
    while (angle < -M_PI)
        angle += 2.0 * M_PI;
    return angle
}

void OdometryPublisher::encoderCallback(const std_msgs::Float32MultiArray::ConstPtr& encoders)
{
	// unpack the encoder message in base_link frame

    int d_enc1_ticks = encoders->data[0] - this.prev_enc1_ticks;
    int d_enc2_ticks = encoders->data[1] - this.prev_enc2_ticks; 
    int d_enc3_ticks = encoders->data[2] - this.prev_enc3_ticks; 
    int d_enc4_ticks = encoders->data[3] - this.prev_enc4_ticks; 

    this.prev_enc1_ticks = encoders->data[0];
    this.prev_enc2_ticks = encoders->data[1];
    this.prev_enc3_ticks = encoders->data[2];
    this.prev_enc4_ticks = encoders->data[3];

    double dx = ( d_enc1_ticks + d_enc2_ticks + d_enc3_ticks + d_enc4_ticks ) / 4.0;
    double dy = ( 0 - d_enc1_ticks + d_enc2_ticks + d_enc3_ticks - d_enc4_ticks ) / 4.0;
    dx =   dx * this.scale_x / this.TICKS_PER_METER;
    dy =  -dy * this.scale_y / this.TICKS_PER_METER;
	
    double dr  = -( d_enc2_ticks + d_enc4_ticks ) / 2.0 / this.TICKS_PER_METER;
    double dl  =  ( d_enc1_ticks + d_enc3_ticks ) / 2.0 / this.TICKS_PER_METER;
    double dth =  ( dr - dl ) / this.BASE_WIDTH;

    double dist = ( dr + dl ) / 4.0;

	ros::Time current_time = ros::Time::now();
    ros::Time dt = ( current_time - this.last_enc_time );
    this.last_enc_time = current_time;

    this.x += dx * cos(this.actual_angle) - dy * sin(this.actual_angle);
    this.y += dx * sin(this.actual_angle) + dy * cos(this.actual_angle);
    this.th = this.normalize_angle(this.th + dth); // probably garbage value

    double vx, vy, vth;

    if ( abs(d_time) < 0.000001 ) {
        vx  =  0.1;
        vy  = -0.1;
        vth =  0.1;
    } else {
        vx  = dx  / dt;
        vy  = dy  / dt;
        vth = dth / dt;
    }

	// since all odometry is 6DOF we'll need a quaternion created from yaw
	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

	// first, we'll publish the transform over tf
	geometry_msgs::TransformStamped odom_trans;
	odom_trans.header.stamp = current_time;
	odom_trans.header.frame_id = "odom";
	odom_trans.child_frame_id = "base_link";

	odom_trans.transform.translation.x = this.x;
	odom_trans.transform.translation.y = this.y;
	odom_trans.transform.translation.z = 0.0; // need some way to check this w/o the quaternion
	odom_trans.transform.rotation = odom_quat;

	// send the transform
	odom_broadcaster.sendTransform(odom_trans);

	// next, we'll publish the odometry message over ROS
	nav_msgs::Odometry odom;
	odom.header.stamp = current_time;
	odom.header.frame_id = "odom";

	// set the position
	odom.pose.pose.position.x  = this.x;
	odom.pose.pose.position.y  = this.y;
	odom.pose.pose.position.z  = 0.0;
	odom.pose.pose.orientation = odom_quat;

	// set the velocity
	odom.child_frame_id        = "base_link";
	odom.twist.twist.linear.x  = vx;
	odom.twist.twist.linear.y  = vy;
	odom.twist.twist.angular.z = vth;

	// publish the message
	odom_pub.publish(odom);
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "odometry_publisher");
	OdometryPublisher odom;

	ros::spin();
}