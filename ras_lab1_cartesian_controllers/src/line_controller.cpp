#include <ros/ros.h>
#include <math.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "line_controller");
	ros::NodeHandle nh;
	ros::Rate rate(10);
	
	ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/motor_controller/twist", 1);
	
	geometry_msgs::Twist twist_msg;
	
	twist_msg.linear.x = 0.2;
	
	while (ros::ok()) {
		pub.publish(twist_msg);
		rate.sleep();
	}

}
