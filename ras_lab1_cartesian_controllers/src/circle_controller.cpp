#include <ros/ros.h>
#include <math.h>
#include <geometry_msgs/Twist.h>


const double T = 10;
const double R = 0.5;

double v = (2 * M_PI * R) / T;
double w = v / R;


int main(int argc, char **argv)
{
	ros::init(argc, argv, "circle_controller");
	ros::NodeHandle nh;
	ros::Rate rate(10);
	
	ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/motor_controller/twist", 1);
	
	geometry_msgs::Twist twist_msg;
	
	// Other elements are set to zero
	twist_msg.linear.x = v;
	twist_msg.angular.z = w;
	
	while (ros::ok()) {
		pub.publish(twist_msg);
		rate.sleep();
	}

}

