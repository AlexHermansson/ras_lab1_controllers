#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <ras_lab1_msgs/ADConverter.h>

const double alpha = 0.025;
const double v = 0.2;

int adc_left;
int adc_right;


void setVelocities(geometry_msgs::Twist& msg) {
	int error = adc_left - adc_right;
	msg.angular.z = int (alpha * error);
}

void ADCallback(const ras_lab1_msgs::ADConverter::ConstPtr& msg) {
	adc_right = msg->ch1;
	adc_left = msg->ch2;
}

int main(int argc, char **argv) 
{
	ros::init(argc, argv, "wall_following_controller");
	ros::NodeHandle nh;
	ros::Rate rate(10);
	
	ros::Publisher twist_pub = nh.advertise<geometry_msgs::Twist>("/motor_controller/twist", 1);
	ros::Subscriber adc_sub = nh.subscribe<ras_lab1_msgs::ADConverter>("/kobuki/adc", 1, ADCallback);
	
	geometry_msgs::Twist twist_msg;
	twist_msg.linear.x = v;
	
	
	while (ros::ok()) {
		ros::spinOnce();
		setVelocities(twist_msg);
		twist_pub.publish(twist_msg);
	
	}
	

}
