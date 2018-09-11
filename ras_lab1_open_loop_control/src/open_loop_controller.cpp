#include <ros/ros.h>
#include <ras_lab1_msgs/PWM.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "open_loop_controller");
  ros::NodeHandle nh;
	
	//Loop rate, 10 Hz.
	ros::Rate rate(10);
	
	//Publish messages of type ras_lab1_msgs::PWM to the topic /kobuki/PWM with queue size of 10.
	ros::Publisher pub = nh.advertise<ras_lab1_msgs::PWM>("/kobuki/pwm", 10);
	
	ras_lab1_msgs::PWM pwm_msg;
	
	pwm_msg.PWM1 = 255;
	pwm_msg.PWM2 = 225;
	
	while (ros::ok()) {
		pub.publish(pwm_msg);
		rate.sleep();
	}
} 
