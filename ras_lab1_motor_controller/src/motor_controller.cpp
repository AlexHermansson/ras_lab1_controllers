#include <ros/ros.h>
#include <math.h>
#include <vector>
#include <ras_lab1_msgs/PWM.h>
#include <ras_lab1_msgs/Encoders.h>
#include <geometry_msgs/Twist.h>

const int control_frequency = 10;
const int ticks_per_rev = 360;
const double wheel_radius = 0.0352;
const double base = 0.23;
const double dt = 1.0 / control_frequency;

const double alpha = 1;
const double beta  = 5.0;

double v_robot_desired;
double w_robot_desired;

// left is first element, right is second.
std::vector<double> int_error (2, 0); // init two elements, each with value 0.
std::vector<double> w_estimate (2, 0); 
std::vector<double> w_desired (2, 0);
std::vector<double> delta_encoder (2, 0);


void encoderCallback(const ras_lab1_msgs::Encoders::ConstPtr& msg) {
	delta_encoder[0] = msg->delta_encoder1;
	delta_encoder[1] = msg->delta_encoder2;
}

void twistCallback(const geometry_msgs::Twist::ConstPtr& msg) {
	v_robot_desired = msg->linear.x;
	w_robot_desired = msg->angular.z;
}

void updateEstimatedSpeed() {
	w_estimate[0] = (delta_encoder[0] * 2 * M_PI * control_frequency) / (ticks_per_rev);
	w_estimate[1] = (delta_encoder[1] * 2 * M_PI * control_frequency) / (ticks_per_rev);
}

void updateDesiredSpeed() {
	w_desired[0] = (v_robot_desired - w_robot_desired * (base / 2)) / wheel_radius;
	w_desired[1] = (v_robot_desired + w_robot_desired * (base / 2)) / wheel_radius;
}

void setPWM(ras_lab1_msgs::PWM& msg) {
	// Update desired and estimated speed. Then calculate PWM and set it.
	updateEstimatedSpeed();
	updateDesiredSpeed();

	double error_left = w_desired[0] - w_estimate[0];
	int_error[0] += error_left * dt;
	msg.PWM1 = int (alpha * error_left + beta * int_error[0]);
	
	double error_right = w_desired[1] - w_estimate[1];
	int_error[1] += error_right * dt;
	msg.PWM2 = int (alpha * error_right + beta * int_error[1]);
}


int main(int argc, char **argv)
{	
	ros::init(argc, argv, "motor_controller");
	ros::NodeHandle nh;
	ros::Rate rate(10);
	
	ros::Publisher pwm_pub = nh.advertise<ras_lab1_msgs::PWM>("/kobuki/pwm", 1);
	
	ros::Subscriber enc_sub = nh.subscribe<ras_lab1_msgs::Encoders>("/kobuki/encoders", 1, 
																																	encoderCallback);
	
	ros::Subscriber twist_sub = nh.subscribe<geometry_msgs::Twist>("/motor_controller/twist", 1, 
																																 twistCallback);
	
	ras_lab1_msgs::PWM pwm_msg;
	
	while (ros::ok()) {
		ros::spinOnce(); // Listen to encoder and twist topics. Callbacks are invoked
		setPWM(pwm_msg);
		pwm_pub.publish(pwm_msg);
		rate.sleep();
	}

}
