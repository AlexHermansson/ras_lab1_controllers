#include <ros/ros.h>
#include <vector>
#include <math.h>
#include <ras_lab1_msgs/PWM.h>
#include <ras_lab1_msgs/Encoders.h>
#include <geometry_msgs/Twist.h>

const int control_frequency = 10;
const int ticks_per_rev = 360;
const double wheel_radius = 0.0352;
const double base = 0.23;
const double dt = 1.0 / control_frequency;

const double alpha = 1.0;
const double beta  = 5.0;

double int_error_left = 0;
double int_error_right = 0;

double v_robot_desired;
double w_robot_desired;

double w_left_desired;
double w_right_desired;
double w_left_estimate;
double w_right_estimate;

double delta_encoder_left;
double delta_encoder_right;


class Encoder {

	public:
		int delta_encoder_left;
		int delta_encoder_right;
		
	void encoderCallback(const ras_lab1_msgs::Encoders::ConstPtr& msg) {
		delta_encoder_left = msg->delta_encoder1;
		delta_encoder_right = msg->delta_encoder2;
	}

};


void encoderCallback(const ras_lab1_msgs::Encoders::ConstPtr& msg) {
	delta_encoder_left = msg->delta_encoder1;
	delta_encoder_right = msg->delta_encoder2;
	ROS_INFO("left encoder : %f", delta_encoder_left);
	ROS_INFO("right encoder : %f", delta_encoder_right);
}

void twistCallback(const geometry_msgs::Twist::ConstPtr& msg) {
	v_robot_desired = msg->linear.x;
	w_robot_desired = msg->angular.z;
}

void estimateSpeed() {
	w_left_estimate = (delta_encoder_left * 2 * M_PI * control_frequency) / (ticks_per_rev);
	w_right_estimate = (delta_encoder_right * 2 * M_PI * control_frequency) / (ticks_per_rev);
	ROS_INFO("left est : %f", w_left_estimate);
	ROS_INFO("right est : %f", w_right_estimate);
}


void desiredSpeed() {
	w_left_desired = (v_robot_desired - w_robot_desired * (base / 2)) / wheel_radius;
	w_right_desired = (v_robot_desired + w_robot_desired * (base / 2)) / wheel_radius;
	ROS_INFO("left des : %f", w_left_desired);
	ROS_INFO("right des : %f", w_right_desired);
}

void setPWM(ras_lab1_msgs::PWM& msg) {
	double error_left = w_left_desired - w_left_estimate;
	int_error_left += error_left * dt;
	msg.PWM1 = int (alpha * error_left + beta * int_error_left);
	
	double error_right = w_right_desired - w_right_estimate;
	int_error_right += error_right * dt;
	msg.PWM2 = int (alpha * error_right + beta * int_error_right);
	
	ROS_INFO("error_left : %f", error_left);
	ROS_INFO("error_right : %f", error_right);
	ROS_INFO("int_error_left : %f", int_error_left);
	ROS_INFO("int_error_right : %f", int_error_right);
	ROS_INFO("pwm_left : %d", msg.PWM1);
	ROS_INFO("pwm_right : %d", msg.PWM2);
	
}


int main(int argc, char **argv)
{	
	ros::init(argc, argv, "motor_controller");
	ros::NodeHandle nh;
	ros::Rate rate(10);
	
	ros::Publisher pwm_pub = nh.advertise<ras_lab1_msgs::PWM>("/kobuki/pwm", 1);
	
	ros::Subscriber enc_sub = nh.subscribe<ras_lab1_msgs::Encoders>("/kobuki/encoders", 1, encoderCallback);
	
	ros::Subscriber twist_sub = nh.subscribe<geometry_msgs::Twist>("/motor_controller/twist", 1, twistCallback);
	
	ras_lab1_msgs::PWM pwm_msg;
	
	while (ros::ok()) {
	
		/*  
		Subscriber listens to the encoder and twist. Updates desired v and w, then updates the
		delta encoder values.
		*/
		ros::spinOnce();
		
		// Update desired and estimated speed. Then calculate PWM and set it.
		estimateSpeed();
		desiredSpeed();
		setPWM(pwm_msg);
		
		pwm_pub.publish(pwm_msg);
		
		rate.sleep();
		
		
	}
	
}
