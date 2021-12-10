#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <cmath>


ros::Subscriber cmd_sub;
ros::Publisher cmd_pub;
int counter = 0;
double vx_1 = 0, vx_2 = 0, ax_1 = 0, ax_2 = 0;
float beta = 0.3;
float a, b, c;
int averaging_method;
bool smooth_zero;

void cmd_vel_callback(const geometry_msgs::TwistConstPtr &msg){
	double vx, ax;
	vx_2 = vx_1;
	ax_2 = ax_1;
	
	if(averaging_method == 1 && (counter > 0 || smooth_zero)){
		vx = beta * vx_1 + (1-beta) * msg->linear.x;
		ax = beta * ax_1 + (1-beta) * msg->angular.z;
	}

	else if(averaging_method == 2 && (counter > 1 || smooth_zero)){
		vx = a * vx_2 + b * vx_1 + c * msg->linear.x;
		ax = a * ax_2 + b * ax_1 + c * msg->angular.z;
	}
	
	ROS_INFO("Original velocity: %f, Output velocity: %f", msg->linear.x, vx);
	
	
	geometry_msgs::Twist msg_out = (*msg);
	msg_out.linear.x = vx;
	msg_out.angular.z = ax;
	cmd_pub.publish(msg_out);
	vx_1 = vx;
	ax_1 = ax;
	counter++;
}

int main(int argc, char *argv[]){

	ros::init(argc, argv, "velocity_smoother");
	ros::NodeHandle nh;

	std::string input_topic, output_topic;
	
    a = beta / (2-beta);
    b = (1-beta) / (2-beta);
    c = (1-beta) / (2-beta);
    
	nh.param<std::string>("twist_topic", input_topic, "/cmd_vel");
	nh.param<std::string>("smooth_twist_topic", output_topic, "/cmd_vel_smoothed");
	if(!ros::param::get("~/beta", beta))
		ROS_ERROR("%s","Beta parameter must be defined");
		//nh.param("beta", beta, 0.3);
	nh.param<int>("averaging_method", averaging_method, 1);
	nh.param<bool>("smooth_with_zero", smooth_zero, false);
	
	if(averaging_method == 1)
		ROS_INFO("Executing %s with beta equal %f", "exponential decay", beta);
	else if(averaging_method == 2)
		ROS_INFO("Executing %s with beta equal %f", "exponential approach", beta);



	if(averaging_method != 1 && averaging_method != 2)
		ROS_ERROR("%s", "velocity_smoother: Averging method is unknown.");

	cmd_sub = nh.subscribe(input_topic, 10, cmd_vel_callback);
	cmd_pub = nh.advertise<geometry_msgs::Twist>(output_topic, 10);
	ros::spin();
	return 0;
}
