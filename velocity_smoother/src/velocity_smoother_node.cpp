#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <cmath>
#include <velocity_smoother/velocity_smootherConfig.h>
#include <dynamic_reconfigure/server.h>

enum Method {decay=1, approch};

ros::Subscriber cmd_sub;
ros::Publisher cmd_pub;
int counter = 0;
double vx_1 = 0, vx_2 = 0, ax_1 = 0, ax_2 = 0;
float beta = 0.3;
float a, b, c;
Method averaging_method = decay;
bool smooth_zero;


//smoother in se2 space
void cmd_vel_callback(const geometry_msgs::TwistConstPtr &msg){
	double vx, ax;
	vx_2 = vx_1;
	ax_2 = ax_1;
	
	if(averaging_method == decay && (counter > 0 || smooth_zero)){
		vx = beta * vx_1 + (1-beta) * msg->linear.x;
		ax = beta * ax_1 + (1-beta) * msg->angular.z;
	}

	else if(averaging_method == approch && (counter > 1 || smooth_zero)){
		vx = a * vx_2 + b * vx_1 + c * msg->linear.x;
		ax = a * ax_2 + b * ax_1 + c * msg->angular.z;
	}
	
	
	geometry_msgs::Twist msg_out = (*msg);
	msg_out.linear.x = vx;
	msg_out.angular.z = ax;
	cmd_pub.publish(msg_out);
	vx_1 = vx;
	ax_1 = ax;
	counter++;
}

void reconfigure_callback(velocity_smoother::velocity_smootherConfig &config, uint32_t level) {
  ROS_INFO("Reconfigure Request: %d %f", config.method, config.beta);
  beta = config.beta;
  averaging_method = (Method)config.method;
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
	int tmp;
	nh.param<int>("averaging_method", tmp, 1);
	averaging_method = (Method)tmp;
	nh.param<bool>("smooth_with_zero", smooth_zero, false);
	
	if(averaging_method == approch)
		ROS_INFO("Executing %s with beta equal %f", "exponential decay", beta);
	else if(averaging_method == decay)
		ROS_INFO("Executing %s with beta equal %f", "exponential approach", beta);



	if(averaging_method != decay && averaging_method != approch)
		ROS_ERROR("%s", "velocity_smoother: Averging method is unknown.");
		
	dynamic_reconfigure::Server<velocity_smoother::velocity_smootherConfig> server;
	dynamic_reconfigure::Server<velocity_smoother::velocity_smootherConfig>::CallbackType f;

	f = boost::bind(&reconfigure_callback, _1, _2);
	server.setCallback(f);

	cmd_sub = nh.subscribe(input_topic, 10, cmd_vel_callback);
	cmd_pub = nh.advertise<geometry_msgs::Twist>(output_topic, 10);
	ros::spin();
	return 0;
}
