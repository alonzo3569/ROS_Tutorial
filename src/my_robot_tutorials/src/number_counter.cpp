#include <ros/ros.h>
#include <std_msgs/Int64.h>
#include <std_srvs/SetBool.h>

int counter = 0;
ros::Publisher pub;

void callback_number(const std_msgs::Int64& msg)
{
	counter += msg.data;
	std_msgs::Int64 new_msg;
	new_msg.data = counter;
	pub.publish(new_msg);
}

bool callback_reset_counter(std_srvs::SetBool::Request &req,
							std_srvs::SetBool::Response &res)
{
	if (req.data) {
		counter = 0;
		res.success = true;
		res.message = "Counter has been successfully reset";
	}
	else {
		res.success = false;
		res.message = "Counter has not been reset";
	}

	return true;
}


int main (int argc, char **argv)
{
	ros::init(argc, argv, "number_counter");
	ros::NodeHandle nh;

	ros::Subscriber sub = nh.subscribe("/number", 1000, callback_number);

	pub = nh.advertise<std_msgs::Int64>("/number_count", 10);

	ros::ServiceServer reset_service = nh.advertiseService("/reset_counter", 
		callback_reset_counter);

	ros::spin();
}
