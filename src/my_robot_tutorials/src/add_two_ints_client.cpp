#include <ros/ros.h>
#include <rospy_tutorials/AddTwoInts.h>

int main (int argc, char **argv)
{
	ros::init(argc, argv, "add_two_ints_client");
	ros::NodeHandle nh;

	ros::ServiceClient client = 
		nh.serviceClient<rospy_tutorials::AddTwoInts>("/add_two_ints");

	rospy_tutorials::AddTwoInts srv;
	srv.request.a = 12;
	srv.request.b = 5;

	if (client.call(srv)) {
		ROS_INFO("Returned sum is %d", (int)srv.response.sum);
	}
	else {
		ROS_WARN("Service call failed");
	}
}
