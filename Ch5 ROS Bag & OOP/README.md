# Chapter **5.**  ROS Bag and OOP

## Replay a Topic With ROS Bags
* A bag is a file format in ROS for storing ROS message data.
* `rosbag` command can record, replay bags.

```console
# Rosbag help
rosbag -h

# Run ros master and node
[alonzo@study ~]$ roscore
[alonzo@study ~]$ rosrun my_robot_tutorials hw_status_publisher.py
[alonzo@study ~]$ rosnode list
[alonzo@study ~]$ rostopic list
[alonzo@study ~]$ rostopic echo /my_robot/hardware_status

# Save the data in one topic
# rosbag record {topic name}
[alonzo@study ~]$ rosbag record /my_robot/hardware_status
# This will generate .bag file

# Replay topic
[alonzo@study ~]$ rosbag play 2018-06-16-08-31-32.bag
# rostopic /my_robot/hardware_status start printing
```

## Use OOP With ROS Python
```python
#!/usr/bin/env python

import rospy
from std_msgs.msg import Int64
from std_srvs.srv import SetBool


class NumberCounter:

	def __init__(self):
		self.counter = 0

		self.number_subscriber = rospy.Subscriber("/number", Int64, self.callback_number)

		self.pub = rospy.Publisher("/number_count", Int64, queue_size=10)

		self.reset_service = rospy.Service("/reset_counter", SetBool, self.callback_reset_counter)

	def callback_number(self, msg):
		self.counter += msg.data
		new_msg = Int64()
		new_msg.data = self.counter
		self.pub.publish(new_msg)

	def callback_reset_counter(self, req):
		if req.data:
			self.counter = 0
			return True, "Counter has been successfully reset"
		return False, "Counter has not been reset"


if __name__ == '__main__':
	rospy.init_node('number_counter')
	NumberCounter()
	rospy.spin()
```

## Use OOP With ROS C++
```cpp
#include <ros/ros.h>
#include <std_msgs/Int64.h>
#include <std_srvs/SetBool.h>

class NumberCounter {

	private:
	int counter;
	ros::Publisher pub;
	ros::Subscriber number_subscriber;
	ros::ServiceServer reset_service;

	public:
	NumberCounter(ros::NodeHandle *nh) {
		counter = 0;
		
		pub = nh->advertise<std_msgs::Int64>("/number_count", 10);
		
		number_subscriber = nh->subscribe("/number", 1000,
			&NumberCounter::callback_number, this);
		
		reset_service = nh->advertiseService("/reset_counter",
			&NumberCounter::callback_reset_counter, this);
	}

	void callback_number(const std_msgs::Int64& msg) {
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
};

int main (int argc, char **argv)
{
	ros::init(argc, argv, "number_counter");
	ros::NodeHandle nh;
	NumberCounter nc = NumberCounter(&nh);
	ros::spin();
}
```

## Work With Multiple Catkin Workspaces

* __Note :__
Only the last setup.bash in ~/.bashrc will be sourced.
Thus, source manually to overwrite enc params or modify bashrc.

