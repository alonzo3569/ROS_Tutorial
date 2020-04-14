# Chapter **2.**  ROS Service

## ROS Service

<div align=center>

<img src="https://github.com/alonzo3569/ROS/blob/master/Ch2%20ROS%20Service/ROS_Service_Concept3.png"/><br></br>
<img src="https://github.com/alonzo3569/ROS/blob/master/Ch2%20ROS%20Service/ROS_Service_Concept.png"/><br></br>
<img src="https://github.com/alonzo3569/ROS/blob/master/Ch2%20ROS%20Service/ROS_Service_Concept2.png"/><br></br>

</div>

* __Service message (Request/Response) :__

<div align=center>

<img src="https://github.com/alonzo3569/ROS/blob/master/Ch2%20ROS%20Service/Service_Msg_Type.png"/><br></br>

</div>

* __Python Server/Client :__
  * Python Server
```python
#!/usr/bin/env python

import rospy
from rospy_tutorials.srv import AddTwoInts 

def handle_add_two_ints(req):
    result = req.a + req.b
    rospy.loginfo("Sum of " + str(req.a) + " and " + str(req.b) + " is " + str(result))
    return result

if __name__ == '__main__':

    rospy.init_node('robot_news_radio_transmitter')
    rospy.loginfo("Add two ints server node created")

    service = rospy.Service("/add_two_ints", AddTwoInts, handle_add_two_ints) # name of the service, usually start with verb
                                                                              # wait for client AddTwoInts class object (int a, int b), in this code, it's req
    rospy.loginfo("Serrvice server has been started")
   
    rospy.spin()
```

<div align=center>

<img src="https://github.com/alonzo3569/ROS/blob/master/Ch2%20ROS%20Service/Service_Call_Example.png"/><br></br>
<b>Fig. Use rosservice call to poke msg to service "/add_two_ints"</b><br></br>
<img src="https://github.com/alonzo3569/ROS/blob/master/Ch2%20ROS%20Service/Service_Call_Example2.png"/><br></br>
<b>Fig. Service "/add_two_ints" response to incoming msg</b><br></br>

</div>

  * Python Client
```python
#!/usr/bin/env python

import rospy
from rospy_tutorials.srv import AddTwoInts 

if __name__ == '__main__':

    rospy.init_node('add_two_ints_client')
    rospy.wait_for_service("/add_two_ints") # Wait for service is setup

    try: # If service is not ready, catch exception
        add_two_ints = rospy.ServiceProxy("/add_two_ints", AddTwoInts) # Create client and give the type of client
        response = add_two_ints(2,6) # Call client, get AddTwoInts class object(int sum) from server and assign to response
        rospy.loginfo("Sum is: " + str(response.sum))
    except rospy.ServiceException as e:
        #rospy.logwarn("Service failed: ", str(e))
        rospy.loginfo(str(e))
```

<div align=center>

<img src="https://github.com/alonzo3569/ROS/blob/master/Ch2%20ROS%20Service/ROS_Service_Server.png"/>
<b>Fig. Run server and activate "/add_two_ints" service</b><br></br>
<img src="https://github.com/alonzo3569/ROS/blob/master/Ch2%20ROS%20Service/ROS_Service_Client.png"/>
<b>Fig. Run client</b><br></br>

</div>

* __C++ Server/Client :__
  * C++ Server
    1. Add library for service message (Remember to add executale and target link as well)
    ```console
    # pkg/src/CMakeLists.txt
    find_package(catkin REQUIRED COMPONENTS
      roscpp
      rospy
      std_msgs
      rospy_tutorials  <= library for service msg
    )

    # pkg/src/package.xml
    <build_depend>rospy_tutorials</build_depend>
    <exec_depend>rospy_tutorials</exec_depend>
    ```
    2. Edit C++ server
    ```cpp
    #include <ros/ros.h>
    #include <rospy_tutorials/AddTwoInts.h>

    bool handle_add_two_ints(rospy_tutorials::AddTwoInts::Request &req,
     rospy_tutorials::AddTwoInts::Response &res)
    {
    int result = req.a + req.b;
    ROS_INFO("%d + %d = %d", (int)req.a, (int)req.b, (int)result);
    res.sum = result;
    return true;
    }

    int main(int argc, char** argv){

        ros::init(argc, argv, "add_two_ints_server"); // Can't be the same with other node
        ros::NodeHandle nh; //To start the node

        ros::ServiceServer server = nh.advertiseService("/add_two_ints", handle_add_two_ints); // Service name & callback

        ros::spin();

    }
    ```
  * C++ Client
```cpp
#include <ros/ros.h>
#include <rospy_tutorials/AddTwoInts.h>

int main(int argc, char** argv){

  ros::init(argc, argv, "add_two_ints_client"); // Can't be the same with other node
  ros::NodeHandle nh;   // Start the node

  ros::ServiceClient client = nh.serviceClient<rospy_tutorials::AddTwoInts>("/add_two_ints");

  rospy_tutorials::AddTwoInts srv;

  srv.request.a = 12;
  srv.request.b = 5;
    
    if(client.call(srv)){ // If server return true
      //process data
      ROS_INFO("Returned sum is %d", (int)srv.response.sum);
    } 
    else{
      ROS_WARN("Service call failed");
    }

  ros::spin();

}
```

* __ROS Service command :__

<div align=center>

<img src="https://github.com/alonzo3569/ROS/blob/master/Ch2%20ROS%20Service/ROS_Service_Command.png"/><br></br>

</div>


* __Python example(Publisher/Subscriber/Service) :__
```python
#!/usr/bin/env python

import rospy
from std_msgs.msg import Int64 
from std_srvs.srv import SetBool # sudo apt-get install ros-melodic-std-srvs

counter = 0
pub = None

# Function for subscriber
def callback(num):
    rospy.loginfo(num)
    global counter
    counter += num.data
    pub.publish(counter)
    rospy.loginfo(counter)

# Service function
def handle_message(msg):
    #rospy.loginfo(msg.response.success) # error: input message doesn't include Response(success,message)
    #rospy.loginfo(msg.response.message)    
    if (msg.data):
        global counter
        counter = 0
        return True,"Counter reset success"
    else:
        return False,"Counter has not been reset"
        
if __name__ == '__main__':

    rospy.init_node('number_counter')

    sub = rospy.Subscriber("/number", Int64, callback)

    pub = rospy.Publisher("/number_count", Int64, queue_size=10)

    service = rospy.Service("/reset_number_count", SetBool, handle_message) # name of the service, usually start with verb

    rospy.spin() #Keep running callbcak function
```
* __Python example service message type :__

<div align=center>

<img src="https://github.com/alonzo3569/ROS/blob/master/Ch2%20ROS%20Service/Python_Service_Example.png"/><br></br>

</div>

* __Python example output :__

<div align=center>

<img src="https://github.com/alonzo3569/ROS/blob/master/Ch2%20ROS%20Service/Python_Service_Example2.png"/><br></br>
<img src="https://github.com/alonzo3569/ROS/blob/master/Ch2%20ROS%20Service/Python_Service_Example3.png"/><br></br>

</div>

