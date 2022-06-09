# kelo_hitbot_driver

Driver for [hitbot Z-Arm S922 manipulator](https://www.hitbotrobot.com/product/s922-robotic-arm/)

## Installation

Install the kelo_hitbot_driver package using the following steps:

~~~ sh
cd ~/<CATKIN_WORKSPACE>/src
git clone 
git clone https://github.com/kelo-robotics/kelo_hitbot_driver.git
catkin build kelo_hitbot_driver

source ~/catkin_ws/devel/setup.bash
~~~

## Usage

There are two methods to use this package:
- ROS topics. The advantage of this method is the ease of use where the robot arm can be controlled simply by publishing ROS topics.
It also publishes the joint states, IO states and other information periodically.
- C++ API. This method provides communication with the robot arm without any dependency to ROS, and therefore has less overhead.
The main disadvantage of this method is that the joint state and sensor data collection need to be done manually.

### C++ API

The explanation of the functions are documented in [HitbotDriver.h](include/kelo_hitbot_driver/HitbotDriver.h).
Here is an example code for a simple [pick and place](src/examples/pick_and_place.cpp) application:

~~~ sh
#include "kelo_hitbot_driver/HitbotDriver.h"

int main(int argc, char** argv)
{
	HitbotDriver driver;
	driver.setupConnection("192.168.58.2", 8080);

	std::vector<float> pos1(6);
	pos1[0] = 446.7;
	pos1[1] = 404.1;
	pos1[2] = 222.7;
	pos1[3] = 179.99;
	pos1[4] = 0.0;
	pos1[5] = -40.0;
	std::vector<float> pos2 = pos1;
	pos2[2] = 122.7;
	std::vector<float> pos3 = pos1;
	pos3[0] = 96.3;
	pos3[1] = 543.3;
	std::vector<float> pos4 = pos3;
	pos4[2] = 122.7;
	std::vector<float> extAxis(4, 0);
	std::vector<float> offset(6, 0);

	driver.movePTP(pos1, 0, 0, 100, 100, 100, extAxis, 0, 0, offset);
	driver.moveGripper(1, 0, 100, 0, 1000);
	driver.moveLin(pos2, 0, 0, 100, 100, 100, 0, extAxis, 0, 0, offset);
	driver.moveGripper(1, 100, 100, 0, 1000);
	driver.moveLin(pos1, 0, 0, 100, 100, 100,0, extAxis, 0, 0, offset);
	driver.moveLin(pos3, 0, 0, 100, 100, 100, 0, extAxis, 0, 0, offset);
	driver.moveLin(pos4, 0, 0, 100, 100, 100, 0, extAxis, 0, 0, offset);
	driver.moveGripper(1, 0, 100,0, 1000);
	driver.moveLin(pos3, 0, 0, 100, 100, 100, 0, extAxis, 0, 0, offset);
	
	return 0;
}
~~~

First of all, create a HitbotDriver object and setup the TCP connection.
The default IP address of the robot arm is 192.168.58.2 and the port is 8080. 
Please make sure that the PC network settings is configured properly.

Once the communication is established, the commands can be executed.
First the arm is positioned above the object and the gripper is opened.
The arm TCP is then moved down for 10cm and then the gripper is closed.
The object is then lifted back for 10cm and moved sideways to the new location.
The object is then placed back to the table, the gripper is reopened and then the arm goes back up for 10cm.


### ROS topics

The hitbot ROS interface can be started with:
 
~~~ sh
roslaunch kelo_hitbot_driver robot.launch
~~~

It will start a ROS node named hitbot_driver which will publish joint states and sensor data periodically and
listens to commands sent via ROS topics. The ROS topics and the related functions are listed in [commands.md](commands.md).
Here is an example of a [pick and place](src/examples/pick_and_place_ROS.cpp) application using ROS topics:

~~~ sh

#include <ros/ros.h>
#include <kelo_hitbot_driver/PTP.h>
#include <kelo_hitbot_driver/GripperMotion.h>

int main(int argc, char** argv)
{
	ros::init (argc, argv, "pick_and_place");
	ros::NodeHandle nh("hitbot_driver");
	
	ros::Publisher movePub = nh.advertise<kelo_hitbot_driver::PTP>("move_PTP", 2);		
	ros::Publisher gripperPub = nh.advertise<kelo_hitbot_driver::GripperMotion>("move_gripper", 2);		
	
	std::vector<float> extAxesPos(4, 0);
	
	kelo_hitbot_driver::PTP msg1;
	msg1.command.pose.x = 446.7;
	msg1.command.pose.y = 404.1;
	msg1.command.pose.z = 222.7;
	msg1.command.pose.rx = 179.99;
	msg1.command.pose.ry = 0.0;
	msg1.command.pose.rz = -40.0;
	msg1.command.toolID = 0;
	msg1.command.workpieceID = 0;
	msg1.command.speed = 100;
	msg1.command.acc = 100;
	msg1.command.extAxesPos = extAxesPos;
	msg1.ovl = 100;
	msg1.blendTime = 0;
	msg1.offsetFlag = 0;
	msg1.dt.x = 0;
	msg1.dt.y = 0;
	msg1.dt.z = 0;
	msg1.dt.rx = 0;
	msg1.dt.ry = 0;
	msg1.dt.rz = 0;
	
	kelo_hitbot_driver::PTP msg2 = msg1;
	msg2.command.pose.z = 122.7;

	kelo_hitbot_driver::PTP msg3 = msg1;
	msg3.command.pose.x = 96.3;
	msg3.command.pose.y = 543.3;

	kelo_hitbot_driver::PTP msg4 = msg3;
	msg4.command.pose.z = 122.7;
	
	kelo_hitbot_driver::GripperMotion openMsg;
	openMsg.ID = 1;
	openMsg.pos = 0;
	openMsg.speed = 100;
	openMsg.force = 0;
	openMsg.maxTime = 1000;
	
	kelo_hitbot_driver::GripperMotion closeMsg = openMsg;
	closeMsg.pos = 100;
	
	movePub.publish(msg1);
	ros::Duration(0.1).sleep();
	gripperPub.publish(openMsg);
	ros::Duration(0.1).sleep();
	movePub.publish(msg2);
	ros::Duration(0.1).sleep();
	gripperPub.publish(closeMsg);
	ros::Duration(0.1).sleep();
	movePub.publish(msg1);
	ros::Duration(0.1).sleep();
	movePub.publish(msg3);
	ros::Duration(0.1).sleep();
	movePub.publish(msg4);
	ros::Duration(0.1).sleep();
	gripperPub.publish(openMsg);
	ros::Duration(0.1).sleep();
	movePub.publish(msg3);
	ros::Duration(0.1).sleep();

	ros::shutdown();
}

~~~


