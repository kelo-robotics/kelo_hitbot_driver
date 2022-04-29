# kelo_hitbot_driver

Driver for hitbot Z-Arm S922 manipulator

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
Here is an example code for a simple pick and place application:

~~~ sh
int main(int argc, char** argv)
{
	HitbotDriver driver;
	driver.setupConnection(192.168.58.2, 8080);

	driver.movePTP(446.7,404.1,222.7,179.99,0.0,-40.0,0,0,100,180,100,0.000,0.000,0.000,0.000,0,0,0,0,0,0,0,0);
	driver.moveGripper(1,0,100,0,1000);
	driver.movePTP(446.7,404.1,122.7,179.99,0.0,-40.0,0,0,100,180,100,0.000,0.000,0.000,0.000,0,0,0,0,0,0,0,0);
	driver.moveGripper(1,100,100,0,1000);
	driver.movePTP(446.7,404.1,222.7,179.99,0.0,-40.0,0,0,100,180,100,0.000,0.000,0.000,0.000,0,0,0,0,0,0,0,0);

	driver.movePTP(96.3,543.3,222.7,179.99,0.0,-12.5,0,0,100,180,100,0.000,0.000,0.000,0.000,0,0,0,0,0,0,0,0);
	driver.movePTP(96.3,543.3,122.7,179.99,0.0,-12.5,0,0,100,180,100,0.000,0.000,0.000,0.000,0,0,0,0,0,0,0,0);
	driver.moveGripper(1,0,100,0,1000);
	driver.movePTP(96.3,543.3,222.7,179.99,0.0,-12.5,0,0,100,180,100,0.000,0.000,0.000,0.000,0,0,0,0,0,0,0,0);
	
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

The ROS topics and the related functions are listed in [commands.md](commands.md)
