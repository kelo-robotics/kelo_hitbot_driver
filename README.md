# kelo_hitbot_driver

Driver for hitbot manipulator

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

There are two methods to use the driver:
- C++ API. This method provides 
- ROS topics. The advantage of this method is the ease of use where the robot arm can be controlled simply by publishing ROS topics.
It also publishes the joint states, IO states and other information periodically.

### C++ API

First of all, create a HitbotDriver object and setup the TCP connection.

~~~ sh
HitbotDriver driver;
driver.setupConnection(<IP_ADDRESS>, <PORT>);
~~~

The default IP address of the robot arm is 192.168.58.2. 
Please make sure that the PC connected is configured properly.

### ROS topics
 
~~~ sh
roslaunch kelo_hitbot_driver robot.launch
~~~
