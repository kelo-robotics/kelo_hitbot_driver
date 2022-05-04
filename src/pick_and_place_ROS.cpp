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
