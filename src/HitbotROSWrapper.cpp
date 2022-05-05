#include <ros/ros.h>
#include "kelo_hitbot_driver/HitbotDriver.h"
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Empty.h>
#include <kelo_hitbot_driver/AnalogOutput.h>
#include <kelo_hitbot_driver/Arc.h>
#include <kelo_hitbot_driver/Collision.h>
#include <kelo_hitbot_driver/DigitalOutput.h>
#include <kelo_hitbot_driver/Jog.h>
#include <kelo_hitbot_driver/JointState.h>
#include <kelo_hitbot_driver/Lin.h>
#include <kelo_hitbot_driver/LT.h>
#include <kelo_hitbot_driver/PTP.h>
#include <kelo_hitbot_driver/Servo.h>
#include <kelo_hitbot_driver/Tool.h>
#include <kelo_hitbot_driver/WaitAnalog.h>
#include <kelo_hitbot_driver/WaitDigital.h>
#include <kelo_hitbot_driver/Weave.h>
#include <kelo_hitbot_driver/Weld.h>
#include <kelo_hitbot_driver/Workpiece.h>
#include <kelo_hitbot_driver/GripperAct.h>
#include <kelo_hitbot_driver/GripperMotion.h>
#include <kelo_hitbot_driver/Config.h>
#include <kelo_hitbot_driver/AxleSensorReg.h>
#include <kelo_hitbot_driver/GuardFT.h>
  
HitbotDriver driver;

std::string IP_address = "192.168.58.2";
int port = 8080;
int auxPort = 8082;

ros::Publisher jointStatePublisher;
ros::Publisher TCPPosePublisher;
ros::Publisher TCPTargetPublisher;
ros::Publisher TCPFlangePublisher;
ros::Publisher TCPOffsetPublisher;
ros::Publisher TCPNumPublisher;
ros::Publisher payloadPublisher;
ros::Publisher payloadCogPublisher;
ros::Publisher jointLimitPublisher;
ros::Publisher FTConfigPublisher;
ros::Publisher axleConfigPublisher;
ros::Publisher DIPublisher;
ros::Publisher toolDIPublisher;
ros::Publisher AIPublisher;
ros::Publisher toolAIPublisher;
ros::Publisher toolCoordPublisher;
ros::Publisher wobjCoordPublisher;
		
void setParams(ros::NodeHandle& nh) {
	nh.getParam("IP_address", IP_address);
	nh.getParam("port", port);
	nh.getParam("aux_port", auxPort);
	
	std::vector<double> negJointLimits;
	std::vector<double> posJointLimits;
	int level;
	if(nh.getParam("negative_joint_limits", negJointLimits)) {
		driver.setLimitNegative(negJointLimits);
	}
	if(nh.getParam("positive_joint_limits", posJointLimits)) {
		driver.setLimitPositive(posJointLimits);
	}
	if(nh.getParam("anti_collision_level", level)) {
		driver.setAntiCollision(level);
	}
}

std::vector<float> convertAxesToVector(kelo_hitbot_driver::Axes axes) {
	std::vector<float> values(6, 0);
	values[0] = axes.x;
	values[1] = axes.y;
	values[2] = axes.z;
	values[3] = axes.rx;
	values[4] = axes.ry;
	values[5] = axes.rz;
	return values;
}

std::vector<double> convertAxesDToVector(kelo_hitbot_driver::AxesD axes) {
	std::vector<double> values(6, 0);
	values[0] = axes.x;
	values[1] = axes.y;
	values[2] = axes.z;
	values[3] = axes.rx;
	values[4] = axes.ry;
	values[5] = axes.rz;
	return values;
}

kelo_hitbot_driver::Axes convertVectorToAxis(std::vector<float> vec) {
	kelo_hitbot_driver::Axes axes;
	if (vec.size() == 6) {
		axes.x = vec[0];
		axes.y = vec[1];
		axes.z = vec[2];
		axes.rx = vec[3];
		axes.ry = vec[4];
		axes.rz = vec[5];
	}
	return axes;
}

//
// Simple movement
//

void movePTPCallback(const kelo_hitbot_driver::PTP msg) {
	driver.movePTP(convertAxesToVector(msg.command.pose), msg.command.toolID, msg.command.workpieceID,
					msg.command.speed, msg.command.acc, msg.ovl, msg.command.extAxesPos, msg.blendTime,
					msg.offsetFlag, convertAxesToVector(msg.offset));
	std::cout << "start PTP movement" << std::endl;
}

void moveLinCallback(const kelo_hitbot_driver::Lin msg) {
	driver.moveLin(convertAxesToVector(msg.command.pose), msg.command.toolID, msg.command.workpieceID,
					msg.command.speed, msg.command.acc, msg.ovl, msg.blendRadius, msg.command.extAxesPos, 
					msg.searchFlag, msg.offsetFlag, convertAxesToVector(msg.offset));
	std::cout << "start linear movement" << std::endl;
}

void moveArcCallback(const kelo_hitbot_driver::Arc msg) {
	driver.moveArc(convertAxesToVector(msg.command1.pose), msg.command1.toolID, msg.command1.workpieceID,
					msg.command1.speed, msg.command1.acc, msg.command1.extAxesPos, convertAxesToVector(msg.command2.pose), 
					msg.command2.toolID, msg.command2.workpieceID, msg.command2.speed, msg.command2.acc, msg.command2.extAxesPos,
					msg.ovl, msg.blendRadius, msg.offsetFlag, convertAxesToVector(msg.offset));
	std::cout << "start arc movement" << std::endl;
}

void moveJogCallback(const kelo_hitbot_driver::Jog msg) {
	driver.startJog(msg.command, msg.ID, msg.direction, msg.vel, msg.acc, msg.maxDistance);
	std::cout << "start jog movement" << std::endl;
}

void moveServoCallback(const kelo_hitbot_driver::Servo msg) {
	driver.servoJ(msg.jointAngle, msg.acc, msg.vel, msg.cycle, msg.lookaheadTime, msg.gain);
	std::cout << "start servo movement" << std::endl;
}

//
// Procedure
//

void procedureCallback(const std_msgs::String& msg) {
	if (msg.data == "start") {
		driver.startProcedure();
		std::cout << "start automatic procedure" << std::endl;
	} else if (msg.data == "stop") {
		driver.stopProcedure();
		std::cout << "stop automatic procedure" << std::endl;
	} else if (msg.data == "pause") {
		driver.pauseProcedure();
		std::cout << "pause automatic procedure" << std::endl;
	} else if (msg.data == "resume") {
		driver.resumeProcedure();
		std::cout << "resume automatic procedure" << std::endl;
	} else
		std::cout << "Automatic procedure command is invalid" << std::endl;
}

void resetCallback(const std_msgs::Empty msg) {
	driver.resetErrors();
	std::cout << "resetting all error flags" << std::endl;
}

void stopCallback(const std_msgs::Empty msg) {	
	driver.stopJog();
	driver.stopLine();
	driver.stopTool();
	driver.stopWorkpiece();
	std::cout << "stopping" << std::endl;
}

//
// IO
//

void publishIOStatus() {
	std_msgs::Int32MultiArray DIMsg;
	std_msgs::Int32MultiArray toolDIMsg;
	
	for (unsigned int i = 0; i < 16; i++) {
		if (driver.getDI(i))
			DIMsg.data.push_back(1);
		else
			DIMsg.data.push_back(0);
	}
	
	for (unsigned int i = 0; i < 5; i++) {
		if (driver.getToolDI(i))
			toolDIMsg.data.push_back(1);
		else
			toolDIMsg.data.push_back(0);
	}
	DIPublisher.publish(DIMsg);
	toolDIPublisher.publish(toolDIMsg);
	
	std_msgs::Float32MultiArray AIMsg;
	std_msgs::Float32MultiArray toolAIMsg;
	for (unsigned int i = 0; i < 2; i++) {
		AIMsg.data.push_back(driver.getAI(i)[0]);
	}
	toolAIMsg.data.push_back(driver.getToolAI(0)[0]);
	AIPublisher.publish(AIMsg);
	toolAIPublisher.publish(toolAIMsg);
}

void waitDICallback(const kelo_hitbot_driver::WaitDigital msg) {
	if (msg.type == 0) {
		driver.waitDI(msg.ID, msg.value, msg.waitTime, msg.alarm);
		std::cout << "waiting for digital input " << msg.ID << " for " << msg.waitTime << "ms" << std::endl; 
	} else if (msg.type == 1) {
		driver.waitToolDI(msg.ID, msg.value, msg.waitTime, msg.alarm);
		std::cout << "waiting for tool digital input " << msg.ID << " for " << msg.waitTime << "ms" << std::endl;
	}
}

void waitAICallback(const kelo_hitbot_driver::WaitAnalog msg) {
	if (msg.type == 0) {
		driver.waitAI(msg.ID, msg.sign, msg.value, msg.waitTime, msg.alarm);
		std::cout << "waiting for analog input " << msg.ID << " for " << msg.waitTime << "ms" << std::endl; 
	} else if (msg.type == 1) {
		driver.waitToolAI(msg.ID, msg.sign, msg.value, msg.waitTime, msg.alarm);
		std::cout << "waiting for tool analog input " << msg.ID << " for " << msg.waitTime << "ms" << std::endl;
	}
}

void setDOCallback(const kelo_hitbot_driver::DigitalOutput msg) {
	if (msg.type == 0) {
		driver.setDO(msg.ID, msg.value, msg.smooth);
		std::cout << "set digital output " << msg.ID << " to " << msg.value << std::endl; 
	} else if (msg.type == 1) {
		driver.setToolDO(msg.ID, msg.value, msg.smooth);
		std::cout << "set tool digital output " << msg.ID << " to " << msg.value << std::endl; 
	}
}

void setAOCallback(const kelo_hitbot_driver::AnalogOutput msg) {
	if (msg.type == 0) {
		driver.setAO(msg.ID, msg.value);
		std::cout << "set analog output " << msg.ID << " to " << msg.value << std::endl; 
	} else if (msg.type == 1) {
		driver.setToolAO(msg.ID, msg.value);
		std::cout << "set tool analog output " << msg.ID << " to " << msg.value << std::endl; 
	}
}

//
// Configurations
//

void modeCallback(const std_msgs::Int32& msg) {
	if (msg.data == 0) {
		driver.switchMode(msg.data);
		std::cout << "switch to automatic mode" << std::endl;
	} else if (msg.data == 1) {
		driver.switchMode(msg.data);
		std::cout << "switch to manual mode" << std::endl;
	} else {
		std::cout << "unknown operation mode" << std::endl;
	}
}

void setSpeedCallback(const std_msgs::UInt8 msg) {
	driver.setSpeed(msg.data);
	std::cout << "set speed to " << msg.data << std::endl;
}

void dragTeachCallback(const std_msgs::UInt8 msg) {
	if (msg.data == 0 || msg.data == 1) {
		driver.switchDragTeach(msg.data);
		if (msg.data == 1)
			std::cout << "set drag teach switch on " << std::endl;
		else
			std::cout << "set drag teach switch off " << std::endl;
	} else 
		std::cout << "set drag teach parameter invalid" << std::endl;
}

void installPosCallback(const std_msgs::UInt8 msg) {
	if (msg.data >=0 && msg.data <= 2) {
		driver.setRobotInstallPos(msg.data);
		if (msg.data == 0)
			std::cout << "robot is mounted upright" << std::endl;
		if (msg.data == 1)
			std::cout << "robot is mounted sideways" << std::endl;
		if (msg.data == 2)
			std::cout << "robot is mounted upside down" << std::endl;
	}
}

void setToolPointCallback(const std_msgs::Int32 msg) {
	driver.setToolPoint(msg.data);
	std::cout << "set tool point on joint " << msg.data << std::endl;
}

void setToolCoordCallback(const kelo_hitbot_driver::Tool msg) {
	driver.setToolCoord(msg.ID, convertAxesDToVector(msg.pose), msg.type, msg.install);
	std::cout << "setting tool coordinate" << std::endl;
}

void setWObjCoordPointCallback(const std_msgs::Int32 msg) {
	driver.setWObjCoordPoint(msg.data);
	std::cout << "set object point on " << msg.data << std::endl;
}

void setWObjCoordCallback(const kelo_hitbot_driver::Workpiece msg) {
	driver.setWObjCoord(msg.ID, convertAxesDToVector(msg.pose));
	std::cout << "setting object coordinate" << std::endl;
}

void setToolListCallback(const kelo_hitbot_driver::Tool msg) {
	driver.setToolList(msg.ID, convertAxesDToVector(msg.pose), msg.type, msg.install);
	std::cout << "setting tool list" << std::endl;
}

void setWObjListCallback(const kelo_hitbot_driver::Workpiece msg) {
	driver.setWObjList(msg.ID, convertAxesDToVector(msg.pose));
	std::cout << "setting object list" << std::endl;
}

//
// Welding
//

void startWeldingCallback(const kelo_hitbot_driver::Weld msg) {
	driver.startArc(msg.ID, msg.waitingTime);
	std::cout << "start welding profile number " << msg.ID << "with maximum waiting time " << msg.waitingTime << "ms" << std::endl;
}

void stopWeldingCallback(const kelo_hitbot_driver::Weld msg) {
	driver.endArc(msg.ID, msg.waitingTime);
	std::cout << "end welding profile number " << msg.ID << "with maximum waiting time " << msg.waitingTime << "ms" << std::endl;
}

void laserOnCallback(const std_msgs::Int32 msg) {
	driver.laserOn(msg.data);
	std::cout << "turning on laser " << msg.data << std::endl;
}

void laserOffCallback(const std_msgs::Empty msg) {
	driver.laserOff();
	std::cout  << "turning off laser" << std::endl;
}

void trackLTOnCallback(const std_msgs::Int32 msg) {
	driver.trackLTOn();
	std::cout << "turning on LT tracking" << std::endl;
}

void trackLTOffCallback(const std_msgs::Empty msg) {
	driver.trackLTOff();
	std::cout << "turning off LT tracking" << std::endl;
}

void startLTSearchCallback(const kelo_hitbot_driver::LT msg) {
	driver.startLTSearch(msg.direction, msg.vel, msg.distance, msg.maxTime);
	std::cout << "start LT search" << std::endl;
}

void stopLTSearchCallback(const std_msgs::Empty msg) {
	driver.stopLTSearch();
	std::cout << "stop LT search" << std::endl;
}

//
// Auxiliary
//

void weaveParamCallback(const kelo_hitbot_driver::Weave msg) {
	driver.setWeaveParam(msg.ID, msg.type, msg.freq, msg.rang, msg.lst, msg.rst);
	std::cout << "set new weave param" << std::endl;
}

void startWeaveCallback(const std_msgs::UInt8 msg) {
	driver.startWeave(msg.data);
	std::cout << "start weave" << std::endl;
}

void endWeaveCallback(const std_msgs::UInt8 msg) {
	driver.endWeave(msg.data);
	std::cout << "end weave" << std::endl;
}

//
// Function
//

void publishConfig() {
	kelo_hitbot_driver::Config FTMsg;
	std::vector<int> FTConfig = driver.getFTConfig();
	FTMsg.ID = FTConfig[0];
	FTMsg.company = FTConfig[1];
	FTMsg.device = FTConfig[2];
	FTMsg.softwareVersion = FTConfig[3];
	FTMsg.bus = FTConfig[4];
	FTConfigPublisher.publish(FTMsg);
	
	kelo_hitbot_driver::Config axleMsg;
	std::vector<int> axleConfig = driver.getAxleSensorConfig();
	axleMsg.ID = axleConfig[0];
	axleMsg.company = axleConfig[1];
	axleMsg.device = axleConfig[2];
	axleMsg.softwareVersion = axleConfig[3];
	axleMsg.bus = axleConfig[4];
	axleConfigPublisher.publish(axleMsg);
}

void activateGripperCallback(const kelo_hitbot_driver::GripperAct msg) {
	driver.activateGripper(msg.ID, msg.act);
	std::cout << "activate gripper " << msg.ID << std::endl;
}

void moveGripperCallback(const kelo_hitbot_driver::GripperMotion msg) {
	driver.moveGripper(msg.ID, msg.pos, msg.speed, msg.force, msg.maxTime);
	std::cout << "move gripper " << msg.ID << std::endl;
}

void configureAxleSensorCallback(const kelo_hitbot_driver::Config msg) {
	driver.configureAxleSensor(msg.ID, msg.company, msg.device, msg.softwareVersion, msg.bus);
	std::cout << "configure axle sensor" << std::endl;
}

void activateAxleSensorCallback(const std_msgs::Int32 msg) {
	driver.activateAxleSensor(msg.data);
	if (msg.data == 0)
		std::cout << "deactivate axle sensor" << std::endl;
	if (msg.data == 1)
		std::cout << "activate axle sensor" << std::endl;
}

void writeAxleSensorCallback(const kelo_hitbot_driver::AxleSensorReg msg) {
	driver.writeAxleSensorReg(msg.addr, msg.regHighAddr, msg.regLowAddr, msg.regNum,msg.data1, msg.data2);
	std::cout << "write axle sensor register" << std::endl;
}

void setFTConfigCallback(const kelo_hitbot_driver::Config msg) {
	driver.setFTConfig(msg.ID, msg.company, msg.device, msg.softwareVersion, msg.bus);
	std::cout << "set FT config" << std::endl;
}

void activateFTCallback(const std_msgs::Int32 msg) {
	driver.activateFT(msg.data);
	if (msg.data == 0)
		std::cout << "deactivate FT" << std::endl;
	if (msg.data == 1)
		std::cout << "activate FT" << std::endl;
}

void guardFTCallback(const kelo_hitbot_driver::GuardFT msg) {
	driver.guardFT(msg.flag, msg.isSelect, convertAxesDToVector(msg.force));
	std::cout << "guard FT" << std::endl;
}

void setFTRCSCallback(const std_msgs::Int32 msg) {
	if (msg.data == 0 || msg.data == 1) {
		driver.setFTRCS(msg.data);
		if (msg.data == 0)
			std::cout << "using tool coordinate system" << std::endl;
		if (msg.data == 1)	
			std::cout << "using base coordinate system" << std::endl;
	} else
		std::cout << "unknown coordinate system" << std::endl;
}

void setFTZeroCallback(const std_msgs::UInt8 msg) {
	if (msg.data == 0 || msg.data == 1) {
		driver.setFTZero(msg.data);
		std::cout << "set FT zero to " << msg.data << std::endl;
	} else
		std::cout << "FT zero parameter is unknown" << std::endl;
}

//
// States
//

void publishJointStates() {
	kelo_hitbot_driver::JointState stateMsg;
	stateMsg.position = driver.getJointPosDegree();
	stateMsg.velocity = driver.getJointSpeedsDegree();
	stateMsg.torque = driver.getJointTorques();
	stateMsg.gripperReady = driver.getGripperMotionDone()[0];
	jointStatePublisher.publish(stateMsg);
	
	std_msgs::Float64MultiArray jointMsg;
	jointMsg.data = driver.getJointSoftLimitDeg();
	jointLimitPublisher.publish(jointMsg);
}

void publishTCPInfo() {
	kelo_hitbot_driver::Axes poseMsg = convertVectorToAxis(driver.getActualTCPPose());
	TCPPosePublisher.publish(poseMsg);
	
	kelo_hitbot_driver::Axes targetMsg = convertVectorToAxis(driver.getTargetTCPPose());
	TCPTargetPublisher.publish(targetMsg);
	
	kelo_hitbot_driver::Axes offsetMsg = convertVectorToAxis(driver.getTCPOffset());
	TCPOffsetPublisher.publish(offsetMsg);	
	
	kelo_hitbot_driver::Axes flangePoseMsg = convertVectorToAxis(driver.getActualToolFlangePose());
	TCPFlangePublisher.publish(flangePoseMsg);
	
	std_msgs::Int32 numMsg;
	std::vector<int> num = driver.getActualTCPNum();
	numMsg.data = num[0];
	TCPNumPublisher.publish(numMsg);
}

void publishPayload() {
	std_msgs::Float32 payloadMsg;
	std::vector<float> payload = driver.getTargetPayload();
	payloadMsg.data = payload[0];
	payloadPublisher.publish(payloadMsg);
	
	std_msgs::Float32MultiArray cogMsg;
	cogMsg.data = driver.getTargetPayloadCog();
	payloadCogPublisher.publish(cogMsg);	
}

void publishCoordinateSystem() {
	kelo_hitbot_driver::Axes toolMsg = convertVectorToAxis(driver.computeTool());
	toolCoordPublisher.publish(toolMsg);
	
	kelo_hitbot_driver::Axes WObjMsg = convertVectorToAxis(driver.computeWObjCoord());
	wobjCoordPublisher.publish(WObjMsg);
}

int main(int argc, char** argv)
{
	ros::init (argc, argv, "hitbot_driver");
	ros::NodeHandle nh("hitbot_driver");
	
	setParams(nh);
	
	//Publishers
	jointStatePublisher = nh.advertise<kelo_hitbot_driver::JointState>("joint_state", 2);
	TCPPosePublisher = nh.advertise<kelo_hitbot_driver::Axes>("TCP_pose", 2);
	TCPTargetPublisher = nh.advertise<kelo_hitbot_driver::Axes>("TCP_target", 2);
	TCPFlangePublisher = nh.advertise<kelo_hitbot_driver::Axes>("TCP_flange_pose", 2);
	TCPOffsetPublisher = nh.advertise<kelo_hitbot_driver::Axes>("TCP_offset", 2);
	TCPNumPublisher = nh.advertise<std_msgs::Int32>("TCP_num", 2);
	payloadPublisher =  nh.advertise<std_msgs::Float32>("target_payload", 2);
	payloadCogPublisher = nh.advertise<std_msgs::Float32MultiArray>("target_payload_cog", 2);
	jointLimitPublisher = nh.advertise<std_msgs::Float64MultiArray>("joint_limits", 2);
	FTConfigPublisher = nh.advertise<kelo_hitbot_driver::Config>("FT_config", 2);
	axleConfigPublisher = nh.advertise<kelo_hitbot_driver::Config>("axle_sensor_config", 2);
	DIPublisher = nh.advertise<std_msgs::Int32MultiArray>("digital_input", 2);
	toolDIPublisher = nh.advertise<std_msgs::Int32MultiArray>("tool_digital_input", 2);
	AIPublisher = nh.advertise<std_msgs::Float32MultiArray>("analog_input", 2);
	toolAIPublisher = nh.advertise<std_msgs::Float32MultiArray>("tool_analog_input", 2);
	toolCoordPublisher = nh.advertise<kelo_hitbot_driver::Axes>("tool_coordinate_frame", 2);
	wobjCoordPublisher = nh.advertise<kelo_hitbot_driver::Axes>("object_coordinate_frame", 2);
			
	//Subscribers
	ros::Subscriber movePTPSubscriber = nh.subscribe("move_PTP", 1000, movePTPCallback);
	ros::Subscriber moveLinSubscriber = nh.subscribe("move_linear", 1000, moveLinCallback);
	ros::Subscriber moveArcSubscriber = nh.subscribe("move_arc", 1000, moveArcCallback);
	ros::Subscriber moveJogSubscriber = nh.subscribe("move_jog", 1000, moveJogCallback);
	ros::Subscriber moveServoSubscriber = nh.subscribe("move_servo", 1000, moveServoCallback);
	ros::Subscriber automaticSubscriber = nh.subscribe("automatic_procedure", 1000, procedureCallback);
	ros::Subscriber resetSubscriber = nh.subscribe("reset_error", 1000, resetCallback);
	ros::Subscriber stopSubscriber = nh.subscribe("stop", 1000, stopCallback);
	ros::Subscriber waitDISubscriber= nh.subscribe("wait_DI", 1000, waitDICallback);
	ros::Subscriber waitAISubscriber= nh.subscribe("wait_AI", 1000, waitAICallback);
	ros::Subscriber setDOSubscriber= nh.subscribe("set_DO", 1000, setDOCallback);
	ros::Subscriber setAOSubscriber= nh.subscribe("set_AO", 1000, setAOCallback);
	ros::Subscriber startWeldingSubscriber = nh.subscribe("start_welding", 1000, startWeldingCallback);
	ros::Subscriber stopWeldingSubscriber = nh.subscribe("stop_welding", 1000, stopWeldingCallback);
	ros::Subscriber laserOnSubscriber = nh.subscribe("laser_on", 1000, laserOnCallback);
	ros::Subscriber laserOffSubscriber = nh.subscribe("laser_off", 1000, laserOffCallback);
	ros::Subscriber trackLTOnSubscriber = nh.subscribe("track_LT_on", 1000, trackLTOnCallback);
	ros::Subscriber trackLTOffSubscriber = nh.subscribe("track_LT_off", 1000, trackLTOffCallback);
	ros::Subscriber startLTSearchSubscriber = nh.subscribe("start_LT_search", 1000, startLTSearchCallback);
	ros::Subscriber stopLTSearchSubscriber = nh.subscribe("stop_LT_search", 1000, stopLTSearchCallback);
	ros::Subscriber startWeaveSubscriber = nh.subscribe("start_weave", 1000, startWeaveCallback);
	ros::Subscriber endWeaveSubscriber = nh.subscribe("end_weave", 1000, endWeaveCallback);
	ros::Subscriber weaveParamSubscriber = nh.subscribe("weave_param", 1000, weaveParamCallback);
	ros::Subscriber modeSubscriber = nh.subscribe("mode", 1000, modeCallback);
	ros::Subscriber setSpeedSubscriber = nh.subscribe("set_speed", 1000, setSpeedCallback);
	ros::Subscriber teachSwitchSubscriber = nh.subscribe("drag_teach_switch", 1000, dragTeachCallback);
	ros::Subscriber installPosSubscriber = nh.subscribe("set_install_position", 1000, installPosCallback);
	ros::Subscriber toolPointSubscriber = nh.subscribe("set_tool_point", 1000, setToolPointCallback);
	ros::Subscriber toolCoordSubscriber = nh.subscribe("set_tool_coord", 1000, setToolCoordCallback);
	ros::Subscriber objectPointSubscriber = nh.subscribe("set_object_point", 1000, setWObjCoordPointCallback);
	ros::Subscriber objectCoordSubscriber = nh.subscribe("set_object_coord", 1000, setWObjCoordCallback);
	ros::Subscriber toolListSubscriber = nh.subscribe("set_tool_list", 1000, setToolListCallback);
	ros::Subscriber objectListSubscriber = nh.subscribe("set_object_list", 1000, setWObjListCallback);
	ros::Subscriber actGripperSubscriber = nh.subscribe("activate_gripper", 1000, activateGripperCallback);
	ros::Subscriber moveGripperSubscriber = nh.subscribe("move_gripper", 1000, moveGripperCallback);
	ros::Subscriber configAxleSubscriber = nh.subscribe("set_axle_config", 1000, configureAxleSensorCallback);
	ros::Subscriber actAxleSubscriber = nh.subscribe("activate_axle_sensor", 1000, activateAxleSensorCallback);
	ros::Subscriber writeAxleRegSubscriber = nh.subscribe("write_axle_sensor_register", 1000, writeAxleSensorCallback);
	ros::Subscriber setFTConfigSubscriber = nh.subscribe("set_FT_config", 1000, setFTConfigCallback);
	ros::Subscriber actFTSubscriber = nh.subscribe("activate_FT", 1000, activateFTCallback);
	ros::Subscriber guardFTSubscriber = nh.subscribe("guard_FT", 1000, guardFTCallback);
	ros::Subscriber setFTRCSSubscriber = nh.subscribe("set_FT_rcs", 1000, setFTRCSCallback);
	ros::Subscriber setFTZeroSubscriber = nh.subscribe("set_FT_zero", 1000, setFTZeroCallback);

	int error = driver.setupConnection(IP_address, port);
	if (error != 0) {
		std::cout << "connection setup failed" << std::endl;
		return -1;
	}
	
	ros::Rate rate(30);
	
	while (nh.ok()) {
		ros::spinOnce();
		
		publishConfig();
		publishIOStatus();
		publishJointStates();
		publishTCPInfo();
		publishCoordinateSystem();
		publishPayload();
		 
		rate.sleep();
	}
	ros::shutdown();
}
