/******************************************************************************
 * Copyright (c) 2022
 * KELO Robotics GmbH
 *
 * Author:
 * Leonardo Tan
 *
 *
 * This software is published under a dual-license: GNU Lesser General Public
 * License LGPL 2.1 and BSD license. The dual-license implies that users of this
 * code may choose which terms they prefer.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 * * Neither the name of Locomotec nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 2.1 of the
 * License, or (at your option) any later version or the BSD license.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License LGPL and the BSD license for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL and BSD license along with this program.
 *
 ******************************************************************************/

#ifndef HITBOTDRIVER_H_
#define HITBOTDRIVER_H_

#include <stdio.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string>
#include <iostream>
#include <cstring>
#include <vector>
#include <sstream>
#include <algorithm>
#include "kelo_hitbot_driver/CommandList.h"

class HitbotDriver {
public:
	//! Constructor of HitbotDriver class
	HitbotDriver();

	//! Destructor of HitbotDriver class
	virtual ~HitbotDriver();

	//! Setup the TCP connection with the hitbot arm
	int setupConnection(std::string IP_address, int port);
	
	//! Point-to-point movement
	//! TCPPose is the desired TCP pose with the format [x, y, z, rx, ry, rz] with x, y, z in mm and rx, ry, rz in degrees
	//! toolNum is the tool ID
	//! workpieceNum is the workpiece ID
	//! speed is the percentage of the maximum speed, ranging from 0 to 100
	//! acc is the percentage of the maximum acceleration, ranging from 0 to 100
	//! ovl is the percentage of the maximum speed, ranging from 0 to 100
	//! extAxisPos is a vector which consists of the extended axis positions [a1, a2, a3, a4]
	//! blendT is the smoothing time, ranging from 0 to 500 ms
	//! offsetFlag determines the type of the offset. 0 for no offset, 1 for workpiece/base coordinate system, 2 for tool coordinate system
	//! offset is the amount of the offset in [x, y, z, rx, ry, rz] with x, y, z in mm and rx, ry, rz in degrees
	bool movePTP(std::vector<float> TCPPose, int toolNum, int workpieceNum, 
				float speed, float acc, int ovl, std::vector<float> extAxisPos, float blendT, 
				uint8_t offsetFlag, std::vector<float> offset);
	
	//! Joint space movement
	//! jointAngles is the desired joint positions in degrees
	//! toolNum is the tool ID
	//! workpieceNum is the workpiece ID
	//! speed is the percentage of the maximum speed, ranging from 0 to 100
	//! acc is the percentage of the maximum acceleration, ranging from 0 to 100
	//! ovl is the percentage of the maximum speed, ranging from 0 to 100
	//! extAxisPos is a vector which consists of the extended axis positions [a1, a2, a3, a4]
	//! blendT is the smoothing time, ranging from 0 to 500 ms
	//! offsetFlag determines the type of the offset. 0 for no offset, 1 for workpiece/base coordinate system, 2 for tool coordinate system
	//! offset is the amount of the offset in [x, y, z, rx, ry, rz] with x, y, z in mm and rx, ry, rz in degrees
	bool moveJoint(std::vector<float> jointAngles, int toolNum, int workpieceNum, 
			float speed, float acc, int ovl, std::vector<float> extAxisPos, float blendT, 
			uint8_t offsetFlag, std::vector<float> offset);
	
	//! Arc circular movement
	//! TCPPose is the desired TCP pose with the format [x, y, z, rx, ry, rz] with x, y, z in mm and rx, ry, rz in degrees
	//! toolNum is the tool ID
	//! workpieceNum is the workpiece ID
	//! speed is the percentage of the maximum speed, ranging from 0 to 100
	//! acc is the percentage of the maximum acceleration, ranging from 0 to 100
	//! ovl is the percentage of the maximum speed, ranging from 0 to 100
	//! extAxisPos is a vector which consists of the extended axis positions [a1, a2, a3, a4]
	//! blendR is the smoothing radius, ranging from 0 to 1000 mm
	//! offsetFlag determines the type of the offset. 0 for no offset, 1 for workpiece/base coordinate system, 2 for tool coordinate system
	//! offset is the amount of the offset in [x, y, z, rx, ry, rz] with x, y, z in mm and rx, ry, rz in degrees
	bool moveArc(std::vector<float> TCPPose1, int toolNum1, int workpieceNum1, float speed1, float acc1, 
				std::vector<float> extAxisPos1, std::vector<float> TCPPose2, int toolNum2, int workpieceNum2, float speed2, 
				float acc2, std::vector<float> extAxisPos2, int ovl, float blendR, uint8_t offsetFlag, std::vector<float> offset);
	
	//! linear movement
	//! TCPPose is the desired TCP pose with the format [x, y, z, rx, ry, rz] with x, y, z in mm and rx, ry, rz in degrees
	//! toolNum is the tool ID
	//! workpieceNum is the workpiece ID
	//! speed is the percentage of the maximum speed, ranging from 0 to 100
	//! acc is the percentage of the maximum acceleration, ranging from 0 to 100
	//! ovl is the percentage of the maximum speed, ranging from 0 to 100
	//! extAxisPos is a vector which consists of the extended axis positions [a1, a2, a3, a4]
	//! blendR is the smoothing radius, ranging from 0 to 1000 mm
	//! searchFlag determines whether the wire seeks. 0 for no, 1 for yes
	//! offsetFlag determines the type of the offset. 0 for no offset, 1 for workpiece/base coordinate system, 2 for tool coordinate system
	//! offset is the amount of the offset in [x, y, z, rx, ry, rz] with x, y, z in mm and rx, ry, rz in degrees
	bool moveLin(std::vector<float> TCPPose, int toolNum, int workpieceNum, float speed, float acc, int ovl, float blendR, 
				std::vector<float> extAxisPos, uint8_t searchFlag, uint8_t offsetFlag, std::vector<float> offset);
	
	//! single axis jog movement
	//! motionCmd determines the reference frame for the axis jog movement. 0 for joint coordinate, 2 for base coordinate and 4 for tool coordinate
	//! jointNum is the joint number or the axis for the jog movement. If motionCmd is 0 it determines which joint should be moved (J1 - J6). Otherwise it will be {x, y, z, rx, ry, rz}
	//! direction determines if the movement is towards the positive or negative axis. 0 is for negative and 1 is for positive
	//! vel is the percentage of the maximum speed, ranging from 0 to 100
	//! acc is the percentage of the maximum acceleration, ranging from 0 to 100
	//! maxDistance is the maximum distance or angle in a single jog command in mm or degrees
	bool startJog(uint8_t motionCmd, uint8_t jointNum, uint8_t direction, float vel, float acc, float maxDistance);
	
	//! Servo to joint position
	//! jointAngle is the list of desired joint position in degrees
	//! vel is the percentage of the maximum speed, ranging from 0 to 100
	//! acc is the percentage of the maximum acceleration, ranging from 0 to 100
	//! t is the instruction cycle in s
	//! lookaheadTime is the filter time in s. Currently unavailable
	//! gain is the proportional gain at the target position. Currently unavailable
	bool servoJ(std::vector<float> jointAngle, float acc, float vel, float t, float lookaheadTime, float gain);
	
	//! Start automatic procedure
	bool startProcedure();
	
	//! Stop automatic procedure
	bool stopProcedure();
	
	//! Pause automatic procedure
	bool pauseProcedure();
	
	//! Resume automatic procedure
	bool resumeProcedure();
	
	//! Reset error flags
	bool resetErrors();
	
	//! Stop jogging movement
	bool stopJog();
	
	//! Stop linear movement
	bool stopLine();
	
	//! Stop tool coordinates jogging 
	bool stopTool();
	
	//! Stop workpiece coordinates jogging
	bool stopWorkpiece();
	
	//! Set the value of a digital output
	//! nIO is the digital output port number
	//! bOpen is the input value
	//! smooth is the flag for the smoothing. If the value is 0 there is no smoothing and 1 otherwise
	bool setDO(int nIO, int bOpen, int smooth);
	
	//! Get the value of a digital input port
	//! nIO is the digital input port number
	bool getDI(int nIO);
	
	//! Set the value of an analog output port
	//! nIO is the analog output port number
	//! value is the input value
	bool setAO(int nIO, float value);
	
	//! Get the value of an analog input port
	//! nIO is the analog input port number
	std::vector<float> getAI(int nIO);
	
	//! Set the value of a tool digital output port
	//! nIO is the tool digital output port number
	//! bOpen is the input value
	//! smooth is the flag for the smoothing. If the value is 0 there is no smoothing and 1 otherwise
	bool setToolDO(int nIO, int bOpen, int smooth);
	
	//! Get the value of a tool digital input port
	//! nIO is the tool digital input port number
	bool getToolDI(int nIO);
	
	//! Set the value of a tool analog output port
	//! nIO is the tool analog output port number
	//! value is the input value
	bool setToolAO(int nIO, float value);
	
	//! Get the value of a tool analog input port
	//! nIO is the tool analog input port number
	std::vector<float> getToolAI(int nIO);
	
	//! Wait for the value of a digital input port to be a certain value
	//! nIO is the digital input port number
	//! bOpen is the desired value of the digital input
	//! time is the maximum waiting time in ms
	//! alarm is the flag that determines the behavior when the waiting time is violated. If the value is 0, once the waiting time is over it will report an error. If the value is 1 then it will ignore the error
	bool waitDI(int nIO, int bOpen, int time, uint8_t alarm);
	
	//! Wait for the value of an analog input port to be above or below a threshold
	//! nIO is the analog input port number
	//! sign determines which comparison operator used. If the value is 0, it will wait until the value is greater than the threshold. If the value is 1, it will wait until the value is less than the threshold
	//! time is the maximum waiting time in ms
	//! alarm is the flag that determines the behavior when the waiting time is violated. If the value is 0, once the waiting time is over it will report an error. If the value is 1 then it will ignore the error
	bool waitAI(int nIO, int sign, int bOpen, int time, uint8_t alarm);
	
	//! Wait for the value of a tool digital input port to be a certain value
	//! nIO is the tool digital input port number
	//! bOpen is the desired value of the digital input
	//! time is the maximum waiting time in ms
	//! alarm is the flag that determines the behavior when the waiting time is violated. If the value is 0, once the waiting time is over it will report an error. If the value is 1 then it will ignore the error
	bool waitToolDI(int nIO, int bOpen, int time, uint8_t alarm);
	
	//! Wait for the value of a tool analog input port to be a certain value
	//! nIO is the analog input port number
	//! sign determines which comparison operator used. If the value is 0, it will wait until the value is greater than the threshold. If the value is 1, it will wait until the value is less than the threshold
	//! time is the maximum waiting time in ms
	//! alarm is the flag that determines the behavior when the waiting time is violated. If the value is 0, once the waiting time is over it will report an error. If the value is 1 then it will ignore the error
	bool waitToolAI(int nIO, int sign, int bOpen, int time, uint8_t alarm);
	
	//! Simple wait command
	//! time is the maximum waiting time in ms
	bool waitMs(int time);
	
	//! Switch operation mode
	//! 0 for automatic and 1 for manual mode
	bool switchMode(int mode);
	
	//! Set the robot arm maximum speed
	//! speed is the percentage of the maximum speed. The value is between 0 to 100
	bool setSpeed(uint8_t speed);
	
	//! Change the drag teach flag
	//! status is the flag. 0 for inactive and 1 for active
	bool switchDragTeach(uint8_t status);
	
	//! Set the robot installation orientation
	//! installPos is the installation orientation of the robot. 0 for upright, 1 for sideways, 2 for upside down installation
	bool setRobotInstallPos(uint8_t installPos);
	
	//! Set anti collision level
	//! level is the sensitivity of the collision avoidance. The value is from 1 to 3 with 1 as the most sensitive
	bool setAntiCollision(int level);
	
	//! Set positive software joint limit
	//! jointLimit is a vector which consists the maximum allowed joint angles in degrees
	bool setLimitPositive(std::vector<double> jointLimit);
	
	//! Set negative software joint limit
	//! jointLimit is a vector which consists the minimum allowed joint angles in degrees
	bool setLimitNegative(std::vector<double> jointLimit);
	
	//! Set tool calibration point
	//! id is the joint number used for the calibration
	bool setToolPoint(int id);
	
	//! Calculate tool coordinate system
	//! The output vector format is [x,y,z,rx,ry,rz]
	//! x, y, z are in mm and rx, ry, rz are in degrees
	std::vector<float> computeTool();
	
	//! Set TCP coordinates
	//! toolNum is the tool coordinate system number
	//! TCPPose is the pose vector in the format [x,y,z,rx,ry,rz] with x, y, z are in mm and rx, ry, rz are in degrees
	//! type 0-tool, 1-sensor
	//! install 0-end of installation, 1-outside the robot
	bool setToolCoord(int toolNum, std::vector<double> TCPPose, int type, int install);
	
	//! Set workpiece calibration point
	//! id is calibration point number. The value is from 1 to 3
	bool setWObjCoordPoint(int id);
	
	//! Calculate workpiece coordinate system
	//! The output vector format is [x,y,z,rx,ry,rz]
	//! x, y, z are in mm and rx, ry, rz are in degrees
	std::vector<float> computeWObjCoord();
	
	//! Set workpiece coordinate system
	//! workpieceNum is the workpiece coordinate system number
	//! pose is the workpiece pose in the format of [x,y,z,rx,ry,rz] with x, y, z are in mm and rx, ry, rz are in degrees
	bool setWObjCoord(int workpieceNum, std::vector<double> pose);
	
	//! Set tool list
	//! toolNum is the tool coordinate system number
	//! TCPPose is the pose vector in the format [x,y,z,rx,ry,rz] with x, y, z are in mm and rx, ry, rz are in degrees
	//! type 0-tool, 1-sensor
	//! install 0-end of installation, 1-outside the robot
	bool setToolList(int toolNum, std::vector<double> TCPPose, int type, int install);
	
	//! Set workpiece list
	//! workpieceNum is the workpiece coordinate system number
	//! pose is the workpiece pose in the format of [x,y,z,rx,ry,rz] with x, y, z are in mm and rx, ry, rz are in degrees
	bool setWObjList(int workpieceNum, std::vector<double> pose);
	
	//! Start arc
	//! arcNum is the welding profile number
	//! maxWT is the maximum waiting time in ms
	bool startArc(uint8_t arcNum, int maxWT);
	
	//! End arc
	//! arcNum is the welding profile number
	//! maxWT is the maximum waiting time in ms
	bool endArc(uint8_t arcNum, int maxWT);
	
	//! Turn on laser
	//! ID is the weld type number
	bool laserOn(uint8_t ID);
	
	//! Turn off laser
	bool laserOff();
	
	//! Start tracking
	bool trackLTOn();
	
	//! Stop tracking
	bool trackLTOff();
	
	//! Start search
	//! direction is the positioning direction. 0 is for positive X axis, 1 is for negative X axis, 2 is for positive Y axis, 3 is for negative Y axis, 4 is for positive Z axis, 5 is for negative Z axis
	//! vel is search speed percentage from 0 to 100
	//! distance is positioning distance in mm
	//! maxtime is the maximum searching time in ms
	bool startLTSearch(uint8_t direction, uint32_t vel, int distance, int maxtime);
	
	//! Stop search
	bool stopLTSearch();
	
	//! Set weave parameter setting
	//! num is the weave parameter configuration number from 0 to 7
	//! type is the swing type. 0 for plane swing, 1 for vertical L swing
	//! freq is the weave frequency in Hz
	//! rang is weave range in mm
	//! lst is weave left dwell time in ms
	//! rst is weave right dwell time in ms
	bool setWeaveParam(uint8_t num, uint8_t type, float freq, float rang, int lst, int rst);
	
	//! Start weave
	//! num is the weave parameter configuration number
	bool startWeave(uint8_t num);
	
	//! End weave
	//! num is the weave parameter configuration number
	bool endWeave(uint8_t num);

	//! Activate gripper
	//! id is the gripper number
	//! act is the activation flag. 0 for deactivate and 1 for activate.
	bool activateGripper(int id, int act);
	
	//! Control gripper
	//! id is the gripper number
	//! pos is the position of the gripper, ranging from 0 to 100 percent
	//! speed is the speed of the gripper movement, ranging from 0 to 100 percent
	//! force is the torque of the gripper, ranging from 0 to 100 percent
	//! maxTime is the maximum waiting time in ms ranging from 0 to 30000
	bool moveGripper(int id, int pos, int speed, int force, int maxTime);
	
	//! Check if the gripper is currently on motion
	//! returns the status of the gripper. 0 if the gripper is still running, 1 if the gripper is idle
	std::vector<int> getGripperMotionDone();
	
	//! Configure axle sensor
	//! id is the sensor number
	//! company is the sensor manufacturer id
	//! device is the sensor device number
	//! softwareVer is the software version
	//! bus is the end position
	bool configureAxleSensor(int id, int company, int device, int softwareVer, int bus);
	
	//! Get axle sensor configuration
	//! Returns a vector of int in the format of [id, company, device, softwareVer, bus]
	//! id is the sensor number
	//! company is the sensor manufacturer id
	//! device is the sensor device number
	//! softwareVer is the software version
	//! bus is the end position
	std::vector<int> getAxleSensorConfig();
	
	//! Activate axle sensor
	//! flag is the activation flag. 0 for inactive, 1 for active
	bool activateAxleSensor(int flag);
	
	//! Write axle sensor register
	//! addr is the device address, ranging from 0x01 to 0x08
	//! regHighAddr is register high address, ranging from 0x00 to 0xFF
	//! regLowAddr is register low address, ranging from 0x00 to 0xFF
	//! regNum is the number of registers, ranging from 0x01 to 0x0F
	bool writeAxleSensorReg(uint8_t addr, uint8_t regHighAddr, uint8_t regLowAddr, 
							uint8_t regNum, uint16_t data1, uint16_t data2);
	
	//! Set force sensor configuration
	//! id is the sensor number, ranging from 1 to 8
	//! company is the sensor manufacturer id
	//! device is the sensor device number
	//! softwareVer is the software version
	//! bus is the end position
	bool setFTConfig(int id, int company, int device, int softwareVer, int bus);
	
	//! Get force sensor configuration
	//! Returns a vector of int in the format of [id, company, device, softwareVer, bus]
	//! id is the sensor number, ranging from 1 to 8
	//! company is the sensor manufacturer id
	//! device is the sensor device number
	//! softwareVer is the software version
	//! bus is the end position
	std::vector<int> getFTConfig();
	
	//! Activate force sensor
	//! flag is the force sensor activation flag. 0 for inactive, 1 for active
	bool activateFT(int flag);
	
	//! Force sensor collision guard
	//! flag determines if the collision guard is active. 0 for off and 1 for on
	//! isSelect is the flag 
	//! collision force is a vector which consists of the collision force threshold in [x, y, z, rx, ry, rz] axes
	//! x, y and z are in N and rx, ry, rz are in Nm
	bool guardFT(int flag, uint8_t isSelect, std::vector<double> collisionForce);
	
	//! Set the force sensor reference coordinate system
	//! rcs is the coordinate system id. 0 for tool coordinate system, 1 for base coordinate system
	bool setFTRCS(int rcs);
	
	//! Calibrate force sensor zero value
	//! flag determines the calibration action. 0 is clear, 1 is zero
	bool setFTZero(uint8_t flag);
	
	//! Get joint angles in degrees
	std::vector<float> getJointPosDegree();
	
	//! Get joint angles in radian
	std::vector<float> getJointPosRadian();
	
	//! Get joint velocities in degrees/s
	std::vector<float> getJointSpeedsDegree();
	
	//! Get joint velocities in radian/s
	std::vector<float> getJointSpeedsRadian();
	
	//! Get current TCP pose
	//! Returns a vector which consists of the TCP pose in the format of [x, y, z, rx, ry, rz] 
	//! x, y and z are in mm and rx, ry, rz are in degrees
	std::vector<float> getActualTCPPose();
	
	//! Get actual TCP number
	std::vector<int> getActualTCPNum();
	
	//! Get actual tool flange pose
	//! Returns a vector which consists of the actual tool flange pose in the format of [x, y, z, rx, ry, rz] 
	//! x, y and z are in mm and rx, ry, rz are in degrees
	std::vector<float> getActualToolFlangePose();

	//! Calculates the required joint angles for a desired TCP pose
	//! Returns a vector which consists of joint positions of the arm in degrees
	//! flag determines the pose type. 0 for absolute pose, 1 for relative pose
	//! TCPPose is the desired TCP pose in the format of [x, y, z, rx, ry, rz]
	//! x, y and z are in mm and rx, ry, rz are in degrees
	//! config is joint configuration. The default is 1
	std::vector<float> getInverseKinematics(uint8_t flag, std::vector<float> TCPPose, int config);
	
	//! Calculates the TCP pose of a specific joint angle configuration
	//! Returns a vector in the format of [x, y, z, rx, ry, rz] with x, y, z in mm and rx, ry, rz in degrees
	//! jointAngles is a vector which consists of joint positions of the arm in degrees
	std::vector<float> getForwardKinematics(std::vector<float> jointAngles);
	
	//! Get joint torques in Nm
	std::vector<float> getJointTorques();
	
	//! Get target payload in kg
	std::vector<float> getTargetPayload();
	
	//! Get target payload center-of-gravity
	//! Returns a vector in the format of [x, y, z] with x, y, z in mm
	std::vector<float> getTargetPayloadCog();
	
	//! Get target TCP pose relative to the base coordinate system
	//! Returns a vector which consists of the target TCP pose in the format of [x, y, z, rx, ry, rz]
	//! x, y and z are in mm and rx, ry, rz are in degrees 
	std::vector<float> getTargetTCPPose();
	
	//! Get the pose of the TCP relative to the end flange
	//! Returns a vector which consists of the TCP offset in the format of [x, y, z, rx, ry, rz]
	//! x, y and z are in mm and rx, ry, rz are in degrees 
	std::vector<float> getTCPOffset();
	
	//! Get software joint limits in degrees
	std::vector<double> getJointSoftLimitDeg();

protected:
	//! Read the buffer and extract the data content as a vector of strings
	std::vector<std::string> getContent();
	
	//! Read the buffer and extract the data content
	//! Returns 1 if the sent command is executed successfully
	bool getContentBool();
	
	//! Read the buffer and extract the data content as a vector of integers
	std::vector<int> getContentInt();
	
	//! Read the buffer and extract the data content as a vector of doubles
	std::vector<double> getContentDouble();
	
	//! Read the buffer and extract the data content as a vector of floats
	std::vector<float> getContentFloat();
	
	//! Create the packet
	void sendCommand(int ID, std::string command);
	void sendSpecialCommand(int ID, std::string command);
	
	int sock;
	struct sockaddr_in serv_addr;
	char buffer[1024];
	unsigned int msgCounter;
	
	std::string startHeader;
	std::string endHeader;
	std::string separator;
};



#endif /* HITBOTDRIVER_H_ */
