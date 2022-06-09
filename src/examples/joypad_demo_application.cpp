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

#include <ros/ros.h>

#include "kelo_hitbot_driver/HitbotDriver.h"
#include <sensor_msgs/Joy.h>
HitbotDriver driver;

std::vector<float> axes(6,0);
std::vector<int> buttons(12,0);
std::vector<int> prev_buttons(12,0);

double autoTime = 40.0;
ros::Time tLast;

bool newStart = true;
unsigned int state = 0;
unsigned int teachState = 0;

void goToFoldedPosition() {
	std::vector<float> jointAngle(6);
	jointAngle[0] = 38.3;
	jointAngle[1] = -177.6;
	jointAngle[2] = 152.5;
	jointAngle[3] = -156.7;
	jointAngle[4] = -89.0;
	jointAngle[5] = 89.0;
	std::vector<float> extAxis(4, 0);
	std::vector<float> offset(6, 0);
	driver.moveJoint(jointAngle, 0, 0, 70, 70, 70, extAxis, 0, 0, offset);	
}

void goToStretchedPosition() {
	std::vector<float> jointAngle(6);
	jointAngle[0] = 38.3;
	jointAngle[1] = -115.26;
	jointAngle[2] = 88.8;
	jointAngle[3] = -151.9;
	jointAngle[4] = -91.6;
	jointAngle[5] = 89.0;
	std::vector<float> extAxis(4, 0);
	std::vector<float> offset(6, 0);
	driver.moveJoint(jointAngle, 0, 0, 70, 70, 70, extAxis, 0, 0, offset);	
}

void inspect() {
	double yawMagnitude = 45;
	double pitchMagnitude = 45;
	std::vector<float> extAxis(4, 0);
	std::vector<float> offset(6, 0);

	std::vector<float> jointAngle = driver.getJointPosDegree();
	jointAngle[4] = jointAngle[4] - yawMagnitude;
	driver.moveJoint(jointAngle, 0, 0, 70, 70, 70, extAxis, 0, 0, offset);
	
	jointAngle[3] = jointAngle[3] + pitchMagnitude;
	driver.moveJoint(jointAngle, 0, 0, 70, 70, 70, extAxis, 0, 0, offset);
	
	jointAngle[4] = jointAngle[4] + 2 * yawMagnitude;
	driver.moveJoint(jointAngle, 0, 0, 70, 70, 70, extAxis, 0, 0, offset);
	
	jointAngle[3] = jointAngle[3] - pitchMagnitude;
	driver.moveJoint(jointAngle, 0, 0, 70, 70, 70, extAxis, 0, 0, offset);
	
	jointAngle[4] = jointAngle[4] - yawMagnitude;
	driver.moveJoint(jointAngle, 0, 0, 70, 70, 70, extAxis, 0, 0, offset);
}

void makeSquare() {	
	// move forward
	driver.startJog(4, 3, 1, 100, 100, 100);
	
	//make square
	driver.startJog(4, 2, 1, 100, 100, 200);
	driver.startJog(4, 1, 1, 100, 100, 300);
	driver.startJog(4, 2, 0, 100, 100, 400);
	driver.startJog(4, 1, 0, 100, 100, 300);
	driver.startJog(4, 2, 1, 100, 100, 200);
}

void lookAround() {
	std::vector<float> jointAngle(6);
	jointAngle[0] = 128.3;
	jointAngle[1] = -115.26;
	jointAngle[2] = 88.8;
	jointAngle[3] = -151.9;
	jointAngle[4] = -91.6;
	jointAngle[5] = 89.0;
	std::vector<float> extAxis(4, 0);
	std::vector<float> offset(6, 0);
	driver.moveJoint(jointAngle, 0, 0, 100, 100, 100, extAxis, 0, 0, offset);

	// inspect
	double yawMagnitude = 45;
	double pitchMagnitude = 45;
	
	jointAngle[4] = jointAngle[4] - yawMagnitude;
	driver.moveJoint(jointAngle, 0, 0, 100, 100, 100, extAxis, 0, 0, offset);
	
	jointAngle[3] = jointAngle[3] + pitchMagnitude;
	driver.moveJoint(jointAngle, 0, 0, 100, 100, 100, extAxis, 0, 0, offset);
	
	jointAngle[4] = jointAngle[4] + 2 * yawMagnitude;
	driver.moveJoint(jointAngle, 0, 0, 100, 100, 100, extAxis, 0, 0, offset);
	
	jointAngle[3] = jointAngle[3] - pitchMagnitude;
	driver.moveJoint(jointAngle, 0, 0, 100, 100, 100, extAxis, 0, 0, offset);
	
	jointAngle[4] = jointAngle[4] - yawMagnitude;
	driver.moveJoint(jointAngle, 0, 0, 100, 100, 100, extAxis, 0, 0, offset);
}

void sporadicMovement() {
	std::vector<float> extAxis(4, 0);
	std::vector<float> offset(6, 0);

	std::vector<float> jointAngle(6);
	jointAngle[0] = 77.39;
	jointAngle[1] = -93.87;
	jointAngle[2] = 101.35;
	jointAngle[3] = -151.85;
	jointAngle[4] = -75.2;
	jointAngle[5] = 89.0;
	driver.moveJoint(jointAngle, 0, 0, 100, 100, 100, extAxis, 0, 0, offset);
	driver.startJog(4, 3, 1, 50, 50, 150);
	driver.waitMs(1000);
	driver.startJog(4, 3, 0, 100, 100, 150);
	
	jointAngle[0] = -16.1;
	jointAngle[1] = -106.74;
	jointAngle[2] = 91.9;
	jointAngle[3] = -177.6;
	jointAngle[4] = 71.7;
	jointAngle[5] = 89.0;
	driver.moveJoint(jointAngle, 0, 0, 100, 100, 100, extAxis, 0, 0, offset);
	driver.startJog(0, 5, 0, 100, 100, 45);
	driver.startJog(0, 5, 1, 100, 100, 45);
}

void stop() {
	driver.stopJog();
	driver.stopLine();
	driver.stopTool();
}

void automatic() {
	goToStretchedPosition();
	makeSquare();
	goToStretchedPosition();
	lookAround();
	sporadicMovement();
	goToStretchedPosition();
	goToFoldedPosition();
	std::cout << "execute automatic operation" << std::endl;
}

void process() {
	if (prev_buttons[8] == 0 && buttons[8] == 1) {
		driver.resetErrors();
	}
	if (buttons[5] == 1) {
		if (teachState == 0 && prev_buttons[9] == 0 && buttons[9] == 1) {
			//switch mode (red button)
			if (state == 0) {
				state = 1;
				std::cout << "switch to automatic mode" << std::endl;
			} else if (state == 1) {
				state = 0;
				std::cout << "switch to manual mode" << std::endl;
			}
		}
	}
	
	if (state == 1) {
		if (newStart || (ros::Time::now() - tLast).toSec() > autoTime) {
			automatic();
			newStart = false;
			tLast = ros::Time::now();
		}
	} else if (state == 0) {
		if (buttons[5] == 1 && (ros::Time::now() - tLast).toSec() > autoTime) {
			if (prev_buttons[0] == 0 && buttons[0] == 1) {
				//stretch out for inspection (blue button)
				goToStretchedPosition();
				std::cout << "go to ready position" << std::endl;
			} else if (prev_buttons[1] == 0 &&buttons[1] == 1) {
				// inspect (green button)
				inspect();
				std::cout << "perform inspection" << std::endl;
			} else if (prev_buttons[3] == 0 &&buttons[3] == 1) {
				// fold back (yellow button)
				goToFoldedPosition();
				std::cout << "go to idle position" << std::endl;
			} else if (fabs(axes[4]) > 0.1) {
				if (buttons[4] == 1) {
					//turn left and right
					if (axes[4] > 0) {
						driver.startJog(4, 4, 0, 50, 50, 20);
					} else {
						driver.startJog(4, 4, 1, 50, 50, 20);
					}
				} else {
					//move left and right
					if (axes[4] > 0) {
						driver.startJog(4, 2, 1, 50, 50, 50);
					} else {
						driver.startJog(4, 2, 0, 50, 50, 50);
					}
				}
			} else if (fabs(axes[5]) > 0.1) {
				if (buttons[4] == 1) {
					//nod up and down
					if (axes[5] > 0) {
						driver.startJog(4, 5, 0, 50, 50, 20);
					} else {
						driver.startJog(4, 5, 1, 50, 50, 20);
					}
				} else {
					//move up and down
					if (axes[5] > 0) {
						driver.startJog(4, 1, 0, 50, 50, 50);
					} else {
						driver.startJog(4, 1, 1, 50, 50, 50);
					}
				}
			} else if (buttons[6] == 1) {
				//move backward (LT)
				driver.startJog(4, 3, 0, 50, 50, 50);

			} else if (buttons[7] == 1) {
				//move forward (RT)
				driver.startJog(4, 3, 1, 50, 50, 50);
			} else if (prev_buttons[2] == 0 && buttons[2] == 1) {
				if (teachState == 0) {
					driver.switchDragTeach(1);
					teachState = 1;
					std::cout << "enable drag teach" << std::endl;
				} else if (teachState == 1) {
					driver.switchDragTeach(0);
					teachState = 0;
					std::cout << "disable drag teach" << std::endl;
				}
			}
		}
	}
	
	prev_buttons = buttons;
}

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy) {
	buttons = joy->buttons;
	axes = joy->axes;
}

int main(int argc, char** argv)
{
	ros::init (argc, argv, "joypad_demo_application");
	ros::NodeHandle nh;
	
	ros::Subscriber joySubscriber = nh.subscribe("/joy", 1000, joyCallback);
	ros::Rate rate(20);
	
	driver.setupConnection("192.168.58.2", 8080);
	
	while (nh.ok()) {
		ros::spinOnce();
		process();
		rate.sleep();
	}
	ros::shutdown();
}
