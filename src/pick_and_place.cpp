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
}
