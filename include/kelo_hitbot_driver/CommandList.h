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
 
#define MoveJ_ID						201
#define MoveC_ID						202
#define MoveL_ID						203
#define StartJOG_ID						232
#define ServoJ_ID						376

#define START_ID						101
#define STOP_ID							102
#define PAUSE_ID						103
#define RESUME_ID						104
#define RESETALLERROR_ID				107
#define STOPJOG_ID						233
#define STOPLINE_ID						234
#define STOPTOOL_ID						235
#define STOPWORKPIECE_ID				241

#define SetDO_ID						204
#define GetDI_ID						212
#define SetAO_ID						209
#define GetAI_ID						214
#define SetToolDO_ID					210
#define GetToolDI_ID					213
#define SetToolAO_ID					211
#define GetToolAI_ID					215
#define WaitDI_ID						218
#define WaitAI_ID						220
#define WaitToolDI_ID					219
#define WaitToolAI_ID					221
#define WaitMs_ID						207

#define Mode_ID							303
#define SetSpeed_ID						206
#define DragTeachSwitch_ID				333
#define SetRobotInstallPos_ID			337
#define SetAnticollision_ID				305
#define SetLimitPositive_ID				308
#define SetLimitNegative_ID				309
#define SetToolPoint_ID					313
#define ComputeTool_ID					314
#define SetToolCoord_ID					316
#define SetWObjCoordPoint_ID			249
#define ComputeWObjCoord_ID				250
#define SetWObjCoord_ID					251
#define SetToolList_ID					319
#define SetWObjList_ID					383

#define ARCStart_ID						247
#define ARCEnd_ID						248
#define LTLaserOn_ID					255
#define LTLaserOff_ID					256
#define LTTrackOn_ID					257
#define LTTrackOff_ID					258
#define LTSearchStart_ID				259
#define LTSearchStop_ID					260

#define WeaveSetPara_ID					252
#define WeaveStart_ID					253
#define WeaveEnd_ID						254

#define ActGripper_ID					227
#define MoveGripper_ID					228
#define GetGripperMotionDone_ID			377
#define AxleSensorConfig_ID				545
#define AxleSensorConfigGet_ID			546
#define AxleSensorActivate_ID			547
#define AxleSensorRegWrite_ID			548
#define FT_SetConfig_ID					526
#define FT_GetConfig_ID					527
#define FT_Activate_ID					524
#define FT_Guard_ID						521
#define FT_SetRCS_ID					525
#define FT_SetZero_ID					528

#define GetActualJointPosDegree_ID		377
#define GetActualJointPosRadian_ID		377
#define GetActualJointSpeedsDegree_ID	377
#define GetActualJointSpeedsRadian_ID	377
#define GetActualTCPPose_ID				377
#define GetActualTCPNum_ID				377
#define GetActualToolFlangePose_ID		377
#define GetInverseKin_ID				377
#define GetForwardKin_ID				377
#define GetJointTorques_ID				377
#define GetTargetPayload_ID				377
#define GetTargetPayloadCog_ID			377
#define GetTargetTCPPose_ID				377
#define GetTCPOffset_ID					377
#define GetJointSoftLimitDeg_ID			377



