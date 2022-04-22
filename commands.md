# Publishers

Here is the list of the published topics:
- joint_state[(kelo_hitbot_driver::JointState)](msg/JointState.msg)
Related function(s): hitbot_driver::getJointPosDegree, hitbot_driver::getJointSpeedsDegree, hitbot_driver::getJointTorques

- TCP_pose[(kelo_hitbot_driver::Axes)](msg/Axes.msg)
Related function(s): hitbot_driver::getActualTCPPose

- TCP_target[(kelo_hitbot_driver::Axes)](msg/Axes.msg)
Related function(s): hitbot_driver::getTargetTCPPose

- TCP_flange_pose[(kelo_hitbot_driver::Axes)](msg/Axes.msg)
Related function(s): hitbot_driver::getActualToolFlangePose

- TCP_offset[(kelo_hitbot_driver::Axes)](msg/Axes.msg)
Related function(s): hitbot_driver::getTCPOffset

- TCP_num(std_msgs::Int32)
Related function(s): hitbot_driver::getActualTCPNum

- target_payload(std_msgs::Float32)
Related function(s): hitbot_driver::getTargetPayload

- target_payload_cog(std_msgs::Float32MultiArray)
Related function(s): hitbot_driver::getTargetPayloadCog

- joint_limits(std_msgs::Float64MultiArray)
Related function(s): hitbot_driver::getJointSoftLimitDeg

- FT_config[(kelo_hitbot_driver::Config)](msg/Config.msg)
Related function(s): hitbot_driver::getFTConfig

- axle_sensor_config[(kelo_hitbot_driver::Config)](msg/Config.msg)
Related function(s): hitbot_driver::getAxleSensorConfig

- digital_input(std_msgs::Int32MultiArray)
Related function(s): hitbot_driver::getDI

- tool_digital_input(std_msgs::Int32MultiArray)
Related function(s): hitbot_driver::getToolDI

- analog_input(std_msgs::Float32MultiArray)
Related function(s): hitbot_driver::getAI

- tool_analog_input(std_msgs::Float32MultiArray)
Related function(s): hitbot_driver::getToolAI

- tool_coordinate_frame[(kelo_hitbot_driver::Axes)](msg/Axes.msg)
Related function(s): hitbot_driver::computeTool

- object_coordinate_frame[(kelo_hitbot_driver::Axes)](msg/Axes.msg)
Related function(s): hitbot_driver::computeWObjCoord


# Subscribers

Here is the list of the subscribed topics:
- move_PTP[(kelo_hitbot_driver::PTP)](msg/PTP.msg)
Related function(s): [hitbot_driver::movePTP](include/kelo_hitbot_driver/HitbotDriver.h#27)

- move_linear[(kelo_hitbot_driver::Lin)](msg/Lin.msg)
Related function(s): hitbot_driver::moveLin

- move_arc[(kelo_hitbot_driver::Arc)](msg/Arc.msg)
Related function(s): hitbot_driver::moveArc

- move_jog[(kelo_hitbot_driver::Jog)](msg/Jog.msg)
Related function(s): hitbot_driver::startJog

- move_servo[(kelo_hitbot_driver::Servo)](msg/Servo.msg)
Related function(s): hitbot_driver::servoJ

- automatic_procedure(std_msgs::String)
Related function(s): hitbot_driver::startProcedure, hitbot_driver::stopProcedure, hitbot_driver::pauseProcedure, hitbot_driver::resumeProcedure

- reset_error(std_msgs::Empty)
Related function(s): hitbot_driver::resetErrors

- stop(std_msgs::Empty)
Related function(s): hitbot_driver::stopJog, hitbot_driver::stopLine, hitbot_driver::stopTool, hitbot_driver::stopWorkpiece

- wait_DI[(kelo_hitbot_driver::WaitDigital)](msg/WaitDigital.msg)
Related function(s): hitbot_driver::waitDI

- wait_AI[(kelo_hitbot_driver::WaitAnalog)](msg/WaitAnalog.msg)
Related function(s): hitbot_driver::waitAI

- set_DO[(kelo_hitbot_driver::DigitalOutput)](msg/DigitalOutput.msg)
Related function(s): hitbot_driver::setDO

- set_AO[(kelo_hitbot_driver::AnalogOutput)](msg/AnalogOutput.msg)
Related function(s): hitbot_driver::setAO

- start_welding[(kelo_hitbot_driver::Weld)](msg/Weld.msg)
Related function(s): hitbot_driver::startArc

- stop_welding[(kelo_hitbot_driver::Weld)](msg/Weld.msg)
Related function(s): hitbot_driver::endArc

- laser_on(std_msgs::Int32)
Related function(s): hitbot_driver::laserOn

- laser_off(std_msgs::Empty)
Related function(s): hitbot_driver::laserOff

- track_LT_on(std_msgs::Int32)
Related function(s): hitbot_driver::trackLTOn

- track_LT_off(std_msgs::Empty)
Related function(s): hitbot_driver::trackLTOff

- start_LT_search[(kelo_hitbot_driver::LT)](msg/LT.msg)
Related function(s): hitbot_driver::startLTSearch

- stop_LT_search(std_msgs::Empty)
Related function(s): hitbot_driver::stopLTSearch

- start_weave(std_msgs::UInt8)
Related function(s): hitbot_driver::startWeave

- end_weave(std_msgs::UInt8)
Related function(s): hitbot_driver::endWeave

- weave_param[(kelo_hitbot_driver::Weave)](msg/Weave.msg)
Related function(s): hitbot_driver::setWeaveParam

- mode(std_msgs::Int32)
Related function(s): hitbot_driver::switchMode

- set_speed(std_msgs::UInt8)
Related function(s): hitbot_driver::setSpeed

- drag_teach_switch(std_msgs::UInt8)
Related function(s): hitbot_driver::switchDragTeach

- set_install_position(std_msgs::UInt8)
Related function(s): hitbot_driver::setRobotInstallPos

- set_tool_point(std_msgs::Int32)
Related function(s): hitbot_driver::setToolPoint

- set_tool_coord[(kelo_hitbot_driver::Tool)](msg/Tool.msg)
Related function(s): hitbot_driver::setToolCoord

- set_object_point(std_msgs::Int32)
Related function(s): hitbot_driver::setWObjCoordPoint

- set_object_coord[(kelo_hitbot_driver::Workpiece)](msg/Workpiece.msg)
Related function(s): hitbot_driver::setWObjCoord

- set_tool_list[(kelo_hitbot_driver::Tool)](msg/Tool.msg)
Related function(s): hitbot_driver::setToolList

- set_object_list[(kelo_hitbot_driver::Workpiece)](msg/Workpiece.msg)
Related function(s): hitbot_driver::setWObjList

- activate_gripper[(kelo_hitbot_driver::GripperAct)](msg/GripperAct.msg)
Related function(s): hitbot_driver::activateGripper

- move_gripper[(kelo_hitbot_driver::GripperMotion)](msg/GripperMotion.msg)
Related function(s): hitbot_driver::moveGripper

- set_axle_config[(kelo_hitbot_driver::Config)](msg/Config.msg)
Related function(s): hitbot_driver::configureAxleSensor

- activate_axle_sensor(std_msgs::Int32)
Related function(s): hitbot_driver::activateAxleSensor

- write_axle_sensor_register[(kelo_hitbot_driver::AxleSensorReg)](msg/AxleSensorReg.msg)
Related function(s): hitbot_driver::writeAxleSensorReg

- set_FT_config[(kelo_hitbot_driver::Config)](msg/Config.msg)
Related function(s): hitbot_driver::setFTConfig

- activate_FT(std_msgs::Int32)
Related function(s): hitbot_driver::activateFT

- guard_FT[(kelo_hitbot_driver::GuardFT)](msg/GuardFT.msg)
Related function(s): hitbot_driver::guardFT

- set_FT_rcs(std_msgs::Int32)
Related function(s): hitbot_driver::setFTRCS

- set_FT_zero(std_msgs::UInt8)
Related function(s): hitbot_driver::setFTZero

