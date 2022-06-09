# Publishers

Here is the list of the published topics:
- joint_state[(kelo_hitbot_driver::JointState)](msg/JointState.msg)  
Related function(s): [hitbot_driver::getJointPosDegree](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L443-L444), [hitbot_driver::getJointSpeedsDegree](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L449-L450), [hitbot_driver::getJointTorques](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L481-L482)

- TCP_pose[(kelo_hitbot_driver::Axes)](msg/Axes.msg)  
Related function(s): [hitbot_driver::getActualTCPPose](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L455-L458)

- TCP_target[(kelo_hitbot_driver::Axes)](msg/Axes.msg)  
Related function(s): [hitbot_driver::getTargetTCPPose](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L491-L494)

- TCP_flange_pose[(kelo_hitbot_driver::Axes)](msg/Axes.msg)  
Related function(s): [hitbot_driver::getActualToolFlangePose](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L463-L466)

- TCP_offset[(kelo_hitbot_driver::Axes)](msg/Axes.msg)  
Related function(s): [hitbot_driver::getTCPOffset](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L496-L499)

- TCP_num(std_msgs::Int32)  
Related function(s): [hitbot_driver::getActualTCPNum](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L459-L461)

- target_payload(std_msgs::Float32)  
Related function(s): [hitbot_driver::getTargetPayload](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L484-L485)

- target_payload_cog(std_msgs::Float32MultiArray)  
Related function(s): [hitbot_driver::getTargetPayloadCog](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L487-L489)

- joint_limits(std_msgs::Float64MultiArray)  
Related function(s): [hitbot_driver::getJointSoftLimitDeg](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L501-L502)

- FT_config[(kelo_hitbot_driver::Config)](msg/Config.msg)  
Related function(s): [hitbot_driver::getFTConfig](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L415-L422)

- axle_sensor_config[(kelo_hitbot_driver::Config)](msg/Config.msg)  
Related function(s): [hitbot_driver::getAxleSensorConfig](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L386-L393)

- digital_input(std_msgs::Int32MultiArray)  
Related function(s): [hitbot_driver::getDI](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L177-L179)

- tool_digital_input(std_msgs::Int32MultiArray)  
Related function(s): [hitbot_driver::getToolDI](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L196-L198)

- analog_input(std_msgs::Float32MultiArray)  
Related function(s): [hitbot_driver::getAI](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L186-L188)

- tool_analog_input(std_msgs::Float32MultiArray)  
Related function(s): [hitbot_driver::getToolAI](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L205-L207)

- tool_coordinate_frame[(kelo_hitbot_driver::Axes)](msg/Axes.msg)  
Related function(s): [hitbot_driver::computeTool](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L273-L276)

- object_coordinate_frame[(kelo_hitbot_driver::Axes)](msg/Axes.msg)  
Related function(s): [hitbot_driver::computeWObjCoord](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L289-L292)


# Subscribers

Here is the list of the subscribed topics:
- move_PTP[(kelo_hitbot_driver::PTP)](msg/PTP.msg)  
Related function(s): [hitbot_driver::movePTP](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L66-L79)

- move_linear[(kelo_hitbot_driver::Lin)](msg/Lin.msg)  
Related function(s): [hitbot_driver::moveLin](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L111-L124)

- move_arc[(kelo_hitbot_driver::Arc)](msg/Arc.msg)  
Related function(s): [hitbot_driver::moveArc](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L96-L109)

- move_jog[(kelo_hitbot_driver::Jog)](msg/Jog.msg)  
Related function(s): [hitbot_driver::startJog](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L126-L133)

- move_servo[(kelo_hitbot_driver::Servo)](msg/Servo.msg)  
Related function(s): [hitbot_driver::servoJ](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L135-L142)

- automatic_procedure(std_msgs::String)  
Related function(s): [hitbot_driver::startProcedure](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L144-L145), [hitbot_driver::stopProcedure](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L147-L148), [hitbot_driver::pauseProcedure](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L150-L151), [hitbot_driver::resumeProcedure](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L153-L154)

- reset_error(std_msgs::Empty)  
Related function(s): [hitbot_driver::resetErrors](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L156-L157)

- stop(std_msgs::Empty)  
Related function(s): [hitbot_driver::stopJog](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L159-L160), [hitbot_driver::stopLine](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L162-L163), [hitbot_driver::stopTool](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L165-L166), [hitbot_driver::stopWorkpiece](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L168-L169)

- wait_DI[(kelo_hitbot_driver::WaitDigital)](msg/WaitDigital.msg)  
Related function(s): [hitbot_driver::waitDI](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L209-L214)

- wait_AI[(kelo_hitbot_driver::WaitAnalog)](msg/WaitAnalog.msg)  
Related function(s): [hitbot_driver::waitAI](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L216-L221)

- set_DO[(kelo_hitbot_driver::DigitalOutput)](msg/DigitalOutput.msg)  
Related function(s): [hitbot_driver::setDO](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L171-L175)

- set_AO[(kelo_hitbot_driver::AnalogOutput)](msg/AnalogOutput.msg)  
Related function(s): [hitbot_driver::setAO](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L181-L184)

- start_welding[(kelo_hitbot_driver::Weld)](msg/Weld.msg)  
Related function(s): [hitbot_driver::startArc](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L311-L314)

- stop_welding[(kelo_hitbot_driver::Weld)](msg/Weld.msg)  
Related function(s): [hitbot_driver::endArc](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L316-L319)

- laser_on(std_msgs::Int32)  
Related function(s): [hitbot_driver::laserOn](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L321-L323)

- laser_off(std_msgs::Empty)  
Related function(s): [hitbot_driver::laserOff](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L325-L326)

- track_LT_on(std_msgs::Int32)  
Related function(s): [hitbot_driver::trackLTOn](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L328-L329)

- track_LT_off(std_msgs::Empty)  
Related function(s): [hitbot_driver::trackLTOff](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L331-L332)

- start_LT_search[(kelo_hitbot_driver::LT)](msg/LT.msg)  
Related function(s): [hitbot_driver::startLTSearch](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L334-L339)

- stop_LT_search(std_msgs::Empty)  
Related function(s): [hitbot_driver::stopLTSearch](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L341-L342)

- start_weave(std_msgs::UInt8)  
Related function(s): [hitbot_driver::startWeave](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L353-L355)

- end_weave(std_msgs::UInt8)  
Related function(s): [hitbot_driver::endWeave](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L357-L359)

- weave_param[(kelo_hitbot_driver::Weave)](msg/Weave.msg)  
Related function(s): [hitbot_driver::setWeaveParam](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L344-L351)

- mode(std_msgs::Int32)  
Related function(s): [hitbot_driver::switchMode](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L241-L243)

- set_speed(std_msgs::UInt8)  
Related function(s): [hitbot_driver::setSpeed](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L245-L247)

- drag_teach_switch(std_msgs::UInt8)  
Related function(s): [hitbot_driver::switchDragTeach](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L249-L251)

- set_install_position(std_msgs::UInt8)  
Related function(s): [hitbot_driver::setRobotInstallPos](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L253-L255)

- set_tool_point(std_msgs::Int32)  
Related function(s): [hitbot_driver::setToolPoint](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L269-L271)

- set_tool_coord[(kelo_hitbot_driver::Tool)](msg/Tool.msg)  
Related function(s): [hitbot_driver::setToolCoord](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L278-L283)

- set_object_point(std_msgs::Int32)  
Related function(s): [hitbot_driver::setWObjCoordPoint](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L285-L287)

- set_object_coord[(kelo_hitbot_driver::Workpiece)](msg/Workpiece.msg)  
Related function(s): [hitbot_driver::setWObjCoord](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L294-L297)

- set_tool_list[(kelo_hitbot_driver::Tool)](msg/Tool.msg)  
Related function(s): [hitbot_driver::setToolList](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L299-L304)

- set_object_list[(kelo_hitbot_driver::Workpiece)](msg/Workpiece.msg)  
Related function(s): [hitbot_driver::setWObjList](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L306-L309)

- activate_gripper[(kelo_hitbot_driver::GripperAct)](msg/GripperAct.msg)  
Related function(s): [hitbot_driver::activateGripper](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L361-L364)

- move_gripper[(kelo_hitbot_driver::GripperMotion)](msg/GripperMotion.msg)  
Related function(s): [hitbot_driver::moveGripper](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L366-L372)

- set_axle_config[(kelo_hitbot_driver::Config)](msg/Config.msg)  
Related function(s): [hitbot_driver::configureAxleSensor](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L378-L384)

- activate_axle_sensor(std_msgs::Int32)  
Related function(s): [hitbot_driver::activateAxleSensor](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L395-L397)

- write_axle_sensor_register[(kelo_hitbot_driver::AxleSensorReg)](msg/AxleSensorReg.msg)  
Related function(s): [hitbot_driver::writeAxleSensorReg](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L399-L405)

- set_FT_config[(kelo_hitbot_driver::Config)](msg/Config.msg)  
Related function(s): [hitbot_driver::setFTConfig](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L407-L413)

- activate_FT(std_msgs::Int32)  
Related function(s): [hitbot_driver::activateFT](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L424-L426)

- guard_FT[(kelo_hitbot_driver::GuardFT)](msg/GuardFT.msg)  
Related function(s): [hitbot_driver::guardFT](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L428-L433)

- set_FT_rcs(std_msgs::Int32)  
Related function(s): [hitbot_driver::setFTRCS](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L435-L437)

- set_FT_zero(std_msgs::UInt8)  
Related function(s): [hitbot_driver::setFTZero](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L439-L441)

