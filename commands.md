# Publishers

Here is the list of the published topics:
- joint_state[(kelo_hitbot_driver::JointState)](msg/JointState.msg)  
Related function(s): [hitbot_driver::getJointPosDegree](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L428-L429), [hitbot_driver::getJointSpeedsDegree](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L434-L435), [hitbot_driver::getJointTorques](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L466-L467)

- TCP_pose[(kelo_hitbot_driver::Axes)](msg/Axes.msg)  
Related function(s): [hitbot_driver::getActualTCPPose](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L440-L443)

- TCP_target[(kelo_hitbot_driver::Axes)](msg/Axes.msg)  
Related function(s): [hitbot_driver::getTargetTCPPose](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L476-L479)

- TCP_flange_pose[(kelo_hitbot_driver::Axes)](msg/Axes.msg)  
Related function(s): [hitbot_driver::getActualToolFlangePose](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L448-L451)

- TCP_offset[(kelo_hitbot_driver::Axes)](msg/Axes.msg)  
Related function(s): [hitbot_driver::getTCPOffset](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L481-L484)

- TCP_num(std_msgs::Int32)  
Related function(s): [hitbot_driver::getActualTCPNum](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L445-L446)

- target_payload(std_msgs::Float32)  
Related function(s): [hitbot_driver::getTargetPayload](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L469-L470)

- target_payload_cog(std_msgs::Float32MultiArray)  
Related function(s): [hitbot_driver::getTargetPayloadCog](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L472-L474)

- joint_limits(std_msgs::Float64MultiArray)  
Related function(s): [hitbot_driver::getJointSoftLimitDeg](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L486-L487)

- FT_config[(kelo_hitbot_driver::Config)](msg/Config.msg)  
Related function(s): [hitbot_driver::getFTConfig](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L400-L407)

- axle_sensor_config[(kelo_hitbot_driver::Config)](msg/Config.msg)  
Related function(s): [hitbot_driver::getAxleSensorConfig](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L371-L378)

- digital_input(std_msgs::Int32MultiArray)  
Related function(s): [hitbot_driver::getDI](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L162-L164)

- tool_digital_input(std_msgs::Int32MultiArray)  
Related function(s): [hitbot_driver::getToolDI](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L181-L183)

- analog_input(std_msgs::Float32MultiArray)  
Related function(s): [hitbot_driver::getAI](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L171-L173)

- tool_analog_input(std_msgs::Float32MultiArray)  
Related function(s): [hitbot_driver::getToolAI](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L190-L192)

- tool_coordinate_frame[(kelo_hitbot_driver::Axes)](msg/Axes.msg)  
Related function(s): [hitbot_driver::computeTool](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L258-L261)

- object_coordinate_frame[(kelo_hitbot_driver::Axes)](msg/Axes.msg)  
Related function(s): [hitbot_driver::computeWObjCoord](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L274-L277)


# Subscribers

Here is the list of the subscribed topics:
- move_PTP[(kelo_hitbot_driver::PTP)](msg/PTP.msg)  
Related function(s): [hitbot_driver::movePTP](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L66-L79)

- move_linear[(kelo_hitbot_driver::Lin)](msg/Lin.msg)  
Related function(s): [hitbot_driver::moveLin](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L96-L109)

- move_arc[(kelo_hitbot_driver::Arc)](msg/Arc.msg)  
Related function(s): [hitbot_driver::moveArc](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L81-L94)

- move_jog[(kelo_hitbot_driver::Jog)](msg/Jog.msg)  
Related function(s): [hitbot_driver::startJog](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L111-L118)

- move_servo[(kelo_hitbot_driver::Servo)](msg/Servo.msg)  
Related function(s): [hitbot_driver::servoJ](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L120-L127)

- automatic_procedure(std_msgs::String)  
Related function(s): [hitbot_driver::startProcedure](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L129-L130), [hitbot_driver::stopProcedure](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L132-L133), [hitbot_driver::pauseProcedure](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L135-L136), [hitbot_driver::resumeProcedure](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L138-L139)

- reset_error(std_msgs::Empty)  
Related function(s): [hitbot_driver::resetErrors](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L141-L142)

- stop(std_msgs::Empty)  
Related function(s): [hitbot_driver::stopJog](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L144-L145), [hitbot_driver::stopLine](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L147-L148), [hitbot_driver::stopTool](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L150-L151), [hitbot_driver::stopWorkpiece](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L153-L154)

- wait_DI[(kelo_hitbot_driver::WaitDigital)](msg/WaitDigital.msg)  
Related function(s): [hitbot_driver::waitDI](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L194-L199)

- wait_AI[(kelo_hitbot_driver::WaitAnalog)](msg/WaitAnalog.msg)  
Related function(s): [hitbot_driver::waitAI](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L201-L206)

- set_DO[(kelo_hitbot_driver::DigitalOutput)](msg/DigitalOutput.msg)  
Related function(s): [hitbot_driver::setDO](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L156-L160)

- set_AO[(kelo_hitbot_driver::AnalogOutput)](msg/AnalogOutput.msg)  
Related function(s): [hitbot_driver::setAO](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L166-L169)

- start_welding[(kelo_hitbot_driver::Weld)](msg/Weld.msg)  
Related function(s): [hitbot_driver::startArc](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L296-L299)

- stop_welding[(kelo_hitbot_driver::Weld)](msg/Weld.msg)  
Related function(s): [hitbot_driver::endArc](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L301-L304)

- laser_on(std_msgs::Int32)  
Related function(s): [hitbot_driver::laserOn](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L306-L308)

- laser_off(std_msgs::Empty)  
Related function(s): [hitbot_driver::laserOff](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L310-L311)

- track_LT_on(std_msgs::Int32)  
Related function(s): [hitbot_driver::trackLTOn](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L313-L314)

- track_LT_off(std_msgs::Empty)  
Related function(s): [hitbot_driver::trackLTOff](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L316-L317)

- start_LT_search[(kelo_hitbot_driver::LT)](msg/LT.msg)  
Related function(s): [hitbot_driver::startLTSearch](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L319-L324)

- stop_LT_search(std_msgs::Empty)  
Related function(s): [hitbot_driver::stopLTSearch](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L326-L327)

- start_weave(std_msgs::UInt8)  
Related function(s): [hitbot_driver::startWeave](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L338-L340)

- end_weave(std_msgs::UInt8)  
Related function(s): [hitbot_driver::endWeave](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L342-L344)

- weave_param[(kelo_hitbot_driver::Weave)](msg/Weave.msg)  
Related function(s): [hitbot_driver::setWeaveParam](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L329-L336)

- mode(std_msgs::Int32)  
Related function(s): [hitbot_driver::switchMode](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L226-L228)

- set_speed(std_msgs::UInt8)  
Related function(s): [hitbot_driver::setSpeed](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L230-L232)

- drag_teach_switch(std_msgs::UInt8)  
Related function(s): [hitbot_driver::switchDragTeach](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L234-L236)

- set_install_position(std_msgs::UInt8)  
Related function(s): [hitbot_driver::setRobotInstallPos](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L238-L240)

- set_tool_point(std_msgs::Int32)  
Related function(s): [hitbot_driver::setToolPoint](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L254-L256)

- set_tool_coord[(kelo_hitbot_driver::Tool)](msg/Tool.msg)  
Related function(s): [hitbot_driver::setToolCoord](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L263-L268)

- set_object_point(std_msgs::Int32)  
Related function(s): [hitbot_driver::setWObjCoordPoint](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L270-L272)

- set_object_coord[(kelo_hitbot_driver::Workpiece)](msg/Workpiece.msg)  
Related function(s): [hitbot_driver::setWObjCoord](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L279-L282)

- set_tool_list[(kelo_hitbot_driver::Tool)](msg/Tool.msg)  
Related function(s): [hitbot_driver::setToolList](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L284-L289)

- set_object_list[(kelo_hitbot_driver::Workpiece)](msg/Workpiece.msg)  
Related function(s): [hitbot_driver::setWObjList](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L291-L294)

- activate_gripper[(kelo_hitbot_driver::GripperAct)](msg/GripperAct.msg)  
Related function(s): [hitbot_driver::activateGripper](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L346-L349)

- move_gripper[(kelo_hitbot_driver::GripperMotion)](msg/GripperMotion.msg)  
Related function(s): [hitbot_driver::moveGripper](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L351-L357)

- set_axle_config[(kelo_hitbot_driver::Config)](msg/Config.msg)  
Related function(s): [hitbot_driver::configureAxleSensor](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L363-L369)

- activate_axle_sensor(std_msgs::Int32)  
Related function(s): [hitbot_driver::activateAxleSensor](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L380-L382)

- write_axle_sensor_register[(kelo_hitbot_driver::AxleSensorReg)](msg/AxleSensorReg.msg)  
Related function(s): [hitbot_driver::writeAxleSensorReg](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L384-L390)

- set_FT_config[(kelo_hitbot_driver::Config)](msg/Config.msg)  
Related function(s): [hitbot_driver::setFTConfig](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L392-L398)

- activate_FT(std_msgs::Int32)  
Related function(s): [hitbot_driver::activateFT](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L409-L411)

- guard_FT[(kelo_hitbot_driver::GuardFT)](msg/GuardFT.msg)  
Related function(s): [hitbot_driver::guardFT](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L413-L418)

- set_FT_rcs(std_msgs::Int32)  
Related function(s): [hitbot_driver::setFTRCS](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L420-L422)

- set_FT_zero(std_msgs::UInt8)  
Related function(s): [hitbot_driver::setFTZero](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L424-L426)

