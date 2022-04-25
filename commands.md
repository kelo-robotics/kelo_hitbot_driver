# Publishers

Here is the list of the published topics:
- joint_state[(kelo_hitbot_driver::JointState)](msg/JointState.msg)  
Related function(s): [hitbot_driver::getJointPosDegree](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L389-L390), [hitbot_driver::getJointSpeedsDegree](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L395-L396), [hitbot_driver::getJointTorques](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L427-L428)

- TCP_pose[(kelo_hitbot_driver::Axes)](msg/Axes.msg)  
Related function(s): [hitbot_driver::getActualTCPPose](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L401-L404)

- TCP_target[(kelo_hitbot_driver::Axes)](msg/Axes.msg)  
Related function(s): [hitbot_driver::getTargetTCPPose](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L437-L440)

- TCP_flange_pose[(kelo_hitbot_driver::Axes)](msg/Axes.msg)  
Related function(s): [hitbot_driver::getActualToolFlangePose](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L409-L412)

- TCP_offset[(kelo_hitbot_driver::Axes)](msg/Axes.msg)  
Related function(s): [hitbot_driver::getTCPOffset](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L442-L445)

- TCP_num(std_msgs::Int32)  
Related function(s): [hitbot_driver::getActualTCPNum](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L406-L407)

- target_payload(std_msgs::Float32)  
Related function(s): [hitbot_driver::getTargetPayload](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L430-L431)

- target_payload_cog(std_msgs::Float32MultiArray)  
Related function(s): [hitbot_driver::getTargetPayloadCog](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L433-L435)

- joint_limits(std_msgs::Float64MultiArray)  
Related function(s): [hitbot_driver::getJointSoftLimitDeg](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L447-L448)

- FT_config[(kelo_hitbot_driver::Config)](msg/Config.msg)  
Related function(s): [hitbot_driver::getFTConfig](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L361-L368)

- axle_sensor_config[(kelo_hitbot_driver::Config)](msg/Config.msg)  
Related function(s): [hitbot_driver::getAxleSensorConfig](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L332-L339)

- digital_input(std_msgs::Int32MultiArray)  
Related function(s): [hitbot_driver::getDI](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L123-L125)

- tool_digital_input(std_msgs::Int32MultiArray)  
Related function(s): [hitbot_driver::getToolDI](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L142-L144)

- analog_input(std_msgs::Float32MultiArray)  
Related function(s): [hitbot_driver::getAI](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L132-L134)

- tool_analog_input(std_msgs::Float32MultiArray)  
Related function(s): [hitbot_driver::getToolAI](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L151-L153)

- tool_coordinate_frame[(kelo_hitbot_driver::Axes)](msg/Axes.msg)  
Related function(s): [hitbot_driver::computeTool](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L219-L222)

- object_coordinate_frame[(kelo_hitbot_driver::Axes)](msg/Axes.msg)  
Related function(s): [hitbot_driver::computeWObjCoord](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L235-L238)


# Subscribers

Here is the list of the subscribed topics:
- move_PTP[(kelo_hitbot_driver::PTP)](msg/PTP.msg)  
Related function(s): [hitbot_driver::movePTP](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L27-L40)

- move_linear[(kelo_hitbot_driver::Lin)](msg/Lin.msg)  
Related function(s): [hitbot_driver::moveLin](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L57-L70)

- move_arc[(kelo_hitbot_driver::Arc)](msg/Arc.msg)  
Related function(s): [hitbot_driver::moveArc](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L42-L55)

- move_jog[(kelo_hitbot_driver::Jog)](msg/Jog.msg)  
Related function(s): [hitbot_driver::startJog](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L72-L79)

- move_servo[(kelo_hitbot_driver::Servo)](msg/Servo.msg)  
Related function(s): [hitbot_driver::servoJ](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L81-L88)

- automatic_procedure(std_msgs::String)  
Related function(s): [hitbot_driver::startProcedure](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L90-L91), [hitbot_driver::stopProcedure](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L93-L94), [hitbot_driver::pauseProcedure](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L96-L97), [hitbot_driver::resumeProcedure](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L99-L100)

- reset_error(std_msgs::Empty)  
Related function(s): [hitbot_driver::resetErrors](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L102-L103)

- stop(std_msgs::Empty)  
Related function(s): [hitbot_driver::stopJog](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L105-L106), [hitbot_driver::stopLine](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L108-L109), [hitbot_driver::stopTool](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L111-L112), [hitbot_driver::stopWorkpiece](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L114-L115)

- wait_DI[(kelo_hitbot_driver::WaitDigital)](msg/WaitDigital.msg)  
Related function(s): [hitbot_driver::waitDI](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L155-L160)

- wait_AI[(kelo_hitbot_driver::WaitAnalog)](msg/WaitAnalog.msg)  
Related function(s): [hitbot_driver::waitAI](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L162-L167)

- set_DO[(kelo_hitbot_driver::DigitalOutput)](msg/DigitalOutput.msg)  
Related function(s): [hitbot_driver::setDO](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L117-L121)

- set_AO[(kelo_hitbot_driver::AnalogOutput)](msg/AnalogOutput.msg)  
Related function(s): [hitbot_driver::setAO](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L127-L130)

- start_welding[(kelo_hitbot_driver::Weld)](msg/Weld.msg)  
Related function(s): [hitbot_driver::startArc](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L257-L260)

- stop_welding[(kelo_hitbot_driver::Weld)](msg/Weld.msg)  
Related function(s): [hitbot_driver::endArc](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L262-L265)

- laser_on(std_msgs::Int32)  
Related function(s): [hitbot_driver::laserOn](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L267-L269)

- laser_off(std_msgs::Empty)  
Related function(s): [hitbot_driver::laserOff](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L271-L272)

- track_LT_on(std_msgs::Int32)  
Related function(s): [hitbot_driver::trackLTOn](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L274-L275)

- track_LT_off(std_msgs::Empty)  
Related function(s): [hitbot_driver::trackLTOff](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L277-L278)

- start_LT_search[(kelo_hitbot_driver::LT)](msg/LT.msg)  
Related function(s): [hitbot_driver::startLTSearch](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L280-L285)

- stop_LT_search(std_msgs::Empty)  
Related function(s): [hitbot_driver::stopLTSearch](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L287-L288)

- start_weave(std_msgs::UInt8)  
Related function(s): [hitbot_driver::startWeave](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L299-L301)

- end_weave(std_msgs::UInt8)  
Related function(s): [hitbot_driver::endWeave](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L303-L305)

- weave_param[(kelo_hitbot_driver::Weave)](msg/Weave.msg)  
Related function(s): [hitbot_driver::setWeaveParam](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L290-L297)

- mode(std_msgs::Int32)  
Related function(s): [hitbot_driver::switchMode](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L187-L189)

- set_speed(std_msgs::UInt8)  
Related function(s): [hitbot_driver::setSpeed](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L191-L193)

- drag_teach_switch(std_msgs::UInt8)  
Related function(s): [hitbot_driver::switchDragTeach](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L195-L197)

- set_install_position(std_msgs::UInt8)  
Related function(s): [hitbot_driver::setRobotInstallPos](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L199-L201)

- set_tool_point(std_msgs::Int32)  
Related function(s): [hitbot_driver::setToolPoint](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L215-L217)

- set_tool_coord[(kelo_hitbot_driver::Tool)](msg/Tool.msg)  
Related function(s): [hitbot_driver::setToolCoord](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L224-L229)

- set_object_point(std_msgs::Int32)  
Related function(s): [hitbot_driver::setWObjCoordPoint](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L231-L233)

- set_object_coord[(kelo_hitbot_driver::Workpiece)](msg/Workpiece.msg)  
Related function(s): [hitbot_driver::setWObjCoord](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L240-L243)

- set_tool_list[(kelo_hitbot_driver::Tool)](msg/Tool.msg)  
Related function(s): [hitbot_driver::setToolList](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L245-L250)

- set_object_list[(kelo_hitbot_driver::Workpiece)](msg/Workpiece.msg)  
Related function(s): [hitbot_driver::setWObjList](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L252-L255)

- activate_gripper[(kelo_hitbot_driver::GripperAct)](msg/GripperAct.msg)  
Related function(s): [hitbot_driver::activateGripper](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L307-L310)

- move_gripper[(kelo_hitbot_driver::GripperMotion)](msg/GripperMotion.msg)  
Related function(s): [hitbot_driver::moveGripper](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L312-L318)

- set_axle_config[(kelo_hitbot_driver::Config)](msg/Config.msg)  
Related function(s): [hitbot_driver::configureAxleSensor](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L324-L330)

- activate_axle_sensor(std_msgs::Int32)  
Related function(s): [hitbot_driver::activateAxleSensor](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L341-L343)

- write_axle_sensor_register[(kelo_hitbot_driver::AxleSensorReg)](msg/AxleSensorReg.msg)  
Related function(s): [hitbot_driver::writeAxleSensorReg](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L345-L351)

- set_FT_config[(kelo_hitbot_driver::Config)](msg/Config.msg)  
Related function(s): [hitbot_driver::setFTConfig](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L353-L359)

- activate_FT(std_msgs::Int32)  
Related function(s): [hitbot_driver::activateFT](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L370-L372)

- guard_FT[(kelo_hitbot_driver::GuardFT)](msg/GuardFT.msg)  
Related function(s): [hitbot_driver::guardFT](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L374-L379)

- set_FT_rcs(std_msgs::Int32)  
Related function(s): [hitbot_driver::setFTRCS](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L381-L383)

- set_FT_zero(std_msgs::UInt8)  
Related function(s): [hitbot_driver::setFTZero](include/kelo_hitbot_driver/HitbotDriver.h?plain=1#L385-L387)

