cmake_minimum_required(VERSION 2.8.3)
project(kelo_hitbot_driver)

set (CMAKE_CXX_STANDARD 11)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)


add_message_files(
  FILES
  Axes.msg
  AxesD.msg
  MotionCommand.msg
  AnalogOutput.msg
  Arc.msg
  Collision.msg
  DigitalOutput.msg
  Jog.msg
  JointState.msg
  Lin.msg
  LT.msg
  PTP.msg
  Servo.msg
  Tool.msg
  WaitAnalog.msg
  WaitDigital.msg
  Weave.msg
  Weld.msg
  Workpiece.msg
  GripperAct.msg
  GripperMotion.msg
  Config.msg
  AxleSensorReg.msg
  GuardFT.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  kelo_hitbot_driver
)

catkin_package(
  CATKIN_DEPENDS std_msgs
  INCLUDE_DIRS include
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_executable(hitbot_driver src/HitbotROSWrapper.cpp src/HitbotDriver.cpp)
target_link_libraries(hitbot_driver
	${catkin_LIBRARIES}
)

add_executable(hitbot_server src/HitbotServer.cpp)
target_link_libraries(hitbot_server
	${catkin_LIBRARIES}
)

