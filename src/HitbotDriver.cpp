#include "kelo_hitbot_driver/HitbotDriver.h"
#include <ros/ros.h>
HitbotDriver::HitbotDriver() {
	sock = 0;
	buffer[1024] = {0};
	msgCounter = 1;
	
	startHeader = "/f/b";
	endHeader = "/b/f";
	separator = "III";

}

HitbotDriver::~HitbotDriver() {
	
}

int HitbotDriver::setupConnection(std::string IP_address, int port) {
	// create socket
	if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0)
	{
		printf("\n Socket creation error \n");
		return -1;
	}

	serv_addr.sin_family = AF_INET;
	serv_addr.sin_port = htons(port);

	// convert IPv4 and IPv6 addresses from text to binary form
	if(inet_pton(AF_INET, IP_address.c_str(), &serv_addr.sin_addr)<=0)
	{
		std::cout << "Invalid address/ Address not supported" << std::endl;
		return -1;
	}

	if (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
	{
		std::cout << "Connection Failed" << std::endl;
		return -1;
	}
	return 0;
}

void HitbotDriver::sendCommand(int ID, std::string command) {
	ros::Time tstart = ros::Time::now();
	std::string sequence = std::to_string(msgCounter);
	std::string instructionType = std::to_string(ID);
	std::string count = std::to_string(command.length());
	std::string packet = startHeader + separator + sequence + separator + instructionType + separator + count + separator + command + separator + endHeader;
	const char *packet_str = packet.c_str();
	send(sock, packet_str , strlen(packet_str) , 0);
	msgCounter++;

	std::cout << "sending: " << packet << std::endl;
	//memset (buffer,'0', 1024);
	//buffer[1024] = {0};
	ros::Time treceive = ros::Time::now();
	int valread = read(sock, buffer, 1024);
	ros::Time tend = ros::Time::now();
	std::cout << "receiving: " << buffer << std::endl << std::endl;
	std::cout << "Sending time = " << (tend - tstart).toSec() << std::endl;
	std::cout << "Receiving time = " << (tend - treceive).toSec() << std::endl;
}

std::vector<std::string> HitbotDriver::getContent() {
	ros::Time tstart = ros::Time::now();
	std::string packet(buffer);
	
	for (unsigned int i = 0; i < 3; i++) {
		packet.erase(0, packet.find(separator) + separator.length());
	}
	std::string commandStr = packet.substr(0, packet.find(separator));
	packet.erase(0, packet.find(separator) + separator.length());
	std::string content = packet.substr(0, packet.find(separator));

	std::vector<std::string> data;
	if (std::stoi(commandStr) == 500) {
		resetErrors();
		std::cout << "Command failed. Resetting error flag" << std::endl;
	} else{
		while (content.find(",") != std::string::npos) {
			data.push_back(content.substr(0, content.find(",")));
			content.erase(0, content.find(",") + 1);
		}
		data.push_back(content.substr(0, content.length() + 1));
	}	
	ros::Time tend = ros::Time::now();
	std::cout << "Get content time = " << (tend - tstart).toSec() << std::endl;
	return data;
}

std::vector<int> HitbotDriver::getContentInt() {
	std::vector<std::string> valuesStr = getContent();
	std::vector<int> values(valuesStr.size());
	std::transform(valuesStr.begin(), valuesStr.end(), values.begin(), [](const std::string& val) { return std::stoi(val); });
	return values;
}

bool HitbotDriver::getContentBool() {
	std::vector<std::string> valuesStr = getContent();
	if (valuesStr.size() == 1 && valuesStr[0] == "1")
		return true;
	else
		return false;
}

std::vector<float> HitbotDriver::getContentFloat() {
	std::vector<std::string> valuesStr = getContent();
	std::vector<float> values(valuesStr.size());
	std::transform(valuesStr.begin(), valuesStr.end(), values.begin(), [](const std::string& val) { return std::stof(val); });
	return values;
}

std::vector<double> HitbotDriver::getContentDouble() {
	std::vector<std::string> valuesStr = getContent();
	std::vector<double> values(valuesStr.size());
	std::transform(valuesStr.begin(), valuesStr.end(), values.begin(), [](const std::string& val) { return std::stod(val); });
	return values;
}

//
// Simple Move Functions
//

bool HitbotDriver::movePTP(std::vector<float> TCPPose, int toolNum, int workpieceNum, 
							float speed, float acc, int ovl, std::vector<float> extAxisPos, float blendT, 
							uint8_t offsetFlag, std::vector<float> offset) {
	std::vector<float> jointAngle = getInverseKinematics(0, TCPPose, -1);							

	std::string command = "MoveJ(";
	for (unsigned int i = 0; i < jointAngle.size(); i++) { command += std::to_string(jointAngle[i]) + ","; }
	for (unsigned int i = 0; i < TCPPose.size(); i++) { command += std::to_string(TCPPose[i]) + ","; }
	command += std::to_string(toolNum) + "," + std::to_string(workpieceNum) + "," + std::to_string(speed) + "," + std::to_string(acc) + "," + std::to_string(ovl);
	for (unsigned int i = 0; i < extAxisPos.size(); i++) { command += std::to_string(extAxisPos[i]) + ","; }
	command += std::to_string(blendT) + "," + std::to_string(offsetFlag) + ",";
	for (unsigned int i = 0; i < offset.size() - 1; i++) { command += std::to_string(offset[i]) + ","; }
	command += std::to_string(offset[offset.size() - 1]) + ")";
	
	sendCommand(MoveJ_ID, command);
	return getContentBool();
}

bool HitbotDriver::moveArc(std::vector<float> TCPPose1, int toolNum1, int workpieceNum1, 
							float speed1, float acc1, std::vector<float> extAxisPos1, 
							std::vector<float> TCPPose2, int toolNum2, int workpieceNum2, float speed2, float acc2, 
							std::vector<float> extAxisPos2, int ovl, float blendR, uint8_t offsetFlag, std::vector<float> offset) {
	std::vector<float> jointAngle1 = getInverseKinematics(0, TCPPose1, -1);
	std::vector<float> jointAngle2 = getInverseKinematics(0, TCPPose2, -1);
								
	std::string command = "MoveC(";
	for (unsigned int i = 0; i < jointAngle1.size(); i++) { command += std::to_string(jointAngle1[i]) + ","; }
	for (unsigned int i = 0; i < TCPPose1.size(); i++) { command += std::to_string(TCPPose1[i]) + ","; }
	command += std::to_string(toolNum1) + "," + std::to_string(workpieceNum1) + "," + std::to_string(speed1) + "," + std::to_string(acc1) + "," ;
	for (unsigned int i = 0; i < extAxisPos1.size(); i++) { command += std::to_string(extAxisPos1[i]) + ","; }
	for (unsigned int i = 0; i < jointAngle2.size(); i++) { command += std::to_string(jointAngle2[i]) + ","; }
	for (unsigned int i = 0; i < TCPPose2.size(); i++) { command += std::to_string(TCPPose2[i]) + ","; }
	command += std::to_string(toolNum2) + "," + std::to_string(workpieceNum2) + "," + std::to_string(speed2) + "," + std::to_string(acc2) + "," ;
	for (unsigned int i = 0; i < extAxisPos2.size(); i++) { command += std::to_string(extAxisPos2[i]) + ","; }
	command += std::to_string(ovl) + "," + std::to_string(blendR) + "," + std::to_string(offsetFlag) + ",";
	for (unsigned int i = 0; i < offset.size() - 1; i++) { command += std::to_string(offset[i]) + ","; }
	command += std::to_string(offset[offset.size() - 1]) + ")";
	
	sendCommand(MoveC_ID, command);
	return getContentBool();
}

bool HitbotDriver::moveLin(std::vector<float> TCPPose, int toolNum, int workpieceNum, 
							float speed, float acc, int ovl, float blendR, std::vector<float> extAxisPos,  
							uint8_t searchFlag, uint8_t offsetFlag, std::vector<float> offset) {
	std::vector<float> jointAngle = getInverseKinematics(0, TCPPose, -1);									
								
	std::string command = "MoveL(";
	for (unsigned int i = 0; i < jointAngle.size(); i++) { command += std::to_string(jointAngle[i]) + ","; }
	for (unsigned int i = 0; i < TCPPose.size(); i++) { command += std::to_string(TCPPose[i]) + ","; }
	command += std::to_string(toolNum) + "," + std::to_string(workpieceNum) + "," + std::to_string(speed) + "," + std::to_string(acc) + ",";
	command += std::to_string(ovl) + "," + std::to_string(blendR) + ",";
	for (unsigned int i = 0; i < extAxisPos.size(); i++) { command += std::to_string(extAxisPos[i]) + ","; }
	command += std::to_string(searchFlag) + "," + std::to_string(offsetFlag) + ",";
	for (unsigned int i = 0; i < offset.size() - 1; i++) { command += std::to_string(offset[i]) + ","; }
	command += std::to_string(offset[offset.size() - 1]) + ")";
	
	sendCommand(MoveL_ID, command);
	return getContentBool();
}

bool HitbotDriver::startJog(uint8_t motionCmd, uint8_t jointNum, uint8_t direction, float vel, float acc, float maxDistance) {
	std::string command = "StartJOG(";
	command += std::to_string(motionCmd) + "," + std::to_string(jointNum) + "," + std::to_string(direction) + ",";
	command += std::to_string(vel) + "," + std::to_string(acc) + "," + std::to_string(maxDistance) + ")";
	
	sendCommand(StartJOG_ID, command);
	return getContentBool();
}

bool HitbotDriver::servoJ(std::vector<float> jointAngle, float acc, float vel, float t, float lookaheadTime, float gain) {
	std::string command = "ServoJ(";
	for (unsigned int i = 0; i < jointAngle.size(); i++) { command += std::to_string(jointAngle[i]) + ","; }
	command += std::to_string(acc) + "," + std::to_string(vel) + "," + std::to_string(t) + ","; 
	command += std::to_string(lookaheadTime) + "," + std::to_string(gain) + ")";
	
	sendCommand(ServoJ_ID, command);
	return getContentBool();
}

//
// Procedure
//

bool HitbotDriver::startProcedure() {
	sendCommand(START_ID, "START");
	return getContentBool();
}

bool HitbotDriver::stopProcedure() {
	sendCommand(STOP_ID, "STOP");
	return getContentBool();
}

bool HitbotDriver::pauseProcedure() {
	sendCommand(PAUSE_ID, "PAUSE");
	return getContentBool();
}

bool HitbotDriver::resumeProcedure() {
	sendCommand(RESUME_ID, "RESUME");
	return getContentBool();
}

bool HitbotDriver::resetErrors() {
	sendCommand(RESETALLERROR_ID, "RESETALLERROR");
	return getContentBool();
}

bool HitbotDriver::stopJog() {
	sendCommand(STOPJOG_ID, "STOPJOG");
	return getContentBool();
}

bool HitbotDriver::stopLine() {
	sendCommand(STOPLINE_ID, "STOPLINE");
	return getContentBool();
}

bool HitbotDriver::stopTool() {
	sendCommand(STOPTOOL_ID, "STOPTOOL");
	return getContentBool();
}

bool HitbotDriver::stopWorkpiece() {
	sendCommand(STOPWORKPIECE_ID, "STOPWORKPIECE_ID");
	return getContentBool();
}

//
// Input Output
//

bool HitbotDriver::setDO(int nIO, int bOpen, int smooth) {
	std::string command = "SetDO(" + std::to_string(nIO) + "," + std::to_string(bOpen) + "," + std::to_string(smooth) + ")";
	sendCommand(SetDO_ID, command);
	return getContentBool();
}

bool HitbotDriver::getDI(int nIO) {
	std::string command = "GetDI(" + std::to_string(nIO) + ")";
	sendCommand(GetDI_ID, command);
	return getContentBool();
}

bool HitbotDriver::setAO(int nIO, float value) {
	std::string command = "SetAO(" + std::to_string(nIO) + "," + std::to_string(value) + ")"; 
	sendCommand(SetAO_ID, command);
	return getContentBool();
}

std::vector<float> HitbotDriver::getAI(int nIO) {
	std::string command = "GetAI(" + std::to_string(nIO) + ")";
	sendCommand(GetAI_ID, command);
	return getContentFloat();
}

bool HitbotDriver::setToolDO(int nIO, int bOpen, int smooth) {
	std::string command = "SetToolDO(" + std::to_string(nIO) + "," + std::to_string(bOpen) + "," + std::to_string(smooth) + ")";
	sendCommand(SetToolDO_ID, command);
	return getContentBool();
}

bool HitbotDriver::getToolDI(int nIO) {
	std::string command = "GetToolDI(" + std::to_string(nIO) + ")"; 
	sendCommand(GetToolDI_ID, command);
	return getContentBool();
}

bool HitbotDriver::setToolAO(int nIO, float value) {
	std::string command = "SetToolAO(" + std::to_string(nIO) + "," + std::to_string(value) + ")"; 
	sendCommand(SetToolAO_ID, command);
	return getContentBool();
}

std::vector<float> HitbotDriver::getToolAI(int nIO) {
	std::string command = "GetToolAI(" +std::to_string(nIO) + ")"; 
	sendCommand(GetToolAI_ID, command);
	return getContentFloat();
}

bool HitbotDriver::waitDI(int nIO, int bOpen, int time, uint8_t alarm) {
	std::string command = "WaitDI(";
	command += std::to_string(nIO) + "," + std::to_string(bOpen) + "," + std::to_string(time) + "," + std::to_string(alarm) + ")"; 
	sendCommand(WaitDI_ID, command);
	return getContentBool();
}

bool HitbotDriver::waitAI(int nIO, int sign, int bOpen, int time, uint8_t alarm) {
	std::string command = "WaitAI(";
	command += std::to_string(nIO) + "," + std::to_string(sign) + "," + std::to_string(bOpen) + "," + std::to_string(time) + "," + std::to_string(alarm) + ")";
	sendCommand(WaitAI_ID, command);
	return getContentBool();
}

bool HitbotDriver::waitToolDI(int nIO, int bOpen, int time, uint8_t alarm) {
	std::string command = "WaitToolDI(";
	command += std::to_string(nIO) + "," + std::to_string(bOpen) + "," + std::to_string(time) + "," + std::to_string(alarm) + ")"; 
	sendCommand(WaitToolDI_ID, command);
	return getContentBool();
}

bool HitbotDriver::waitToolAI(int nIO, int sign, int bOpen, int time, uint8_t alarm) {
	std::string command = "WaitToolAI(";
	command += std::to_string(nIO) + "," + std::to_string(sign) + "," + std::to_string(bOpen) + "," + std::to_string(time) + "," + std::to_string(alarm) + ")";
	sendCommand(WaitToolAI_ID, command);
	return getContentBool();
}

bool HitbotDriver::waitMs(int time) {
	std::string command = "WaitMs(" + std::to_string(time) + ")";
	sendCommand(WaitMs_ID, command);
	return getContentBool();
}

//
// Configuration
//

bool HitbotDriver::switchMode(int mode) {
	std::string command = "Mode(" + std::to_string(mode) + ")";
	sendCommand(Mode_ID, command);
	return getContentBool();
}

bool HitbotDriver::setSpeed(uint8_t speed) {
	std::string command = "SetSpeed(" + std::to_string(speed) + ")";
	sendCommand(SetSpeed_ID, command);
	return getContentBool();
}

bool HitbotDriver::switchDragTeach(uint8_t status) {
	std::string command = "DragTeachSwitch(" + std::to_string(status) + ")";
	sendCommand(DragTeachSwitch_ID, command);
	return getContentBool();
}

bool HitbotDriver::setRobotInstallPos(uint8_t installPos) {
	std::string command = "SetRobotInstallPos(" + std::to_string(installPos) + ")";
	sendCommand(SetRobotInstallPos_ID, command);
	return getContentBool();
}

bool HitbotDriver::setAntiCollision(int level) {
	std::string command = "SetAntiCollision(" + std::to_string(level) + ")";
	sendCommand(SetAnticollision_ID, command);
	return getContentBool();
}

bool HitbotDriver::setLimitPositive(std::vector<double> jointLimit) {
	std::string command = "SetLimitPositive(";
	for (unsigned int i = 0; i < jointLimit.size() - 1; i++) { command += std::to_string(jointLimit[i]) + ","; }
	command += std::to_string(jointLimit[jointLimit.size() - 1]) + ")";
	sendCommand(SetLimitPositive_ID, command);
	return getContentBool();
}

bool HitbotDriver::setLimitNegative(std::vector<double> jointLimit) {
	std::string command = "SetLimitNegative(";
	for (unsigned int i = 0; i < jointLimit.size() - 1; i++) { command += std::to_string(jointLimit[i]) + ","; }
	command += std::to_string(jointLimit[jointLimit.size() - 1]) + ")";
	sendCommand(SetLimitNegative_ID, command);
	return getContentBool();
}

bool HitbotDriver::setToolPoint(int id) {
	std::string command = "SetToolPoint(";
	command += std::to_string(id) + ")";
	sendCommand(SetToolPoint_ID, command);
	return getContentBool();
}

std::vector<float> HitbotDriver::computeTool() {
	sendCommand(ComputeTool_ID, "ComputeTool()");
	return getContentFloat();
}

bool HitbotDriver::setToolCoord(int toolNum, std::vector<double> TCPPose, int type, int install) {
	std::string command = "SetToolCoord(";
	command += std::to_string(toolNum) + ",";
	for (unsigned int i = 0; i < TCPPose.size(); i++) { command += std::to_string(TCPPose[i]) + ","; }
	command += std::to_string(type) + "," + std::to_string(install) + ")";
	sendCommand(SetToolPoint_ID, command);
	return getContentBool();
}

bool HitbotDriver::setWObjCoordPoint(int id) {
	std::string command = "SetWObjCoordPoint(";
	command += std::to_string(id) + ")";
	sendCommand(SetWObjCoordPoint_ID, command);
	return getContentBool();
}

std::vector<float> HitbotDriver::computeWObjCoord() {
	sendCommand(ComputeWObjCoord_ID, "ComputeWObjCoord()");
	return getContentFloat();
}

bool HitbotDriver::setWObjCoord(int workpieceNum, std::vector<double> pose) {
	std::string command = "SetWObjCoord(";
	command += std::to_string(workpieceNum) + ",";
	for (unsigned int i = 0; i < pose.size() - 1; i++) { command += std::to_string(pose[i]) + ","; }
	command += std::to_string(pose[pose.size() - 1]) + ")";
	sendCommand(SetWObjCoord_ID, command);
	return getContentBool();
}

bool HitbotDriver::setToolList(int toolNum, std::vector<double> TCPPose, int type, int install) {
	std::string command = "SetToolList(";
	command += std::to_string(toolNum) + ",";
	for (unsigned int i = 0; i < TCPPose.size(); i++) { command += std::to_string(TCPPose[i]) + ","; }
	command += std::to_string(type) + "," + std::to_string(install) + ")";
	sendCommand(SetToolList_ID, command);
	return getContentBool();
}

bool HitbotDriver::setWObjList(int workpieceNum, std::vector<double> pose) {
	std::string command = "SetWObjList(";
	command += std::to_string(workpieceNum) + ",";
	for (unsigned int i = 0; i < pose.size() - 1; i++) { command += std::to_string(pose[i]) + ","; }
	command += std::to_string(pose[pose.size() - 1]) + ")";
	sendCommand(SetWObjList_ID, command);
	return getContentBool();
}

//
// Welding commands
//

bool HitbotDriver::startArc(uint8_t arcNum, int maxWT) {
	std::string command = "ARCStart(" + std::to_string(arcNum) + "," + std::to_string(maxWT) + ")";
	sendCommand(ARCStart_ID, command);
	return getContentBool();
}

bool HitbotDriver::endArc(uint8_t arcNum, int maxWT) {
	std::string command = "ARCEnd(" + std::to_string(arcNum) + "," + std::to_string(maxWT) + ")";
	sendCommand(ARCEnd_ID, command);
	return getContentBool();
}

bool HitbotDriver::laserOn(uint8_t ID) {
	std::string command = "LTLaserOn(";
	command += std::to_string(ID) + ")";
	sendCommand(LTLaserOn_ID, command);
	return getContentBool();
}

bool HitbotDriver::laserOff() {
	sendCommand(LTLaserOff_ID, "LTLaserOff()");
	return getContentBool();
}

bool HitbotDriver::trackLTOn() {
	sendCommand(LTTrackOn_ID, "LTTrackOn()");
	return getContentBool();
}

bool HitbotDriver::trackLTOff() {
	sendCommand(LTTrackOff_ID, "LTTrackOff()");
	return getContentBool();
}

bool HitbotDriver::startLTSearch(uint8_t direction, uint32_t vel, int distance, int maxtime) {
	std::string command = "LTSearchStart(";
	command += std::to_string(direction) + "," + std::to_string(vel) + "," + std::to_string(distance) + "," + std::to_string(maxtime) + ")";
	sendCommand(LTSearchStart_ID, command);
	return getContentBool();
}

bool HitbotDriver::stopLTSearch() {
	sendCommand(LTSearchStop_ID, "LTSearchStop()");
	return getContentBool();
}

//
// Auxiliary commands
//

bool HitbotDriver::setWeaveParam(uint8_t num, uint8_t type, float freq, float rang, int lst, int rst) {
	std::string command = "WeaveSetPara(";
	command += std::to_string(num) + "," + std::to_string(freq) + "," + std::to_string(rang) + ",";
	command += std::to_string(lst) + "," + std::to_string(rst) + ")";
	sendCommand(WeaveSetPara_ID, command);
	return getContentBool();
}

bool HitbotDriver::startWeave(uint8_t num) {
	std::string command = "WeaveStart(" + std::to_string(num) + ")";		
	sendCommand(WeaveStart_ID, command);
	return getContentBool();
}

bool HitbotDriver::endWeave(uint8_t num) {
	std::string command = "WeaveEnd(" + std::to_string(num) + ")";
	sendCommand(WeaveEnd_ID, command);
	return getContentBool();
}

//
// Function commands
//


bool HitbotDriver::activateGripper(int id, int act) {
	std::string command = "ActGripper(" + std::to_string(id) + "," + std::to_string(act)  + ")";
	sendCommand(ActGripper_ID, command);
	return getContentBool();
}

bool HitbotDriver::moveGripper(int id, int pos, int speed, int force, int maxTime) {
	std::string command = "MoveGripper(" + std::to_string(id) + "," + std::to_string(pos)  + ",";
	command += std::to_string(speed) + "," + std::to_string(force)  + "," + std::to_string(maxTime) + ")";
	sendCommand(MoveGripper_ID, command);
	return getContentBool();
}

std::vector<int> HitbotDriver::getGripperMotionDone() {
	sendCommand(GetGripperMotionDone_ID, "GetGripperMotionDone()");
	return getContentInt();
}

bool HitbotDriver::configureAxleSensor(int id, int company, int device, int softwareVer, int bus) {
	std::string command = "AxleSensorConfig(" + std::to_string(id) + "," + std::to_string(company) + ",";
	command += std::to_string(device) + "," + std::to_string(softwareVer) + "," + std::to_string(bus) + ")";
	sendCommand(AxleSensorConfig_ID, command);
	return getContentBool();
}

std::vector<int> HitbotDriver::getAxleSensorConfig() {
	sendCommand(AxleSensorConfigGet_ID, "AxleSensorConfigGet()");
	return getContentInt();
}

bool HitbotDriver::activateAxleSensor(int flag) {
	std::string command = "AxleSensorActivate(" + std::to_string(flag) + ")";
	sendCommand(AxleSensorActivate_ID, command);
	return getContentBool();
}

bool HitbotDriver::writeAxleSensorReg(uint8_t addr, uint8_t regHighAddr, uint8_t regLowAddr, 
									uint8_t regNum, uint16_t data1, uint16_t data2) {
	std::string command = "AxleSensorRegWrite(" + std::to_string(addr) + "," + std::to_string(regHighAddr) + ",";
	command += std::to_string(regLowAddr) + "," + std::to_string(regNum) + "," + std::to_string(data1) + "," + std::to_string(data2) + ")";
	sendCommand(AxleSensorRegWrite_ID, command);
	return getContentBool();
}

bool HitbotDriver::setFTConfig(int id, int company, int device, int softwareVer, int bus) {
	std::string command = "FT_SetConfig(" + std::to_string(id) + "," + std::to_string(company) + ",";
	command += std::to_string(device) + "," + std::to_string(softwareVer) + "," + std::to_string(bus) + ")";
	sendCommand(FT_SetConfig_ID, command);
	return getContentBool();
}

std::vector<int> HitbotDriver::getFTConfig() {
	sendCommand(FT_GetConfig_ID, "FT_GetConfig()");
	return getContentInt();
}

bool HitbotDriver::activateFT(int flag) {
	std::string command = "FT_Activate(" + std::to_string(flag) + ")";
	sendCommand(FT_Activate_ID, command);
	return getContentBool();
}

bool HitbotDriver::guardFT(int flag, uint8_t isSelect, std::vector<double> collisionForce) {
	std::string command = "FT_Guard(" + std::to_string(flag) + "," + std::to_string(isSelect) + ",";
	for (unsigned int i = 0; i < collisionForce.size() - 1; i++) { command += std::to_string(collisionForce[i]) + ","; }
	command += std::to_string(collisionForce[collisionForce.size() - 1]) + ")";
	sendCommand(FT_Guard_ID, command);
	return getContentBool();
}

bool HitbotDriver::setFTRCS(int rcs) {
	std::string command = "FT_SetRCS(" + std::to_string(rcs) + ")";
	sendCommand(FT_SetRCS_ID, command);
	return getContentBool();
}

bool HitbotDriver::setFTZero(uint8_t flag) {
	std::string command = "FT_SetZero(" + std::to_string(flag) + ")";
	sendCommand(FT_SetZero_ID, command);
	return getContentBool();
}

//
// Joint States
//

std::vector<float> HitbotDriver::getJointPosDegree() {
	sendCommand(GetActualJointPosDegree_ID, "GetActualJointPosDegree()");
	return getContentFloat();
}

std::vector<float> HitbotDriver::getJointPosRadian() {
	sendCommand(GetActualJointPosRadian_ID, "GetActualJointPosRadian()");
	return getContentFloat();
}

std::vector<float> HitbotDriver::getJointSpeedsDegree() {
	sendCommand(GetActualJointSpeedsDegree_ID, "GetActualJointSpeedsDegree()");
	return getContentFloat();
}

std::vector<float> HitbotDriver::getJointSpeedsRadian() {
	sendCommand(GetActualJointSpeedsRadian_ID, "GetActualJointSpeedsRadian()");
	return getContentFloat();
}

std::vector<float> HitbotDriver::getActualTCPPose() {
	sendCommand(GetActualTCPPose_ID, "GetActualTCPPose()");
	return getContentFloat();
}

std::vector<int> HitbotDriver::getActualTCPNum() {
	sendCommand(GetActualTCPNum_ID, "GetActualTCPNum()");
	return getContentInt();
}

std::vector<float> HitbotDriver::getActualToolFlangePose() {
	sendCommand(GetActualToolFlangePose_ID,  "GetActualToolFlangePose()");
	return getContentFloat();
}

std::vector<float> HitbotDriver::getInverseKinematics(uint8_t flag, std::vector<float> TCPPose, int config) {
	std::string command = "GetInverseKin(" + std::to_string(flag) + ",";
	for (unsigned int i = 0; i < TCPPose.size(); i++) { command += std::to_string(TCPPose[i]) + ","; }
	command += std::to_string(config) + ")";
	sendCommand(GetInverseKin_ID, command);
	return getContentFloat();
}

std::vector<float> HitbotDriver::getForwardKinematics(std::vector<float> jointAngles) {
	std::string command = "GetForwardKin(";
	for (unsigned int i = 0; i < jointAngles.size(); i++) { command += std::to_string(jointAngles[i]) + ","; }
	command += ")";
	sendCommand(GetForwardKin_ID, command);
	return getContentFloat();
}

std::vector<float> HitbotDriver::getJointTorques() {
	sendCommand(GetJointTorques_ID, "GetJointTorques()");
	return getContentFloat();
}

std::vector<float> HitbotDriver::getTargetPayload() {
	sendCommand(GetTargetPayload_ID, "GetTargetPayload()");
	return getContentFloat();
}

std::vector<float> HitbotDriver::getTargetPayloadCog() {
	sendCommand(GetTargetPayloadCog_ID, "GetTargetPayloadCog()");
	return getContentFloat();
}

std::vector<float> HitbotDriver::getTargetTCPPose() {
	sendCommand(GetTargetTCPPose_ID, "GetTargetTCPPose()");
	return getContentFloat();
}

std::vector<float> HitbotDriver::getTCPOffset() {
	sendCommand(GetTCPOffset_ID, "GetTCPOffset()");
	return getContentFloat();
}

std::vector<double> HitbotDriver::getJointSoftLimitDeg() {
	sendCommand(GetJointSoftLimitDeg_ID, "GetJointSoftLimitDeg()");
	return getContentDouble();
}

