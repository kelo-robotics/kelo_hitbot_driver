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
