#include "dwa/dwa_general.h"



int main(int argc, char **argv) {
	ros::init(argc, argv, "dwagen");
	ros::NodeHandle n;

	DWAGen dwa = DWAGen("dwagen", n);
	dwa.run();
}
