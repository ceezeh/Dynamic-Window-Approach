#include "dwa/map.h"
#include "dwa/dwa.h"



int main(int argc, char **argv) {
	ros::init(argc, argv, "dwa");
	ros::NodeHandle n;

	DWA dwa = DWA("DWA", n);
	dwa.run();
}
