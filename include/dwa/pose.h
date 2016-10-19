#ifndef POSE_H
#define POSE_H
#include <map/helper.h>
using namespace std;
struct Pose {
	float x;
	float y;
	float th;
	Pose(float xt, float yt, float tht = 0) :
			x(xt), y(yt), th(tht) {
	}
	Pose() {
		x = y = th = 0;
	}
	bool operator==(Pose pose) // copy/move constructor is called to construct arg
			{
		if (equals(this->x,
				pose.x) && equals(this->x, pose.x) && equals(this->x, pose.x)) {
			return true;
		} else {
			return false;
		}
	}
	Pose operator+(Pose pose) // copy/move constructor is called to construct arg
			{
		pose.x += this->x;
		pose.y += this->y;
		pose.th = angAdd(pose.th, this->th);
		return pose;
	}
	float bearingToPose(Pose pose) {
		float y = pose.y - this->y;
		float x = pose.x - this->x;
		float bearing = atan2(y, x);
		return bearing;
	}

};
#endif
