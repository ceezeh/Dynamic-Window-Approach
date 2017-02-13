#ifndef DISTANCE_H
#define DISTANCE_H
#include <map/helper.h>

using namespace std;
struct Distance {
	float lin;
	float ang;
	Distance() {
		ang = -1;
		lin = -1;
	}

	Distance(float lin_t, float ang_t) {
		lin = lin_t;
		ang = ang_t;
	}
	bool operator==(Distance dist) // copy/move constructor is called to construct arg
			{
		if (equals(this->lin, dist.lin) && equals(this->ang, dist.ang)) {
			return true;
		} else {
			return false;
		}
	}
	Distance operator=(Distance dist) // copy/move constructor is called to construct arg
			{
		this->lin = dist.lin;
		this->ang = dist.ang;
		return *this;
	}
};
#endif
