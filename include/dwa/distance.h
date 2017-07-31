#ifndef DISTANCE_H
#define DISTANCE_H
#include <costmap/helper.h>

using namespace std;
struct Distance {
	float lookahead;
	float clearance;
	float ang;
	Distance() {
		lookahead = -1;
		clearance = -1;
		ang =-1;
	}

	Distance(float l_t, float c_t, float a_t) {
		clearance = c_t;
		lookahead = l_t;
		ang = a_t;
	}
	bool operator==(Distance dist) // copy/move constructor is called to construct arg
			{
		if (equals(this->clearance, dist.clearance) && equals(this->lookahead, dist.lookahead)&& equals(this->ang, dist.ang)) {
			return true;
		} else {
			return false;
		}
	}
	Distance operator=(Distance dist) // copy/move constructor is called to construct arg
			{
		this->clearance = dist.clearance;
		this->lookahead = dist.lookahead;
		this->ang = dist.ang;
		return *this;
	}
};
#endif
