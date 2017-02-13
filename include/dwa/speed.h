#ifndef SPEED_H
#define SPEED_H
#include <map/helper.h>
#define MAX_LIN_VEL 0.4 //m/s
#define MIN_LIN_VEL 0.4 //m/s

#define MAX_ANG_VEL .7 //rads/s
#define MIN_ANG_VEL .7 //rads/s

using namespace std;
struct Speed {
	float v;
	float w;

	Speed(const Speed& speed) :
			v(speed.v), w(speed.w) {
	}
	Speed(float vt, float wt) :
			v(vt), w(wt) {
	}
	Speed() {
		w = v = 0;
	}
	Speed operator*(float s) // copy/move constructor is called to construct arg
				{
			Speed speed = Speed();
			speed.v = this->v * s;
			speed.w = this->w * s;
			return speed;
		}
	Speed operator+(Speed rhs) // copy/move constructor is called to construct arg
			{
		Speed speed = Speed();
		speed.v = this->v + rhs.v;
		speed.w = this->w + rhs.w;
		return speed;
	}

	Speed operator-(Speed rhs) // copy/move constructor is called to construct arg
				{
			Speed speed = Speed();
			speed.v = this->v - rhs.v;
			speed.w = this->w - rhs.w;
			return speed;
		}
	bool operator==(Speed s) // copy/move constructor is called to construct arg
			{
		if (equals(this->v,s.v) && equals(this->w, s.w)) {
			return true;
		} else {
			return false;
		}
	}
};
#endif
