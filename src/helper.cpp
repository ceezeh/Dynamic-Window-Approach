#include "dwa/helper.h"
using namespace std;
float DeOscillator::lin_dist_thres = .1;
float DeOscillator::ang_dist_thres_lower = M_PI * 10 / 180;
float DeOscillator::ang_dist_thres_upper = M_PI * 110 / 180;
// ---------------Trigonometry----------------------
//float angDiff(float a1, float a2) {
//	float a = a1 - a2;
//	a = fmod((a + M_PI), (2 * M_PI)) - M_PI;
//	return wraparound(a);
//}

float vectorNorm(Speed p) {
	return sqrt(magSquared(p));
}
float magSquared(Speed p) {
	std::vector<float> v { p.v, p.w };
	float res = inner_product(v.begin(), v.end(), v.begin(), 0.0f);
	return res;
}
float sqrt_approx(float z) {
	int val_int = *(int*) &z; /* Same bits, but as an int */
	/*
	 * To justify the following code, prove that
	 *
	 * ((((val_int / 2^m) - b) / 2) + b) * 2^m = ((val_int - 2^m) / 2) + ((b + 1) / 2) * 2^m)
	 *
	 * where
	 *
	 * b = exponent bias
	 * m = number of mantissa bits
	 *
	 * .
	 */

	val_int -= 1 << 23; /* Subtract 2^m. */
	val_int >>= 1; /* Divide by 2. */
	val_int += 1 << 29; /* Add ((b + 1) / 2) * 2^m. */

	return *(float*) &val_int; /* Interpret again as float */
}
void rotateFromBody(Pose T, RealPoint *pose) {
	float x = pose->x;
	float y = pose->y;

	float x2 = cos(T.th) * x - sin(T.th) * y;
	float y2 = sin(T.th) * x + cos(T.th) * y;

	pose->x = x2 + T.x;
	pose->y = y2 + T.y;
}

void rotateFromBody(Pose T, IntPoint *pose) {
	float x = pose->x;
	float y = pose->y;

	float x2 = cos(T.th) * x - sin(T.th) * y;
	float y2 = sin(T.th) * x + cos(T.th) * y;

	pose->x = int(x2 + T.x + 0.5);
	pose->y = int(y2 + T.y + 0.5);
}
void toBodyFrame(Pose T, RealPoint &pose) {
	float x = pose.x - T.x;
	float y = pose.y - T.y;

	pose.x = cos(-T.th) * x - sin(-T.th) * y;
	pose.y = sin(-T.th) * x + cos(-T.th) * y;

}
// inverse of normalise speed.
Speed getRealSpeed(Speed speed_old) {
	Speed speed = Speed(speed_old);
	if (speed.v >= 0) {
		speed.v *= (MAX_LIN_VEL);
	} else {
		speed.v *= (MIN_LIN_VEL);
	}
	if (speed.w >= 0) {
		speed.w *= (MAX_ANG_VEL);
	} else {
		speed.w *= (MIN_ANG_VEL);
	}
	return speed;
}

Speed normaliseSpeed(Speed speed_old) {
	Speed speed = Speed(speed_old);
	if (speed.v >= 0) {
		speed.v /= (MAX_LIN_VEL);
	} else {
		speed.v /= (MIN_LIN_VEL);
	}
	if (speed.w >= 0) {
		speed.w /= (MAX_ANG_VEL);
	} else {
		speed.w /= (MIN_ANG_VEL);
	}
	return speed;
}

////Assumes lower to upper forms a continuous region.
//float wraparound (float ang) { // [-pi, pi]
//    if (equals(ang, 0) || equals(fabs(ang), M_PI)) return ang;
//    if ((ang <= M_PI)&& (ang>=-M_PI)) return ang;
//    if (ang > M_PI) {
//        ang -= 2*M_PI;
//    }
//    if (ang < -M_PI ) {
//        ang +=  2*M_PI;
//    }
//    return wraparound(ang);
//}
bool isAngleInRegion(float ang, float upper, float lower) {
	upper = wraparound(upper);
	lower = wraparound(lower);
	ang = wraparound(ang);
	bool wrapped = false;
	if (upper < lower) { // wraparound has occurred.
		upper += 2 * M_PI;
		wrapped = true;
	}
	if ((ang < 0) && wrapped)
		ang += 2 * M_PI;
	if ((upper >= ang) && (lower <= ang)) {
		return true;
	} else {
		return false;
	}
}
