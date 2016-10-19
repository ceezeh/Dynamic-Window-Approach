#ifndef SHAREDDWA_HELPER_H
#define SHAREDDWA_HELPER_H
#define  USE_MATH_DEFINES
//#define 	DEBUG
#include <unistd.h>
#include <stdlib.h>
#include <vector>
#include <math.h>
#include <numeric>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/TwistStamped.h"
#include "nav_msgs/OccupancyGrid.h"
#include <map/point.h>
#include <map/helper.h>
#include "nav_msgs/Odometry.h"
#include "dwa/pose.h"
#include "dwa/speed.h"

#define NULL_POSE Pose(0,0,0)

using namespace std;

float vectorNorm(Pose p);
int getQuadrant(float upper);
float vectorNorm(Pose p);
float vectorNorm(Speed p);
float magSquared(Speed p);
float sqrt_approx(float z);
Speed getRealSpeed(Speed speed_old);
Speed normaliseSpeed(Speed speed_old);
template<typename Ty>
void rotateFromBody(Ty &pose, Pose T);
int getQuadrant(float upper);
bool isAngleInRegion(float ang, float upper, float lower);

// Assumes pose is to be rotated to a coordinate system with T as origin coordinate.

void rotateFromBody(RealPoint *pose, Pose T);
void rotateFromBody(IntPoint *pose, Pose T);
//***********************************************************************//
/*
 * Needed explicit parameters:
 * Wheelchair dimensions, centre of wheelchair motion,
 * occupancy gridsize,
 * wheelchair kinematic parameters such as acc, max speed,
 *
 */
typedef enum {
	Joystick, Button
} InterfaceType; // We use this to inform on the probability distribution for the user's intention from input interface.
//***********************************************************************//

//***********************************************************************//

//***********************************************************************//
struct DynamicWindow {
	Speed upperbound;
	Speed lowerbound;
};
//***********************************************************************//
class DeOscillator {
private:
	float upperbound, lowerbound;
	Pose start_pose; // The pose at the start of deoscillation.
	static float lin_dist_thres, ang_dist_thres_lower, ang_dist_thres_upper;
public:
	DeOscillator() {
		upperbound = M_PI + .01;
		lowerbound = -M_PI - .01;
		start_pose = Pose();
	}
	// This function examines if we have travelled far enough to ensure deoscillation.
	void updateOdom(const nav_msgs::Odometry& cmd) {
		float xt = cmd.pose.pose.position.x;
		float yt = cmd.pose.pose.position.y;
		float tht = atan2(cmd.twist.twist.angular.z, cmd.twist.twist.linear.x);

		float lin_dist = sqrt(
				pow(start_pose.x - xt, 2) + pow(start_pose.y - yt, 2));
		float ang_dist = abs(angDiff(start_pose.th, tht));
		if ((lin_dist > lin_dist_thres) || ((ang_dist > ang_dist_thres_lower)
//					&& (ang_dist < ang_dist_thres_upper)
				)) {
			start_pose = Pose(xt, yt, tht);
			cout << "NEW DIRECTION" << endl;
		}

	}

	void getAdmissibleDirection(float& upperbound, float& lowerbound) {
		upperbound = start_pose.th+ M_PI * 60 / 180;
		upperbound = wraparound(upperbound);
		lowerbound = start_pose.th - M_PI * 60 / 180;
		lowerbound = wraparound(lowerbound);
	}
};
#endif
