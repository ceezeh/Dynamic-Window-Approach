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
#define INVALID_DIR -12
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
	bool front; // front = true;
	Pose start_pose; // The pose at the start of deoscillation.
	float velDir; // instantanouse direction of motion in the body frame.
	static float lin_dist_thres, ang_dist_thres_lower, ang_dist_thres_upper;
	bool first;
public:
	DeOscillator() {
		upperbound = M_PI + .01;
		lowerbound = -M_PI - .01;
		start_pose = Pose();
		velDir = 0;
		front = true;
		first = true;
	}
	// This function examines if we have travelled far enough to ensure deoscillation.
	void updateOdom(const nav_msgs::Odometry& cmd) {
		float xt = cmd.pose.pose.position.x;
		float yt = cmd.pose.pose.position.y;
		float tht = cmd.pose.pose.position.z;

		if (first) {
			start_pose = Pose(xt, yt, tht);
			first = false;
			return;
		}
		float lin_dist = sqrt(
				pow(start_pose.x - xt, 2) + pow(start_pose.y - yt, 2));
		float ang_dist = abs(angDiff(start_pose.th, tht));
		if ((lin_dist > lin_dist_thres) || ((ang_dist > ang_dist_thres_lower)
//					&& (ang_dist < ang_dist_thres_upper)
				)) {
			start_pose = Pose(xt, yt, tht);
			if (front) {
				velDir = atan2(cmd.twist.twist.angular.z,
						cmd.twist.twist.linear.x);
			} else {
				velDir = atan2(-cmd.twist.twist.angular.z,
						cmd.twist.twist.linear.x);
			}
			cout << "NEW DIRECTION" << endl;
		}

	}

	void getAdmissibleDirection(float& upperbound, float& lowerbound) {
		cout << "velDir" << velDir << endl;
		upperbound = velDir + M_PI * 60 / 180;
		upperbound = wraparound(upperbound);
		lowerbound = velDir - M_PI * 60 / 180;
		lowerbound = wraparound(lowerbound);
	}

	// This is called only once anytime the goal pose changes.
	void changeDir(Pose currPose, Pose goalPose, float dir= INVALID_DIR) {
		float bearing = currPose.bearingToPose(goalPose);
		float trueBearing = angDiff(currPose.th, bearing);
		bool front_t = true;
		if (fabs(trueBearing) > M_PI / 2) {
			front_t = false;
		}

		cout << "Computing dir.... bearingToGoal: " << bearing
				<< ", true bearing: " << trueBearing << ". current th: "<<currPose.th<<endl;
		if (dir != INVALID_DIR) {
			velDir = dir;
		}
		if (front ^ front_t) {
			// Useful when DWA is used and complex update of velDir is not required.
			// This is because DWA only takes one goal and does not change its goal.
			if (dir == INVALID_DIR) {
				if (front_t) {
					velDir = 0;
				} else {
					velDir = M_PI;
				}
			}
			front = front_t;
			cout << "CHANGING DIRECTION, isFront= " << front << ". Veldir: "
					<< velDir << " !!!" << endl;
		} else {
			cout << "New goal but direction Remains the same. isFront= "
					<< front << ". Veldir: " << velDir << endl;
		}

	}

	bool isFront() const {
		return front;
	}
}
;
#endif
