#include "ros/ros.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_datatypes.h"
#include <unistd.h>
#include <math.h>
#include <sstream>
#include <stdlib.h>
#include <numeric>
#include <map/helper.h>
#include "dwa/dwa.h"
#include <chrono>

#define SIM_ON
#define DEBUG
//#define USER_CMD_BIT 2
#define ODOM_BIT 1
#define MAP_BIT 0

using namespace std;
using namespace std::chrono;



DWA::DWA(const char * topic, ros::NodeHandle &n_t) {
	this->goalPose = Pose(10, 0);
	// Trajectories.
	for (float i = -M_PI; i < M_PI; i += 0.5235987756) { // Split into 12 angles  for each quadrant.
		trajectories.push_back(i);
	}
	dt = 0.2; // seconds.

	//	 TODO: Verify these parameters.
	horizon = 20 / dt; // 10 seconds
	refresh_period = 0; // 1 / dt;
	// WC kinematics
	acc_lim_v = 0.06 * 20; // 0.06 original but tooooo small. Tooooo.
	acc_lim_w = 3;
	decc_lim_v = -0.96;
	decc_lim_w = -3;
	max_trans_vel = MIN_LIN_VEL;
	min_trans_vel = -MIN_LIN_VEL;
	max_rot_vel = MAX_ANG_VEL;
	min_rot_vel = -MIN_ANG_VEL;
	// WC dimensions.
	wc_length = 1.3; //m
	wc_width = .6; //m

	vstep = (max_trans_vel - min_trans_vel) / 5;
	wstep = (max_rot_vel - min_rot_vel) / 5;

	/*
	 * These parameters are used for accessing the right back side of the wheelchair
	 * as the start position to fill or check occupancy.
	 */
	length_offset = .5;
	width_offset = wc_width / 2;

	odom = Speed(0, 0);
	float gridsize = 0.05;
	float mapsize = 4;
	dwa_map = MapContainerPtr(new MapContainer(gridsize, mapsize));

	deOscillator = DeOscillator();
	deOscillator.changeDir(Pose(0,0),goalPose);
	// ROS
	DATA_COMPLETE = 3;
	n = n_t;
	command_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);

	odom_sub = n.subscribe("odom", 1, &DWA::odomCallback, this);

	occupancy_sub = n.subscribe("local_map", 1, &DWA::occupancyCallback, this);

	goalPose_sub = n.subscribe("goalpose", 1, &DWA::goalPoseCallback, this);
	dataflag = 0;

}

//DWA::~DWA() {
//	delete this->dwa_map;
//}

void DWA::odomCallback(const nav_msgs::Odometry& cmd) {
	// Update dataflag.
	dataflag |= (1 << ODOM_BIT);
	// update v
	this->odom.v = cmd.twist.twist.linear.x;
//#ifdef DEBUG
	ROS_INFO("I heard something, v= %f", this->odom.v);
//#endif

	this->odom.w = cmd.twist.twist.angular.z; // scaling factor that maps user's command to real world units.
//#ifdef DEBUG
	ROS_INFO("I heard something, w= %f", this->odom.w);
//#endif

	tf::Quaternion q;
	tf::quaternionMsgToTF(cmd.pose.pose.orientation, q);
	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);
//		wraparoundGlobalPose (globalPose);

	odom_all = nav_msgs::Odometry(cmd);
	odom_all.pose.pose.position.z = yaw;
	deOscillator.updateOdom(odom_all);
}

void DWA::occupancyCallback(const nav_msgs::OccupancyGrid& og) {
	// Update dataflag.
	dataflag |= (1 << MAP_BIT);
	this->dwa_map->updateMap(og);

//	for (int i = 0; i < this->dwa_map->getNoOfGrids(); i++) {
//		for (int j = 0; j < this->dwa_map->getNoOfGrids(); j++) {
//			int val = this->dwa_map->at(i, j);
//			if (val != 0) {
//				cout << "(" << i << "," << j << "), ";
//			}
//		}
//	}
//	cout << endl;
}
void DWA::goalPoseCallback(const geometry_msgs::Pose& p) {
	tf::Quaternion q;
		tf::quaternionMsgToTF(p.orientation, q);
		tf::Matrix3x3 m(q);
		double roll, pitch, yaw;
		m.getRPY(roll, pitch, yaw);
	Pose goal = Pose(p.position.x, p.position.y, yaw);
	updateGoalPose(goal);
}
/*
 * Heading is defined in the first paper on DWA
 * as the bearing of the robot���s direction from the goal,
 * such that heading is maximum when the robot is facing the goal.
 * It is a measure to motivate the robot to progress towards the goal.
 * It is computed at the position the robot will be in after maximum deceleration from the next time step.
 *
 * We normalise this value to the -1,1 range.
 */
float DWA::computeHeading(Speed candidateSpeed, Pose goal) {
	// Compute goal from user command.

	// All calculations is done is local frame.
	// Normalise user's speed first.
	float x, y, th;
	x = y = th = 0;

	// Here we are assuming the robot will use this speed for the next few time steps. // This could be because of latency
	for (int i = 0; i < 10; i++) {

		x += candidateSpeed.v * cos(th) * dt;
		y += candidateSpeed.v * sin(th) * dt;
		th += candidateSpeed.w * dt;
		th = wraparound(th);
	}
	Speed prevSpeed = candidateSpeed;
	Speed nextSpeed = candidateSpeed;
	nextSpeed.v -= copysign(decc_lim_v, nextSpeed.v) * dt;
	nextSpeed.w -= copysign(decc_lim_w, nextSpeed.w) * dt;
	// Now exert maximum deceleration.
	while ((abs(nextSpeed.v) < abs(prevSpeed.v))
			|| (abs(nextSpeed.w) < abs(prevSpeed.w))) {
		if (abs(nextSpeed.v) < abs(prevSpeed.v)) {
			x += nextSpeed.v * cos(th) * dt;
			y += nextSpeed.v * sin(th) * dt;
			prevSpeed.v = nextSpeed.v;
			nextSpeed.v -= copysign(decc_lim_v, nextSpeed.v) * dt;
		}
		if (abs(nextSpeed.w) < abs(prevSpeed.w)) {
			th += nextSpeed.w * dt;
			th = wraparound(th);
			prevSpeed.w = nextSpeed.w;
			nextSpeed.w -= copysign(decc_lim_w, nextSpeed.w) * dt;
		}
		prevSpeed = nextSpeed;

	}
	Pose currentpose = Pose(odom_all.pose.pose.position.x,
			odom_all.pose.pose.position.y, odom_all.pose.pose.position.z);
	Pose endPose = currentpose + Pose(x, y, th);

	float bearingToGoal = endPose.bearingToPose(goal);
	bool front = this->deOscillator.isFront();
	float heading;
	if (front) {
		heading = M_PI - angDiff(bearingToGoal, endPose.th);
	} else {
		heading = - angDiff(bearingToGoal, endPose.th);
	}
		/*
	 * normalise the test speed as well.
	 * Here we are asking what speed would closely match the user's
	 * intention given his joystick angle of deflection.
	 * We assume that the user is aiming for an instantenous speed
	 * rather than that speed over a long distance.
	 * Thus there is no need to predict intention as a position.
	 * Rather it is sufficient to consider prediction over intended
	 * instantaneous direction and magnitude of motion (i.e over velocity space)
	 */

//	float goalth = goal.w*horizon*dt; // gets the starting motion direction from the user's input.
//	Speed test = candidateSpeed;
//	float candididateth =test.w*horizon*dt;
//
//	// Compute heading here.
//	float heading = M_PI - angDiff(candididateth, goalth);
	// Normalise heading to [0,1]
	heading = wraparound(heading);
	heading = fabs(heading) / M_PI;
	return heading;
}

/*
 * Clearance measures the distance to the closest obstacle on the trajectory.
 * TODO: Populate values on a lookup table to prevent re-evaluation.
 * Clearance is now dynamic.
 */

float DWA::computeClearance(Speed candidateSpeed) {
	// clearance is normalised to [0,1] by d
	if (candidateSpeed == Speed(0, 0)) {
		return 1;
	}
	float x = computeDistToNearestObstacle(candidateSpeed);
#ifdef DEBUG
	ROS_INFO("Pre clearance: %f", x);
#endif

	x = (x < SAFEZONE) ? 0 : x / 2.550;
	return x;

}

float DWA::computeDistToNearestObstacle(Speed candidateSpeed) {
	// Compute rectangular dimension depicting wheelchair in  occupancy map.
	// Compute and normalize clearance.
	float x, y, th;
	x = y = th = 0;
	float clearance = 2.55; // 2.55m is maximum detectable distance by sonar
	int count = refresh_period; // we want 2 seconds = 2/dt counts

	for (int i = 0; i < horizon; i++) {
		x += candidateSpeed.v * cos(th) * dt;
		y += candidateSpeed.v * sin(th) * dt;
		th += candidateSpeed.w * dt;
		th = wraparound(th);
		if ((fabs(x / dwa_map->getResolution()) > dwa_map->getNoOfGrids() / 2)
				|| (fabs(y / dwa_map->getResolution())
						> dwa_map->getNoOfGrids() / 2)) {
			cout << "Horizon at edge of localmap!!!" << endl;
			break;
		}
		// the below is expensive to compute so
		// compute only after every 2 seconds into the future.
		if (count-- <= 0) {
			count = refresh_period;

			Pose pose = Pose(x, y, th);
			bool isObstacle = onObstacle(pose);
			if (isObstacle) {
				clearance = vectorNorm(pose);
				return clearance;
			}
		}
	}
	return clearance;
}

/*
 * Checks if there is any obstacle in the occupancy grid to obstruct the
 * wheelchair if its centre where at pose. Returns the location of the obstacles.
 */
bool DWA::onObstacle(Pose pose) {
	// Get corners of wheelchair rectangle.
	RealPoint topLeft = RealPoint(-length_offset, width_offset);
	RealPoint topRight = RealPoint(wc_length - length_offset, width_offset);
	RealPoint bottomRight = RealPoint(wc_length - length_offset, -width_offset);
	RealPoint bottomLeft = RealPoint(-length_offset, -width_offset);

	for (int k = 0; k < 1; k++) {
		float delta = k * dwa_map->getResolution();
		topLeft += RealPoint(-delta, delta);
		topRight += RealPoint(-delta, -delta);
		bottomRight += RealPoint(-delta, delta);
		bottomLeft += RealPoint(delta, delta);

		// Transform corners into pose coordinate.
		rotateFromBody(&topLeft, pose);
		rotateFromBody(&topRight, pose);
		rotateFromBody(&bottomLeft, pose);
		rotateFromBody(&bottomRight, pose);

		IntPoint topLeftInt;
		dwa_map->realToMap(topLeft, topLeftInt);
		IntPoint topRightInt;
		dwa_map->realToMap(topRight, topRightInt);
		IntPoint bottomLeftInt;
		dwa_map->realToMap(bottomLeft, bottomLeftInt);
		IntPoint bottomRightInt;
		dwa_map->realToMap(bottomRight, bottomRightInt);

		// Now get the map equivalent of points.

		// Compute the outline of the rectangle.

		vector<IntPoint> outline;
		bresenham(topLeftInt.x, topLeftInt.y, topRightInt.x, topRightInt.y,
				outline);

		bresenham(topRightInt.x, topRightInt.y, bottomRightInt.x,
				bottomRightInt.y, outline);
		bresenham(bottomRightInt.x, bottomRightInt.y, bottomLeftInt.x,
				bottomLeftInt.y, outline);
		bresenham(bottomLeftInt.x, bottomLeftInt.y, topLeftInt.x, topLeftInt.y,
				outline);

		for (int i = 0; i < outline.size(); i++) {
			IntPoint point = outline[i];
//			cout << "Probbing Grid.. X:"<<point.x<<", Y: "<<point.y<<endl;
			if (point.x < 0 || point.x > dwa_map->getNoOfGrids() || point.y < 0
					|| point.y > dwa_map->getNoOfGrids()) {
				continue;
			}
			if (this->dwa_map->at(point.x, point.y) > 0) {
				cout << "Probbing...Obstacle found!" << endl;
				return true;
			}
		}
	}
	return false;
}

/*
 *  This function normalises s the forward velocity of the robot and supports fast motion.
 *  We want to include angular velocity as well so as to obey the user's commands whilst supporting faster motion.
 *
 */
float DWA::computeVelocity(Speed candidateSpeed) {
	// Normalise velocity to [0, 1] range.
	Speed speed = normaliseSpeed(candidateSpeed);
	return fabs(speed.v);
}
/*
 * Function gets all possible admissible velocities according to the paper.
 * The returned velocities are indexed to align with the trajectory vector variable.
 *
 * This function really establishes an upperbound on velocity since all very slow speeds are theoretically reachable.
 * We optimize this function by only considering trajectories a certain degree of user input.
 */
vector<Speed> DWA::getAdmissibleVelocities(vector<Speed> admissibles,
		float upperbound = M_PI + 1, float lowerbound = -M_PI - 1) {
	admissibles.clear();
	// We search for velocities for by keeping a list of radii of curvature.
	for (int i = 0; i < trajectories.size(); i++) {
		// Construct a speed object from angle and find the clearance.

		if (!isAngleInRegion(trajectories[i], upperbound, lowerbound)) {
			admissibles.emplace_back(0, 0); // This is because the list of admissibles must match with the list of trajectories.
			continue;
		}
		Speed trajectory;
		if ((trajectories[i] < M_PI / 2) && ((trajectories[i] >= -M_PI / 2))) { // +ve v space
			trajectory.v =  max_trans_vel;
		} else {
			trajectory.v = min_trans_vel;
		}

		// need to convert velocities from normalised to real values.
		trajectory.w = trajectory.v * tan(trajectories[i]);

		float dist = computeDistToNearestObstacle(trajectory);
		dist = (dist<SAFEZONE)? 0 : dist;
		float va = copysign(sqrt(fabs(2 * .6 * dist * decc_lim_v)),
				trajectory.v);
		// Here we are simply getting the velocity restriction for each trajectory.
		// In this case, wa is bound to va so a slight deviation from the paper's wa calculation.
		float wa = va * tan(trajectories[i]);

		admissibles.emplace_back(va, wa);
	}
	return admissibles;
}

DynamicWindow DWA::computeDynamicWindow(DynamicWindow dw) {
	dw.upperbound.v = odom.v + acc_lim_v * 5 * dt;
	dw.upperbound.w = odom.w + acc_lim_w * 5 * dt;
	dw.lowerbound.v = odom.v - acc_lim_v * 5 * dt;
	dw.lowerbound.w = odom.w - acc_lim_w * 5 * dt;
	return dw;
}

vector<Speed> DWA::getResultantVelocities(vector<Speed> resultantVelocities,
		float upperbound = M_PI + 1, float lowerbound = -M_PI - 1) {

	DynamicWindow dw;
	dw = computeDynamicWindow(dw);
	vector<Speed> admissibles;
	admissibles.clear();
	admissibles = getAdmissibleVelocities(admissibles, upperbound, lowerbound);

	bool front = deOscillator.isFront();
	// Check direction of goal is in front or behind.
	bool zeroVisited = false; // ensures that we add v=w= 0 only once.
	for (int i = 0; i < trajectories.size(); i++) {
		cout << "ADMISSIBLE traj: [v="<<admissibles[i].v<<",w=" <<admissibles[i].w<<"]"<<endl;
		if (front) {
			if (fabs(trajectories[i]) > M_PI / 2) {
				continue;
			}
		} else {
			if (fabs(trajectories[i]) < M_PI / 2) {
				continue;
			}
		}
		if (!zeroVisited) {
			resultantVelocities.emplace_back(0, 0);
			cout << "Resultant traj: [v="<<0<<",w=" <<0<<endl;
			zeroVisited = true;
		}
		cout << "Trajectory Heading : " << trajectories[i] << endl;
//		if ((angDiff(trajectories[i], upperbound) > 0)
//				|| (angDiff(trajectories[i], lowerbound) < 0)) {
//			continue;
//		}
		if (!isAngleInRegion(trajectories[i], upperbound, lowerbound))
			continue;
		cout << "Passed trajectory Heading : " << trajectories[i] << endl;
		// for very large tan, the data becomes skewed so just use the dw as boundary
		//Here trajectory is either pi/2 or -PI/2
		if (equals(abs(trajectories[i]), M_PI / 2)) {
			float vel = 0;
			float upperbound_w, lowerbound_w;
			if (equals(trajectories[i], M_PI / 2)) {
				upperbound_w = dw.upperbound.w;
				lowerbound_w = (dw.lowerbound.w < 0) ? 0 : dw.lowerbound.w;
			} else {
				upperbound_w = (dw.upperbound.w > 0) ? 0 : dw.upperbound.w;
				lowerbound_w = dw.lowerbound.w;
			}
			upperbound_w = min(upperbound_w, max_rot_vel);
			lowerbound_w = max(lowerbound_w, min_rot_vel);
			float stepw = (upperbound_w - lowerbound_w) / 6;
			if (stepw == 0)
				continue;
//			cout << "upperbound: " << upperbound << "lowerbound" << lowerbound
//					<< "upperbound_w: " << upperbound_w << "lowerbound_w"
//					<< lowerbound_w << endl;
			for (float w = lowerbound_w; w < upperbound_w; w += stepw) {
				if ((w >= lowerbound_w) && ((w <= upperbound_w))) {
//					cout
//							<< "isAngleInRegion(atan2(w, vel), upperbound, lowerbound)"
//							<< isAngleInRegion(atan2(w, vel), upperbound,
//									lowerbound) << endl;
					if (zeroVisited && equals(vel, 0) && equals(w, 0)) {
						continue;
					} else if (equals(vel, 0) && (equals(w, 0))) {
						zeroVisited = true;
					}
					if (isAngleInRegion(atan2(w, vel), upperbound,
							lowerbound)) {
						resultantVelocities.emplace_back(vel, w);
						cout << "Resultant traj: [v="<<vel<<",w=" <<w<<endl;
					}
				}
			}
			continue;
		}

		// For small tan near zero, w = 0

		if (abs(tan(trajectories[i])) < 0.001) {
			float w = 0;
			float lowerbound_v, upperbound_v;
			if ((trajectories[i] < M_PI / 2)
					&& ((trajectories[i] >= -M_PI / 2))) { // +ve v space
				// So compute lowerbound on trajectories.

				upperbound_v = min(dw.upperbound.v, admissibles[i].v);
				lowerbound_v = 0;
			} else {
				upperbound_v = 0;
				lowerbound_v = max(dw.lowerbound.v, admissibles[i].v);
			}
			upperbound_v = min(upperbound_v, max_trans_vel);
			lowerbound_v = max(lowerbound_v, min_trans_vel);
			/*
			 * We are assuming focus within a narrow angle so that 0 and PI can not both be within focus.
			 * Thus if zero is present, pi is not.
			 */
			if (isAngleInRegion(0, upperbound, lowerbound)) { // If 0 is present.
				lowerbound_v = (lowerbound_v < 0) ? 0 : lowerbound_v;
			} else {
				upperbound_v = (upperbound_v > 0.0) ? 0 : upperbound_v;
			}
			float step = (upperbound_v - lowerbound_v) / 3;
			if (step == 0)
				continue;
			for (float vel = lowerbound_v; vel <= upperbound_v; vel += step) {
				if (zeroVisited && equals(vel, 0) && equals(w, 0)) {
					continue;
				} else if (equals(vel, 0) && (equals(w, 0))) {
					zeroVisited = true;
				}
				if (isAngleInRegion(atan2(w, vel), upperbound, lowerbound))
					resultantVelocities.emplace_back(vel, w);
				cout << "Resultant traj: [v="<<vel<<",w=" <<w<<endl;
			}
			continue;
		}

		float lowerbound_v, upperbound_v;
		if ((trajectories[i] < M_PI / 2) && ((trajectories[i] >= -M_PI / 2))) { // +ve v space
			// So compute lowerbound on trajectories.

			upperbound_v = min(dw.upperbound.v, admissibles[i].v);
			lowerbound_v = 0;
		} else {
			upperbound_v = 0;
			lowerbound_v = max(dw.lowerbound.v, admissibles[i].v);
		}
		upperbound_v = min(upperbound_v, max_trans_vel);
		lowerbound_v = max(lowerbound_v, min_trans_vel);

		float step = (upperbound_v - lowerbound_v) / 3;
		if (step == 0)
			continue;
		for (float vel = lowerbound_v; vel <= upperbound_v; vel += step) {

			// For small tan near zero, w = 0
			float w = vel * tan(trajectories[i]);

			float upperbound_w, lowerbound_w;
			if (tan(trajectories[i] < 0)) {
				upperbound_w = dw.upperbound.w;
				lowerbound_w = max(dw.lowerbound.w, admissibles[i].w);
			} else {
				upperbound_w = min(dw.upperbound.w, admissibles[i].w);
				lowerbound_w = dw.lowerbound.w;
			}
			upperbound_w = min(upperbound_w, max_rot_vel);
			lowerbound_w = max(lowerbound_w, min_rot_vel);
			if ((w >= lowerbound_w) && ((w <= upperbound_w))) {
				if (zeroVisited && equals(vel, 0) && equals(w, 0)) {
					continue;
				} else if (equals(vel, 0) && (equals(w, 0))) {
					zeroVisited = true;
				}
				if (isAngleInRegion(atan2(w, vel), upperbound, lowerbound))
					resultantVelocities.emplace_back(vel, w);
				cout << "Resultant traj: [v="<<vel<<",w=" <<w<<endl;
			}

		}

	}

//	ROS_INFO("Printing resultant velocities ...\n");
//	for (int i = 0; i < resultantVelocities.size(); i++) {
//		ROS_INFO("Vel[%d]; [v = %f, w= %f]", i, resultantVelocities[i].v,
//				resultantVelocities[i].w);
//	}
//	ROS_INFO("Printing resultant velocities ended\n");
	return resultantVelocities;
}

//void DWA::restrictVelocitySpace(float &upperbound, float &lowerbound,
//		Speed input) {
//	float upperboundt, lowerboundt;
//	deOscillator.getAdmissibleDirection(upperbound, lowerbound);
//	float ang = atan2(input.w, input.v);
//
//	upperboundt = ang + M_PI * 60 / 180;
//	upperboundt = wraparound(upperboundt);
//	lowerboundt = ang - M_PI * 60 / 180;
//	lowerboundt = wraparound(lowerboundt);
//
//	float upper, lower;
////	upper = upperboundt;
////	lower = lowerboundt;
////
//	if (isAngleInRegion(upperboundt, upperbound, lowerbound)) {
//		upper = upperboundt;
//	} else {
//		upper = upperbound;
//	}
//	if (isAngleInRegion(lowerboundt, upperbound, lowerbound)) {
//		lower = lowerboundt;
//	} else {
//		lower = lowerbound;
//	}
//	upperbound = upper;
//	lowerbound = lower;
//}

/*
 * This is the part that does the probabilistic conditioning based on the user's input.
 */

/*
 * Let G (v,w) = Fn( �� �� heading + ����clearance + ����velocity), where Fn is arbitrary.
 * C = G(v,w) ��� exp(-1/(2*s)*(f-h)(f-h)') ���  W(H), for each of our possible goal location.
 */
Speed DWA::computeNextVelocity(Speed chosenSpeed) {

	vector<Speed> resultantVelocities;
	resultantVelocities.clear();

	/*
	 * Create optimized search space.
	 */


	// Convert to body angle.
//	float dir = -odom_all.pose.pose.position.z;
//	float v = 0.5;
//	Speed goal = Speed(v,v * tan(dir));
	float upperbound = -M_PI - 1;
	float lowerbound = M_PI + 1;


//	restrictVelocitySpace(upperbound, lowerbound, goal);
	deOscillator.getAdmissibleDirection(upperbound, lowerbound);
	ROS_INFO("upperbound: %f, lowerbound: %f", upperbound, lowerbound);
	resultantVelocities = getResultantVelocities(resultantVelocities,
			upperbound, lowerbound);
	float maxCost = 0;
	// Put weightings here.
	float alpha = 0.5;	// For heading.
	float beta = 0.4;	// For clearance.
	float gamma = 1;	// For velocity.
	float final_clearance = 0;
	cout << "Number of resultant velocities" << resultantVelocities.size()
			<< endl;
	for (int i = 0; i < resultantVelocities.size(); i++) {

		Speed realspeed = resultantVelocities[i];
		float heading = computeHeading(realspeed, goalPose);
		float clearance = computeClearance(realspeed);

		float velocity = computeVelocity(realspeed);
		float G = alpha * heading + beta * clearance + gamma * velocity;

		float cost = G;

#ifdef DEBUG

		ROS_INFO("Printing out DWA parameters for specific velocity ...");
		ROS_INFO("RealVel[v = %f, w= %f], heading=%f, clearance=%f, "
				"velocity = %f, cost = %f, Pose (x: %f, y: %f, th: %f)",
				realspeed.v, realspeed.w, heading, clearance, velocity, cost,
				odom_all.pose.pose.position.x, odom_all.pose.pose.position.y,
				odom_all.pose.pose.position.z);
#endif

		if (cost > maxCost) {
			maxCost = cost;
			chosenSpeed = realspeed;
			final_clearance = clearance;
		}
	}
	chosenSpeed = (equals(final_clearance, 0)) ? Speed(0, 0) : chosenSpeed;
	ROS_INFO("Chosen speed: [v=%f, w=%f]", chosenSpeed.v, chosenSpeed.w);
	return chosenSpeed;
}

void DWA::getData() {
	while (dataflag != this->DATA_COMPLETE && ros::ok()) { // Data bits are arranged n order of testing priority.
		ros::spinOnce();

	}
	dataflag = 0;

}

void DWA::updateGoalPose(Pose goal, float dir) {
	if (!(goal == goalPose)) {
		Pose currentPose = Pose(odom_all.pose.pose.position.x,
						odom_all.pose.pose.position.y, odom_all.pose.pose.position.z);
		this->deOscillator.changeDir(currentPose, goal, dir);
		goalPose = goal;
	}
}

void DWA::run() {
	ros::Rate loop_rate(1 / dt);

	Speed chosenSpeed;
	int maxduration;
	int aveduration;
	MyTimer timer = MyTimer();

	while (ros::ok()) {
		timer.start();
		getData();
		chosenSpeed = this->computeNextVelocity(chosenSpeed);
		timer.stop();

		geometry_msgs::Twist motorcmd;
		motorcmd.linear.x = chosenSpeed.v;
#ifdef	SIM_ON
		motorcmd.angular.z = -chosenSpeed.w;
#else
		motorcmd.angular.z = chosenSpeed.w;
#endif
		command_pub.publish(motorcmd);

		loop_rate.sleep();

		ROS_INFO("DWA Max Duration: %d", timer.getMaxDuration());
		ROS_INFO("DWA Average Duration: %d ", timer.getAveDuration());
	}

}
