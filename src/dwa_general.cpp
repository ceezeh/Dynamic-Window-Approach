/*
 * dwa_general.cpp
 *
 *  Created on: 14 Aug 2017
 *      Author: wheelchair
 */

#include "dwa/dwa_general.h"
#include "dwa/escapevelocity.h"
#include <cppopt/newton_raphson.h>
#include <cppopt/gauss_newton.h>
#include <cppopt/gradient_descent.h>
#define AGENT3_BIT 6
#define AGENT2_BIT 5
#define AGENT1_BIT 4
#define AGENT0_BIT 3
#define POSE_BIT 2

using namespace cv;

DWAGen::DWAGen(const char * topic, ros::NodeHandle &n_t) :
		DWA(topic, n_t) {
	distFromObstacle.push_back(Distance(NULLDIST, NULLDIST, NULLDIST));

	dynamic_obstacles.push_back(nav_msgs::Odometry());
	dynamic_obstacles.push_back(nav_msgs::Odometry());
	dynamic_obstacles.push_back(nav_msgs::Odometry());
	dynamic_obstacles.push_back(nav_msgs::Odometry());
	this->agentSub0 = n.subscribe("odom0", 4, &DWAGen::getDynamicObstacle,
			this);
	this->agentSub1 = n.subscribe("odom1", 4, &DWAGen::getDynamicObstacle,
			this);
	this->agentSub2 = n.subscribe("odom2", 4, &DWAGen::getDynamicObstacle,
			this);
	this->agentSub3 = n.subscribe("odom3", 4, &DWAGen::getDynamicObstacle,
			this);
	this->odomSub = n.subscribe("odom", 4, &DWAGen::odomCallback, this);
	this->DATA_COMPLETE = 31;

	this->max_trans_vel = 1;
	this->min_trans_vel = -1;

	this->max_rot_vel = 1.5;
	this->min_rot_vel = -1.5;
	this->horizon = 20; // 10 seconds
}

void DWAGen::odomCallback(const nav_msgs::Odometry& odom) {
	dataflag |= (1 << POSE_BIT);
	this->currPose = Pose(odom.pose.pose.position.x, odom.pose.pose.position.y,
			getYaw(odom.pose.pose.orientation));
}

void DWAGen::getDynamicObstacle(const nav_msgs::Odometry odom) {
	if (odom.header.frame_id == "agent0") {
		dataflag |= (1 << AGENT0_BIT);
		dynamic_obstacles[0] = nav_msgs::Odometry(odom);
	} else if (odom.header.frame_id == "agent1") {
		dataflag |= (1 << AGENT1_BIT);
		dynamic_obstacles[1] = nav_msgs::Odometry(odom);
	} else if (odom.header.frame_id == "agent2") {
		dataflag |= (1 << AGENT2_BIT);
		dynamic_obstacles[2] = nav_msgs::Odometry(odom);
	} else if (odom.header.frame_id == "agent3") {
		dataflag |= (1 << AGENT3_BIT);
		dynamic_obstacles[3] = nav_msgs::Odometry(odom);
	}
}

Distance DWAGen::computeDistToNearestStaticObstacle(Speed candidateSpeed) {
	// Compute rectangular dimension depicting wheelchair in  occupancy map.
	// Compute and normalize clearance.

	//	if (equals(candidateSpeed.w, 0)) {
	//		for (int i = 0; i < this->dwa_map->getMap().data.size(); i++) {
	//			if (this->dwa_map->getMap().data[i] > lo_occ_thres) {
	//				int x, y;
	//				this->dwa_map->getOccXY(i, x, y);
	//				cout << "Searching....obstacle found at x=" << x << ",y=" << y
	//						<< endl;
	//			}
	//		}
	//	}
	bool foundObst = false;
	float traj = atan2(candidateSpeed.w, candidateSpeed.v);
	float x, y, th;
	x = y = th = 0;
	float v = copysign(0.1, candidateSpeed.v);
	float w = v * tan(traj);
	if (equals(fabs(traj), M_PI / 2)) {
		w = copysign(0.2, traj);
		v = 0;
	}

	float clearance = 2.55; // 2.55m is maximum detectable distance by sonar
	int count = refresh_period; // we want 2 seconds = 2/dt counts
	float acc = 0;
	for (int i = 0; i < 1; i++) {
		float dx = odom.v * cos(th) * dt;
		float dy = odom.v * sin(th) * dt;
		float ds = vectorNorm(Pose(dx, dy));
		//		acc += ds;
		x += dx;
		y += dy;
		th += odom.w * dt;
		th = wraparound(th);
	}

	for (int i = 0; i < horizon; i++) {
		float dx, dy, ds, dth;
		if (i > 0) {
			dx = v * cos(th) * dt;
			dy = v * sin(th) * dt;
			ds = vectorNorm(Pose(dx, dy));
			acc += ds;
			dth = w * dt;
		} else {
			dth = candidateSpeed.w * dt;
			dx = candidateSpeed.v * cos(th + dth) * dt;
			dy = candidateSpeed.v * sin(th + dth) * dt;
			ds = vectorNorm(Pose(dx, dy));
		}
		x += dx;
		y += dy;
		th += dth;
		th = wraparound(th);
		//		cout<<"[Compute Clearance] x="<<x<<",y="<<y<<",th="<<th<<", i="<<i<<endl;
		//		timestep = i;
		if ((fabs(x / dwa_map->getResolution()) > dwa_map->getNoOfGrids() / 2)
				|| (fabs(y / dwa_map->getResolution())
						> dwa_map->getNoOfGrids() / 2)) {
			// cout << "Horizon at edge of localmap!!!" << endl;
			break;
		}
		// the below is expensive to compute so
		// compute only after every 2 seconds into the future.

		Pose pose = Pose(x, y, th);
		if (i > -1) { //For laser sensor.
			bool isObstacle = onObstacle(pose, candidateSpeed,
					(i == 0) ? true : false);

			if (isObstacle) {
				foundObst = true;
				break;
			}
		}

	}
	if (foundObst) {
		float dx = candidateSpeed.v * cos(th) * dt;
		float dy = candidateSpeed.v * sin(th) * dt;
		float ds = vectorNorm(Pose(dx, dy));
		th = fabs(th);
		if (equals(traj, 0) || equals(traj, M_PI))
			return Distance(acc, acc + ds, 2 * M_PI);
		if (equals(fabs(traj), M_PI / 2)) {
			if (equals(candidateSpeed.w, 0))
				// cout << "@@Using ang dist..." << endl;
			return Distance(th * 0.55, th * 0.55 + ds, th); // Using Centre to back metric...
		} else
			return Distance(acc, acc + ds, th);
	} else {
		return Distance(clearance, clearance, 2 * M_PI);
	}
}

Distance DWAGen::computeDistToNearestObstacle(Speed candidateSpeed) {
	Distance d1, d2;
#pragma omp simp sections
	{
		{
			d1 = computeDistToNearestDynamicObstacle(candidateSpeed);
		}
#pragma opm section
		{
			d2 = computeDistToNearestStaticObstacle(candidateSpeed);
		}
	}
	if (d1.clearance < d2.clearance) {
		return d1;
	} else {
		return d2;
	}

}
Distance DWAGen::computeDistToNearestDynamicObstacle(Speed candidateSpeed) {
	float dmin = 4;
	float obstRadiusSq = 1.5; //0.7 *root(2)
	EscapeVelocity escapeVelocity = EscapeVelocity(*(this->wcDimensions.get()),
			.2, .4, .2, .2, .2, .4);
#pragma omp simd parallel for
	for (int i = 0; i < this->dynamic_obstacles.size(); i++) {
		// Convert obstacle position to the wheelchair's body frame.
		nav_msgs::Odometry om = this->dynamic_obstacles[i];
		RealPoint obst = RealPoint(om.pose.pose.position.x,
				om.pose.pose.position.y);

		toBodyFrame(this->currPose, obst);

		// If obstacle is very far away,

		// Need to find relative speed.

		// Rotate from obstacle body to global and then from global to wheelchair body frame.

		float dtheta = angDiff(getYaw(om.pose.pose.orientation),
				this->currPose.th);
//		cout << "om.pose.pose.orientation.w" << om.pose.pose.orientation.w
//				<< "om.pose.pose.orientation.x" << om.pose.pose.orientation.x
//				<< "om.pose.pose.orientation.y" << om.pose.pose.orientation.y
//				<< "om.pose.pose.orientation.z" << om.pose.pose.orientation.z
//				<< endl;
		Pose T = Pose(0, 0, dtheta);
		RealPoint speed = RealPoint(om.twist.twist.linear.x, 0);
		rotateFromBody(T, &speed);

		//Relative Linear Speed;
		RealPoint linSpeed = RealPoint(candidateSpeed.v - speed.x, -speed.y);

		// Obtain initial theta in the body frame.
		float th = atan2(linSpeed.y, linSpeed.x);
		float x, y;
		x = y = 0;
		// TODO, need to decide if wheelchair is going backwards or forwards with relative speed.
		float v = vectorNorm(linSpeed);
		float w = candidateSpeed.w - om.twist.twist.angular.z; // Relative angular speed

//		cout << "Candidate Speed [v.x=" << candidateSpeed.v << ",v.y=0, w="
//				<< candidateSpeed.w << "], Agent Speed [v.x=" << speed.x
//				<< ",v.y=" << speed.y << ", w=" << om.twist.twist.angular.z
//				<< "], "
//						"Relative Speed [v.x=" << linSpeed.x << ",v.y="
//				<< linSpeed.y << ", w=" << w << "]" << endl;
		// Assemble all vertices in a matrix.
		Mat P =
				(Mat_<float>(8, 3) << this->wcDimensions->getTopLeftCorner1().x, this->wcDimensions->getTopLeftCorner1().y, 1, this->wcDimensions->getTopRightCorner1().x, this->wcDimensions->getTopRightCorner1().y, 1, this->wcDimensions->getTopLeftCorner2().x, this->wcDimensions->getTopLeftCorner2().y, 1, this->wcDimensions->getTopRightCorner2().x, this->wcDimensions->getTopRightCorner2().y, 1, this->wcDimensions->getBottomRightCorner2().x, this->wcDimensions->getBottomRightCorner2().y, 1, this->wcDimensions->getBottomLeftCorner2().x, this->wcDimensions->getBottomLeftCorner2().y, 1, this->wcDimensions->getBottomRightCorner1().x, this->wcDimensions->getBottomRightCorner1().y, 1, this->wcDimensions->getBottomLeftCorner1().x, this->wcDimensions->getBottomLeftCorner1().y, 1);
		P = P.t();
		float dist = 0;
		//Todo:Use angular distance as well.
		float t = 0;

		float ratio = fabs(w / v);

		v = copysign(0.2, v);
		w = copysign(v * ratio, w);
		for (int i = 0; i < this->horizon; i++) {
			float ds = this->dt * v;
			dist += ds;
			x += ds * cos(th);
			y += ds * sin(th);
			th += dt * w;
			t += dt;

			// Create Transformation matrix.
			Mat T = (Mat_<float>(2, 3) << cos(th), -sin(th), x, sin(th), cos(
					th), y);
			// Format the coordinate matrix
			Mat X = T * P;
			X = X.t();

			// Centre of Obstacle
			Mat O =
					(Mat_<float>(8, 2) << obst.x, obst.y, obst.x, obst.y, obst.x, obst.y, obst.x, obst.y, obst.x, obst.y, obst.x, obst.y, obst.x, obst.y, obst.x, obst.y);

			Mat A = X.colRange(0, 2) - O;
			A = A.mul(A);
			Mat d;
			reduce(A, d, 1, CV_REDUCE_SUM);
			bool isBreak = false;
			for (int i = 0; i < 8; i++) {
				if (d.at<float>(i, 0) <= obstRadiusSq) {
					for (float delta = 0; delta < M_PI * .16; delta += 0.05) { // Todo: This is a hack!! Warning.
						Pose obstPose = Pose(obst.x, obst.y, 0);
						float startAng = obstPose.bearingToPose(Pose());
						float r = d.at<float>(i, 0);

						RealPoint ob1 = RealPoint(
								obst.x + r * cos(startAng + delta),
								obst.y + r * sin(startAng + delta));

						RealPoint ob2 = RealPoint(
								obst.x + r * cos(startAng - delta),
								obst.y + r * sin(startAng - delta));

						if (!escapeVelocity.isInSafetyZone(ob1)
								|| !escapeVelocity.isInSafetyZone(ob1)) {

							if (dist < dmin) {
								dmin = dist;
							}
							isBreak = true;
							 cout << " Seen terminal obstacle" << endl;
							break;
						} else {
							 cout << " Seen obstacle but it is not terminal"
									<< endl;
						}

						// Populate the escape velocity class with obstacles

						cout << "Obstacle at [x=" << ob1.x << ", y=" << ob1.y
								<< "]" << endl;
						cout << "Obstacle at [x=" << ob2.x << ", y=" << ob2.y
								<< "]" << endl;
						escapeVelocity.updateZones(ob1);
						escapeVelocity.updateZones(ob2);
					}
				}
			}
			if (isBreak) {
				isBreak = false;
				break;
			}

		}
	}
	if (escapeVelocity.isEscapeVelocity(candidateSpeed)) {
		return Distance(dmin, dmin, 2 * M_PI);
	} else {
		return Distance(0, 0, 0);
	}

}
//Distance DWAGen::computeDistToNearestDynamicObstacle(Speed candidateSpeed) {
//	float dmin = 2.55;
//	float ang = 2 * M_PI;
//	for (int i = 0; i < this->dynamic_obstacles.size(); i++) {
//
//		// Convert obstacle position to the wheelchair's body frame.
//		nav_msgs::Odometry odom = this->dynamic_obstacles[i];
//		RealPoint obst = RealPoint(odom.pose.pose.position.x,
//				odom.pose.pose.position.y);
//
//		toBodyFrame(this->currPose, obst);
//		// If obstacle is very far away, ignore it.
//		if (vectorNorm(obst) > 3)
//			break;
//
//		// Get orientation of obstacle.
//		float heading = getYaw(odom.pose.pose.orientation);
//		heading = angDiff(heading, this->currPose.th); // Heading in body frame.
//
//		const float Cc = odom.twist.twist.linear.x * cos(heading);
//		const float Cs = odom.twist.twist.angular.z * sin(heading);
//
//		const float vr = candidateSpeed.v;
//		const float wr = candidateSpeed.w;
//
//		const float xob = obst.x;
//		const float yob = obst.y;
//
//		const float r = abs(vr / wr);
//		float sigt = 1;
//		if (equals(copysign(1.0, vr), copysign(1.0, wr))) {
//			sigt = -1;
//		}
//		const float sig = sigt;
//		cppopt::F df =
//				[&](const cppopt::Matrix &x) -> cppopt::Matrix {
//					cppopt::Matrix d(1, 1);
//
//					d(0) =
//					(Cs - r*wr*cos(sig*M_PI/2 + x(0)*wr))*((yob) - (r)*(sin(sig*M_PI/2 + (x(0))*(wr)) -sig* 1)
//							+ (Cs)*(x(0))) + ((Cc) + sin(sig*M_PI/2 + (x(0))*(wr))*(r)*(wr))*(xob
//							+ Cc*x(0) - r*cos(sig*M_PI/2 + x(0)*wr)) + ((Cs) - cos(sig*M_PI/2 + (x(0))*(wr))*(r)*(wr))*(yob
//							+ Cs*x(0) - r*(sin(sig*M_PI/2 + x(0)*wr) -sig* 1)) + (Cc + r*wr*sin(sig*M_PI/2 + x(0)*wr))*((xob)
//							- cos(sig*M_PI/2 + (x(0))*(wr))*(r) + (Cc)*(x(0)));
//
//					return d;
//				};
////		cppopt::F ddf =
////				[&](const cppopt::Matrix &x) -> cppopt::Matrix {
////					cppopt::Matrix d(1, 1);
////
////					d(0) = 2*((Cs) - cos(sig*M_PI/2 + (x(0))*(wr))*(r)*(wr))*(Cs - r*wr*cos(sig*M_PI/2 + x(0)*wr))
////					+ 2*((Cc) + sin(sig*M_PI/2 + (x(0))*(wr))*(r)*(wr))*(Cc + r*wr*sin(sig*M_PI/2 + x(0)*wr))
////					+ r*pow(wr,2)*cos(sig*M_PI/2 + x(0)*wr)*((xob) - cos(sig*M_PI/2 + (x(0))*(wr))*(r) + (Cc)*(x(0)))
////					+ cos(sig*M_PI/2 + (x(0))*(wr))*(r)*pow(wr,2)*(xob + Cc*x(0) - r*cos(sig*M_PI/2 + x(0)*wr))
////					+ r*pow(wr,2)*sin(sig*M_PI/2 + x(0)*wr)*((yob) - (r)*(sin(sig*M_PI/2 + (x(0))*(wr)) -sig)
////							+ (Cs)*(x(0))) + sin(sig*M_PI/2 + (x(0))*(wr))*(r)*pow(wr,2)*(yob + Cs*x(0) -
////							r*(sin(sig*M_PI/2 + x(0)*wr) -sig));
////					return d;
////				};
//		cppopt::Matrix x(1, 1);
//		x(0) = 0.0;
//
//		// Iterate while norm of the first order derivative is greater than some predefined threshold.
//		cppopt::ResultInfo ri = cppopt::SUCCESS;
//		while (ri == cppopt::SUCCESS && df(x).norm() > 0.01f) {
//			ri = cppopt::gradientDescent(df, x, 0.1);
////			std::cout << std::fixed << std::setw(3) << "Parameters: "
////					<< x.transpose() << " Error: " << df(x).norm() << std::endl;
//		}
//
////		std::cout << "Found a " << (ddf(x)(0) < 0.f ? "Maximum" : "Minimum")
////				<< std::endl;
//
//		float d;
//		if (x(0) > 0) {
//
//			float t = x(0);
//			d = pow(
//					pow(yob - r * (sin(t * wr + sig * M_PI / 2) - sig) + Cs * t,
//							2)
//							+ pow(
//									xob + Cc * t
//											- r * cos(t * wr + sig * M_PI / 2),
//									2), 0.5);
//
//			// How do we ensure or guarantee non collision?
//			if (t < 6 && d < 1.5) {
//				d = 0;
//				ang = 0;
//			}
//		} else {
//			d = 2.55;
//		}
//
//		if (d < dmin) {
//			dmin = d;
//
//		}
//	}
//	cout << "Dmin:" << dmin << endl;
//	return Distance(dmin, dmin, ang);
//
//}
