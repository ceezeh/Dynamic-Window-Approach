/*
 * escapevelocity.h
 *
 *  Created on: 7 Aug 2017
 *      Author: wheelchair
 */

#ifndef DWA_INCLUDE_DWA_ESCAPEVELOCITY_H_
#define DWA_INCLUDE_DWA_ESCAPEVELOCITY_H_

#include "dwa/speed.h"
#include "costmap/point.h"
#include <vector>

using namespace std;

typedef enum Zones_t {
	forward,
	farForward,
	fwdLeft,
	fwdRight,
	bwdLeft,
	bwdRight,
	backward,
	farBackward,
	invalid
} Zone;

class EscapeVelocity {
public:
	EscapeVelocity(WCDimensions wcd_t) :
			wcd(wcd_t) {
		fwdD = .1;
		farFwdD = .25;
		rightD = leftD = .05;
		farRightD = farLeftD = 0.2;
		bwdD = .1;
		farBwdD = .25;
		for (int i = 0; i < 9; i++) {
			invalidZones.push_back(false);
		}
	}

	EscapeVelocity(WCDimensions wcd_t, float forward, float farForward,
			float side, float farside, float backward, float farBackward) :
			wcd(wcd_t) {
		fwdD = forward;
		farFwdD = farForward;
		rightD = leftD = side;
		farRightD = farLeftD = farside;
		bwdD = backward;
		farBwdD = farBackward;
		for (int i = 0; i < 9; i++) {
			invalidZones.push_back(false);
		}
	}
	// Returns true if speed is valid
	// the position of the obstacle.
	void updateZones(RealPoint p) {
		Zone zone = getZone(p);
		invalidZones[zone] = true;
	}
	bool isInSafetyZone(RealPoint p) {
		if ((p.x < wcd.getBottomRightCorner2().x + farFwdD)
				&& (p.x > (wcd.getBottomLeftCorner1().x - farBwdD))
				&& (p.y > (wcd.getBottomRightCorner1().y - farRightD))
				&& (p.y < (wcd.getTopRightCorner1().y + farRightD))) {
			return true;
		} else {
			return false;
		}
	}
	bool isEscapeVelocity(Speed s) {

//		// cout << "Traj-V:" << s.v << ", W:" << s.w << endl;
		if (invalidZones[Zone::forward]) {
			if (!equals(s.w, 0) || (s.v > 0)) {
				// cout << "Zone forward" << endl;
				return false;
			}
		}
		if (invalidZones[Zone::farForward]) {
			if (s.v > 0) {
				// cout << "Zone far forward" << endl;
				return false;
			}
		}
		if (invalidZones[Zone::fwdRight] || invalidZones[Zone::bwdLeft]) {
			if (s.w < 0 && !equals(s.w, 0)) {
				if (invalidZones[Zone::fwdRight]) {
					// cout << "Zone fwd right" << endl;
				}
				if (invalidZones[Zone::bwdLeft]) {
					// cout << "Zone bwd left" << endl;
				}
				return false;
			}
		}
		if (invalidZones[Zone::fwdLeft] || invalidZones[Zone::bwdRight]) {
			if (s.w > 0 && !equals(s.w, 0)) {
				if (invalidZones[Zone::fwdLeft]) {
					// cout << "Zone fwd left" << endl;
				}
				if (invalidZones[Zone::bwdRight]) {
					// cout << "Zone bwd right" << endl;
				}

				return false;
			}

		}
		if (invalidZones[Zone::backward]) {
			if (!equals(s.w, 0) || (s.v < 0)) {
				// cout << "Zone bwd" << endl;
				return false;
			}
		}
		// cout << "no zone" << endl;
		return true;

	}

private:

// Takes point in the body frame.
	Zone getZone(RealPoint p) {	//TODO: Verify this.
		if (p.x<(wcd.getTopRightCorner2().x + fwdD) && p.x>(
				wcd.getTopLeftCorner2().x - rightD)
				&& (p.y > (wcd.getBottomRightCorner1().y - rightD))
				&& p.y < (wcd.getTopRightCorner1().y + rightD)) {
			return Zone::forward;
		}
		if (p.x<(wcd.getTopRightCorner2().x + farFwdD) && p.x>(
				wcd.getTopLeftCorner2().x + fwdD)
				&& (p.y > (wcd.getBottomRightCorner1().y - rightD))
				&& p.y < (wcd.getTopRightCorner1().y + rightD)) {
			return Zone::farForward;
		}
		if ((p.x<(wcd.getBottomRightCorner2().x + farFwdD) && p.x>(
				wcd.getBottomLeftCorner2().x - rightD)
				&& (p.y < (wcd.getBottomRightCorner1().y - rightD))
				&& p.y > (wcd.getBottomRightCorner1().y - farRightD))
				|| (p.x < (wcd.getTopRightCorner1().x - rightD) && p.x > 0
						&& (p.y < (wcd.getBottomRightCorner1().y + rightD))
						&& p.y > (wcd.getBottomRightCorner1().y - farRightD))) {
			return Zone::fwdRight;
		}
		if ((p.x<(wcd.getTopRightCorner2().x + farFwdD) && p.x>(
				wcd.getTopRightCorner2().x - leftD)
				&& (p.y > (wcd.getTopRightCorner1().y - leftD))
				&& p.y < (wcd.getTopRightCorner1().y + farLeftD))
				|| (p.x < (wcd.getTopRightCorner1().x - leftD) && p.x > 0
						&& (p.y > (wcd.getTopRightCorner1().y - leftD))
						&& p.y < (wcd.getTopRightCorner1().y + farLeftD))) {
			return Zone::fwdLeft;
		}
		if (p.x > (wcd.getTopLeftCorner1().x - bwdD)
				&& p.x < (wcd.getTopLeftCorner1().x + leftD)
				&& (p.y > (wcd.getBottomLeftCorner1().y - rightD))
				&& p.y < (wcd.getTopLeftCorner1().y + rightD)) {
			return Zone::backward;
		}
		if (p.x > (wcd.getTopLeftCorner1().x - farBwdD)
				&& p.x < (wcd.getTopLeftCorner1().x - bwdD)
				&& (p.y > (wcd.getBottomRightCorner1().y - rightD))
				&& p.y < (wcd.getTopRightCorner1().y + rightD)) {
			return Zone::farBackward;
		}
		if ((p.x > (wcd.getBottomRightCorner1().x - farBwdD)
				&& p.x<
						(wcd.getBottomLeftCorner1().x + rightD)
								&& (p.y
										< (wcd.getBottomLeftCorner1().y - rightD))
								&& p.y>(
						wcd.getBottomLeftCorner1().y - farRightD))
				|| (p.x > (wcd.getBottomLeftCorner1().x + rightD)
						&& p.x<
								0
										&& (p.y
												< (wcd.getBottomRightCorner1().y
														+ rightD)) && p.y>(
								wcd.getBottomRightCorner1().y - farRightD))) {
			return Zone::bwdRight;
		}
		if ((p.x > (wcd.getTopLeftCorner1().x - farBwdD)
				&& p.x < (wcd.getTopLeftCorner1().x + leftD)
				&& (p.y > (wcd.getTopLeftCorner1().y + leftD))
				&& p.y < (wcd.getTopLeftCorner1().y + farLeftD))
				|| (p.x > (wcd.getTopLeftCorner1().x + leftD) && p.x < 0
						&& (p.y > (wcd.getTopLeftCorner1().y - leftD))
						&& p.y < (wcd.getTopLeftCorner1().y + farLeftD))) {
			return Zone::bwdLeft;
		}
		return invalid;
	}

private:
	float fwdD, farFwdD, rightD, farRightD, leftD, farLeftD, bwdD, farBwdD;
	WCDimensions wcd;
	vector<bool> invalidZones;
}
;

#endif /* COSTMAP_INCLUDE_COSTMAP_ESCAPEVELOCITY_H_ */
