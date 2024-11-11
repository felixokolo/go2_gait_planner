/*
 * Kinematics.cpp
 *
 *  Created on: 6 Jun 2024
 *      Author: Felix
 */

#include "Quadruped/Kinematics.h"
#include "Quadruped/Leg.h"


Kinematics::Kinematics(Leg *leg): leg(leg)
{
}

Eigen::Vector3d Kinematics::getCurPosition()
{
	return getPosition(leg->getTransformation());
}

Eigen::Vector3d Kinematics::getPosition(Eigen::Matrix4d trans)
{
	Eigen::Vector3d pos;
	pos <<  trans(0, 3),
			trans(1, 3),
			trans(2, 3);
	return pos;
}


Eigen::Vector3d Kinematics::getPosition(double hipAngle,
										double thighAngle,
										double calfAngle)
{
	return getPosition(Eigen::Vector3d(hipAngle, thighAngle, calfAngle));
}

Eigen::Vector3d Kinematics::getPosition(Eigen::Vector3d angles)
{
	return getPosition(leg->getTransformation(angles));
}

Eigen::Vector3d Kinematics::jointDelta_q(Eigen::Vector3d delta_x)
{
	return leg->getJacobian().inverse() * delta_x;
}

Eigen::Vector3d Kinematics::jointAngleCompute(Eigen::Vector3d curAngles,
		Eigen::Vector3d delta_x)
{
	Eigen::Vector3d finalAngles = curAngles + jointDelta_q(delta_x);
	leg->enforceJointLim(finalAngles);
	return finalAngles;
}

Eigen::Vector3d Kinematics::jointAngleCompute(Eigen::Vector3d delta_x)
{
    Eigen::Vector3d curAngles = leg->getAngles();
	curAngles = curAngles + jointDelta_q(delta_x);
	leg->enforceJointLim(curAngles);
	return curAngles;
}

Kinematics::~Kinematics()
{
	// TODO Auto-generated destructor stub
}

