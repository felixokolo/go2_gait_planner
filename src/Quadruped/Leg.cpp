/*
 * Leg.cpp
 *
 *  Created on: 2 Jun 2024
 *      Author: Priyanka
 *      Cheked: Felix
 */

#include "Quadruped/Leg.h"
#include <iostream>
#include <cmath>


Leg::Leg(double baseLength, double hipLength, double thighLength, double calfLength, LegType type)
    : hip(baseLength/2, hipLength, JointType::HIP), thigh(hipLength, thighLength, JointType::THIGH),
	  calf(thighLength, calfLength, JointType::CALF),legType(type),
	  kinematics(this){
	//kinematics = new Kinematics(legTransformation, legJacobian);

	//setLegTransformation(hip.getTransformation()*thigh.getTransformation()*calf.getTransformation());
	/*switch (legType) {

	case LegType::FL:
		std::cout << "FL Leg" << std::endl;
		break;

	case LegType::FR:
			std::cout << "FR Leg" << std::endl;
			break;

	case LegType::RL:
			std::cout << "RL Leg" << std::endl;
			break;

	case LegType::RR:
			std::cout << "RR Leg" << std::endl;
			break;
	}*/
	joints[HIP] = &hip;
	joints[THIGH] = &thigh;
	joints[CALF] = &calf;

	switch (legType)
	{
	case FL:
		transVector = FL_TRANS;
		calculateJacobian = [this](double qhip, double qthigh, double qcalf) {
			return this->calculateJacobian_FL(qhip, qthigh, qcalf);
		};
		break;

	case FR:
		transVector = FR_TRANS;
		calculateJacobian = [this](double qhip, double qthigh, double qcalf) {
			return this->calculateJacobian_FR(qhip, qthigh, qcalf);
			};
		break;

	case RL:
		transVector = RL_TRANS;
		calculateJacobian = [this](double qhip, double qthigh, double qcalf) {
			return this->calculateJacobian_RL(qhip, qthigh, qcalf);
		};
		break;

	case RR:
		transVector = RR_TRANS;
		calculateJacobian = [this](double qhip, double qthigh, double qcalf) {
			return this->calculateJacobian_RR(qhip, qthigh, qcalf);
		};
		break;

	default:
		throw std::invalid_argument("Invalid Leg type.");
		break;

	}

	/* calculateTransformation = [this](double qhip, double qthigh, double qcalf) {

		auto ret =  getBaseTransformation() *
					hip.getTransformation(qhip) *
					thigh.getTransformation(qthigh) *
					calf.getTransformation(qcalf);
		
		std::cout << "Gets Here" << std::endl;
		std::cout << qhip << " " << qthigh << " " << qcalf << std::endl;
		std::cout << ret << std::endl;
		
		return ret;
	}; */

	setAngles(DEFAULT_ANGLE_HIP, DEFAULT_ANGLE_THIGH, DEFAULT_ANGLE_CALF);

}

Eigen::Matrix<double, 4, 4> Leg::calculateTransformation(double qhip,
														 double qthigh,
														 double qcalf)
{
	auto ret =  getBaseTransformation() *
				hip.getTransformation(qhip) *
				thigh.getTransformation(qthigh) *
				calf.getTransformation(qcalf);
	
	/* std::cout << "Gets Here" << std::endl;
	std::cout << qhip << " " << qthigh << " " << qcalf << std::endl;
	std::cout << ret << std::endl; */
	
	return ret;
}

Eigen::Matrix<double, 3, 3> Leg::calculateJacobian_FL(double qhip, double qthigh,
		double qcalf)
{
	Eigen::Matrix3d	Jacobian;
	Jacobian << 0, - LEN_CALF*cos(qcalf + qthigh + CALF_OFFSET) - LEN_THIGH*cos(qthigh),                  -LEN_CALF*cos(qcalf + qthigh + CALF_OFFSET),
				LEN_THIGH*cos(qhip)*cos(qthigh) - LEN_HIP*sin(qhip) - LEN_CALF*sin(qcalf + CALF_OFFSET)*cos(qhip)*sin(qthigh) + LEN_CALF*cos(qcalf + CALF_OFFSET)*cos(qhip)*cos(qthigh), -sin(qhip)*(LEN_CALF*sin(qcalf + qthigh + CALF_OFFSET) + LEN_THIGH*sin(qthigh)), -LEN_CALF*sin(qcalf + qthigh + CALF_OFFSET)*sin(qhip),
				LEN_HIP*cos(qhip) + LEN_THIGH*cos(qthigh)*sin(qhip) + LEN_CALF*cos(qcalf + CALF_OFFSET)*cos(qthigh)*sin(qhip) - LEN_CALF*sin(qcalf + CALF_OFFSET)*sin(qhip)*sin(qthigh),  cos(qhip)*(LEN_CALF*sin(qcalf + qthigh + CALF_OFFSET) + LEN_THIGH*sin(qthigh)),  LEN_CALF*sin(qcalf + qthigh + CALF_OFFSET)*cos(qhip);

	return Jacobian;
}

Eigen::Matrix<double, 3, 3> Leg::calculateJacobian_FR(double qhip, double qthigh,
		double qcalf)
{
	Eigen::Matrix3d	Jacobian;
	Jacobian << 0, - LEN_CALF*cos(qcalf + qthigh + CALF_OFFSET) - LEN_THIGH*cos(qthigh), -LEN_CALF*cos(qcalf + qthigh + CALF_OFFSET),
				LEN_HIP*sin(qhip) + LEN_THIGH*cos(qhip)*cos(qthigh) - LEN_CALF*sin(qcalf + CALF_OFFSET)*cos(qhip)*sin(qthigh) + LEN_CALF*cos(qcalf + CALF_OFFSET)*cos(qhip)*cos(qthigh), -sin(qhip)*(LEN_CALF*sin(qcalf + qthigh + CALF_OFFSET) + LEN_THIGH*sin(qthigh)), -LEN_CALF*sin(qcalf + qthigh + CALF_OFFSET)*sin(qhip),
				LEN_THIGH*cos(qthigh)*sin(qhip) - LEN_HIP*cos(qhip) + LEN_CALF*cos(qcalf + CALF_OFFSET)*cos(qthigh)*sin(qhip) - LEN_CALF*sin(qcalf + CALF_OFFSET)*sin(qhip)*sin(qthigh),  cos(qhip)*(LEN_CALF*sin(qcalf + qthigh + CALF_OFFSET) + LEN_THIGH*sin(qthigh)),  LEN_CALF*sin(qcalf + qthigh + CALF_OFFSET)*cos(qhip);

	return Jacobian;
}

Eigen::Matrix<double, 3, 3> Leg::calculateJacobian_RL(double qhip, double qthigh,
		double qcalf)
{
	Eigen::Matrix3d	Jacobian;
	Jacobian << 0, - LEN_CALF*cos(qcalf + qthigh + CALF_OFFSET) - LEN_THIGH*cos(qthigh), -LEN_CALF*cos(qcalf + qthigh + CALF_OFFSET),
				LEN_THIGH*cos(qhip)*cos(qthigh) - LEN_HIP*sin(qhip) - LEN_CALF*sin(qcalf + CALF_OFFSET)*cos(qhip)*sin(qthigh) + LEN_CALF*cos(qcalf + CALF_OFFSET)*cos(qhip)*cos(qthigh), -sin(qhip)*(LEN_CALF*sin(qcalf + qthigh + CALF_OFFSET) + LEN_THIGH*sin(qthigh)), -LEN_CALF*sin(qcalf + qthigh + CALF_OFFSET)*sin(qhip),
				LEN_HIP*cos(qhip) + LEN_THIGH*cos(qthigh)*sin(qhip) + LEN_CALF*cos(qcalf + CALF_OFFSET)*cos(qthigh)*sin(qhip) - LEN_CALF*sin(qcalf + CALF_OFFSET)*sin(qhip)*sin(qthigh),  cos(qhip)*(LEN_CALF*sin(qcalf + qthigh + CALF_OFFSET) + LEN_THIGH*sin(qthigh)),  LEN_CALF*sin(qcalf + qthigh + CALF_OFFSET)*cos(qhip);

	return Jacobian;
}

Eigen::Matrix<double, 3, 3> Leg::calculateJacobian_RR(double qhip, double qthigh,
		double qcalf)
{
	Eigen::Matrix3d	Jacobian;
	Jacobian << 0, - LEN_CALF*cos(qcalf + qthigh + CALF_OFFSET) - LEN_THIGH*cos(qthigh), -LEN_CALF*cos(qcalf + qthigh + CALF_OFFSET),
				LEN_HIP*sin(qhip) + LEN_THIGH*cos(qhip)*cos(qthigh) - LEN_CALF*sin(qcalf + CALF_OFFSET)*cos(qhip)*sin(qthigh) + LEN_CALF*cos(qcalf + CALF_OFFSET)*cos(qhip)*cos(qthigh), -sin(qhip)*(LEN_CALF*sin(qcalf + qthigh + CALF_OFFSET) + LEN_THIGH*sin(qthigh)), -LEN_CALF*sin(qcalf + qthigh + CALF_OFFSET)*sin(qhip),
				LEN_THIGH*cos(qthigh)*sin(qhip) - LEN_HIP*cos(qhip) + LEN_CALF*cos(qcalf + CALF_OFFSET)*cos(qthigh)*sin(qhip) - LEN_CALF*sin(qcalf + CALF_OFFSET)*sin(qhip)*sin(qthigh),  cos(qhip)*(LEN_CALF*sin(qcalf + qthigh + CALF_OFFSET) + LEN_THIGH*sin(qthigh)),  LEN_CALF*sin(qcalf + qthigh + CALF_OFFSET)*cos(qhip);

	return Jacobian;
}


void Leg::setAngles(double hip_angle, double thigh_angle, double calf_angle)
{
	    // Calculating each joint transformation and then multiplying to get complete transformation
	    hip.setAngle(hip_angle);
	    thigh.setAngle(thigh_angle);
	    calf.setAngle(calf_angle);

	    legTransformation = 	getBaseTransformation() *
	    						hip.getTransformation() *
	    						thigh.getTransformation() *
								calf.getTransformation();


	    /*std::cout << "Hip Transformation matrix:\n" << getBaseTransformation()*hip.getTransformation() << std::endl;
	    std::cout << "Thigh Transformation matrix:\n" << thigh.getTransformation() << std::endl;
	    std::cout << "Calf Transformation matrix:\n" << calf.getTransformation() << std::endl;
	    std::cout << "Leg Transformation matrix from base:\n" << legTransformation << std::endl;*/


	    //calling
	    legJacobian = calculateJacobian(hip_angle, thigh_angle, calf_angle);

}

Eigen::Matrix4d Leg::getTransformation()
{
	return legTransformation;
}



Eigen::Vector3d Leg::getPosition()
{
	return kinematics.getCurPosition();
}

Eigen::Vector3d Leg::getAngles()
{
	return Eigen::Vector3d(hip.getAngle(), thigh.getAngle(), calf.getAngle());
}

Eigen::Matrix4d Leg::getTransformation(Eigen::Vector3d angles)
{
	/* std::cout << "Gets transformation" << std::endl;
	std::cout << angles << std::endl; */
	auto ret = calculateTransformation(angles[0], angles(1), angles(2));
	//std::cout << ret << std::endl;
	return ret;
}

Eigen::MatrixXd Leg::getJacobian()
{
	return legJacobian;
}

Eigen::MatrixXd Leg::getJacobian(Eigen::Vector3d angles)
{
	return calculateJacobian(angles[0], angles(1), angles(2));
}

Eigen::Vector3d Leg::getPosition(Eigen::Vector3d angles)
{
	return kinematics.getPosition(angles);
}

Eigen::Matrix4d Leg::getBaseTransformation()
{
	Eigen::Matrix4d tmp;
		tmp << 0, 0, 1, transVector[0],
				0, 1, 0, transVector[1],
				-1, 0, 0, transVector[2],
				0, 0, 0, 1;
		return tmp;
}

Kinematics* Leg::getKinematics()
{
	return &kinematics;
}

Eigen::Vector3d Leg::enforceJointLim(Eigen::Vector3d estJointAngles)
{
	Eigen::Vector3d limitedAngles;
	// std::cout << "sending " << estJointAngles << std::endl;
	for (int joint = 0; joint < JOINT_NUM; ++joint)
	{
		limitedAngles[joint] = joints[joint]->enforceLim(estJointAngles[joint]);
	}
	// std::cout << "received " << limitedAngles << std::endl;
	return limitedAngles;
}

Eigen::Vector3d Leg::getJointVels()
{
    return Eigen::Vector3d(hip.getQd(),
						   thigh.getQd(),
						   calf.getQd());
}

Eigen::Vector3d Leg::getJointTaus()
{
    return Eigen::Vector3d(hip.getTau(),
						   thigh.getTau(),
						   calf.getTau());
}

Eigen::Vector3d Leg::getJointKps()
{
    return Eigen::Vector3d(hip.getKp(),
						   thigh.getKp(),
						   calf.getKp());
}

Eigen::Vector3d Leg::getJointKds()
{
    return Eigen::Vector3d(hip.getKd(),
						   thigh.getKd(),
						   calf.getKd());
}

void Leg::setJointVels(Eigen::Vector3d values)
{
	hip.setQd(values[0]);
	thigh.setQd(values[1]);
	calf.setQd(values[2]);
}

void Leg::setJointTaus(Eigen::Vector3d values)
{
	hip.setTau(values[0]);
	thigh.setTau(values[1]);
	calf.setTau(values[2]);
}

void Leg::setJointKps(Eigen::Vector3d values)
{
	hip.setKp(values[0]);
	thigh.setKp(values[1]);
	calf.setKp(values[2]);
}

void Leg::setJointKds(Eigen::Vector3d values)
{
	hip.setKd(values[0]);
	thigh.setKd(values[1]);
	calf.setKd(values[2]);
}

Eigen::Vector3d Leg::calculateTau(Eigen::Vector3d f_ext)
{
	/* std::cout << legJacobian.transpose() <<std::endl;
	std::cout << f_ext <<std::endl; */
	return legJacobian.transpose() * f_ext;
}

/*Eigen::Vector3d Leg::getAngles( Eigen::MatrixXd Jacobian)
{
	Eigen::Vector3d angles;
	angles = Jacobian * position;
	std::cout<< "angles is" <<angles <<std::endl;
	return angles;

}




LegType Leg::getType() const
{
	return legType;
}

void Leg::setType(LegType type)
{
	this->legType = type;
}*/
