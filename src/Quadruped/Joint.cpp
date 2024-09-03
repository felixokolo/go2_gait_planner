/*
 * Joint.cpp
 *
 *  Created on: 2 Jun 2024
 *      Author: Priyanka
 *      Cheked: Felix
 */

#include "Quadruped/Joint.h"
#include <cmath>
#include <iostream>


Joint::Joint(double parentLength, double childLength, JointType type):
			 parentLink(parentLength), childLink(childLength), type(type)
{
	 switch (type) {
		case HIP:
			jointLoLim = HIP_LO_LIM;
			jointUpLim = HIP_UP_LIM;
			defaultAngle = DEFAULT_ANGLE_HIP;

			calculateTransformation = [this](double angle)
			{
				return this->calculateHipTransformation(angle);
			};
			break;
		case THIGH:
			jointLoLim = THIGH_LO_LIM;
			jointUpLim = THIGH_UP_LIM;
			defaultAngle = DEFAULT_ANGLE_THIGH;

			calculateTransformation = [this](double angle)
			{
				return this->calculateThighTransformation(angle);
			};
			break;
		case CALF:

			jointLoLim = CALF_LO_LIM;
			jointUpLim = CALF_UP_LIM;
			defaultAngle = DEFAULT_ANGLE_CALF;

			calculateTransformation = [this](double angle)
			{
				return this->calculateCalfTransformation(angle);
			};
			break;
		default:
			throw std::invalid_argument("Invalid Joint type.");
	}

	setAngle(defaultAngle);
}

void Joint::setAngle(double angle)  {
	jointAngle = enforceLim(angle);
	jointTransformation = calculateTransformation(jointAngle);
}


double Joint::enforceLim(double estJointAngle)
{
	if (estJointAngle < jointLoLim)
	{
		std::cout << "Low limit Triggered " << estJointAngle << std::endl;
		// std::exit(-1);
		throw 0;
		estJointAngle = defaultAngle;
		// estJointAngle = jointAngle;
	}
		
	else if (estJointAngle > jointUpLim)
	{
		std::cout << "Uplimit Triggered " << estJointAngle << std::endl;
		// std::exit(-1);
		throw 0;
		estJointAngle = defaultAngle;
		// estJointAngle = jointAngle;
	}
	return estJointAngle;	
}

Eigen::Matrix<double, 4, 4> Joint::calculateHipTransformation(double angle) {

	Eigen::Matrix<double, 4, 4> jointTransformation;

	jointTransformation << 	cos(angle),		0,		-sin(angle),	0,
						 	sin(angle),		0,  	cos(angle),		0,
						    0, 				-1,			0, 			ladj,
							0,				0,			0,    		1 ;
	return jointTransformation;
}

Eigen::Matrix<double, 4, 4> Joint::calculateThighTransformation(double angle) {

	Eigen::Matrix<double, 4, 4> jointTransformation;

    jointTransformation <<  cos(angle), -sin(angle),	0,
    										childLink.getLength()*cos(angle),
							 sin(angle),  cos(angle),	0,
							 	 	 	 	 childLink.getLength()*sin(angle),
							      0,          0,		1,
								  	  	  	  parentLink.getLength(),
								  0,		  0, 		0,			1;

    return jointTransformation;
}


Eigen::Matrix<double, 4, 4> Joint::calculateCalfTransformation(double angle) {

	Eigen::Matrix<double, 4, 4> jointTransformation;

    jointTransformation << cos(angle + CALF_OFFSET), -sin(angle + CALF_OFFSET),
    					   0, childLink.getLength()*cos(angle + CALF_OFFSET),

						   sin(angle + CALF_OFFSET),  cos(angle + CALF_OFFSET),
						   0, childLink.getLength()*sin(angle + CALF_OFFSET),

						   0,			0, 			1,			0,
						   0,			0,			0,			1;

    return jointTransformation;
}

const Eigen::Matrix4d& Joint::getTransformation() const {

    return jointTransformation;
}

const Eigen::Matrix4d Joint::getTransformation(double angle) const {

    return calculateTransformation(angle);
}


Joint::~Joint() {
    // Destructor
}

double Joint::getAngle()
{
	return jointAngle;
}

double Joint::getQd()
{
    return dq;
}

double Joint::getTau()
{
    return tau;
}

double Joint::getKp()
{
    return Kp;
}

double Joint::getKd()
{
    return Kd;
}

void Joint::setQd(double value)
{
	dq = value;
}

void Joint::setTau(double value)
{
	tau = value;
}

void Joint::setKp(double value)
{
	Kp = value;
}

void Joint::setKd(double value)
{
	Kd = value;
}
