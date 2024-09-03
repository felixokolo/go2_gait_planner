/*
 * Robot.cpp
 *
 *  Created on: 2 Jun 2024
 *      Author: Priyanka
 *      Cheked: Felix
 */

#include "Quadruped/Robot.h"

Robot::Robot()
    : FL(LEN_BASE, LEN_HIP, LEN_THIGH, LEN_CALF, LegType::FL),
      FR(LEN_BASE, -LEN_HIP, LEN_THIGH, LEN_CALF, LegType::FR),
      RL(LEN_BASE, LEN_HIP, LEN_THIGH, LEN_CALF, LegType::RL),
      RR(LEN_BASE, -LEN_HIP, LEN_THIGH, LEN_CALF, LegType::RR)
{
}

Eigen::Matrix4d Robot::getTransformation_FL()
{
	return FL.getTransformation();
}

Eigen::Matrix4d Robot::getTransformation_FR()
{
	return FR.getTransformation();
}

Eigen::Matrix4d Robot::getTransformation_RL()
{
	return RL.getTransformation();
}

Eigen::Matrix4d Robot::getTransformation_RR()
{
	return RR.getTransformation();
}

void Robot::setAngles_FL(double hip_angle, double thigh_angle, double calf_angle)
{
	FL.setAngles(hip_angle, thigh_angle, calf_angle);
}

void Robot::setAngles_FR(double hip_angle, double thigh_angle, double calf_angle)
{
	FR.setAngles(hip_angle, thigh_angle, calf_angle);
}

void Robot::setAngles_RL(double hip_angle, double thigh_angle, double calf_angle)
{
	RL.setAngles(hip_angle, thigh_angle, calf_angle);
}

void Robot::setAngles_RR(double hip_angle, double thigh_angle, double calf_angle)
{
	RR.setAngles(hip_angle, thigh_angle, calf_angle);
}


Eigen::Vector3d Robot::getPosition_FL()
{
	return FL.getPosition();
}

Eigen::Vector3d Robot::getPosition_FR()
{
	return FR.getPosition();
}

Eigen::Vector3d Robot::getPosition_RL()
{
	return RL.getPosition();
}

Eigen::Vector3d Robot::getPosition_RR()
{
	return RR.getPosition();
}

/*Eigen::Vector3d  Robot::getPosition(const std::vector<double> &angles, const LegType leg) {
    if (angles.size() != 3) {
        throw std::invalid_argument("Expected exactly 3 angles.");
    }
    Eigen::Vector3d position;
    switch (leg) {
        case LegType::FL:
        	 position = FL.setangle(angles);
            break;
        case LegType::FR:
            position = FR.setangle(angles);
            break;
        case LegType::RL:
            position = RL.setangle(angles);
            break;
        case LegType::RR:
            position = RR.setangle(angles);
            break;
        default:
            throw std::invalid_argument("Invalid leg type.");

    }
    return position;
}*/

void Robot::setAngles(std::vector<double> angles)
{
    if (angles.size() < 12)
    {
        std::cerr << "Incomplete angles";
        throw std::invalid_argument("Invalid joint angles");
        return;
    }

    for (int i = 0; i < LEG_NUM; i++)
    {
        int cur_leg = i * JOINT_NUM;
        legs[i]->setAngles(angles[cur_leg + HIP], angles[cur_leg + THIGH], angles[cur_leg + CALF]);
    }
}

void Robot::setVels(std::vector<double> values)
{
    if (values.size() < 12)
    {
        std::cerr << "Incomplete values";
        throw std::invalid_argument("Incomplete values");
        return;
    }

    for (int i = 0; i < LEG_NUM; i++)
    {
        int cur_leg = i * JOINT_NUM;
        legs[i]->setJointVels(Eigen::Vector3d(values[cur_leg + HIP], values[cur_leg + THIGH], values[cur_leg + CALF]));
    }
}

void Robot::setTaus(std::vector<double> values)
{
    if (values.size() < 12)
    {
        std::cerr << "Incomplete values";
        throw std::invalid_argument("Incomplete values");
        return;
    }

    for (int i = 0; i < LEG_NUM; i++)
    {
        int cur_leg = i * JOINT_NUM;
        legs[i]->setJointTaus(Eigen::Vector3d(values[cur_leg + HIP], values[cur_leg + THIGH], values[cur_leg + CALF]));
    }
}

void Robot::setKps(std::vector<double> values)
{
    if (values.size() < 12)
    {
        std::cerr << "Incomplete values";
        throw std::invalid_argument("Incomplete values");
        return;
    }

    for (int i = 0; i < LEG_NUM; i++)
    {
        int cur_leg = i * JOINT_NUM;
        legs[i]->setJointKps(Eigen::Vector3d(values[cur_leg + HIP], values[cur_leg + THIGH], values[cur_leg + CALF]));
    }
}

void Robot::setKds(std::vector<double> values)
{
    if (values.size() < 12)
    {
        std::cerr << "Incomplete values";
        throw std::invalid_argument("Incomplete values");
        return;
    }

    for (int i = 0; i < LEG_NUM; i++)
    {
        int cur_leg = i * JOINT_NUM;
        legs[i]->setJointKds(Eigen::Vector3d(values[cur_leg + HIP], values[cur_leg + THIGH], values[cur_leg + CALF]));
    }
}

std::vector<double> Robot::getAngles()
{
    std::vector<double> ret;
    for (int leg = 0; leg < LEG_NUM; leg++)
    {
        auto jointValues = legs[leg]->getAngles();
        for (int joint = 0; joint < JOINT_NUM; joint++)
            ret.push_back(jointValues[joint]);
    }
    return ret;
}

std::vector<double> Robot::getVels()
{
    std::vector<double> ret;
    for (int leg = 0; leg < LEG_NUM; leg++)
    {
        auto jointValues = legs[leg]->getJointVels();
        for (int joint = 0; joint < JOINT_NUM; joint++)
            ret.push_back(jointValues[joint]);
    }
    return ret;
}

std::vector<double> Robot::getTaus()
{
    std::vector<double> ret;
    for (int leg = 0; leg < LEG_NUM; leg++)
    {
        auto jointValues = legs[leg]->getJointTaus();
        for (int joint = 0; joint < JOINT_NUM; joint++)
            ret.push_back(jointValues[joint]);
    }
    return ret;
}

std::vector<double> Robot::getKps()
{
    std::vector<double> ret;
    for (int leg = 0; leg < LEG_NUM; leg++)
    {
        auto jointValues = legs[leg]->getJointKps();
        for (int joint = 0; joint < JOINT_NUM; joint++)
            ret.push_back(jointValues[joint]);
    }
    return ret;
}

std::vector<double> Robot::getKds()
{
    std::vector<double> ret;
    for (int leg = 0; leg < LEG_NUM; leg++)
    {
        auto jointValues = legs[leg]->getJointKds();
        for (int joint = 0; joint < JOINT_NUM; joint++)
            ret.push_back(jointValues[joint]);
    }
    return ret;
}

void Robot::setOrientation(float roll, float pitch, float yaw)
{
    bodyOrientation = {roll, pitch, yaw};
}

Eigen::Vector3d Robot::getOrientation()
{
    return Eigen::Vector3d((double*) &bodyOrientation[0]);
}

Eigen::Vector<double, 12> Robot::calculateTaus(Eigen::Vector3d f_ext)
{
    std::vector<double> ret;
    for (int leg = 0; leg < LEG_NUM; leg++)
    {
        auto taus = legs[leg]->calculateTau(f_ext);
        for (int joint = 0; joint < JOINT_NUM; joint++)
            ret.push_back(taus[joint]);
    }
    return Eigen::Vector<double, 12>((double*) &ret[0]);
}

void Robot::limitTaus()
{
    for (int i = 0; i < JOINT_NUM * LEG_NUM; i++)
    {
        if (cntrlTau[i] > maxTaus[i])
            cntrlTau[i] = maxTaus[i];

        if (cntrlTau[i] < -maxTaus[i])
            cntrlTau[i] = maxTaus[i];

    }
}

/* void Robot::setValues(const std::function<void(Eigen::Vector3d)>& func, std::vector<double> values)
{
    if (values.size() < 12)
    {
        std::cerr << "Incomplete values";
        throw std::invalid_argument("Incomplete values");
        return;
    }

    for (int i = 0; i < LEG_NUM; i++)
    {
        int cur_leg = i * JOINT_NUM;
        func(Eigen::Vector3d(values[cur_leg + HIP], values[cur_leg + THIGH], values[cur_leg + CALF]));
    }
} */
