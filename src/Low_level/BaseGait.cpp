/*
 * BaseGait.cpp
 *
 *  Created on: 16 Aug 2024
 *      Author: Felix
 */

#include "Low_level/BaseGait.h"

BaseGait::BaseGait(Robot *robotModel, std::string nodeName):
BodyMover(robotModel, nodeName), robotModel(robotModel)
{
    gaitTimer_ = this->create_wall_timer(std::chrono::milliseconds(1), std::bind(&BaseGait::gaitCallback, this));
    gait_params_sub = this->create_subscription<go2_scratch::msg::GaitParam>(
        gait_params_topic, 10, std::bind(&BaseGait::gaitParamsCallback, this, std::placeholders::_1));
    cmdPubTimer = this->create_wall_timer(std::chrono::milliseconds(1),
                                          std::bind(&BaseGait::publishLowCmd, this));
}


void BaseGait::gaitParamsCallback(go2_scratch::msg::GaitParam::SharedPtr gaitParamsMsg)
{
    setGaitMotion(static_cast<GaitMotion>(gaitParamsMsg->movement));
    setStanceDuration(static_cast<int>(gaitParamsMsg->stance_duration * 500));
    setStanceDepth(gaitParamsMsg->stance_depth);
    setSwingHeight(gaitParamsMsg->swing_height);
    // std::cout << "new time type " << gaitParamsMsg->stance_duration * 1000 << std::endl;
    if (!writeFile)
    {
        writeFile = true;
        startTime = curTime;
    }
    
}

void BaseGait::setStanceDuration(int val)
{
    if (val < (1000/PUB_RATE) * ANGLE_RES)
        return;
    stance_duration = val;
}

void BaseGait::setStanceDepth(float val)
{
    if (val > 0 || val < -0.05)
        return;
    stanceDepth = val;
}

void BaseGait::setSwingHeight(float val)
{
    if (val < 0 || val > (NOMINAL_HEIGHT - 0.05))
        return;
    swingHeight = val;
}

void BaseGait::setGaitMotion(GaitMotion val)
{
    gaitMotion = val > GAIT_MOTION_NUM? STOP : val;
    gaitStartTime = curTime;
    phase = 0;
    // std::cout << "start Time " << startTime << std::endl;
}


BaseGait::~BaseGait()
{
    
}