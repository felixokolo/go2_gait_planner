/*
 * LegMover.cpp
 *
 *  Created on: 23 Jun 2024
 *      Author: Felix
 */

#include "Low_level/BodyMover.h"

BodyMover::BodyMover(Robot *robotModel, std::string nodeName):
StateMonitor(robotModel, nodeName), robotModel(robotModel)
{
    /* FR_legMover = std::make_shared<LegMover>(robotModel, FR);
    FL_legMover = std::make_shared<LegMover>(robotModel, FL);
    RR_legMover = std::make_shared<LegMover>(robotModel, RR);
    RL_legMover = std::make_shared<LegMover>(robotModel, RL); */

    assert(qTarg.size() == JOINT_NUM * LEG_NUM);

    FR_legMover = new LegMover(robotModel->legs[FR], qTarg.begin() + (FR * JOINT_NUM));
    FL_legMover = new LegMover(robotModel->legs[FL], qTarg.begin() + (FL * JOINT_NUM));
    RR_legMover = new LegMover(robotModel->legs[RR], qTarg.begin() + (RR * JOINT_NUM));
    RL_legMover = new LegMover(robotModel->legs[RL], qTarg.begin() + (RL * JOINT_NUM));

    legMovers[0] = FR_legMover;
    legMovers[1] = FL_legMover;
    legMovers[2] = RR_legMover;
    legMovers[3] = RL_legMover;

    standPos = STAND_POS;
    sitPos = SIT_POS;

    FR_moveTimer_ = this->create_wall_timer(std::chrono::milliseconds((int)(1000/PUB_RATE)),
                                            std::bind(&LegMover::mover, FR_legMover));
    FL_moveTimer_ = this->create_wall_timer(std::chrono::milliseconds((int)(1000/PUB_RATE)),
                                            std::bind(&LegMover::mover, FL_legMover));
    RR_moveTimer_ = this->create_wall_timer(std::chrono::milliseconds((int)(1000/PUB_RATE)),
                                            std::bind(&LegMover::mover, RR_legMover));
    RL_moveTimer_ = this->create_wall_timer(std::chrono::milliseconds((int)(1000/PUB_RATE)),
                                            std::bind(&LegMover::mover, RL_legMover));

    cmdPubTimer = this->create_wall_timer(std::chrono::milliseconds(1),
                                          std::bind(&BodyMover::publishLowCmd, this));
    mover_sub = this->
    create_subscription<go2_gait_planner::msg::MoveLeg>(mover_topic, 10,
                                                   std::bind(&BodyMover::moverCallback,
                                                             this, std::placeholders::_1));
    standSit_sub = this->
    create_subscription<std_msgs::msg::Int32>(standSit_topic, 10,
                                                   std::bind(&BodyMover::standSitCallback,
                                                             this, std::placeholders::_1));
}

void BodyMover::publishLowCmd()
{
    int c = 0;
    for (int leg = 0; leg < LEG_NUM; ++leg)
    {
        if (!legMovers[leg]->straightPhase)
            ++c;
    }
    if (standCmd && c > 3)
    {
        standing = true;
        standCmd = false;
    }
        
    // std::cout << "Hi from BodyMover" << std::endl;
    StateMonitor::publishLowCmd();

}

void BodyMover::moverCallback(go2_gait_planner::msg::MoveLeg::SharedPtr moveMsg)
{
    startTime = curTime;
    // writeFile = true;
    MotionType motionType = (MotionType)moveMsg->motion_type;
    Eigen::Vector3d direction(moveMsg->direction[0],
                              moveMsg->direction[1],
                              moveMsg->direction[2]);
    int legType = moveMsg->leg_type;

    auto curPosition = robotModel->legs[legType]->getPosition();

    legMovers[legType]->moveLegPosition(curPosition + direction,
                                        moveMsg->duration, motionType,
                                        moveMsg->swing_height, 0);

    /* switch (motionType)
    {
        case STRAIGHT:
            // std::cout << "Executing straight " << legType << std::endl;
            legMovers[legType]->moveLegStraight(direction, moveMsg->duration, 0);

        break;

        case SWING:
            legMovers[legType]->moveLegSwing(direction, moveMsg->swing_height, moveMsg->duration, 0);
            std::cout << "Executing swing" << std::endl;
        break;

        default:
            throw std::invalid_argument("Invalid motion type.");
        break;
    } */
}

void BodyMover::standSitCallback(std_msgs::msg::Int32::SharedPtr standSitMsg)
{
    if (paramsNotSet)
        return;
    auto targPos = standPos;
    
    switch ((StandSitPos) standSitMsg->data)
    {
        case STAND:
            legMovers[FR]->moveLegPosition(FR_STAND, STAND_SIT_DURATION, STRAIGHT, 0, 0);
            legMovers[FL]->moveLegPosition(FL_STAND, STAND_SIT_DURATION, STRAIGHT, 0, 0);
            legMovers[RR]->moveLegPosition(RR_STAND, STAND_SIT_DURATION, STRAIGHT, 0, 0);
            legMovers[RL]->moveLegPosition(RL_STAND, STAND_SIT_DURATION, STRAIGHT, 0, 0);
            standCmd = true;
        break;

        case SIT:
            legMovers[FR]->moveLegPosition(FR_SIT, STAND_SIT_DURATION, STRAIGHT, 0, 0);
            legMovers[FL]->moveLegPosition(FL_SIT, STAND_SIT_DURATION, STRAIGHT, 0, 0);
            legMovers[RR]->moveLegPosition(RR_SIT, STAND_SIT_DURATION, STRAIGHT, 0, 0);
            legMovers[RL]->moveLegPosition(RL_SIT, STAND_SIT_DURATION, STRAIGHT, 0, 0);
            targPos = sitPos;
            standing = false;
            // return;
        break;

        default:
        break;
    }

    /* for (int leg = 0; leg < LEG_NUM; ++leg)
    {
        int curLegPos = JOINT_NUM * leg;
        Eigen::Vector3d pos(
                            targPos[curLegPos + 0],
                            targPos[curLegPos + 1],
                            targPos[curLegPos + 2]);
        legMovers[leg]->moveLegPosition(pos, STAND_SIT_DURATION, STRAIGHT, 0, 0);
    } */

    
}

BodyMover::~BodyMover()
{
    delete FR_legMover;
    delete FL_legMover;
    delete RR_legMover;
    delete RL_legMover;
}