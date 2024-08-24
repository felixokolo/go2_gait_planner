/*
 * LegMover.cpp
 *
 *  Created on: 23 Jun 2024
 *      Author: Felix
 */

#include "Low_level/LegMover.h"

LegMover::LegMover(Leg *leg, std::vector<double>::iterator qTarg):
            leg(leg), leg_qTarg(qTarg)
{
    F_Ext = Eigen::Vector3d({0.0, 0.0, -15.0}); // 45N Force in the z-axis
    /* mover_sub = this->create_subscription<go2_scratch::msg::MoveLeg>(
        mover_topic, 10, std::bind(&LegMover::moverCallback, this, std::placeholders::_1));
    
        
    moveTimer_ = this->create_wall_timer(std::chrono::milliseconds((int)(1000/PUB_RATE)), std::bind(&LegMover::mover, this));
    // swingTimer_ = this->create_wall_timer(std::chrono::milliseconds((int)(1000/PUB_RATE)), std::bind(&LegMover::swingMover, this));
    // cmdPubTimer = this->create_wall_timer(std::chrono::milliseconds(1), std::bind(&LegMover::publishLowCmd, this));
    // controllLoopTimer = this->create_wall_timer(std::chrono::milliseconds(1), std::bind(&LegMover::controllLoop, this));
    F_Ext = Eigen::Vector3d({0.0, 0.0, -15.0}); // 45N Force in the z-axis
    cmdTaus = Eigen::Vector<double, 12>({0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}); */
}

void LegMover::moveLegStraight(Eigen::Vector3d direction, int duration, int phaseOffset)
{
    if (duration < 1000/PUB_RATE)
        duration = 5; // Set to 5 ms
    waitTime = ((duration / ANGLE_RES) / (1000 / PUB_RATE)) - 1;
    
    straightPhase = ANGLE_RES;
   swingPhase = 0;
    this->straightDirection_d = direction / straightPhase;
    motionType = STRAIGHT;
}

void LegMover::straightMover()
{
    if (straightPhase == 0) // Return when the phase is complete
    {
        countDown = 0;
        motionType = MOTION_TYPE_NUM;

        /* Eigen::Vector3d curAngles = get_qTarg();
        Eigen::Vector3d straightDirection_d = targPos - leg->getPosition();
        curAngles = leg->getKinematics()->jointAngleCompute(curAngles, straightDirection_d);
        set_qTarg(leg->enforceJointLim(curAngles)); */

        return;
    }
    if (countDown == 0)
    {
        countDown = waitTime;
    }
        
    else
    {
        countDown--;
        return;
    }
    
    straightPhase--;
    Eigen::Vector3d curAngles = get_qTarg();
    curAngles = leg->getKinematics()->jointAngleCompute(curAngles, straightDirection_d);
    set_qTarg(leg->enforceJointLim(curAngles));
}

void LegMover::moveLegPosition(Eigen::Vector3d position, int duration,
                               MotionType motionType, float swingHeight, int phaseOffset)
{
    auto direction = position - leg->getPosition();
    targPos = position;
    switch (motionType)
    {
    case STRAIGHT:
        moveLegStraight(direction, duration, phaseOffset);
        break;
    case SWING:
        moveLegSwing(direction, swingHeight, duration, phaseOffset);
        break;
    
    default:
        break;
    }
}

void LegMover::mover()
{
    // std::cout << "From " << legType << std::endl;
    // std::lock_guard<std::mutex> guard(guard_mutex);

    switch (motionType)
    {
        case STRAIGHT:
            straightMover();
        break;
        case SWING:
            swingMover();
        break;
        default:
            return;
        break;
    }
}

Eigen::Vector3d LegMover::get_qTarg()
{
    Eigen::Vector3d qt(*(leg_qTarg + HIP), *(leg_qTarg + THIGH), *(leg_qTarg + CALF));
    return qt;
}

void LegMover::set_qTarg(Eigen::Vector3d angles)
{
    for (int joint = 0; joint < JOINT_NUM; ++joint)
    {
        *(leg_qTarg + joint) = angles[joint];
    }
}


void LegMover::moveLegSwing(Eigen::Vector3d direction, float swingHeight, int duration, int phaseOffset)
{
    
    // x direction must be greater than 0
    bool use_x = true;
    if ((int)(abs(direction[0] * 100)) == 0)
        use_x = false;
    if (!use_x)
        if ((int)(abs(direction[1] * 100)) == 0)
            return;
    
    if (duration < (1000/PUB_RATE) * ANGLE_RES)
        duration = (1000/PUB_RATE) * ANGLE_RES; // Set to minimum duration
    swingPhase = ANGLE_RES;
    straightPhase = 0;
    waitTime = ((duration / ANGLE_RES) / (1000 / PUB_RATE)) - 1;


    this->swingDirection_d = direction / ANGLE_RES;
    // this->swingLegType = legType;
    motionType = SWING;

    // Set orientation/stability control legs

    /* if (legType == FR || legType == RL)
        curOrientCntrlLeg = FL_RR;
    else
        curOrientCntrlLeg = FR_RL;
    orientTarg = robotModel->getOrientation();

    cntrl1 = curOrientCntrlLeg == FL_RR? FL * JOINT_NUM : FR * JOINT_NUM;
    cntrl2 = curOrientCntrlLeg == FL_RR? RR * JOINT_NUM : RL * JOINT_NUM;

    cntrlMask[cntrl1] = false;
    cntrlMask[cntrl2] = false;
    // lowCmdMsg.motor_cmd[cntrl1].kp = 0;
    // lowCmdMsg.motor_cmd[cntrl2].kp = 0;

    orientCntrl = true;
 */
    // Get interpolation points
    Eigen::VectorXd x(5);
    Eigen::VectorXd y(5);

    Eigen::Vector3d curAngles = get_qTarg();
    auto curPos = leg->getPosition(curAngles);

    
    auto to_use_x = use_x ? curPos[0] : curPos[1];
    auto to_use_dir = use_x ? direction[0] : direction[1];
    // Define swinging profile (Foot trajectory)
	x <<    to_use_x, // Initial Positions
            to_use_x + (to_use_dir * 0.15), // 25%
            to_use_x + (to_use_dir * 0.65), // Mid Positions
            to_use_x + (to_use_dir * 0.85), // 75%
            to_use_x + to_use_dir; //Final Positions

    y <<    curPos[2], // Initial Positions
            // curPos[2] + direction[2] + swingHeight * 0.75, // 75%
            curPos[2] + (swingHeight * 0.75), // 75%
            // curPos[2] + direction[2] + swingHeight,// Mid Positions
            curPos[2] + swingHeight, // Mid Positions
            // curPos[2] + direction[2] + swingHeight * 0.75, // 75%
            // curPos[2] + (direction[2] * 0.5), // 75%
            // curPos[2] + direction[2]; //Final Positions
            curPos[2] + swingHeight + (direction[2] - swingHeight) * 0.5,
            curPos[2] + direction[2];
    curSwing = to_use_x; // To keep track of the current interpolation point in x or y
    swing_dir = use_x ? 0 : 1; // Swinging in the x:0 or y:1 direction


    // Final Position
    targPos = curPos + direction;
	
    
    // std::cout << "X " << x << std::endl;
    // std::cout << "Y " << y << std::endl;

	swingSpline = Eigen::SplineFitting<Eigen::Spline<double, 1>>::Interpolate(y.transpose(), 4, x);
    
    //std::cout << "I'm here" << std::endl;

    

    
}

void LegMover::swingMover()
{
    

    if (swingPhase == 0) // Return when the phase is complete
    {
        countDown = 0;
        motionType = MOTION_TYPE_NUM;
        
        //orientCntrl = false;

        /* cntrlMask[cntrl1] = true;
        cntrlMask[cntrl2] = true;
        lowCmdMsg.motor_cmd[cntrl1].kp = robotModel->getKps()[cntrl1];
        lowCmdMsg.motor_cmd[cntrl2].kp = robotModel->getKps()[cntrl2];
        writeFile = false; */
        return;
    }
    if (countDown == 0)
        countDown = waitTime;
    else
    {
        countDown--;
        return;
    }

    swingPhase--;
    Eigen::Vector3d curAngles = get_qTarg();
    auto curPos = leg->getPosition(curAngles);
    curSwing += swingDirection_d[swing_dir];
    Eigen::RowVectorXd z_interp = swingSpline(curSwing);

    //std::cout << curPos[0] + swingDirection_d[0] << " " << z_interp << std::endl;
    // Enforce final leg height
    if (swingPhase == 0)
    {
        // curPos = leg->getPosition();
        swingDirection_d = targPos - curPos;
        z_interp[0] = targPos[2];

    }
        

    swingDirection_d[2] = z_interp[0] - curPos[2];

    // std::cout << z_interp[0] << " " << curSwing << std::endl;

    curAngles = leg->getKinematics()->jointAngleCompute(curAngles, swingDirection_d);
    set_qTarg(leg->enforceJointLim(curAngles));
    // To be commented out
    // robotModel->legs[swingLegType]->setAngles(qt[0], qt[1], qt[2]);
    
    //

    //std::cout << curPos << std::endl;

    /* modifying = true;

    for (int j = 0; j < JOINT_NUM; j++)
    {
        qTarg[curLegPos + j] = qt[j];
        lowCmdMsg.motor_cmd[curLegPos + j].q = qt[j];
    } */

    

    //for (int i = curLegPos, j = 0; i < curLegPos + JOINT_NUM; i++, j++)
    //    lowCmdMsg.motor_cmd[i].q = legAngles[j];

    // modifying = false;
    //publishLowCmd();

}

