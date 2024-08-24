/*
 * TrotGait.cpp
 *
 *  Created on: 16 Aug 2024
 *      Author: Felix
 */

#include "Low_level/TrotGait.h"

TrotGait::TrotGait(Robot *robotModel, std::string nodeName):
BaseGait(robotModel, nodeName), robotModel(robotModel)
{
    
}

void TrotGait::gaitCallback()
{
    if (delay > 0)
    {
        --delay;
        return;
    }
    else
    {    
        if (legMovers[FR]->straightPhase || legMovers[FL]->straightPhase ||
            legMovers[RR]->straightPhase || legMovers[RL]->straightPhase)
            return;

        if (standing)
        {
            switch (gaitMotion)
            {
            case STOP:
                stand();
                if ((curTime - gaitStartTime) > 1000)
                    gaitMotion = (GAIT_MOTION_NUM); // Make it stop after 5 secss
                else
                    writeFile = false;
            break;

            case FORWARD:
                forward();
            break;

            case BACKWARD:
                backward();
            break;

            case LEFT:
                left();
            break;

            case RIGHT:
                right();
            break;

            case JUMP:
                jump();
            break;
            
            
            default:
                break;
            }
        }}
    
}

void TrotGait::stand()
{
    switch (phase)
    {
        case 0:
            if (legMovers[FL]->swingPhase || legMovers[RR]->swingPhase)
                return;
            if (!legMovers[FR]->swingPhase) legMovers[FR]->moveLegPosition(FR_STAND, stance_duration, SWING, swingHeight, 0);
            if (!legMovers[RL]->swingPhase) legMovers[RL]->moveLegPosition(RL_STAND, stance_duration, SWING, swingHeight, 0);

            phase = 1;
            delay = 500;

        break;

        case 1:
            if (legMovers[FR]->swingPhase || legMovers[RL]->swingPhase)
                return;
            if (!legMovers[FL]->swingPhase) legMovers[FL]->moveLegPosition(FL_STAND, stance_duration, SWING, swingHeight, 0);
            if (!legMovers[RR]->swingPhase) legMovers[RR]->moveLegPosition(RR_STAND, stance_duration, SWING, swingHeight, 0);

            phase = 0;
            delay = 500;
        break;
    }
}

void TrotGait::forward()
{
    switch (phase)
    {
        case 0:
            if (!legMovers[FR]->swingPhase) legMovers[FR]->moveLegPosition(FR_FRONT, stance_duration, SWING, swingHeight, 0);
            if (!legMovers[RL]->swingPhase) legMovers[RL]->moveLegPosition(RL_FRONT, stance_duration, SWING, swingHeight, 0);

            if (!legMovers[FL]->straightPhase) legMovers[FL]->moveLegPosition(FL_BACK, stance_duration, STRAIGHT, stanceDepth, 0);
            if (!legMovers[RR]->straightPhase) legMovers[RR]->moveLegPosition(RR_BACK, stance_duration, STRAIGHT, stanceDepth, 0);

            // standing = false;

            phase = 1;
            delay = 500;

        break;

        case 1:
            if (!legMovers[FL]->swingPhase) legMovers[FL]->moveLegPosition(FL_FRONT, stance_duration, SWING, swingHeight, 0);
            if (!legMovers[RR]->swingPhase) legMovers[RR]->moveLegPosition(RR_FRONT, stance_duration, SWING, swingHeight, 0);

            if (!legMovers[FR]->straightPhase) legMovers[FR]->moveLegPosition(FR_BACK, stance_duration, STRAIGHT, stanceDepth, 0);
            if (!legMovers[RL]->straightPhase) legMovers[RL]->moveLegPosition(RL_BACK, stance_duration, STRAIGHT, stanceDepth, 0);

            // standing = false;

            phase = 0;
            delay = 500;
        break;

        /* case 2:
            if (!legMovers[FL]->swingPhase) legMovers[FL]->moveLegPosition(FL_FRONT, stance_duration, SWING, swingHeight, 0);
            if (!legMovers[RR]->swingPhase) legMovers[RR]->moveLegPosition(RR_FRONT, stance_duration, SWING, swingHeight, 0);

            if (!legMovers[FR]->straightPhase) legMovers[FR]->moveLegPosition(FR_BACK, stance_duration, STRAIGHT, stanceDepth, 0);
            if (!legMovers[RL]->straightPhase) legMovers[RL]->moveLegPosition(RL_BACK, stance_duration, STRAIGHT, stanceDepth, 0);

            // standing = false;

            phase = 0;
        break; */
    }
}

void TrotGait::backward()
{
    switch (phase)
    {
        case 0:
            if (!legMovers[FR]->swingPhase) legMovers[FR]->moveLegPosition(FR_BACK, stance_duration, SWING, swingHeight, 0);
            if (!legMovers[RL]->swingPhase) legMovers[RL]->moveLegPosition(RL_BACK, stance_duration, SWING, swingHeight, 0);

            if (!legMovers[FL]->straightPhase) legMovers[FL]->moveLegPosition(FL_FRONT, stance_duration, STRAIGHT, stanceDepth, 0);
            if (!legMovers[RR]->straightPhase) legMovers[RR]->moveLegPosition(RR_FRONT, stance_duration, STRAIGHT, stanceDepth, 0);

            // standing = false;

            phase = 1;
            delay = 500;

        break;

        case 1:
            if (!legMovers[FL]->swingPhase) legMovers[FL]->moveLegPosition(FL_BACK, stance_duration, SWING, swingHeight, 0);
            if (!legMovers[RR]->swingPhase) legMovers[RR]->moveLegPosition(RR_BACK, stance_duration, SWING, swingHeight, 0);

            if (!legMovers[FR]->straightPhase) legMovers[FR]->moveLegPosition(FR_FRONT, stance_duration, STRAIGHT, stanceDepth, 0);
            if (!legMovers[RL]->straightPhase) legMovers[RL]->moveLegPosition(RL_FRONT, stance_duration, STRAIGHT, stanceDepth, 0);

            // standing = false;

            phase = 0;
            delay = 500;
        break;
    }
}

void TrotGait::right()
{
    switch (phase)
    {
        case 0:
            if (!legMovers[FR]->swingPhase) legMovers[FR]->moveLegPosition(FR_SIDE_RIGHT, stance_duration, SWING, swingHeight, 0);
            if (!legMovers[RL]->swingPhase) legMovers[RL]->moveLegPosition(RL_SIDE_RIGHT, stance_duration, SWING, swingHeight, 0);

            if (!legMovers[FL]->straightPhase) legMovers[FL]->moveLegPosition(FL_SIDE_LEFT, stance_duration, STRAIGHT, stanceDepth, 0);
            if (!legMovers[RR]->straightPhase) legMovers[RR]->moveLegPosition(RR_SIDE_LEFT, stance_duration, STRAIGHT, stanceDepth, 0);

            // standing = false;

            phase++;
            if (phase == 2)
                phase = 0;

        break;

        case 1:
            if (!legMovers[FL]->swingPhase) legMovers[FL]->moveLegPosition(FL_SIDE_RIGHT, stance_duration, SWING, swingHeight, 0);
            if (!legMovers[RR]->swingPhase) legMovers[RR]->moveLegPosition(RR_SIDE_RIGHT, stance_duration, SWING, swingHeight, 0);

            if (!legMovers[FR]->straightPhase) legMovers[FR]->moveLegPosition(FR_SIDE_LEFT, stance_duration, STRAIGHT, stanceDepth, 0);
            if (!legMovers[RL]->straightPhase) legMovers[RL]->moveLegPosition(RL_SIDE_LEFT, stance_duration, STRAIGHT, stanceDepth, 0);

            // standing = false;

            phase++;
            if (phase == 2)
                phase = 0;
        break;
    }
}

void TrotGait::jump()
{
    
    switch (phase)
    {
        case 0:
             if (!legMovers[FR]->straightPhase) legMovers[FR]->moveLegPosition(FR_BENT, 1000, STRAIGHT, stanceDepth, 0);
             if (!legMovers[FL]->straightPhase) legMovers[FL]->moveLegPosition(FL_BENT, 1000, STRAIGHT, stanceDepth, 0);
             if (!legMovers[RR]->straightPhase) legMovers[RR]->moveLegPosition(RR_BENT, 1000, STRAIGHT, stanceDepth, 0);
             if (!legMovers[RL]->straightPhase) legMovers[RL]->moveLegPosition(RL_BENT, 1000, STRAIGHT, stanceDepth, 0);
            // standing = false; 
            // std::cout<<"I Bent"<<std::endl;
             phase++;
             if (phase == 4)
                phase = 0;

        break;

        case 1:
            if (!legMovers[FR]->straightPhase) legMovers[FR]->moveLegPosition(FR_JUMP, stance_duration, STRAIGHT, swingHeight, 0);
            if (!legMovers[FL]->straightPhase) legMovers[FL]->moveLegPosition(FL_JUMP, stance_duration, STRAIGHT, swingHeight, 0);
            if (!legMovers[RR]->straightPhase) legMovers[RR]->moveLegPosition(RR_JUMP, stance_duration, STRAIGHT, swingHeight, 0);
            if (!legMovers[RL]->straightPhase) legMovers[RL]->moveLegPosition(RL_JUMP, stance_duration, STRAIGHT, swingHeight, 0);
            // std::cout<<"I jumped"<<std::endl;
            phase++;
            if (phase == 4)
               phase = 0;
        break;

         case 2:
            if (!legMovers[FR]->straightPhase) legMovers[FR]->moveLegPosition(FR_MIDDLE, 200, STRAIGHT, swingHeight, 0);
            if (!legMovers[FL]->straightPhase) legMovers[FL]->moveLegPosition(FL_MIDDLE, 200, STRAIGHT, swingHeight, 0);
            if (!legMovers[RR]->straightPhase) legMovers[RR]->moveLegPosition(RR_MIDDLE, 200, STRAIGHT, swingHeight, 0);
            if (!legMovers[RL]->straightPhase) legMovers[RL]->moveLegPosition(RL_MIDDLE, 200, STRAIGHT, swingHeight, 0);
            // std::cout<<"I jumped"<<std::endl;
            phase++;
            if (phase == 4)
               phase = 0;
        break;       

        case 3:
            if (!legMovers[FR]->straightPhase) legMovers[FR]->moveLegPosition(FR_STAND, 1000, STRAIGHT, swingHeight, 0);
            if (!legMovers[FL]->straightPhase) legMovers[FL]->moveLegPosition(FL_STAND, 1000, STRAIGHT, swingHeight, 0);
            if (!legMovers[RR]->straightPhase) legMovers[RR]->moveLegPosition(RR_STAND, 1000, STRAIGHT, swingHeight, 0);
            if (!legMovers[RL]->straightPhase) legMovers[RL]->moveLegPosition(RL_STAND, 1000, STRAIGHT, swingHeight, 0);
             
            // std::cout<<"I stood"<<std::endl;
            phase++;
            if (phase == 4)
               phase = 0;
            gaitMotion = GAIT_MOTION_NUM;
        break;

    
         
   }
}

void TrotGait::left()
{
    switch (phase)
    {
        case 0:
            if (!legMovers[FR]->swingPhase) legMovers[FR]->moveLegPosition(FR_SIDE_LEFT, stance_duration, SWING, swingHeight, 0);
            if (!legMovers[RL]->swingPhase) legMovers[RL]->moveLegPosition(RL_SIDE_LEFT, stance_duration, SWING, swingHeight, 0);

            if (!legMovers[FL]->straightPhase) legMovers[FL]->moveLegPosition(FL_SIDE_RIGHT, stance_duration, STRAIGHT, stanceDepth, 0);
            if (!legMovers[RR]->straightPhase) legMovers[RR]->moveLegPosition(RR_SIDE_RIGHT, stance_duration, STRAIGHT, stanceDepth, 0);

            // standing = false;

            phase++;
            if (phase == 2)
                phase = 0;

        break;

        case 1:
            if (!legMovers[FL]->swingPhase) legMovers[FL]->moveLegPosition(FL_SIDE_LEFT, stance_duration, SWING, swingHeight, 0);
            if (!legMovers[RR]->swingPhase) legMovers[RR]->moveLegPosition(RR_SIDE_LEFT, stance_duration, SWING, swingHeight, 0);

            if (!legMovers[FR]->straightPhase) legMovers[FR]->moveLegPosition(FR_SIDE_RIGHT, stance_duration, STRAIGHT, stanceDepth, 0);
            if (!legMovers[RL]->straightPhase) legMovers[RL]->moveLegPosition(RL_SIDE_RIGHT, stance_duration, STRAIGHT, stanceDepth, 0);

            // standing = false;

            phase++;
            if (phase == 2)
                phase = 0;
        break;
    }
}


TrotGait::~TrotGait()
{
    
}