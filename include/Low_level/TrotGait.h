/*
 * TrotGait.h
 *
 *  Created on: 16 Aug 2024
 *      Author: Felix
 */

#ifndef TROT_GAIT
#define TROT_GAIT

#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "Low_level/BaseGait.h"

#include <eigen3/unsupported/Eigen/Splines>

#define REAR_OFFSET -0.00
#define SWING_DISTANCE 0.1

/* #define FR_FRONT {0.27, -0.14, -NOMINAL_HEIGHT}
#define FR_BACK {0.07, -0.14, -NOMINAL_HEIGHT}

#define FL_FRONT {0.29, 0.147, -NOMINAL_HEIGHT}
#define FL_BACK {0.09, 0.147, -NOMINAL_HEIGHT}

#define RR_FRONT {-0.09, -0.145, -NOMINAL_HEIGHT + REAR_OFFSET}
#define RR_BACK {-0.29, -0.145, -NOMINAL_HEIGHT + REAR_OFFSET}

#define RL_FRONT {-0.11, 0.145, -NOMINAL_HEIGHT + REAR_OFFSET}
#define RL_BACK {-0.31, 0.145, -NOMINAL_HEIGHT + REAR_OFFSET} */

/* #define SIT_POS {\
                 0.22244, -0.14438, -0.103629,\
                 0.221357, 0.144401, -0.103878,\
                 -0.216491, -0.195797, -0.103444,\
                 -0.216367, 0.195798, -0.103516\

                } */

/* #define FR_BENT {0.269434, -0.139713, -0.1}
#define FL_BENT {0.269236, 0.147767, -0.1}
#define RR_BENT {-0.119182, -0.113354, -0.1}
#define RL_BENT {-0.118096, 0.144176, -0.1} */

#define FR_BENT {0.219434, -0.145806, -0.1}
#define FL_BENT {0.219236, 0.145221, -0.1}
#define RR_BENT {-0.169182, -0.166028, -0.09}
#define RL_BENT {-0.168096, 0.166349, -0.09}

#define FR_JUMP {0.119434, -0.139713, -NOMINAL_HEIGHT - 0.1}
#define FL_JUMP {0.119236, 0.147767, -NOMINAL_HEIGHT - 0.1}
#define RR_JUMP {-0.269182, -0.113354, -NOMINAL_HEIGHT - 0.1}
#define RL_JUMP {-0.268096, 0.144176, -NOMINAL_HEIGHT - 0.1}

#define FR_FRONT {0.22, -0.14, -NOMINAL_HEIGHT}
#define FR_BACK {0.12, -0.14, -NOMINAL_HEIGHT}

#define FL_FRONT {0.24, 0.147, -NOMINAL_HEIGHT}
#define FL_BACK {0.14, 0.147, -NOMINAL_HEIGHT}

#define RR_FRONT {-0.14, -0.145, -NOMINAL_HEIGHT + REAR_OFFSET}
#define RR_BACK {-0.24, -0.145, -NOMINAL_HEIGHT + REAR_OFFSET}

#define RL_FRONT {-0.16, 0.145, -NOMINAL_HEIGHT + REAR_OFFSET}
#define RL_BACK {-0.26, 0.145, -NOMINAL_HEIGHT + REAR_OFFSET}

#define FR_SIDE_RIGHT {0.17, -0.19, -NOMINAL_HEIGHT}
#define FR_SIDE_LEFT {0.17, -0.09, -NOMINAL_HEIGHT}

#define FL_SIDE_RIGHT {0.19, 0.097, -NOMINAL_HEIGHT}
#define FL_SIDE_LEFT {0.19, 0.197, -NOMINAL_HEIGHT}

#define RR_SIDE_RIGHT {-0.19, -0.195, -NOMINAL_HEIGHT + REAR_OFFSET}
#define RR_SIDE_LEFT {-0.19, -0.095, -NOMINAL_HEIGHT + REAR_OFFSET}

#define RL_SIDE_RIGHT {-0.21, 0.095, -NOMINAL_HEIGHT + REAR_OFFSET}
#define RL_SIDE_LEFT {-0.21, 0.195, -NOMINAL_HEIGHT + REAR_OFFSET}

#define FR_MIDDLE {0.169964 + 0.05, -0.139713, -NOMINAL_HEIGHT * 0.75}
#define FL_MIDDLE {0.190386 + 0.05, 0.147767, -NOMINAL_HEIGHT * 0.75}
#define RR_MIDDLE {-0.189777 + 0.05, -0.113354, -NOMINAL_HEIGHT * 0.75}
#define RL_MIDDLE {-0.209861 + 0.05, 0.144176, -NOMINAL_HEIGHT * 0.75}

/**
 * @class TrotGait
 * @brief Implements the trot gait for a quadruped robot.
 *
 * The TrotGait class is derived from the BaseGait class and provides specific implementations for
 * the trot gait motion. This class manages the different phases of the trot gait, including forward,
 * backward, left, right, standing, and jumping motions.
 */

class TrotGait : public BaseGait
{
public:
    /**
     * @brief Constructs a TrotGait instance.
     *
     * @param robotModel Pointer to the Robot object used for managing robot state.
     * @param nodeName Name of the ROS 2 node.
     */
    TrotGait(Robot *robotModel, std::string nodeName);
    /**
     * @brief Destructs the TrotGait instance.
     */
    ~TrotGait();

private:
    Robot *robotModel; /**< Pointer to the Robot object used for managing robot state. */

    /**
     * @brief Callback function for gait updates.
     *
     * This function is responsible for updating the gait state based on the current phase and
     * the requested motion.
     */
    void gaitCallback();

    /**
     * @brief Executes the forward trot gait.
     *
     * This method calculates and applies the necessary leg movements for the forward trot motion.
     */
    void forward();

    /**
     * @brief Executes the backward trot gait.
     *
     * This method calculates and applies the necessary leg movements for the backward trot motion.
     */
    void backward();

    /**
     * @brief Executes the left turn trot gait.
     *
     * This method calculates and applies the necessary leg movements for the left turn trot motion.
     */
    void left();

    /**
     * @brief Executes the right turn trot gait.
     *
     * This method calculates and applies the necessary leg movements for the right turn trot motion.
     */
    void right();

    /**
     * @brief Executes the stand position for the trot gait.
     *
     * This method calculates and applies the necessary leg movements to bring the robot to a standing position.
     */
    void stand();

    /**
     * @brief Executes the jump motion for the trot gait.
     *
     * This method calculates and applies the necessary leg movements for the jump motion.
     */
    void jump();
    /* void gaitParamsCallback(go2_gait_planner::msg::GaitParam::SharedPtr gaitParamsMsg);
    void setStanceDuration(int val);
    void setStanceDepth(float val);
    void setSwingHeight(float val);
    void setTrotMotion(TrotMotion val);
    void publishLowCmd() override; */

    // rclcpp::TimerBase::SharedPtr gaitTimer_;
    // rclcpp::Subscription<go2_gait_planner::msg::GaitParam>::SharedPtr gait_params_sub;

    // std::string gait_params_topic = "/go2_gait_planner/gait_msg";
};

#endif // TROT_GAIT