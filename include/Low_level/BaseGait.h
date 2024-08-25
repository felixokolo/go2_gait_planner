/*
 * BaseGait.h
 *
 *  Created on: 16 Aug 2024
 *      Author: Felix
 */

#ifndef BASE_GAIT
#define BASE_GAIT

#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

#include "Quadruped/Robot.h"
#include "Low_level/BodyMover.h"

#include <eigen3/unsupported/Eigen/Splines>

/**
 * @enum GaitMotion
 * @brief Enumeration of different gait motions for the robot.
 */

enum GaitMotion
{
    STOP,
    FORWARD,
    BACKWARD,
    LEFT,
    RIGHT,
    JUMP,
    GAIT_MOTION_NUM
};

/**
 * @class BaseGait
 * @brief Base class for implementing different gait patterns for a quadruped robot.
 *
 * The BaseGait class is responsible for managing and executing different gait patterns
 * such as walking forward, backward, turning left, right, or jumping. It acts as a base
 * class that can be extended to implement specific gait behaviors.
 */

class BaseGait : public BodyMover
{
public:
    /**
     * @brief Constructor for the BaseGait class.
     * @param robotModel Pointer to the Robot model used for the gait.
     * @param nodeName Name of the ROS2 node associated with this gait.
     */
    BaseGait(Robot *robotModel, std::string nodeName);
    /**
     * @brief Destructor for the BaseGait class.
     */
    ~BaseGait();

protected:
    int stance_duration = 300;    /**< Duration of the stance phase in milliseconds. */
    float swingHeight = 0.05;     /**< Height of the swing phase in meters. */
    float stanceDepth = -0.0;     /**< Depth of the stance phase in meters. */
    int phase = 0;                /**< Current phase of the gait cycle. */
    long long gaitStartTime = 0;  /**< Start time of the gait cycle. */
    long long delay = 0;          /**< Delay the next phase from starting */
    GaitMotion gaitMotion = STOP; /**< Current gait motion being executed. */

    /**
     * @brief Virtual function to handle gait-specific callbacks.
     *
     * This function should be overridden by derived classes to implement specific gait behaviors.
     */
    virtual void gaitCallback() = 0;

private:
    Robot *robotModel; /**< Pointer to the Robot model associated with this gait. */

    /**
     * @brief Callback function for gait parameter updates.
     * @param gaitParamsMsg Shared pointer to the GaitParam message containing updated parameters.
     */
    void gaitParamsCallback(go2_gait_planner::msg::GaitParam::SharedPtr gaitParamsMsg);

    /**
     * @brief Set the duration of the stance phase.
     * @param val Duration in milliseconds.
     */
    void setStanceDuration(int val);

    /**
     * @brief Set the depth of the stance phase.
     * @param val Depth in meters.
     */
    void setStanceDepth(float val);

    /**
     * @brief Set the height of the swing phase.
     * @param val Height in meters.
     */
    void setSwingHeight(float val);

    /**
     * @brief Set the current gait motion.
     * @param val The GaitMotion value to set.
     */
    void setGaitMotion(GaitMotion val);

    void publishLowCmd() override;

    rclcpp::TimerBase::SharedPtr gaitTimer_;
    rclcpp::Subscription<go2_gait_planner::msg::GaitParam>::SharedPtr gait_params_sub;

    /**< Topic name for gait parameter updates. */
    std::string gait_params_topic = "/go2_gait_planner/gait_msg";
};

#endif // BASE_GAIT