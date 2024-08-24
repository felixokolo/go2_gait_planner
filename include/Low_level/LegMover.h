/*
 * LegMover.h
 *
 *  Created on: 23 Jun 2024
 *      Author: Felix
 */

#ifndef LEG_MOVER
#define LEG_MOVER

#include <iostream>
#include <mutex>

#include "rclcpp/rclcpp.hpp"

#include "Quadruped/Robot.h"

#include <eigen3/unsupported/Eigen/Splines>

#define PUB_RATE 1000
#define ANGLE_RES 50

/**
 * @enum MotionType
 * @brief Specifies the type of motion for the leg movement.
 *
 * This enumeration defines the different types of motion that can be applied
 * to the leg: straight motion and swinging motion.
 */

enum MotionType
{
    STRAIGHT = 0,
    SWING,
    MOTION_TYPE_NUM
};

/**
 * @enum OrientCntrlLegs
 * @brief Defines the pairs of legs for orientation control.
 *
 * This enumeration is used to specify the pairs of legs for controlling
 * their orientation together.
 */
enum OrientCntrlLegs
{
    FL_RR = 0,
    FR_RL
};

class LegMover
{
public:
    /**
     * @brief Constructor for the LegMover class.
     *
     * @param leg Pointer to the Leg object associated with this LegMover.
     * @param qTarg Iterator to the target joint angles vector.
     */
    LegMover(Leg *leg, std::vector<double>::iterator qTarg);

    int straightPhase = 0; /**< The current phase of the straight movement. */
    int swingPhase = 0;    /**< The current phase of the swinging movement. */

    /**
     * @brief Moves the leg to a specified position with a given motion type.
     *
     * @param position Target position for the leg.
     * @param duration Duration of the movement in milliseconds.
     * @param motionType Type of motion to be applied (e.g., straight or swing).
     * @param swingHeight Height of the swing if applicable.
     * @param phaseOffset Offset for the motion phase.
     */
    void moveLegPosition(Eigen::Vector3d position, int duration, MotionType motionType, float swingHeight, int phaseOffset);

    /**
     * @brief Moves the leg in a straight direction.
     *
     * @param direction Direction vector for the straight movement.
     * @param duration Duration of the movement in milliseconds.
     * @param phaseOffset Offset for the movement phase.
     */
    void moveLegStraight(Eigen::Vector3d direction, int duration, int phaseOffset);
    /**
     * @brief Moves the leg in a swinging motion.
     *
     * @param direction Direction vector for the swing.
     * @param swingHeight Height of the swing.
     * @param duration Duration of the movement in milliseconds.
     * @param phaseOffset Offset for the swing phase.
     */

    void moveLegSwing(Eigen::Vector3d direction, float swingHeight, int duration, int phaseOffset);
    /**
     * @brief Updates the leg's movement based on the current phase and motion type.
     */
    void mover();

private:
    Leg *leg;                                /**< Pointer to the Leg object associated with this LegMover. */
    int phaseOffset = 0;                     /**< Offset for the motion phase. */
    MotionType motionType = MOTION_TYPE_NUM; /**< The type of motion being applied. */

    int waitTime = 0;                        /**< Time to wait before starting the next phase. */
    int countDown = 0;                       /**< Countdown timer for phase transitions. */
    Eigen::Vector3d straightDirection_d;     /**< Delta direction for straight movement. */
    Eigen::Vector3d swingDirection_d;        /**< Delta direction for swinging movement. */
    Eigen::Vector3d F_Ext;                   /**< External forces applied to the leg. */
    std::vector<double>::iterator leg_qTarg; /**< Iterator to the target joint angles. */
    Eigen::Vector3d cmdTaus;                 /**< Command torques for the leg joints. */
    Eigen::Vector3d targPos;                 /**< Target position for the leg. */
    double curSwing = 0;                     /**< Current swing height. */
    unsigned int swing_dir = 0;              /**< Direction of the swing. */
    Eigen::Spline<double, 1> swingSpline;    /**< Spline representation of the swing trajectory. */

    std::mutex guard_mutex; /**< Mutex for synchronizing access to shared data. */

    /**
     * @brief Retrieves the current target joint angles.
     *
     * @return The current target joint angles as an Eigen::Vector3d.
     */
    Eigen::Vector3d get_qTarg();

    /**
     * @brief Sets the target joint angles.
     *
     * @param angles The target joint angles to be set.
     */
    void set_qTarg(Eigen::Vector3d angles);

    /**
     * @brief Handles the movement of the leg in a straight direction.
     */
    void straightMover();

    /**
     * @brief Handles the swinging movement of the leg.
     */
    void swingMover();

    /*



     */

    /* OrientCntrlLegs curOrientCntrlLeg = FL_RR;

    LegType straightLegType;
    LegType swingLegType;


    Eigen::Vector3d F_Ext;
     */

    /*
    void paramsCallback(go2_scratch::msg::ParamsSet::SharedPtr paramsMsg);
    void controllLoop();
    void mover();
    void orientControllLoop(); */

    // Publishers and Subscribers
    /*
    rclcpp::TimerBase::SharedPtr straightTimer_;
    rclcpp::TimerBase::SharedPtr swingTimer_;

    rclcpp::TimerBase::SharedPtr controllLoopTimer;



    rclcpp::Subscription<go2_scratch::msg::ParamsSet>::SharedPtr params_sub;

     */
};

#endif // LEG_MOVER