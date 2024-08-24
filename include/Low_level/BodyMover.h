/*
 * BodyMover.h
 *
 *  Created on: 04 Aug 2024
 *      Author: Felix
 */

#ifndef BODY_MOVER
#define BODY_MOVER

#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

#include "Quadruped/Robot.h"
#include "Low_level/StateMonitor.h"
#include "go2_gait_planner/msg/move_leg.hpp"
#include "go2_gait_planner/msg/params_set.hpp"
#include "Low_level/LegMover.h"

#include <eigen3/unsupported/Eigen/Splines>

#define NOMINAL_HEIGHT 0.25

/* #define SIT_POS {\
                 0.22244, -0.14438, -0.103629,\
                 0.221357, 0.144401, -0.103878,\
                 -0.216491, -0.195797, -0.103444,\
                 -0.216367, 0.195798, -0.103516\
                 }
#define STAND_POS {\
                   0.176581, -0.132595, -NOMINAL_HEIGHT,\
                   0.181651, 0.147318, -NOMINAL_HEIGHT,\
                   -0.211777, -0.135967, -NOMINAL_HEIGHT,\
                   -0.206297, 0.148259, -NOMINAL_HEIGHT\
                   } */

#define SIT_POS {               \
    0.219434, -0.145806, -0.1,  \
    0.219236, 0.145221, -0.1,   \
    -0.169182, -0.166028, -0.1, \
    -0.168096, 0.166349, -0.1}

//
#define STAND_POS {                        \
    0.169964, -0.139713, -NOMINAL_HEIGHT,  \
    0.190386, 0.147767, -NOMINAL_HEIGHT,   \
    -0.189777, -0.113354, -NOMINAL_HEIGHT, \
    -0.209861, 0.144176, -NOMINAL_HEIGHT}
/*
#define FR_SIT {0.219434, -0.145806, -0.1}
#define FL_SIT {0.219236, 0.145221, -0.1}
#define RR_SIT {-0.169182, -0.166028, -0.1}
#define RL_SIT {-0.168096, 0.166349, -0.1}

#define FR_STAND {0.169964, -0.139713, -NOMINAL_HEIGHT}
#define FL_STAND {0.190386, 0.147767, -NOMINAL_HEIGHT}
#define RR_STAND {-0.189777, -0.113354, -NOMINAL_HEIGHT}
#define RL_STAND {-0.209861, 0.144176, -NOMINAL_HEIGHT} */

#define FR_SIT {0.219434, -0.145806, -0.0897835}
#define FL_SIT {0.219236, 0.145221, -0.0926759}
#define RR_SIT {-0.169182, -0.166028, -0.0664779}
#define RL_SIT {-0.168096, 0.166349, -0.0652987}

#define FR_STAND {0.192498, -0.134639, -NOMINAL_HEIGHT}
#define FL_STAND {0.192429, 0.135241, -NOMINAL_HEIGHT}
#define RR_STAND {-0.191475, -0.139506, -NOMINAL_HEIGHT}
#define RL_STAND {-0.187648, 0.139064, -NOMINAL_HEIGHT}

#define STAND_SIT_DURATION 1000

/**
 * @enum StandSitPos
 * @brief Enumeration to represent the possible positions of the robot (sitting or standing).
 */
enum StandSitPos
{
    SIT,         /**< Sitting position */
    STAND,       /**< Standing position */
    SITSTAND_NUM /**< Number of positions available (sitting and standing) */
};

/**
 * @class BodyMover
 * @brief Class for managing the body movements of a quadruped robot.
 *
 * The BodyMover class is responsible for controlling the robot's transitions
 * between different body positions, such as sitting and standing. It extends
 * the StateMonitor class and handles the movement of individual legs through LegMover instances.
 */

class BodyMover : public StateMonitor
{
public:
    /**
     * @brief Constructor for the BodyMover class.
     * @param robotModel Pointer to the Robot model used for movement control.
     * @param nodeName Name of the ROS2 node associated with this body mover.
     */
    BodyMover(Robot *robotModel, std::string nodeName);
    /**
     * @brief Destructor for the BodyMover class.
     */
    ~BodyMover();
    LegMover *legMovers[LEG_NUM]; /**< Array of LegMover pointers, one for each leg. */
    bool standing = false;        /**< Boolean flag indicating if the robot is standing. */

protected:
    float nominalHeight = NOMINAL_HEIGHT; /**< Nominal height of the robot when standing. */

    /**
     * @brief Publishes low-level commands to control the robot's joints.
     */
    void publishLowCmd() override;

private:
    Robot *robotModel; /**< Pointer to the Robot model associated with this body mover. */
    /* std::shared_ptr<LegMover> FR_legMover;
    std::shared_ptr<LegMover> FL_legMover;
    std::shared_ptr<LegMover> RR_legMover;
    std::shared_ptr<LegMover> RL_legMover; */
    LegMover *FR_legMover; /**< LegMover instance for the front-right leg. */
    LegMover *FL_legMover; /**< LegMover instance for the front-left leg. */
    LegMover *RR_legMover; /**< LegMover instance for the rear-right leg. */
    LegMover *RL_legMover; /**< LegMover instance for the rear-left leg. */

    /**
     * @brief Callback function for processing movement commands.
     * @param moveMsg Shared pointer to the MoveLeg message containing movement commands.
     */
    void moverCallback(go2_gait_planner::msg::MoveLeg::SharedPtr moveMsg);

    /**
     * @brief Callback function for processing stand/sit commands.
     * @param standSitMsg Shared pointer to the Int32 message indicating the stand/sit command.
     */
    void standSitCallback(std_msgs::msg::Int32::SharedPtr standSitMsg);

    Eigen::Vector<double, 12> sitPos;   /**< Vector representing the robot's sitting position. */
    Eigen::Vector<double, 12> standPos; /**< Vector representing the robot's standing position. */
    bool standCmd = false;              /**< Boolean flag indicating if the stand command has been issued. */

    rclcpp::TimerBase::SharedPtr FR_moveTimer_; /**< ROS2 Timer for triggering front-right leg movements. */
    rclcpp::TimerBase::SharedPtr FL_moveTimer_; /**< ROS2 Timer for triggering front-left leg movements. */
    rclcpp::TimerBase::SharedPtr RR_moveTimer_; /**< ROS2 Timer for triggering rear-right leg movements. */
    rclcpp::TimerBase::SharedPtr RL_moveTimer_; /**< ROS2 Timer for triggering rear-left leg movements. */

    rclcpp::Subscription<go2_gait_planner::msg::MoveLeg>::SharedPtr mover_sub; /**< ROS2 Subscription for receiving movement commands. */
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr standSit_sub;   /**< ROS2 Subscription for receiving stand/sit commands. */
    // rclcpp::Subscription<go2_gait_planner::msg::ParamsSet>::SharedPtr params_sub override;

    std::string mover_topic = "/go2_gait_planner/move";         /**< ROS2 topic for movement commands. */
    std::string standSit_topic = "/go2_gait_planner/stand_sit"; /**< ROS2 topic for stand/sit commands. */
};

#endif // BODY_MOVER