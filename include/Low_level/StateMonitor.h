/*
 * StateMonitor.h
 *
 *  Created on: 23 Jun 2024
 *      Author: Felix
 */

#ifndef STATEMONITOR
#define STATEMONITOR

#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>
#include <exception>

#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg/low_state.hpp"
#include "unitree_go/msg/low_cmd.hpp"
#include "go2_scratch/msg/move_leg.hpp"
#include "go2_scratch/msg/params_set.hpp"
#include "go2_scratch/msg/gait_param.hpp"
#include "go2_scratch/msg/joints_set.hpp"
#include <std_msgs/msg/string.hpp>
#include "motor_crc.h"

#define MAX_PHASE_VALUE 50

#include "Quadruped/Robot.h"

class StateMonitor : public rclcpp::Node
{
public:
    /**
     * @brief Constructs a StateMonitor instance.
     *
     * @param robotModel Pointer to the Robot object used for managing robot state.
     * @param nodeName Name of the ROS 2 node.
     */
    StateMonitor(Robot *robotModel, std::string nodeName);
    std::vector<double> qTarg; /**< Target joint angles for the robot. */

protected:
    unitree_go::msg::LowCmd lowCmdMsg; /**< Message to publish low-level commands. */

    /**
     * @brief Publishes the low-level command message.
     */
    virtual void publishLowCmd();

    long long curTime = 0;   /**< Current time in milliseconds. */
    long long startTime = 0; /**< Start time for the operation in milliseconds. */

    bool paramsNotSet = true;  /**< Flag indicating whether parameters are set. */
    bool firstRun = true;      /**< Flag indicating whether this is the first run. */
    bool writeFile = false;    /**< Flag to enable or disable file writing. */
    Eigen::Vector3d targetPos; /**< Target position for the robot. */

    /**
     * @brief Computes CRC32 checksum.
     *
     * @param ptr Pointer to the data buffer.
     * @param len Length of the data buffer.
     * @return Computed CRC32 checksum.
     */
    uint32_t crc32_core(uint32_t *ptr, uint32_t len);

    // Timers
    rclcpp::TimerBase::SharedPtr cmdPubTimer; /**< Timer for publishing commands. */

    std::string params_topic = "/go2_scratch/params"; /**< ROS 2 topic for parameters. */

    std::mutex guard_mutex; /**< Mutex for synchronizing access to shared data. */

private:
    Robot *robotModel;                                             /**< Pointer to the Robot object used for managing robot state. */
    const std::string imuFileName = "src/go2_scratch/imu_values_"; /**< Base name for IMU file output. */
    const std::string imuFileNameExt = ".txt";                     /**< Extension for IMU file output. */

    /**
     * @brief Callback function for state update messages.
     *
     * @param msg Shared pointer to the received LowState message.
     */
    void stateUpdateCallback(const unitree_go::msg::LowState::SharedPtr msg);

    /**
     * @brief Initializes command messages.
     */
    void init_cmd();

    /**
     * @brief Increments the phase value.
     */
    void phaseIncr();

    /**
     * @brief Converts quaternion to Euler angles.
     *
     * @param quaternions Vector of quaternion values.
     * @return Vector of Euler angles.
     */
    std::vector<float> quatToEuler(std::vector<float> quaternions);
    /*
     * @brief Callback function for joint set messages.
     *
     * @param jointsMsg Shared pointer to the received JointsSet message.
     */

    void jointsCallback(go2_scratch::msg::JointsSet::SharedPtr jointsMsg);
    /**
     * @brief Callback function for parameters set messages.
     *
     * @param paramsMsg Shared pointer to the received ParamsSet message.
     */
    void paramsCallback(go2_scratch::msg::ParamsSet::SharedPtr paramsMsg);

    int legType = 0;                  /**< Type of leg currently being processed. */
    const long long simTime = 200000; /**< Simulation time in milliseconds (20s). */

    /**
     * @brief Writes diagnostic data to a file.
     */
    void writeToFile();

    uint32_t phase = 0; /**< Current phase of the robot's operation. */

    // Publishers and Subscribers
    rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr lowState_sub;
    rclcpp::Publisher<unitree_go::msg::LowCmd>::SharedPtr lowCmd_pub;
    rclcpp::Subscription<go2_scratch::msg::JointsSet>::SharedPtr joints_sub;
    rclcpp::Subscription<go2_scratch::msg::ParamsSet>::SharedPtr params_sub;

    std::string joint_state_topic = "/lowstate";      /**< ROS 2 topic for joint state updates. */
    std::string joint_cmd_topic = "/lowcmd";          /**< ROS 2 topic for joint commands. */
    std::string joints_topic = "/go2_scratch/joints"; /**< ROS 2 topic for joints set messages. */
};

#endif // STATEMONITOR