/*
 * FakePublisher.h
 *
 *  Created on: 23 Jun 2024
 *      Author: Felix
 */

#ifndef FAKE_PUBLISHER
#define FAKE_PUBLISHER

#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg/low_state.hpp"

/**
 * @class FakePublisher
 * @brief A class that simulates the publishing of a robot's low-level state.
 *
 * The FakePublisher class is used to simulate a publisher node in ROS2, which
 * periodically publishes fake data representing the robot's low-level state.
 */
class FakePublisher : public rclcpp::Node
{
public:
    /**
     * @brief Constructor for the FakePublisher class.
     *
     * Initializes the ROS2 node and sets up the timer for periodic publishing
     * of the low-level state.
     */
    FakePublisher();

private:
    /**
     * @brief Publishes a fake low-level state message.
     *
     * This method is periodically called by a timer to publish a simulated
     * low-level state message to a specific ROS2 topic.
     */
    void fakeStatePublisher();

    unitree_go::msg::LowState lowStateMsg; /**< The message object for storing and publishing the low-level state. */

    // Publishers and Subscribers
    rclcpp::Publisher<unitree_go::msg::LowState>::SharedPtr lowState_pub; /**< ROS2 Publisher for the low-level state message. */
    rclcpp::TimerBase::SharedPtr timer_;                                  /**< Timer to periodically trigger the fakeStatePublisher function. */
    std::string joint_state_topic = "/lowstate";                          /**< The topic name where the low-level state messages are published. */
};

#endif // FAKE_PUBLISHER