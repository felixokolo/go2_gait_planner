/*
 * FakePublisher.cpp
 *
 *  Created on: 23 Jun 2024
 *      Author: Felix
 */

#include "Low_level/FakePublisher.h"

FakePublisher::FakePublisher() :
Node("fake_publisher")
{
    lowState_pub = this->create_publisher<unitree_go::msg::LowState>(joint_state_topic, 10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(5), std::bind(&FakePublisher::fakeStatePublisher, this));
}

void FakePublisher::fakeStatePublisher()
{
    //std::cout << "Fake publisher publishing fake states" << std::endl;
    for (int i = 0; i < 12; i++)
    {
        lowState_pub->publish(lowStateMsg);
    }
}