/*
 * state_monitor_node.cpp
 *
 *  Created on: 23 Jun 2024
 *      Author: Felix
 */

// Standard (system) header files
#include <iostream>
#include <cstdlib>
#include <vector>
#include "Quadruped/Robot.h"
#include "Quadruped/Leg.h"
#include "Quadruped/Joint.h"
#include "Quadruped/Link.h"

#include "Low_level/FakePublisher.h"
#include "Low_level/StateMonitor.h"
#include <eigen3/unsupported/Eigen/Splines>
 


int main(int argc, char **argv) {

	rclcpp::init(argc, argv); 
	Robot myRobot;
	

	auto stateMonitorNode = std::make_shared<StateMonitor>(&myRobot, "state_monitor");

	rclcpp::spin(stateMonitorNode);
	rclcpp::shutdown();
	return 0;
}
