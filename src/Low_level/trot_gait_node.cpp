/*
 * trot_gait_node.cpp
 *
 *  Created on: 16 Aug 2024
 *      Author: Felix
 */

// Standard (system) header files
#include <iostream>
#include <cstdlib>
#include <vector>
#include "Quadruped/Robot.h"

#include "Low_level/TrotGait.h"
#include <eigen3/unsupported/Eigen/Splines>



int main(int argc, char **argv) {

	rclcpp::init(argc, argv); 
	Robot myRobot;
	auto moverNode = std::make_shared<TrotGait>(&myRobot, "trot_gait");

	// rclcpp::spin(moverNode);

	rclcpp::executors::MultiThreadedExecutor executor;
	executor.add_node(moverNode);
	executor.spin();
	std::cout << "Passed out" << std::endl;
	rclcpp::shutdown();
	return 0;


return 0;
}
