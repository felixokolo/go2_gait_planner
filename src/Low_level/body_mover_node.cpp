/*
 * body_mover_node.cpp
 *
 *  Created on: 23 Jun 2024
 *      Author: Felix
 */

// Standard (system) header files
#include <iostream>
#include <cstdlib>
#include <vector>
#include "Quadruped/Robot.h"

#include "Low_level/BodyMover.h"
#include <eigen3/unsupported/Eigen/Splines>



int main(int argc, char **argv) {

	rclcpp::init(argc, argv); 
	Robot myRobot;
	auto moverNode = std::make_shared<BodyMover>(&myRobot, "body_mover");

	// rclcpp::spin(moverNode);

	rclcpp::executors::MultiThreadedExecutor executor;
	executor.add_node(moverNode);
	executor.spin();
	std::cout << "Passed out" << std::endl;
	rclcpp::shutdown();
	return 0;



	/* auto FL_pos = myRobot.getPosition_FL();
	std::cout 	<< "The current position of the Front-Left leg is " << std::endl
				<< FL_pos << std::endl
				<< "with respect to the robot's base" << std::endl;

	auto FR_pos = myRobot.getPosition_FR();
	std::cout 	<< "The current position of the Front-Right leg is " << std::endl
				<< FR_pos << std::endl
				<< "with respect to the robot's base" << std::endl;

	auto RL_pos = myRobot.getPosition_RL();
	std::cout 	<< "The current position of the Rear-Left leg is " << std::endl
				<< RL_pos << std::endl
				<< "with respect to the robot's base" << std::endl;

	auto RR_pos = myRobot.getPosition_RR();
	std::cout 	<< "The current position of the Rear-Right leg is " << std::endl
				<< RR_pos << std::endl
				<< "with respect to the robot's base" << std::endl; */

	/*Eigen::VectorXd x(5);
	x << 0, 1, 2, 3, 4;
	Eigen::VectorXd y(5);
	y << 0, 1, 4, 9, 16;

	Eigen::Spline<double, 1> spline = Eigen::SplineFitting<Eigen::Spline<double, 1>>::Interpolate(y.transpose(), 3, x);

	double x_interp = 2.5;
	Eigen::RowVectorXd y_interp = spline(x_interp);
	std::cout << "Interpolated value at " << x_interp << " is " << y_interp << std::endl;*/

return 0;
}
