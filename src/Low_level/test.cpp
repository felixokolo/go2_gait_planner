#include <iostream>
#include <cstdlib>
#include <vector>
#include "Quadruped/Robot.h"

#include "Low_level/LegMover.h"
#include <eigen3/unsupported/Eigen/Splines>



int main(int argc, char **argv) {

    rclcpp::init(argc, argv); 
	Robot myRobot;
	auto legMover = std::make_shared<LegMover>(&myRobot, "Test_Node");



    /* std::vector<double> jointAngles({
        -0.13000117242336273,
        1.2262111902236938,
        -2.724439859390259,
        0.12999489903450012,
        1.226211428642273,
        -2.724479913711548,
        -0.48181259632110596,
        1.2460269927978516,
        -2.7230396270751953,
        0.4821029305458069,
        1.245980143547058,
        -2.7229092121124268
}); */



    std::vector<double> jointAngles({
        -0.0264951,
        0.998995,
        -1.77479,
        -0.0244377,
        0.999868,
        -1.77521,
        -0.0183021,
        0.9962,
        -1.7774,
        -0.0320323,
        0.995105,
        -1.77852
    });
    
    myRobot.setAngles(jointAngles);
    for (int i = 0; i < 12; i++)
        legMover->qTarg[i] = jointAngles[i];
    //legMover->moveLegStraight(LegType::FL, Eigen::Vector3d(0, 0, -0.1), 1000);
    legMover->moveLegSwing(LegType::FL, Eigen::Vector3d(std::stof(argv[1]), std::stof(argv[2]), std::stof(argv[3])), std::stof(argv[4]), 1000);
    while (true)
    //legMover->straightMover();
    legMover->swingMover();
    
	
	return 0;
}