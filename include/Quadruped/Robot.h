/**
 *      @file:      Robot.h
 *
 *      @date:    2 Jun 2024  
      @author:    Priyanka
      @cheked:    Felix
 */

#ifndef ROBOT_H
#define ROBOT_H

#include <vector>
#include <iostream>
#include "Quadruped/Leg.h"
#include "Quadruped/Kinematics.h"

#define MAX_TAU_HIP 10
#define MAX_TAU_THIGH 10
#define MAX_TAU_CALF 5

/**
 * @class Robot
 * @brief Represents a quadruped robot.
 *
 * The Robot class encapsulates the properties and behaviors of a quadruped robot,
 * providing methods to manipulate and retrieve information about its legs, orientation, and control signals.
 */

class Robot
{
public:
    /**
     * @brief Default constructor for the Robot class.
     */
    Robot();
    /**
     * @brief Array of pointers to the legs of the robot.
     */

    Leg *legs[LEG_NUM] = {&FR, &FL, &RR, &RL};
    /**
     * @brief Get the transformation matrix for the Front Left (FL) leg.
     * @return The transformation matrix of the FL leg.
     */
    Eigen::Matrix4d getTransformation_FL();
    /**
     * @brief Get the transformation matrix for the Front Right (FR) leg.
     * @return The transformation matrix of the FR leg.
     */
    Eigen::Matrix4d getTransformation_FR();
    /**
     * @brief Get the transformation matrix for the Rear Left (RL) leg.
     * @return The transformation matrix of the RL leg.
     */
    Eigen::Matrix4d getTransformation_RL();
    /**
     * @brief Get the transformation matrix for the Rear Right (RR) leg.
     * @return The transformation matrix of the RR leg.
     */
    Eigen::Matrix4d getTransformation_RR();

    /**
     * @brief Set the joint angles for the Front Left (FL) leg.
     * @param hip_angle Angle for the hip joint.
     * @param thigh_angle Angle for the thigh joint.
     * @param calf_angle Angle for the calf joint.
     */
    void setAngles_FL(double hip_angle, double thigh_angle, double calf_angle);

    /**
     * @brief Set the joint angles for the Front Right (FR) leg.
     * @param hip_angle Angle for the hip joint.
     * @param thigh_angle Angle for the thigh joint.
     * @param calf_angle Angle for the calf joint.
     */
    void setAngles_FR(double hip_angle, double thigh_angle, double calf_angle);

    /**
     * @brief Set the joint angles for the Rear Left (RL) leg.
     * @param hip_angle Angle for the hip joint.
     * @param thigh_angle Angle for the thigh joint.
     * @param calf_angle Angle for the calf joint.
     */
    void setAngles_RL(double hip_angle, double thigh_angle, double calf_angle);

    /**
     * @brief Set the joint angles for the Rear Right (RR) leg.
     * @param hip_angle Angle for the hip joint.
     * @param thigh_angle Angle for the thigh joint.
     * @param calf_angle Angle for the calf joint.
     */
    void setAngles_RR(double hip_angle, double thigh_angle, double calf_angle);

    /**
     * @brief Set the orientation of the robot's body.
     * @param roll Roll angle of the body.
     * @param pitch Pitch angle of the body.
     * @param yaw Yaw angle of the body.
     */
    void setOrientation(float roll, float pitch, float yaw);

    /**
     * @brief Get the position of the Front Left (FL) leg's end-effector.
     * @return The position as a 3D vector.
     */
    Eigen::Vector3d getPosition_FL();

    /**
     * @brief Get the position of the Front Right (FR) leg's end-effector.
     * @return The position as a 3D vector.
     */
    Eigen::Vector3d getPosition_FR();

    /**
     * @brief Get the position of the Rear Left (RL) leg's end-effector.
     * @return The position as a 3D vector.
     */
    Eigen::Vector3d getPosition_RL();

    /**
     * @brief Get the position of the Rear Right (RR) leg's end-effector.
     * @return The position as a 3D vector.
     */
    Eigen::Vector3d getPosition_RR();

    /**
     * @brief Get the orientation of the robot's body.
     * @return The orientation as a 3D vector (roll, pitch, yaw).
     */
    Eigen::Vector3d getOrientation()      @date:      
      @author:    
      @cheked:    

        /* Eigen::Vector3d move_FL(Eigen::Vector3d position);
        Eigen::Vector3d move_FR(Eigen::Vector3d position);
        Eigen::Vector3d move_RL(Eigen::Vector3d position);
        Eigen::Vector3d move_RR(Eigen::Vector3d position); */

        /**
         * @brief Set the joint angles for all legs.
         * @param angles A vector containing the desired angles for all joints.
         */
        void setAngles(std::vector<double> angles);

    /**
     * @brief Set the joint velocities for all legs.
     * @param values A vector containing the desired velocities for all joints.
     */
    void setVels(std::vector<double> values);

    /**
     * @brief Set the joint torques for all legs.
     * @param values A vector containing the desired torques for all joints.
     */
    void setTaus(std::vector<double> values);

    /**
     * @brief Set the proportional gains (Kp) for all legs.
     * @param values A vector containing the desired Kp values for all joints.
     */
    void setKps(std::vector<double> values);

    /**
     * @brief Set the derivative gains (Kd) for all legs.
     * @param values A vector containing the desired Kd values for all joints.
     */
    void setKds(std::vector<double> values);

    /**
     * @brief Get the current joint angles for all legs.
     * @return A vector containing the angles of all joints.
     */
    std::vector<double> getAngles();

    /**
     * @brief Get the current joint velocities for all legs.
     * @return A vector containing the velocities of all joints.
     */
    std::vector<double> getVels();

    /**
     * @brief Get the current joint torques for all legs.
     * @return A vector containing the torques of all joints.
     */
    std::vector<double> getTaus();

    /**
     * @brief Get the current proportional gains (Kp) for all legs.
     * @return A vector containing the Kp values of all joints.
     */
    std::vector<double> getKps();

    /**
     * @brief Get the current derivative gains (Kd) for all legs.
     * @return A vector containing the Kd values of all joints.
     */
    std::vector<double> getKds();
    /**
     * @brief Limit the joint torques to predefined maximum values.
     */
    void limitTaus();

    /**
     * @brief Calculate the joint torques for all legs based on an external force.
     * @param f_ext The external force applied to the robot.
     * @return A vector containing the torques of all joints.
     */
    Eigen::Vector<double, 12> calculateTaus(Eigen::Vector3d f_ext);
    Eigen::Vector<double, 12> cntrlTau; /**< Control torques for the joints */
    Eigen::Vector<double, 12> cmdTaus;  /**< Commanded torques for the joints */

    /*Eigen::Vector3d  getPosition(const std::vector<double> &angles, LegType leg);
    Eigen::Vector3d  getAngle(const std::vector<double> &angles, LegType leg);*/

private:
    Leg FL, FR, RL, RR;
    /**
     * @brief Helper function to set values using a specific function for all joints.
     * @param func A function to set a value for the joints.
     * @param values A vector containing the values to set.
     */
    void setValues(const std::function<void(Eigen::Vector3d)> &func, std::vector<double> values);
    std::vector<double> bodyOrientation = {0, 0, 0}; /**< Orientation of the robot's body (roll, pitch, yaw) */
    const std::vector<double> maxTaus = {
        MAX_TAU_HIP,
        MAX_TAU_THIGH,
        MAX_TAU_CALF,
        MAX_TAU_HIP,
        MAX_TAU_THIGH,
        MAX_TAU_CALF,
        MAX_TAU_HIP,
        MAX_TAU_THIGH,
        MAX_TAU_CALF,
        MAX_TAU_HIP,
        MAX_TAU_THIGH,
        MAX_TAU_CALF,
    }; /**< Maximum allowable torques for the joints */

    // Kinematics kinematicModel;
};

#endif // ROBOT_H
