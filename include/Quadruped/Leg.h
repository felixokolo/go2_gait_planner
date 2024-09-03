/**
 *      @file:      Leg.h
 *
 *      @date:      2 Jun 2024
 *      @author:    Priyanka
 *      @cheked:    Felix
 */

/**
 *
 */

#ifndef LEG_H
#define LEG_H

#define FL_TRANS Eigen::Vector3d(LEN_BASE / 2, WIDTH_BASE / 2, HEIGHT_BASE)
#define FR_TRANS Eigen::Vector3d(LEN_BASE / 2, -WIDTH_BASE / 2, HEIGHT_BASE)
#define RL_TRANS Eigen::Vector3d(-LEN_BASE / 2, WIDTH_BASE / 2, HEIGHT_BASE)
#define RR_TRANS Eigen::Vector3d(-LEN_BASE / 2, -WIDTH_BASE / 2, HEIGHT_BASE)

/**
 * @enum LegType
 * @brief Defines the types of legs in the quadruped robot.
 *
 * This enumeration is used to specify the type of leg, such as Front Right (FR), Front Left (FL), Rear Right (RR), or Rear Left (RL).
 */

enum LegType
{
    FR = 0, // Front Right
    FL,     // Front Left
    RR,     // Rear Right
    RL,     // Rear Left
    LEG_NUM
};

#include "Quadruped/Joint.h"
#include "Quadruped/Kinematics.h"

/**
 * @class Leg
 * @brief Represents a leg of a quadruped robot.
 *
 * The Leg class encapsulates the behavior and properties of a single leg in a quadruped robot, including its joints, kinematics, and transformations.
 */

class Leg
{
public:
    /**
     * @brief Constructor for the Leg class.
     * @param baseLength Length of the base.
     * @param hipLength Length of the hip.
     * @param thighLength Length of the thigh.
     * @param calfLength Length of the calf.
     * @param Type Type of the leg (FR, FL, RR, or RL).
     */
    Leg(double baseLength, double hipLength, double thighLength,
        double calfLength, LegType Type);
    Joint *joints[JOINT_NUM];

    /**
     * @brief Set the angles of the leg's joints.
     * @param hip_angle Angle of the hip joint.
     * @param thigh_angle Angle of the thigh joint.
     * @param calf_angle Angle of the calf joint.
     */
    void setAngles(double hip_angle, double thigh_angle, double calf_angle);
    /**
     * @brief Get the current angles of the leg's joints.
     * @return A vector containing the hip, thigh, and calf joint angles.
     */
    Eigen::Vector3d getAngles();
    /**
     * @brief Get the transformation matrix of the leg based on the current joint angles.
     * @return The transformation matrix.
     */
    Eigen::Matrix4d getTransformation();
    /**
     * @brief Get the transformation matrix of the leg for the given joint angles.
     * @param angles A vector containing the hip, thigh, and calf joint angles.
     * @return The transformation matrix.
     */
    Eigen::Matrix4d getTransformation(Eigen::Vector3d angles);
    /**
     * @brief Get the Jacobian matrix of the leg based on the current joint angles.
     * @return The Jacobian matrix.
     */
    Eigen::MatrixXd getJacobian();
    /**
     * @brief Get the Jacobian matrix of the leg for the given joint angles.
     * @param angles A vector containing the hip, thigh, and calf joint angles.
     * @return The Jacobian matrix.
     */
    Eigen::MatrixXd getJacobian(Eigen::Vector3d angles);
    /**
     * @brief Get the current position of the leg's end-effector.
     * @return The position as a 3D vector.
     */
    Eigen::Vector3d getPosition();
    /**
     * @brief Get the position of the leg's end-effector for the given joint angles.
     * @param angles A vector containing the hip, thigh, and calf joint angles.
     * @return The position as a 3D vector.
     */
    Eigen::Vector3d getPosition(Eigen::Vector3d angles);
    /**
     * @brief Get the Kinematics object associated with this leg.
     * @return Pointer to the Kinematics object.
     */
    Kinematics *getKinematics();
    /**
     * @brief Enforce joint limits on the estimated joint angles.
     * @param estJointAngles The estimated joint angles.
     * @return The joint angles within the enforced limits.
     */
    Eigen::Vector3d enforceJointLim(Eigen::Vector3d estJointAngles);

    /**
     * @brief Get the current velocities of the leg's joints.
     * @return A vector containing the joint velocities.
     */
    Eigen::Vector3d getJointVels();
    /**
     * @brief Get the current torques of the leg's joints.
     * @return A vector containing the joint torques.
     */
    Eigen::Vector3d getJointTaus();
    /**
     * @brief Get the proportional gains (Kp) of the leg's joints.
     * @return A vector containing the proportional gains.
     */
    Eigen::Vector3d getJointKps();
    /**
     * @brief Get the derivative gains (Kd) of the leg's joints.
     * @return A vector containing the derivative gains.
     */
    Eigen::Vector3d getJointKds();
    /**
     * @brief Calculate the joint torques required to resist an external force.
     * @param f_ext The external force as a 3D vector.
     * @return The calculated joint torques.
     */
    Eigen::Vector3d calculateTau(Eigen::Vector3d f_ext);
    /**
     * @brief Set the velocities of the leg's joints.
     * @param values A vector containing the joint velocities to set.
     */
    void setJointVels(Eigen::Vector3d values);
    /**
     * @brief Set the torques of the leg's joints.
     * @param values A vector containing the joint torques to set.
     */
    void setJointTaus(Eigen::Vector3d values);
    /**
     * @brief Set the proportional gains (Kp) of the leg's joints.
     * @param values A vector containing the proportional gains to set.
     */
    void setJointKps(Eigen::Vector3d values);
    /**
     * @brief Set the derivative gains (Kd) of the leg's joints.
     * @param values A vector containing the derivative gains to set.
     */
    void setJointKds(Eigen::Vector3d values);

private:
    Joint hip;                         /**< Hip joint of the leg */
    Joint thigh;                       /**< Thigh joint of the leg */
    Joint calf;                        /**< Calf joint of the leg */
    LegType legType;                   /**< Type of the leg (FR, FL, RR, or RL) */
    Eigen::Matrix4d legTransformation; /**< Transformation matrix of the leg */
    Eigen::Matrix3d legJacobian;       /**< Jacobian matrix of the leg */
    Kinematics kinematics;             /**< Kinematics object associated with the leg */
    Eigen::Vector3d position;          /**< Position of the leg's end-effector */
    Eigen::Vector3d transVector;       /**< Transformation vector for the leg */

    std::function<Eigen::Matrix<double, 3, 3>(double,
                                              double,
                                              double)>
        calculateJacobian;

    /**
     * @brief Calculate the Jacobian matrix for the Front Left (FL) leg.
     * @param qhip Angle of the hip joint.
     * @param qthigh Angle of the thigh joint.
     * @param qcalf Angle of the calf joint.
     * @return The Jacobian matrix for the FL leg.
     */

    Eigen::Matrix<double, 3, 3> calculateJacobian_FL(double qhip,
                                                     double qthigh,
                                                     double qcalf);
    /**
     * @brief Calculate the Jacobian matrix for the Front Right (FR) leg.
     * @param qhip Angle of the hip joint.
     * @param qthigh Angle of the thigh joint.
     * @param qcalf Angle of the calf joint.
     * @return The Jacobian matrix for the FR leg.
     */
    Eigen::Matrix<double, 3, 3> calculateJacobian_FR(double qhip,
                                                     double qthigh,
                                                     double qcalf);

    /**
     * @brief Calculate the Jacobian matrix for the Rear Left (RL) leg.
     * @param qhip Angle of the hip joint.
     * @param qthigh Angle of the thigh joint.
     * @param qcalf Angle of the calf joint.
     * @return The Jacobian matrix for the RL leg.
     */
    Eigen::Matrix<double, 3, 3> calculateJacobian_RL(double qhip,
                                                     double qthigh,
                                                     double qcalf);
    /**
     * @brief Calculate the Jacobian matrix for the Rear Right (RR) leg.
     * @param qhip Angle of the hip joint.
     * @param qthigh Angle of the thigh joint.
     * @param qcalf Angle of the calf joint.
     * @return The Jacobian matrix for the RR leg.
     */

    Eigen::Matrix<double, 3, 3> calculateJacobian_RR(double qhip,
                                                     double qthigh,
                                                     double qcalf);

    /**
     * @brief Calculate the Jacobian matrix for leg.
     * @param qhip Angle of the hip joint.
     * @param qthigh Angle of the thigh joint.
     * @param qcalf Angle of the calf joint.
     * @return The Jacobian matrix for the leg.
     */

    Eigen::Matrix<double, 4, 4> calculateTransformation(double qhip,
                                                        double qthigh,
                                                        double qcalf);

    /**
     * @brief gets the Jacobian matrix for the base.
     * @return The Jacobian matrix for the base.
     */
    Eigen::Matrix4d getBaseTransformation();
};

#endif // LEG_H
