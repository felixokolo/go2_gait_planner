/**
 *      @file:      Joint.h
 *
 *      @date:      2 Jun 2024
 *      @author:    Priyanka
 *      @cheked:    Felix
 */

#ifndef JOINT_H
#define JOINT_H

#include "Quadruped/Link.h"
#include <eigen3/Eigen/Dense>

#define LEN_THIGH 0.2130
#define LEN_HIP 0.0955
#define LEN_CALF 0.23
#define ladj 0
#define LEN_BASE 0.3868
#define WIDTH_BASE 0.092
#define HEIGHT_BASE -0.000

#define HIP_UP_LIM 0.7
#define HIP_LO_LIM -0.7
#define DEFAULT_ANGLE_HIP 0

#define THIGH_UP_LIM 2.0  // 1.7453
#define THIGH_LO_LIM -0.1 //-0.7854
#define DEFAULT_ANGLE_THIGH 1.3

#define CALF_UP_LIM -0.05 //-0.15
#define CALF_LO_LIM -2.9
#define DEFAULT_ANGLE_CALF -2.8

#define CALF_OFFSET 0.0873
// #define CALF_OFFSET 0.0

/**
 * @enum JointType
 * @brief Defines the types of joints in the quadruped robot.
 *
 * This enumeration is used to specify the type of joint, such as HIP, THIGH, or CALF.
 */

enum JointType
{
    HIP = 0,
    THIGH,
    CALF,
    JOINT_NUM
};

/**
 * @class Joint
 * @brief Represents a joint in the quadruped robot.
 *
 * The Joint class encapsulates the behavior and properties of a joint in a quadruped robot,
 * including transformations, angles, and control parameters.
 */

class Joint
{
public:
    /**
     * @brief Constructor for the Joint class.
     * @param parentLength Length of the parent link.
     * @param childLength Length of the child link.
     * @param type Type of the joint (HIP, THIGH, or CALF).
     */

    Joint(double parentLength, double childLength, JointType type);

    /**
     * @brief Destructor for the Joint class.
     */
    ~Joint();

    /**
     * @brief Get the transformation matrix of the joint.
     * @return The current transformation matrix.
     */
    const Eigen::Matrix4d &getTransformation() const;

    /**
     * @brief Get the transformation matrix for a specified angle.
     * @param angle The angle of the joint.
     * @return The transformation matrix corresponding to the given angle.
     */

    const Eigen::Matrix4d getTransformation(double angle) const;
    /**
     * @brief Set the angle of the joint.
     * @param angle The angle to set.
     */
    void setAngle(double angle);

    /**
     * @brief Get the current angle of the joint.
     * @return The current joint angle.
     */

    double getAngle();

    /**
     * @brief Get the desired joint velocity (qd).
     * @return The desired joint velocity.
     */
    double getQd();
    /**
     * @brief Get the current torque of the joint.
     * @return The current joint torque.
     */
    double getTau();
    /**
     * @brief Get the proportional gain (Kp) of the joint.
     * @return The proportional gain.
     */
    double getKp();
    /**
     * @brief Get the derivative gain (Kd) of the joint.
     * @return The derivative gain.
     */
    double getKd();

    /**
     * @brief Set the desired joint velocity (qd).
     * @param value The desired joint velocity to set.
     */
    void setQd(double value);

    /**
     * @brief Set the torque of the joint.
     * @param value The torque value to set.
     */
    void setTau(double value);

    /**
     * @brief Set the proportional gain (Kp) of the joint.
     * @param value The proportional gain value to set.
     */
    void setKp(double value);
    /**
     * @brief Set the derivative gain (Kd) of the joint.
     * @param value The derivative gain value to set.
     */

    void setKd(double value);

    /**
     * @brief Enforce the joint angle limits.
     * @param estJointAngle The estimated joint angle.
     * @return The joint angle within the enforced limits.
     */
    double enforceLim(double estJointAngle);

private:
    Link parentLink;                     /**< The parent link associated with this joint */
    Link childLink;                      /**< The child link associated with this joint */
    JointType type;                      /**< The type of the joint */
    double jointAngle;                   /**< The current angle of the joint */
    double defaultAngle;                 /**< The default angle of the joint */
    double jointUpLim, jointLoLim;       /**< Upper and lower limits of the joint angle */
    double dq = 0;                       /**< Desired joint velocity */
    double tau = 0;                      /**< Current torque of the joint */
    double Kp = 0, Kd = 0;               /**< Proportional and derivative gains */
    Eigen::Matrix4d jointTransformation; /**< Transformation matrix of the joint */

    /**
     * @brief Function to calculate the transformation matrix based on joint angle.
     */

    std::function<Eigen::Matrix<double, 4, 4>(double)> calculateTransformation;

    /**
     * @brief Calculate the transformation matrix for the hip joint.
     * @param angle The angle of the hip joint.
     * @return The transformation matrix for the given angle.
     */
    Eigen::Matrix<double, 4, 4> calculateHipTransformation(double angle);
    /**
     * @brief Calculate the transformation matrix for the thigh joint.
     * @param angle The angle of the thigh joint.
     * @return The transformation matrix for the given angle.
     */
    Eigen::Matrix<double, 4, 4> calculateThighTransformation(double angle);

    /**
     * @brief Calculate the transformation matrix for the calf joint.
     * @param angle The angle of the calf joint.
     * @return The transformation matrix for the given angle.
     */
    Eigen::Matrix<double, 4, 4> calculateCalfTransformation(double angle);
};

#endif // JOINT_H
