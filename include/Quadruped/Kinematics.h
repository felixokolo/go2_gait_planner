/**
 * Kinematics.h
 *
 *      @date:      6 Jun 2024
 *      @author:    Felix
 *      @cheked:    
 */

#ifndef KINEMATICS_H_
#define KINEMATICS_H_

#include <iostream>
#include <eigen3/Eigen/Dense>

class Leg;

/**
 * @class Kinematics
 * @brief Handles the kinematics calculations for a leg of the quadruped robot.
 *
 * The Kinematics class provides methods to compute the current and desired
 * positions of the leg based on joint angles, as well as to calculate the
 * necessary joint angles to achieve a desired position.
 */

class Kinematics
{
public:
	/**
	 * @brief Constructor for the Kinematics class.
	 * @param leg A pointer to the Leg object associated with these kinematics.
	 */
	Kinematics(Leg *leg);
	/**
	 * @brief Get the current position of the leg.
	 * @return A 3D vector representing the current position of the leg.
	 */
	Eigen::Vector3d getCurPosition();
	/**
	 * @brief Get the position of the leg given a transformation matrix.
	 * @param trans A 4x4 transformation matrix.
	 * @return A 3D vector representing the position of the leg.
	 */
	Eigen::Vector3d getPosition(Eigen::Matrix4d trans);
	/**
	 * @brief Calculate the position of the end-effector based on joint angles.
	 * @param hipAngle Angle of the hip joint.
	 * @param thighAngle Angle of the thigh joint.
	 * @param calfAngle Angle of the calf joint.
	 * @return The position as a 3D vector.
	 */
	Eigen::Vector3d getPosition(double hipAngle,
								double thighAngle,
								double calfAngle);

	/**
	 * @brief Calculate the position of the end-effector based on a vector of joint angles.
	 * @param angles A vector containing the hip, thigh, and calf joint angles.
	 * @return The position as a 3D vector.
	 */
	Eigen::Vector3d getPosition(Eigen::Vector3d angles);

	/**
	 * @brief Compute the new joint angles given the current angles and a desired change in position.
	 * @param curAngles Current joint angles as a 3D vector.
	 * @param delta_x Desired change in position as a 3D vector.
	 * @return The new joint angles as a 3D vector.
	 */
	Eigen::Vector3d jointAngleCompute(Eigen::Vector3d curAngles,
									  Eigen::Vector3d delta_x);

	/**
	 * @brief Compute the new joint angles given a desired change in position.
	 * @param delta_x Desired change in position as a 3D vector.
	 * @return The new joint angles as a 3D vector.
	 */
	Eigen::Vector3d jointAngleCompute(Eigen::Vector3d delta_x);

	/**
	 * @brief Destructor for the Kinematics class.
	 */
	virtual ~Kinematics();

private:
	Leg *leg; /**< Pointer to the Leg object associated with this Kinematics instance */

	/**
	 * @brief Compute the change in joint angles required to achieve a desired change in position.
	 * @param delta_x Desired change in position as a 3D vector.
	 * @return The required change in joint angles as a 3D vector.
	 */
	Eigen::Vector3d jointDelta_q(Eigen::Vector3d delta_x);
};

#endif /* KINEMATICS_H_ */
