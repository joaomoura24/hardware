/**
 *      @file  gamma.h
 *      @brief  Class for gamma force sensor
 *
 *
 *     @author  joão moura, Joao.Moura@ed.ac.uk
 *
 *   @internal
 *     Created  05-Mar-2017
 *    Revision  ---
 *    Compiler  gcc/g++
 *     Company  Edinburgh Centre for Robotics
 * ===============================================================
 */
#ifndef _GAMMA_
#define _GAMMA_

// Library for vectors, matrices, and algebra operations
#include <Eigen/Dense>
// ROS header
#include <ros/ros.h>
// messages
#include <geometry_msgs/WrenchStamped.h>

class Gamma
{
	public:
		Gamma(); // class constructor
		virtual ~Gamma(); // destructor
		Eigen::Matrix<double, 6, 1> force; // force measured after calibration
		Eigen::Matrix< double, 6, 1> forceFiltered; // filtered force
		Eigen::Matrix< double, 6, 1> forceDerivative; // derivative of filtered force
		bool gotForceSensing = false; // did not get force readings
		/**¬
		 * @brief Initialize class
		 * @param TransSensorToContact: 6x6 tranformation matrix from sensor to contact point
		 * @param fMin: maximum negative force for safety
		 * @param fMax: maximum positive force for safety
		 * @param alpha: filtering contant (alpha=1 no filtering; alpha=1 contant force)
		 */
		void Init(Eigen::Matrix<double, 6, 6> TransSensorToContact, Eigen::Matrix<double, 6, 1> fMin, Eigen::Matrix<double, 6, 1> fMax, double alpha);
	private:
		double alpha_; // filtering constant
		//-----------------------------------------
		// ROS variables
		//-----------------------------------------
		ros::NodeHandle nh_; // ROS node handle
		ros::Subscriber subForceSensing_;
		//-----------------------------------------
		// Forse sensor related variables
		//-----------------------------------------
		Eigen::Matrix<double, 6, 6> TransSensorToContact_;
		Eigen::Matrix<double, 6, 1> fMax_;
		Eigen::Matrix<double, 6, 1> fMin_;
		/**
		 * @brief function to check limit of a variable and terminate program in case the limit is reached
		 * @param name: error message
		 * @param var: vector to control
		 * @param var_min: minimum values
		 * @param var_max: maximum values
		 */
		template <typename Derived>
		void maxLimError_(const std::string name, const Eigen::DenseBase<Derived>& var, const Eigen::DenseBase<Derived>& var_min, const Eigen::DenseBase<Derived>& var_max);
		/**¬
		 * @brief Reads force sensor values and calibrates sensor
		 * @param msg: force sensor message
		 */
		void callbackForceSensing_(const geometry_msgs::WrenchStamped::ConstPtr& msg);
};

#endif // _GAMMA_
