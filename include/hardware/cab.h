/**
 *      @file  cab.h
 *      @brief  Class for kuka robot
 *
 *
 *     @author  joão moura, Joao.Moura@ed.ac.uk
 *
 *   @internal
 *     Created  28-Feb-2017
 *    Revision  ---
 *    Compiler  gcc/g++
 *     Company  Edinburgh Centre for Robotics
 * ===============================================================
 */
#ifndef _CAB_
#define _CAB_

// standard string
#include <string>
// Messages
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
// Kinematics library
#include <serial_arm_lib/serialArmKin.h>

#ifndef VAL_SAT
#define VAL_SAT(val,min,max)    ((val)>(max)?(max):((val)<(min)?(min):(val)))
#endif
#define MIN_ERROR 0.001

class Cab
{
	public:
		Cab(); // class constructor
		virtual ~Cab(); // destructor
		// Kinematic related variables
		SerialArmKin kin;
		Eigen::Matrix<double, Eigen::Dynamic, 1> jointValues; // read joint positions
		bool gotRobotState;
		/**¬
		 * @brief Initialize class
		 * @param toolLink: name of the last link
		 */
		void Init(std::string &toolLink);
		/**¬
		 * @brief Commands the robot arm joint positions
		 * @param q: joint positions
		 */
		void setJointPos(const Eigen::Matrix<double, Eigen::Dynamic, 1> &q);
		/**¬
		 * @brief Commands the robot arm joint velocities
		 * @param q: joint velocities
		 */
		void setJointVel(const Eigen::Matrix<double, Eigen::Dynamic, 1> &dq);
	private:
		double virtualScale_;
		//-----------------------------------------
		// ROS variables
		//-----------------------------------------
		sensor_msgs::JointState msgJointVel_;
		ros::Publisher pubJointsVel_;
		ros::Subscriber subJointsState_;
		ros::NodeHandle nh_; // ROS node handle
		ros::Timer cycleTimer_;
		double freq_;
		//-----------------------------------------
		// Proportional controller variables
		//-----------------------------------------
		Eigen::Matrix<double, Eigen::Dynamic, 1> Kp_;
		Eigen::Matrix<double, Eigen::Dynamic, 1> dqMax_;
		/**¬
		 * @brief Reads joints position and stores it in private vector
		 * @param msg: robot state message
		 */
		void callbackJointsState_(const sensor_msgs::JointState::ConstPtr& msg);
		/**¬
		 * @brief Reads joints position and stores it in private vector
		 * @param msg: robot state message
		 */
		Eigen::Matrix<double, Eigen::Dynamic, 1> readVecFromParam_(std::string paramName, int vecSize, double defaultValue);

		// auxiliar vectors
		Eigen::Matrix<double, 7, 1> x_;
        // Publish end-effector pose
        std_msgs::Float64MultiArray logPoseMsg_;
        ros::Publisher pubPoseLog_;
};

#endif // _CAB_
