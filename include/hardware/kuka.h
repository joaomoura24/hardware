/**
 *      @file  kuka.h
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
#ifndef _KUKA_
#define _KUKA_

// standard string
#include <string>
// Messages
#include <ipab_lwr_msgs/FriState.h>
#include <ipab_lwr_msgs/FriCommandJointPosition.h>
#include <geometry_msgs/WrenchStamped.h>
// Kinematics library
#include <serial_arm_lib/serialArmKin.h>

#ifndef VAL_SAT
#define VAL_SAT(val,min,max)    ((val)>(max)?(max):((val)<(min)?(min):(val)))
#endif

class Kuka
{
	public:
		Kuka(); // class constructor
		virtual ~Kuka(); // destructor
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
		//-----------------------------------------
		// ROS variables
		//-----------------------------------------
		ipab_lwr_msgs::FriCommandJointPosition msgJointsState_;
		ros::Publisher pubJointsState_;
		ros::Subscriber subJointsState_;
		ros::NodeHandle nh_; // ROS node handle
		/**¬
		 * @brief Reads joints position and stores it in private vector
		 * @param msg: robot state message
		 */
		void callbackJointsState_(const ipab_lwr_msgs::FriState::ConstPtr& msg);
};

#endif // _KUKA_
