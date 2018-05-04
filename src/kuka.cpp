/**
 *      @file  kuka.cpp
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
#include <hardware/kuka.h>

Kuka::Kuka()
{
}

Kuka::~Kuka()
{
}

void Kuka::Init(std::string &toolLink)
{
	gotRobotState = false; // did not get robot state
	// Initialize kinematics class from the ROS robot_description parameter
	kin.initFromParam("robot_description",toolLink);
	// fill in msgJointsState_ with joint names
	msgJointsState_.joint_names = kin.jointNames;
	// Initialize joint position vector
	jointValues.resize(kin.nrJoints); jointValues.setZero();
	// Publish Kuka operation mode in joint position
	pubJointsState_ = nh_.advertise<ipab_lwr_msgs::FriCommandJointPosition>("/lwr/commandJointPosition", 1, true);
	// Subscribe to robot state
	subJointsState_ = nh_.subscribe<ipab_lwr_msgs::FriState>("/kuka_lwr_state", 1, &Kuka::callbackJointsState_, this);
}

void Kuka::callbackJointsState_(const ipab_lwr_msgs::FriState::ConstPtr& msg)
{
	if(msg->jointPosition.size()==kin.nrJoints){
		for(int idx=0; idx<kin.nrJoints; idx++){
			jointValues(idx) = msg->jointPosition[idx];
		}
		if(!gotRobotState) gotRobotState = true;
	}
	else ROS_WARN_STREAM("Joint position vector with wrong size");
}

void Kuka::setJointPos(const Eigen::Matrix<double, Eigen::Dynamic, 1> &q)
{
	// Fill in message:
	msgJointsState_.header.stamp = ros::Time::now();
	msgJointsState_.jointPosition.clear();
	for(int idx=0; idx<q.size(); idx++){
		msgJointsState_.jointPosition.push_back(VAL_SAT(q(idx), kin.jointLowerLimits(idx), kin.jointUpperLimits(idx)));
	}
	pubJointsState_.publish(msgJointsState_);
}

void Kuka::setJointVel(const Eigen::Matrix<double, Eigen::Dynamic, 1> &dq)
{
	Eigen::Matrix<double, Eigen::Dynamic, 1> q; q.resize(dq.size());
	for(int idx=0; idx<dq.size(); idx++) q(idx) = jointValues(idx) + VAL_SAT(dq(idx), -kin.jointVelocity(idx), kin.jointVelocity(idx));
	setJointPos(q);
}
