/**
 *      @file  cab.cpp
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
#include <hardware/cab.h>

Cab::Cab()
{
}

Cab::~Cab()
{
}

void Cab::Init(std::string &toolLink)
{
	gotRobotState = false; // did not get robot state
	// Initialize kinematics class from the ROS robot_description parameter
	kin.initFromParam("robot_description",toolLink);
	// fill in msgJointsState_ with joint names
	msgJointVel_.name = kin.jointNames;
	// Initialize joint position vector
	jointValues.resize(kin.nrJoints); jointValues.setZero();
	// Publish Cab operation mode in joint position
	pubJointsVel_ = nh_.advertise<sensor_msgs::JointState>("/driver/command", 1, true);
	// Subscribe to robot state
	subJointsState_ = nh_.subscribe<sensor_msgs::JointState>("/joint_states", 1, &Cab::callbackJointsState_, this);
    // Publisher to publish end-effector pose for logging
    pubPoseLog_ = nh_.advertise<std_msgs::Float64MultiArray>("/log/pose", 1);
	// read proportional gain and maximum output
	Kp_ = readVecFromParam_("~Kjoints", kin.nrJoints, 0.0);
	dqMax_ = readVecFromParam_("~dqMax", kin.nrJoints, 10.0);
	// get cycle frequency
	ros::param::param<double>("~freq", freq_, 50.0);
	// get artificial scale:
	ros::param::param<double>("~virtualScale", virtualScale_, 1.0);
}

void Cab::callbackJointsState_(const sensor_msgs::JointState::ConstPtr& msg)
{
	if(msg->position.size()==kin.nrJoints){
		for(int idx=0; idx<kin.nrJoints; idx++){
			jointValues(idx) = msg->position[idx];
			// Get robot pose and orientation
			Eigen::Matrix<double, 3, 1> pos;
			Eigen::Matrix<double, 4, 1> quat;
			kin.getFK(pos, quat, jointValues);
			// store as a vector
			x_.head(3) = pos; x_.tail(4) = quat;
			// Publish message
			logPoseMsg_.data.clear();
			for(int idx=0; idx<7; idx++) logPoseMsg_.data.push_back(x_(idx));
			pubPoseLog_.publish(logPoseMsg_);
		}
		if(!gotRobotState) gotRobotState = true;
	}
	else ROS_WARN_STREAM("Joint position vector with wrong size");
}

void Cab::setJointPos(const Eigen::Matrix<double, Eigen::Dynamic, 1> &q)
{
	if(gotRobotState){
		Eigen::Matrix<double, Eigen::Dynamic, 1> dq; dq.resize(q.size()); dq.setZero();
		for(int idx=0; idx<q.size()-1; idx++){
			dq(idx) = VAL_SAT((Kp_(idx)*(q(idx) - jointValues(idx))), -dqMax_(idx), dqMax_(idx));
		}
		setJointVel(dq);
	}
}

void Cab::setJointVel(const Eigen::Matrix<double, Eigen::Dynamic, 1> &dq)
{
    // Publish messages:
    msgJointVel_.header.stamp = ros::Time::now();
	msgJointVel_.velocity.clear();
    for(int idx=0; idx<(dq.size()-1); idx++){
		if(idx<3) msgJointVel_.velocity.push_back(dq(idx)*virtualScale_);
		else msgJointVel_.velocity.push_back(dq(idx));
    }
	msgJointVel_.velocity.push_back(0.0); // last joint is a virtual joint
    pubJointsVel_.publish(msgJointVel_);
}

Eigen::Matrix<double, Eigen::Dynamic, 1> Cab::readVecFromParam_(std::string paramName, int vecSize, double defaultValue)
{
	std::vector<double> vect(vecSize);
	std::vector<double> vect0(vecSize, defaultValue);
	ros::param::param<std::vector<double>>(paramName, vect, vect0);
	return Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, 1>>((vect).data(), (vect).size());
}
