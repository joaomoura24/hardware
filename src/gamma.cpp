/**
 *      @file  Gamma.cpp
 *      @brief  Class for force sensor gamma
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
#include <hardware/gamma.h>

Gamma::Gamma()
{
}

Gamma::~Gamma()
{
}

void Gamma::Init(Eigen::Matrix<double, 6, 6> TransSensorToContact, Eigen::Matrix<double, 6, 1> fMin, Eigen::Matrix<double, 6, 1> fMax, double alpha)
{
		// Initialize force variables
		force.setZero(); forceDerivative.setZero(); forceFiltered.setZero();
		// Copy input to private variables:
		TransSensorToContact_ = TransSensorToContact;
		fMax_ = fMax;
		fMin_ = fMin;
		alpha_ = alpha;
		// Subscribe to force sensor data
		subForceSensing_ = nh_.subscribe<geometry_msgs::WrenchStamped>("/netft_data", 1, &Gamma::callbackForceSensing_, this);
}

void Gamma::callbackForceSensing_(const geometry_msgs::WrenchStamped::ConstPtr& msg)
{
	// Storing sensor msg in variable
	force[0] = msg->wrench.force.x;
	force[1] = msg->wrench.force.y;
	force[2] = msg->wrench.force.z;
	force[3] = msg->wrench.torque.x;
	force[4] = msg->wrench.torque.y;
	force[5] = msg->wrench.torque.z;
	// Initial calibration process
	static int count = 0;
	static Eigen::Matrix<double, 6, 1> fcal = Eigen::Matrix<double, 6, 1>::Zero();
	if(!gotForceSensing){
		fcal += (1.0/++count) * (force - fcal); // average first values
		if(count == 1) ROS_INFO_STREAM("Motions: Calibrating force sensor...");
		if(count == 200){
			ROS_INFO_STREAM("Motions: Calibration values:");
			ROS_INFO_STREAM(fcal.transpose());
			gotForceSensing = true; // callibration done
		}
	}
	else{
		// Discount initial reading of force sensor
		force -= fcal;
		// Transform 6D force from sensor to contact point
		force = TransSensorToContact_*force;
		// filtering force
		static Eigen::Matrix<double, 6, 1> fPrev = Eigen::Matrix<double, 6, 1>::Zero();
		fPrev = forceFiltered;
		forceFiltered += alpha_*(force - forceFiltered);
		// computing the derivative
		forceDerivative = forceFiltered - fPrev;
	}
    //-----------------------------------------------------------------
    // SAFETY CONDITIONS
    //-----------------------------------------------------------------
    // End-effector force (force sensor)
    maxLimError_("END-EFFECTOR FORCE", force, fMin_, fMax_);
    //-----------------------------------------------------------------
}

template <typename Derived>
void Gamma::maxLimError_(const std::string name, const Eigen::DenseBase<Derived>& var, const Eigen::DenseBase<Derived>& var_min, const Eigen::DenseBase<Derived>& var_max)
{
    for(int idx=0; idx<var.size(); idx++){
        if(var[idx] < var_min[idx] || var[idx] > var_max[idx]){
            ROS_ERROR_STREAM("SweepingRobot: LIMIT " << name << " REACHED");
            ROS_ERROR_STREAM("MIN " << name << ": " << var_min.transpose());
            ROS_ERROR_STREAM(name << ": " << var.transpose());
            ROS_ERROR_STREAM("MAX " << name << ": " << var_max.transpose());
            ros::shutdown();
        }
    }
}

