#ifndef CART_FORCE_CONTROLLER_H
#define CART_FORCE_CONTROLLER_H

#include <vigir_compliant_ros_controller/CustomTypes.h>
#include <ros/ros.h>
#include <boost/scoped_ptr.hpp>
#include <atlas_msgs/AtlasState.h>

#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>

namespace compliant_controller {
    class CartForceController {
    public:
        CartForceController() : initialized_(false), jac_updated_(false) {}
        bool init(const ros::NodeHandle& node, const std::string& root_name, const std::string& tip_name, const Matrix6d& Kp, const Matrix6d& Kd, const Matrix6d& Ki, double step_size);
        bool init(const ros::NodeHandle& node, const std::string& root_name, const std::string& tip_name, double kp, double kd, double ki, double step_size);
        void updateJointState(const VectorNd& q, const VectorNd& qdot);
        void updateJointState(const atlas_msgs::AtlasState::ConstPtr &state);
        bool calcCorrectionVector(const Vector6d& xd, const Vector6d& xdotd, Vector6d& force);
        Vector6d calcCorrectionVector(const Vector6d& xd, const Vector6d& xdotd);
        bool calcTorques(const Vector6d& force, VectorNd& torques);
        VectorNd calcTorques(const Vector6d& force);
        // for testing
        void getTipPose(KDL::Frame& pose);
    private:
        void calcCartError(const Vector6d& xd, const Vector6d& x, Vector6d& x_err) const;
        bool initialized_;

        std::string root_name_;
        std::string tip_name_;

        KDL::Tree  kdl_tree_;
        KDL::Chain kdl_chain_;

        // robot state
        boost::scoped_ptr<KDL::JntArray> q_;
        boost::scoped_ptr<KDL::JntArray> qdot_;

        boost::scoped_ptr<KDL::ChainFkSolverPos>    jnt_to_pose_solver_;
        boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_;
        Jacobian J_;
        bool jac_updated_;

        Matrix6d Kp_;
        Matrix6d Kd_;
        Matrix6d Ki_;

        Vector6d integral_;
        double step_size_;
    };
}

#endif
