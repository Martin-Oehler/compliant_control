#ifndef TJAC_INV_KIN_CONTROLLER_H
#define TJAC_INV_KIN_CONTROLLER_H

#include <ros/ros.h>

#include <boost/scoped_ptr.hpp>

#include <vigir_compliant_ros_controller/CustomTypes.h>

// KDL
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>

namespace compliant_controller {
    class TJacInvKinController {
    public:
        bool init(const ros::NodeHandle& nh, const std::string root_name, const std::string tip_name);
        void starting();
        bool update(const Vector6d& xd, VectorNd& joint_positions);
        void stopping();
        bool updateJointState(const VectorNd& position);
        bool getTipPose(Transform& pose);

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    private:
        bool initialized_;

        std::string root_name_;
        std::string tip_name_;

        KDL::Tree  kdl_tree_;
        KDL::Chain kdl_chain_;
        unsigned int num_joints_;

        boost::scoped_ptr<KDL::ChainFkSolverPos>    jnt_to_pose_solver_;
        boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_;

        // robot state
        VectorNd q_;
        VectorNd qdot_;
        Jacobian J_;

        // pre-allocated temp variables
        VectorNd qstep_;
        KDL::Frame xtmp_;
        KDL::Jacobian Jtmp_;
        KDL::JntArray qtmp_;
    };
}
#endif
