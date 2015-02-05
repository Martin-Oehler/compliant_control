#ifndef JOINT_ADMITTANCE_CONTROLLER_H
#define JOINT_ADMITTANCE_CONTROLLER_H

#include <compliant_ros_controller/CustomTypes.h>
#include <ros/ros.h>
#include <boost/scoped_ptr.hpp>

#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>

namespace compliant_controller {
    class JointAdmittanceController {
    public:
        JointAdmittanceController() : initialized_(false) {}
        bool init(ros::NodeHandle& node, std::string root_name, std::string tip_name, VectorNd& inertia, VectorNd& damping, VectorNd& stiffness, double step_size);
        //bool init(ros::NodeHandle& nh, std::string root_name, std::string tip_name, double inertia, double damping, double stiffness, double step_size);
        void updateJointState(VectorNd &q);
        void calcCompliantPosition(const VectorNd& q0, const Vector6d& fext, VectorNd& qd_out, VectorNd& qdotd_out);
        Vector3d getTipPosition(const VectorNd& q);

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    private:
        VectorNd getE1() const;
        VectorNd getE2() const;
        VectorNd f(const VectorNd& fext) const;

        bool initialized_;
        VectorNd e_;

        VectorNd inertia_;
        VectorNd damping_;
        VectorNd stiffness_;
        double step_size_;

        VectorNd q_;

        std::string root_name_;
        std::string tip_name_;

        KDL::Tree  kdl_tree_;
        KDL::Chain kdl_chain_;

        boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_;
        boost::scoped_ptr<KDL::ChainFkSolverPos>    jnt_to_pose_solver_;
    };
}

#endif
