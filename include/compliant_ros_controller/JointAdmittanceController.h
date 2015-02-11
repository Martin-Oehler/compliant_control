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
        bool init(std::string root_name, std::string tip_name, Vector6d& cart_inertia, Vector6d& cart_damping, Vector6d& cart_stiffness);
        bool init(std::string root_name, std::string tip_name, double cart_inertia, double cart_damping, double cart_stiffness);
        void updateJointState(VectorNd &q);
        void calcCompliantPosition(const VectorNd& q0, const Vector6d& fext, VectorNd& qd_out, VectorNd& qdotd_out, double step_size);
        Vector3d getTipPosition(const VectorNd& q);

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    private:
        void getE1(VectorNd &e1);
        void getE2(VectorNd &e2);
        VectorNd f(const VectorNd& fext);

        bool initialized_;

        VectorNd e_;
        VectorNd e1_;
        VectorNd e2_;

        Vector6d inertia_;
        Vector6d damping_;
        Vector6d stiffness_;

        VectorNd q_;

        std::string root_name_;
        std::string tip_name_;

        KDL::Tree  kdl_tree_;
        KDL::Chain kdl_chain_;

        boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_;
        boost::scoped_ptr<KDL::ChainFkSolverPos>    jnt_to_pose_solver_;

        KDL::Jacobian Jtmp_;
        KDL::JntArray qtmp_;
    };
}

#endif
