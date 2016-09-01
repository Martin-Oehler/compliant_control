#ifndef CART_VEL_CONTROLLER_H
#define CART_VEL_CONTROLLER_H

#include <ros/ros.h>

#include <boost/scoped_ptr.hpp>
#include <kdl/chain.hpp>
#include <kdl/tree.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chainiksolver.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

#include <compliant_ros_controller/CustomTypes.h>

namespace compliant_controller {
    class CartVelController {
    public:
        CartVelController() : initialized_(false) {}
        bool init(ros::NodeHandle& nh, std::string root_name, std::string endeffector_name);
        bool update(const Vector6d &command, VectorNd& velocities);
        void updatePosition(const VectorNd& position);
        void getTipPose(KDL::Frame& pose);

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    private:
        void setCommand(const Vector6d& command);
        bool initialized_;

        std::string root_name_;
        std::string endeffector_name_;

        KDL::Tree kdl_tree_;
        KDL::Chain kdl_chain_;

        boost::scoped_ptr<KDL::ChainIkSolverVel> chain_ik_solver_vel_;
        boost::scoped_ptr<KDL::ChainFkSolverPos> chain_fk_solver_;

        Vector6d cmd_twist_;
        VectorNd q_;
        // pre-allocated variables
        KDL::Twist twist_tmp_;
        KDL::JntArray joint_vel_tmp_;
        KDL::JntArray qtmp_;
    };

}

#endif
