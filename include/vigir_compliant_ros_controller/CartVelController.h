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

#include <vigir_compliant_ros_controller/CustomTypes.h>

namespace compliant_controller {
    class CartVelController {
    public:
        CartVelController() : initialized_(false) {}
        bool init(ros::NodeHandle& nh, std::string root_name, std::string endeffector_name);
        bool update(const KDL::Twist &command, VectorNd& velocities);
        void updatePosition(const VectorNd& position);
        void getTipPose(KDL::Frame& pose);
    private:
        void setCommand(const KDL::Twist& command);
        bool initialized_;

        std::string root_name_;
        std::string endeffector_name_;

        KDL::Tree kdl_tree_;
        KDL::Chain kdl_chain_;

        boost::scoped_ptr<KDL::ChainIkSolverVel> chain_ik_solver_vel_;
        boost::scoped_ptr<KDL::ChainFkSolverPos> chain_fk_solver_;

        KDL::Twist cmd_linear_twist_;
        KDL::Twist cmd_angular_twist_;

        boost::scoped_ptr<KDL::JntArray> q_;
    };

}

#endif
