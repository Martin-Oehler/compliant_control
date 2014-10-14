#include <vigir_compliant_ros_controller/CartVelController.h>

#include <kdl_parser/kdl_parser.hpp>
#include <vigir_compliant_ros_controller/ConversionHelper.h>

namespace compliant_controller {
     bool CartVelController::init(ros::NodeHandle& nh, std::string root_name, std::string endeffector_name) {
         std::string robot_description;

         if (!nh.getParam("/robot_description", robot_description)) {
             ROS_ERROR("Failed to get robot description from parameter server.");
             return false;
         }

         endeffector_name_ = endeffector_name;
         root_name_ = root_name;

        if (!kdl_parser::treeFromString(robot_description,kdl_tree_)) {
            ROS_ERROR("Error constructing kdl tree from robot_description.");
            return false;
        }

        if (!kdl_tree_.getChain(root_name_,endeffector_name_,kdl_chain_)) {
            ROS_ERROR("Error constructing kdl chain from kdl tree.");
            return false;
        }

        chain_ik_solver_vel_.reset(new KDL::ChainIkSolverVel_pinv(kdl_chain_));
        chain_fk_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
        qtmp_.resize(kdl_chain_.getNrOfJoints());
        joint_vel_tmp_.resize(kdl_chain_.getNrOfJoints());
        q_.resize(kdl_chain_.getNrOfJoints());
        std::stringstream debug;
        debug << "Initialized cartesian velocity controller with segments (joint): " << std::endl;
        for (unsigned int i = 0; i < kdl_chain_.getNrOfJoints(); i++) {
            debug << i << ": " << kdl_chain_.getSegment(i).getName() << " (" << kdl_chain_.getSegment(i).getJoint().getName() << ")" << std::endl;
        }
        std::cout << debug.str();
        initialized_ = true;
        return true;
     }

    bool CartVelController::update(const Vector6d& command, VectorNd& velocities) {
        // init
        if (!initialized_) {
            ROS_ERROR("Controller wasn't initilized before calling 'update'.");
            return false;
        }
        setCommand(command);
        if (velocities.size() != kdl_chain_.getNrOfJoints()) {
            ROS_ERROR_STREAM("Size of velocities vector (" << velocities.size() << ") doesn't match number of joints (" << kdl_chain_.getNrOfJoints() << ").");
            return false;
        }
        for (unsigned int i = 0; i < kdl_chain_.getNrOfJoints(); i++) {
            velocities(i) = 0.0;
        }

        // forward kinematics
//        KDL::Frame frame_tip_pose;
//        if(chain_fk_solver_->JntToCart(*q_, frame_tip_pose) < 0) {
//            ROS_ERROR("Unable to compute forward kinematics");
//            return false;
//        }
        // transform vel command to root frame // not necessary, command is in root frame
//        KDL::Frame frame_tip_pose_inv = frame_tip_pose.Inverse();
//        KDL::Twist linear_twist = frame_tip_pose_inv * cmd_linear_twist_;
//        KDL::Twist angular_twist = frame_tip_pose_inv.M * cmd_angular_twist_;
//        KDL::Twist twist(linear_twist.vel, angular_twist.rot);

        // cartesian to joint space
        ConversionHelper::eigenToKdl(q_, qtmp_);
        ConversionHelper::eigenToKdl(cmd_twist_, twist_tmp_);
        if(chain_ik_solver_vel_->CartToJnt(qtmp_, twist_tmp_, joint_vel_tmp_) < 0) {
            ROS_ERROR("Unable to compute cartesian to joint velocity");
            return false;
        }

        // assign values to output.
        ConversionHelper::kdlToEigen(joint_vel_tmp_, velocities);
        return true;
    }

    void CartVelController::updatePosition(const VectorNd& position) {
        if (q_.size() != position.size()) {
            ROS_ERROR_STREAM("Size of joint position vector (" << position.size() << ") doesn't match chain size (" << kdl_chain_.getNrOfJoints() << ").");
            return;
        }
        q_ = position;
    }

    void CartVelController::setCommand(const Vector6d &command) {
        cmd_twist_ = command;
    }

    void CartVelController::getTipPose(KDL::Frame& pose) {
        ConversionHelper::eigenToKdl(q_, qtmp_);
        chain_fk_solver_->JntToCart(qtmp_, pose);
    }
}
