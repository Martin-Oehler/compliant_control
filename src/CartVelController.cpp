#include <vigir_compliant_ros_controller/CartVelController.h>

#include <kdl_parser/kdl_parser.hpp>

namespace control {
     bool CartVelController::init(ros::NodeHandle& nh, std::string root_name, std::string endeffector_name) {
         std::string robot_description;

         if (!nh.getParam("robot_description", robot_description)) {
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
        q_.reset(new KDL::JntArray(kdl_chain_.getNrOfJoints()));
        std::stringstream debug;
        debug << "Initialized Cartesian controller with segments (joint): " << std::endl;
        for (unsigned int i = 0; i < kdl_chain_.getNrOfJoints(); i++) {
            debug << i << ": " << kdl_chain_.getSegment(i).getName() << " (" << kdl_chain_.getSegment(i).getJoint().getName() << ")" << std::endl;
        }
        std::cout << debug.str();
        initialized = true;
        return true;
     }

    bool CartVelController::update(const KDL::Twist& command, VectorNd& velocities) {
        // init
        if (!initialized) {
            ROS_ERROR("Controller wasn't initilized before calling 'update'.");
            return false;
        }
        setCommand(command);
        velocities.resize(kdl_chain_.getNrOfJoints());
        for (unsigned int i = 0; i < kdl_chain_.getNrOfJoints(); i++) {
            velocities(i) = 0.0;
        }

        // forward kinematics
        KDL::Frame frame_tip_pose;
        if(chain_fk_solver_->JntToCart(*q_, frame_tip_pose) < 0) {
            ROS_ERROR("Unable to compute forward kinematics");
            return false;
        }

        // transform vel command to root frame // not necessary, command is in root frame
//        KDL::Frame frame_tip_pose_inv = frame_tip_pose.Inverse();
//        KDL::Twist linear_twist = frame_tip_pose_inv * cmd_linear_twist_;
//        KDL::Twist angular_twist = frame_tip_pose_inv.M * cmd_angular_twist_;
//        KDL::Twist twist(linear_twist.vel, angular_twist.rot);
        KDL::Twist twist(cmd_linear_twist_.vel, cmd_angular_twist_.rot);

        // cartesian to joint space
        KDL::JntArray joint_vel(kdl_chain_.getNrOfJoints());
        if(chain_ik_solver_vel_->CartToJnt(*q_, twist, joint_vel) < 0) {
            ROS_ERROR("Unable to compute cartesian to joint velocity");
            return false;
        }

        // assign values to output.
        std::stringstream debug;
        debug << "Computed velocities per joint: " << std::endl;
        for (unsigned int i = 0; i < kdl_chain_.getNrOfJoints(); i++) {
            velocities(i) = joint_vel(i);
            debug << kdl_chain_.getSegment(i).getJoint().getName() << "\t\t" << velocities(i) << std::endl;
        }
        ROS_INFO_STREAM_THROTTLE(1,debug.str());
        return true;
    }

    void CartVelController::updatePosition(const VectorNd& position) {
        if (q_->rows() != position.size()) {
            ROS_ERROR("Given position vector doesn't match chain size");
            return;
        }
        for (unsigned int i = 0; i < q_->rows(); i++) {
            (*q_)(i) = position(i);
        }
    }

    void CartVelController::setCommand(const KDL::Twist &command) {
        for (unsigned int i = 0; i < 3; i++) {
            cmd_linear_twist_.vel(i) = command.vel(i);
            cmd_linear_twist_.rot(i) = 0.0;

            cmd_angular_twist_.rot(i) = command.rot(i);
            cmd_angular_twist_.vel(i) = 0.0;
        }
    }
}
