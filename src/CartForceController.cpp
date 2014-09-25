#include <vigir_compliant_ros_controller/CartForceController.h>

#include <kdl_parser/kdl_parser.hpp>
#include <vigir_compliant_ros_controller/ConversionHelper.h>

namespace compliant_controller {
    bool CartForceController::init(const ros::NodeHandle& node, const std::string& root_name, const std::string& tip_name, double kp, double kd, double ki, double step_size) {
        Vector6d kp_vector;
        Vector6d kd_vector;
        Vector6d ki_vector;
        for (unsigned int i = 0; i < 6; i++) {
            kp_vector(i) = kp;
            kd_vector(i) = kd;
            ki_vector(i) = ki;
        }
        return init(node, root_name, tip_name, kp_vector.asDiagonal(), kd_vector.asDiagonal(), ki_vector.asDiagonal(), step_size);
    }

    bool CartForceController::init(const ros::NodeHandle &node, const std::string &root_name, const std::string &tip_name, const Matrix6d &Kp, const Matrix6d &Kd, const Matrix6d &Ki, double step_size) {
        std::string robot_description;

        if (!node.getParam("/robot_description", robot_description)) {
            ROS_ERROR("Failed to get robot description from parameter server.");
            return false;
        }

        tip_name_ = tip_name;
        root_name_ = root_name;

       if (!kdl_parser::treeFromString(robot_description,kdl_tree_)) {
           ROS_ERROR("Error constructing kdl tree from robot_description.");
           return false;
       }

       if (!kdl_tree_.getChain(root_name_,tip_name_,kdl_chain_)) {
           ROS_ERROR("Error constructing kdl chain from kdl tree.");
           return false;
       }

       jnt_to_pose_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
       jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));

       q_.reset(new KDL::JntArray(kdl_chain_.getNrOfJoints()));
       qdot_.reset(new KDL::JntArray(kdl_chain_.getNrOfJoints()));
       Kp_ = Kp;
       Kd_ = Kd;
       Ki_ = Ki;
       integral_ = Vector6d::Zero();
       step_size_ = step_size;

       std::stringstream debug;
       debug << "Initialized Cartesian controller with segments (joint): " << std::endl;
       for (unsigned int i = 0; i < kdl_chain_.getNrOfJoints(); i++) {
           debug << i << ": " << kdl_chain_.getSegment(i).getName() << " (" << kdl_chain_.getSegment(i).getJoint().getName() << ")" << std::endl;
       }
       std::cout << debug.str();
       initialized_ = true;
       return true;
    }

    void CartForceController::updateJointState(const VectorNd& q, const VectorNd& qdot) {
        jac_updated_ = false;
        if (q_->rows() != q.size() || qdot_->rows() != qdot.size()) {
            return;
        }
        for (unsigned int i = 0; i < q_->rows(); i++) {
            (*q_)(i) = q(i);
            (*qdot_)(i) = qdot(i);
        }
    }

    void CartForceController::updateJointState(const atlas_msgs::AtlasState::ConstPtr &state) {
        if (kdl_chain_.getNrOfJoints() != 6) {
            return ;
        }
        VectorNd q(6);
        for (unsigned int i = 0; i < 6; i++) {
            q(i) = state->position[22+i];
        }
        VectorNd qdot(6);
        for (unsigned int i = 0; i < 6; i++) {
            qdot(i) = state->velocity[22+i];
        }
        updateJointState(q,qdot);
    }

    bool CartForceController::calcCorrectionVector(const Vector6d& xd, const Vector6d& xdotd, Vector6d& force) {
        Vector6d x, xdot;
        VectorNd qdot;
        KDL::Frame xtmp;
        KDL::Jacobian Jtmp(kdl_chain_.getNrOfJoints());
        if (jnt_to_pose_solver_->JntToCart(*q_,xtmp) < 0) {
            ROS_ERROR("Failed to compute forward kinematics.");
            return false;
        }
        ConversionHelper::kdlToEigen(xtmp, x);
        if (!jac_updated_) {
            jnt_to_jac_solver_->JntToJac(*q_,Jtmp);
            ConversionHelper::kdlToEigen(Jtmp, J_);
            jac_updated_ = true;
        }
        ConversionHelper::kdlToEigen(*qdot_, qdot);

        xdot = J_ * qdot;

        // calculate errors
        Vector6d pos_error;
        calcCartError(xd, x, pos_error);
        Vector6d vel_error;
        calcCartError(xdotd, xdot, vel_error);

        // calculate integral
        integral_ = integral_ + step_size_*Ki_*pos_error;
        force = Kp_*pos_error + Kd_*vel_error + integral_;

        return true;
    }

    Vector6d CartForceController::calcCorrectionVector(const Vector6d& xd, const Vector6d& xdotd) {
        Vector6d force;
        calcCorrectionVector(xd, xdotd, force);
        return force;
    }

    VectorNd CartForceController::calcTorques(const Vector6d& force) {
        VectorNd torques(kdl_chain_.getNrOfJoints());
        calcTorques(force, torques);
        return torques;
    }

    bool CartForceController::calcTorques(const Vector6d &force, VectorNd &torques) {
        // initialize output to 0 (in case of error)
        torques.resize(kdl_chain_.getNrOfJoints());
        for (unsigned int i = 0; i < torques.size(); i++) {
            torques(i) = 0;
        }
        KDL::Jacobian Jtmp(kdl_chain_.getNrOfJoints());
        if (!jac_updated_) {
            jnt_to_jac_solver_->JntToJac(*q_,Jtmp);
            ConversionHelper::kdlToEigen(Jtmp, J_);
            jac_updated_ = true;
        }

        // calculate integral
        torques = J_.transpose() * force;

        return true;
    }

    void CartForceController::calcCartError(const Vector6d& xd, const Vector6d& x, Vector6d& x_err) const {
        x_err.block<3,1>(0,0) = xd.block<3,1>(0,0) - x.block<3,1>(0,0);

        KDL::Rotation xd_rot = KDL::Rotation::RPY(xd(3), xd(4), xd(5));
        KDL::Rotation x_rot = KDL::Rotation::RPY(x(3), x(4), x(5));
        KDL::Vector rot_error_tmp = -0.5 * (xd_rot.UnitX() * x_rot.UnitX() + xd_rot.UnitY() * x_rot.UnitY() + xd_rot.UnitZ() * x_rot.UnitZ());
        Vector3d rot_error;
        ConversionHelper::kdlToEigen(rot_error_tmp, rot_error);
        x_err.block<3,1>(3,0) = rot_error;
    }

    void CartForceController::getTipPose(KDL::Frame& pose) {
        jnt_to_pose_solver_->JntToCart(*q_,pose);
    }
}
