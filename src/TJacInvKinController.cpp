#include <vigir_compliant_ros_controller/TJacInvKinController.h>

#include <kdl_parser/kdl_parser.hpp>
#include <vigir_compliant_ros_controller/ConversionHelper.h>

namespace compliant_controller {
bool TJacInvKinController::init(const ros::NodeHandle& nh, const std::string root_name, const std::string tip_name) {
    std::string robot_description;

    if (!nh.getParam("/robot_description", robot_description)) {
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

   // resize dynamic objects
   num_joints_ = kdl_chain_.getNrOfJoints();
   q_.resize(num_joints_);
   qstep_.resize(num_joints_);
   qdot_.resize(num_joints_);
   Jtmp_.resize(num_joints_);
   qtmp_.resize(num_joints_);

   std::stringstream debug;
   debug << "Initialized Transpose Jacobian Inverse Kinematics controller with segments (joint): " << std::endl;
   for (unsigned int i = 0; i < kdl_chain_.getNrOfJoints(); i++) {
       debug << i << ": " << kdl_chain_.getSegment(i).getName() << " (" << kdl_chain_.getSegment(i).getJoint().getName() << ")" << std::endl;
   }
   std::cout << debug.str();
   initialized_ = true;
   return true;
}

void TJacInvKinController::starting() {

}

bool TJacInvKinController::update(const Vector6d &xd, VectorNd &joint_positions) {
    // Calc current end-effector position
    Vector6d x;
    ConversionHelper::eigenToKdl(q_,qtmp_);
    if (jnt_to_pose_solver_->JntToCart(qtmp_, xtmp_) < 0) {
        ROS_ERROR("Failed to compute forward kinematics.");
        return false;
    }
    ConversionHelper::kdlToEigen(xtmp_, x);

    // Calc jacobian
    jnt_to_jac_solver_->JntToJac(qtmp_,Jtmp_);
    ConversionHelper::kdlToEigen(Jtmp_, J_);

    // calculate error
    Vector6d pos_error = xd - x;

    // calculate alpha
    double num = pos_error.dot(J_ * J_.transpose() * pos_error);
    double denom = (J_ * J_.transpose() * pos_error).dot(J_ * J_.transpose() * pos_error);
    double alpha = num / denom;

    // calculate q step;
    qstep_ = alpha * J_.transpose() * pos_error;
    if (joint_positions.size() != num_joints_) {
        ROS_ERROR_STREAM("Position output vector (size " << joint_positions.size() << ") doesn't have the right dimension (" << num_joints_ << ").");
        return false;
    }
    joint_positions = joint_positions + qstep_;
    return true;
}

void TJacInvKinController::stopping() {

}

bool TJacInvKinController::updateJointState(const VectorNd& position) {
    if (position.size() == num_joints_) {
        q_ = position;
        return true;
    } else {
        ROS_ERROR_STREAM("Position update vector (size " << position.size() << ") doesn't have the right dimension (" << num_joints_ << ").");
        return false;
    }
}

bool TJacInvKinController::getTipPose(Transform& pose) {
    ConversionHelper::eigenToKdl(q_, qtmp_);
    jnt_to_pose_solver_->JntToCart(qtmp_,xtmp_);
    ConversionHelper::kdlToEigen(xtmp_,pose);
}

}
