#include <vigir_compliant_ros_controller/InvKinController.h>
#include <vigir_compliant_ros_controller/ConversionHelper.h>
#include <eigen_conversions/eigen_msg.h>

namespace compliant_controller {
    bool InvKinController::init(std::string group_name) {

        robot_model_loader_.reset(new robot_model_loader::RobotModelLoader());
        robot_model_ = robot_model_loader_->getModel();
        robot_state_.reset(new robot_state::RobotState(robot_model_));
        robot_state_->setToDefaultValues();

        // load joint model group
        joint_model_group_ = robot_state_->getJointModelGroup(group_name);
        if (joint_model_group_ == NULL) {
            ROS_ERROR_STREAM("Joint model group '" << group_name << "' doesn't exist.");
            return false;
        }
        joint_names_ = joint_model_group_->getJointModelNames();

        // Retrieve solver
        const kinematics::KinematicsBaseConstPtr& solver = joint_model_group_->getSolverInstance();
        if (!solver){
          ROS_ERROR("No IK solver loaded for group %s, cannot set group configuration via IK.", joint_model_group_->getName().c_str());
          return false;
        }
        tip_frame_ = solver->getTipFrame();

        // resize dynamic variables
        q_.resize(joint_names_.size(), 0); // init with 0
        solution_.resize(joint_names_.size(), 0);
        poses_.resize(joint_names_.size());

        // Debug output
        std::stringstream debug;
        debug << "Initialized inverse kinematics controller with joint group '" << group_name << "'. " << std::endl;
        debug << "Tip frame: " << tip_frame_ << std::endl;
        debug << "Base frame: " << solver->getBaseFrame() << std::endl;
        debug << "Joints:" << std::endl;
        for (unsigned int i = 0; i < joint_names_.size(); i++) {
            debug << i << ": " << joint_names_[i] << std::endl;
        }
        ROS_INFO_STREAM(debug.str());
        return true;
    }

    bool InvKinController::calcInvKin(const Vector6d& xd, VectorNd& joint_positions) {
        Eigen::Affine3d pose;
        ConversionHelper::eigenToEigen(xd, pose);
        geometry_msgs::Pose pose_msg;
        tf::poseEigenToMsg(pose, pose_msg);

        moveit_msgs::MoveItErrorCodes error_code;
        if (!joint_model_group_->getSolverInstance()->searchPositionIK(pose_msg,q_, 0.1, solution_, error_code)) {
            ROS_ERROR_STREAM_THROTTLE(1, "Computing IK failed. Error code: " << error_code.val);
            return false;
        }

        // Check output vector size
        if (q_.size() != joint_positions.size()) {
            ROS_ERROR_STREAM("Number of joints in group (" << q_.size() << ") doesn't match dimension of joint position vector (" << joint_positions.size() << ").");
            return false;
        }

        for (unsigned int i = 0; i < solution_.size(); i++) {
            joint_positions(i) = solution_[i];
        }
        return true;
    }

    bool InvKinController::updateJointState(const VectorNd& q) {
        if (q.size() != q_.size()) {
            ROS_ERROR_STREAM("Given joint state size (" << q.size() << ") doesn't match number of joints (" << q_.size() << ").");
            return false;
        }
        for (unsigned int i = 0; i < q.size(); i++) {
            q_[i] = q(i);
        }
        return true;
    }

    bool InvKinController::getTipTransform(Eigen::Affine3d& tip_transform) {
        if (!joint_model_group_->getSolverInstance()->getPositionFK(joint_names_,q_,poses_)) {
            ROS_ERROR("Computing FK failed.");
            return false;
        }
        tf::poseMsgToEigen(poses_[poses_.size()-1],tip_transform);
    }
}