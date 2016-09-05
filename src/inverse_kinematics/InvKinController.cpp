#include <compliant_ros_controller/inverse_kinematics/InvKinController.h>
#include <compliant_ros_controller/ConversionHelper.h>
#include <eigen_conversions/eigen_msg.h>

#include <math.h>

namespace compliant_controller {
    bool InvKinController::init(std::string group_name) {
        publish_state_ = false;

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

    bool InvKinController::calcInvKin(const ros::Time &time, const Vector6d& xd, VectorNd& joint_positions) {
        calcInvKin(time, xd, solution_);

        for (unsigned int i = 0; i < solution_.size(); i++) {
            joint_positions(i) = solution_[i];
        }

        return true;
    }

    bool InvKinController::calcInvKin(const ros::Time &time, const Vector6d &xd, std::vector<double> &joint_positions) {
        Eigen::Affine3d pose;
        ConversionHelper::eigenToEigen(xd, pose);
        geometry_msgs::Pose pose_msg;
        tf::poseEigenToMsg(pose, pose_msg);

        moveit_msgs::MoveItErrorCodes error_code;
        if (!joint_model_group_->getSolverInstance()->searchPositionIK(pose_msg,q_, 0.0003, solution_, error_code)) {
            ROS_WARN_STREAM_THROTTLE(1, "Computing IK from " << joint_model_group_->getSolverInstance()->getBaseFrame() << " to " <<
                                     joint_model_group_->getSolverInstance()->getTipFrame() << " failed. Error code: " << error_code.val << " (" << moveitErrCodeToString(error_code.val) << ")");
            return false;
        }

        double limit = 10 * M_PI / 180;

        // find maximum position change
        double min_change_factor = 1;
        double requested_change = 0;
        for (unsigned int i = 0; i < solution_.size(); i++) {
            double change = std::abs(q_[i] - solution_[i]);
            if (change != 0.0) {
                double factor = limit/change;
                if (factor < min_change_factor) {
                    min_change_factor = factor;
                    requested_change = change;
                }
            }
        }

        // limit joint angle change
        if (min_change_factor < 1) {
            for (unsigned int i = 0; i < solution_.size(); i++) {
                double change = solution_[i] - q_[i];
                solution_[i] =  q_[i] + (change * min_change_factor);
            }
            ROS_WARN_STREAM_THROTTLE(1,"Joint angle change (" << requested_change << ") bigger than max (" << limit << "). Limiting speed with factor: " << min_change_factor << ".");
            std::stringstream debug;
            debug << "Current\t | \t Requested" << std::endl;
            for (unsigned int i = 0; i < solution_.size(); i++) {
                debug << q_[i] << "\t\t" << solution_[i] << std::endl;
            }
            ROS_WARN_STREAM_THROTTLE(1,debug.str());
        }

        // Check output vector size
        if (q_.size() != joint_positions.size()) {
            ROS_ERROR_STREAM("Number of joints in group (" << q_.size() << ") doesn't match dimension of joint position vector (" << joint_positions.size() << ").");
            return false;
        }
        joint_positions = solution_;

        if (publish_state_) {
            publishState(time, joint_positions);
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
    bool InvKinController::updateJointState(const std::vector<double>& q) {
        if (q.size() != q_.size()) {
            ROS_ERROR_STREAM("Given joint state size (" << q.size() << ") doesn't match number of joints (" << q_.size() << ").");
            return false;
        }
        q_ = q;
        return true;
    }

    bool InvKinController::getTipTransform(Eigen::Affine3d& tip_transform) {
        if (!joint_model_group_->getSolverInstance()->getPositionFK(joint_model_group_->getSolverInstance()->getTipFrames(), q_,poses_)) {
            ROS_ERROR_STREAM("Computing FK failed.");
            return false;
        }
        tf::poseMsgToEigen(poses_[0],tip_transform);
    }

    void InvKinController::activateStatePublishing(ros::NodeHandle& nh) {
        joint_state_publisher_ = nh.advertise<sensor_msgs::JointState>("joint_cmd", 1000);
        publish_state_ = true;
        seq_counter_ = 0;
    }

    void InvKinController::publishState(const ros::Time& time, const VectorNd& state) {
        sensor_msgs::JointState state_msg;
        state_msg.header.stamp = time;
        state_msg.header.seq = seq_counter_; seq_counter_++;
        state_msg.position.resize(state.size());
        state_msg.effort.resize(state.size());
        state_msg.velocity.resize(state.size());
        for (unsigned int i = 0; i < state.size(); i++) {
            state_msg.position[i] = state(i);
            state_msg.effort[i] = 0;
            state_msg.velocity[i] = 0;
        }
        joint_state_publisher_.publish(state_msg);
    }

   void InvKinController::publishState(const ros::Time &time, const std::vector<double>& state) {
       sensor_msgs::JointState state_msg;
       state_msg.header.stamp = time;
       state_msg.header.seq = seq_counter_; seq_counter_++;
       state_msg.position = state;
       state_msg.effort.resize(state.size(), 0);
       state_msg.velocity.resize(state.size(), 0);
       joint_state_publisher_.publish(state_msg);
   }

   std::string InvKinController::moveitErrCodeToString(int32_t code) {
      switch (code) {
        case moveit_msgs::MoveItErrorCodes::FAILURE:
            return "FAILURE";
        case moveit_msgs::MoveItErrorCodes::TIMED_OUT:
            return "TIME_OUT";
        default:
            return "";
      }
      return "";
   }
}
