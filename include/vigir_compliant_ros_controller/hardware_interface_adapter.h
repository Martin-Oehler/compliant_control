#ifndef HARDWARE_INTERFACE_ADAPTER_H
#define HARDWARE_INTERFACE_ADAPTER_H

// ROS
#include <ros/node_handle.h>
#include <ros/time.h>

// hardware_interface
#include <hardware_interface/joint_command_interface.h>

// compliant control
#include <vigir_compliant_ros_controller/ConversionHelper.h>
#include <vigir_compliant_ros_controller/CustomTypes.h>
#include <vigir_compliant_ros_controller/CartForceController.h>
#include <vigir_compliant_ros_controller/CartVelController.h>
#include <vigir_compliant_ros_controller/InvKinController.h>

template <class HardwareInterface, class State>
class HardwareInterfaceAdapter {
public:
  bool init(std::vector<std::string> segment_names, std::vector<typename HardwareInterface::ResourceHandleType>& joint_handles, ros::NodeHandle& controller_nh)
  {
    return false;
  }

  void starting(const ros::Time& time) {}
  void stopping(const ros::Time& time) {}

  void updateCommand(const ros::Time&     time,
                     const ros::Duration& period,
                     const State&         desired_state) {}
  compliant_controller::Matrix3d getTipRotation() {return compliant_controller::Matrix3d::Identity();}
};

template <>
class HardwareInterfaceAdapter<hardware_interface::VelocityJointInterface, compliant_controller::CartState> {
public:
    bool init(std::vector<std::string> segment_names, std::vector<hardware_interface::JointHandle>& joint_handles, ros::NodeHandle& controller_nh) {
        joint_handles_ptr_ = &joint_handles;
        // resize pre-allocated variables
        velocities_.resize(joint_handles_ptr_->size());
        joint_velocities_.resize(joint_handles_ptr_->size());
        joint_positions_.resize(joint_handles_ptr_->size());

        // init controllers
        if (!cart_force_controller_.init(controller_nh, segment_names[0], segment_names[segment_names.size()-1], 10, 0, 0)) {
            return false;
        }
        if (!cart_vel_controller_.init(controller_nh, segment_names[0], segment_names[segment_names.size()-1])) {
            return false;
        }
        return true;
    }

    void starting(const ros::Time& time) {}
    void stopping(const ros::Time& time) {
        for (unsigned int i = 0; i < joint_handles_ptr_->size(); i++) {
            (*joint_handles_ptr_)[i].setCommand(0);
        }
    }

    void updateCommand(const ros::Time& time, const ros::Duration& period, const compliant_controller::CartState& desired_state) {
        updateJointState();
        // calculate correction vector in cartesian space
        compliant_controller::Vector6d twist;
        cart_force_controller_.calcCorrectionVector(desired_state.position, desired_state.velocity, twist, period.toSec());
        //ROS_INFO_STREAM("calculated twist: " << std::endl << twist);
        // transform cartesian vector to joint velocities
        cart_vel_controller_.update(twist, velocities_);

        // assign joint velocity command
        for (unsigned int i = 0; i < joint_handles_ptr_->size(); i++) {
            (*joint_handles_ptr_)[i].setCommand(velocities_(i));
        }
        //ROS_INFO_STREAM("joint velocity cmds: " << std::endl << velocities_);
    }

    // not realtime safe
    compliant_controller::Matrix3d getTipRotation() {
        updateJointState();
        KDL::Frame pose;
        cart_force_controller_.getTipPose(pose);
        compliant_controller::Matrix3d matrix;
        compliant_controller::ConversionHelper::kdlToEigen(pose.M, matrix);
        return matrix;
    }

private:
    void updateJointState() {
        // update joint state
        for (unsigned int i = 0; i < joint_positions_.size(); i++) {
            joint_positions_(i) = (*joint_handles_ptr_)[i].getPosition();
            joint_velocities_(i) = (*joint_handles_ptr_)[i].getVelocity();
        }
        cart_force_controller_.updateJointState(joint_positions_, joint_velocities_);
        cart_vel_controller_.updatePosition(joint_positions_);
    }

    std::vector<hardware_interface::JointHandle>* joint_handles_ptr_;
    compliant_controller::CartForceController cart_force_controller_;
    compliant_controller::CartVelController cart_vel_controller_;

    //pre-allocated variables
    compliant_controller::VectorNd velocities_;
    compliant_controller::VectorNd joint_positions_;
    compliant_controller::VectorNd joint_velocities_;
};

template <>
class HardwareInterfaceAdapter<hardware_interface::EffortJointInterface, compliant_controller::CartState>  {
public:
     bool init(std::vector<std::string> segment_names, std::vector<hardware_interface::JointHandle>& joint_handles, ros::NodeHandle& controller_nh) {
        joint_handles_ptr_ = &joint_handles;
        // resize pre-allocated variables
        torques_.resize(joint_handles_ptr_->size());
        joint_velocities_.resize(joint_handles_ptr_->size());
        joint_positions_.resize(joint_handles_ptr_->size());
        // init controller
        if (!cart_force_controller_.init(controller_nh, segment_names[0], segment_names[segment_names.size()-1], 10, 0, 0)) {
            return false;
        }
        return true;
     }

     void starting(const ros::Time& time) {}
     void stopping(const ros::Time& time) {
         for (unsigned int i = 0; i < joint_handles_ptr_->size(); i++) {
             (*joint_handles_ptr_)[i].setCommand(0);
         }
     }

     void updateCommand(const ros::Time& time, const ros::Duration& period, const compliant_controller::CartState& desired_state) {
         // calculate correction force in cartesian space
         compliant_controller::Vector6d force;
         cart_force_controller_.calcCorrectionVector(desired_state.position, desired_state.velocity, force, period.toSec());

         // map force to joint efforts
         cart_force_controller_.calcTorques(force, torques_);

         // assign effort cmd
         for (unsigned int i = 0; i < joint_handles_ptr_->size(); i++) {
             (*joint_handles_ptr_)[i].setCommand(torques_(i));
         }

     }

     compliant_controller::Matrix3d getTipRotation() {
         updateJointState();
         KDL::Frame pose;
         cart_force_controller_.getTipPose(pose);
         compliant_controller::Matrix3d matrix;
         compliant_controller::ConversionHelper::kdlToEigen(pose.M, matrix);
         return matrix;
     }
private:
     void updateJointState() {
         // update joint state
         for (unsigned int i = 0; i < joint_positions_.size(); i++) {
             joint_positions_(i) = (*joint_handles_ptr_)[i].getPosition();
             joint_velocities_(i) = (*joint_handles_ptr_)[i].getVelocity();
         }
         cart_force_controller_.updateJointState(joint_positions_, joint_velocities_);
     }

    std::vector<hardware_interface::JointHandle>* joint_handles_ptr_;
    compliant_controller::CartForceController cart_force_controller_;

    //pre-allocated variables
    compliant_controller::VectorNd torques_;
    compliant_controller::VectorNd joint_positions_;
    compliant_controller::VectorNd joint_velocities_;
};

template <>
class HardwareInterfaceAdapter<hardware_interface::PositionJointInterface, compliant_controller::CartState> {
public:
  bool init(std::vector<std::string> segment_names, std::vector<hardware_interface::JointHandle>& joint_handles, ros::NodeHandle& controller_nh)
  {
      joint_handles_ptr_ = &joint_handles;
      joint_positions_.resize(joint_handles.size());
      joint_cmds_.resize(joint_handles.size());
      if (!inv_kin_controller_.init("r_arm_group")) {
          return false;
      }
    return true;
  }

  void starting(const ros::Time& time) {}
  void stopping(const ros::Time& time) {}

  void updateCommand(const ros::Time& time, const ros::Duration& period, const compliant_controller::CartState& desired_state) {
      for (unsigned int i = 0; i < joint_positions_.size(); i++) {
          joint_positions_(i) = (*joint_handles_ptr_)[i].getPosition();
      }
      inv_kin_controller_.updateJointState(joint_positions_);
      inv_kin_controller_.calcInvKin(desired_state.position, joint_cmds_);
      for (unsigned int i = 0; i < joint_cmds_.size(); i++) {
          (*joint_handles_ptr_)[i].setCommand(joint_cmds_(i));
      }
  }
  compliant_controller::Matrix3d getTipRotation() {
      for (unsigned int i = 0; i < joint_positions_.size(); i++) {
          joint_positions_(i) = (*joint_handles_ptr_)[i].getPosition();
      }
      inv_kin_controller_.updateJointState(joint_positions_);

      Eigen::Affine3d pose;
      inv_kin_controller_.getTipTransform(pose);
      return pose.rotation();
  }

private:
  std::vector<hardware_interface::JointHandle>* joint_handles_ptr_;
  compliant_controller::InvKinController inv_kin_controller_;

  //pre-allocated variables
  compliant_controller::VectorNd joint_positions_;
  compliant_controller::VectorNd joint_cmds_;
};


#endif
