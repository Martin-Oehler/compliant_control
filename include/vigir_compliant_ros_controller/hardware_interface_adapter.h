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

// atlas
#include <vigir_atlas_interfaces/vigir_atlas_joint_iface_adapter.h>

// reflexxes
#include<ReflexxesAPI.h>
#include <urdf/model.h>

namespace compliant_controller {

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
  Transform getTipPose() {return Transform();}
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
    Transform getTipPose() {
        updateJointState();
        KDL::Frame pose;
        cart_force_controller_.getTipPose(pose);
        Transform transform;
        compliant_controller::ConversionHelper::kdlToEigen(pose, transform);
        return transform;
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

     Transform getTipPose() {
         updateJointState();
         KDL::Frame pose;
         cart_force_controller_.getTipPose(pose);
         Transform transform;
         compliant_controller::ConversionHelper::kdlToEigen(pose, transform);
         return transform;
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
      // resize pre-allocated variables
      joint_handles_ptr_ = &joint_handles;
      joint_positions_.resize(joint_handles.size());
      joint_cmds_.resize(joint_handles.size());
      // init IK
      std::string moveit_group;
      if (!controller_nh.getParam("moveit_group", moveit_group)) {
        ROS_ERROR_STREAM("Couldn't find param 'moveit_group' in namespace " << controller_nh.getNamespace() << ".");
        return false;
      }

      if (!inv_kin_controller_.init(moveit_group)) { // add moveit group to config
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
  Transform getTipPose() {
      for (unsigned int i = 0; i < joint_positions_.size(); i++) {
          joint_positions_(i) = (*joint_handles_ptr_)[i].getPosition();
      }
      inv_kin_controller_.updateJointState(joint_positions_);

      Eigen::Affine3d pose;
      inv_kin_controller_.getTipTransform(pose);
      Transform transform;
      transform.rotation = pose.rotation();
      transform.translation = pose.translation();
      return transform;
  }

private:
  std::vector<hardware_interface::JointHandle>* joint_handles_ptr_;
  compliant_controller::InvKinController inv_kin_controller_;

  //pre-allocated variables
  compliant_controller::VectorNd joint_positions_;
  compliant_controller::VectorNd joint_cmds_;
};

template <>
class HardwareInterfaceAdapter<hardware_interface::VigirAtlasJointInterface, CartState> {
public:
  bool init(std::vector<std::string> segment_names, std::vector<hardware_interface::VigirAtlasJointHandle>& joint_handles, ros::NodeHandle& controller_nh)
  {
      // resize pre-allocated variables
      joint_handles_ptr_ = &joint_handles;
      joint_positions_.resize(joint_handles.size());
      joint_cmds_.resize(joint_handles.size());
      desired_joint_state_ = JointState(joint_handles.size());
      state_error_ = JointState(joint_handles.size());

      // init IK controller
      if (!controller_nh.getParam("moveit_group", moveit_group_)) {
        ROS_ERROR_STREAM("Couldn't find param 'moveit_group' in namespace " << controller_nh.getNamespace() << ".");
        return false;
      }

      if (!inv_kin_controller_.init(moveit_group_)) { // add moveit group to config
          return false;
      }
      // init reflexxes
      if (!setUpRML(controller_nh)) {
          return false;
      }

      // init vigir joint interface adapter
    return joint_if_adapter_.init(joint_handles,controller_nh);
  }

  void starting(const ros::Time& time) {
      joint_if_adapter_.starting(time);
      for (unsigned int i = 0; i < joint_handles_ptr_->size(); i++) {
          desired_joint_state_.position[i] = (*joint_handles_ptr_)[i].getPosition();
          desired_joint_state_.velocity[i] = 0.0;
          desired_joint_state_.acceleration[i] = 0.0;
      }
  }
  void stopping(const ros::Time& time) {joint_if_adapter_.stopping(time);}

  void updateCommand(const ros::Time& time, const ros::Duration& period, const CartState& desired_state) {
      for (unsigned int i = 0; i < joint_positions_.size(); i++) {
          joint_positions_(i) = (*joint_handles_ptr_)[i].getPosition();
      }
      inv_kin_controller_.updateJointState(joint_positions_);
  /*    if (!inv_kin_controller_.calcInvKin(desired_state.position, joint_cmds_)) {
          ROS_ERROR_STREAM_THROTTLE(1, moveit_group_ << ": Desired state: " << std::endl << desired_state.position);
          ROS_ERROR_STREAM_THROTTLE(1, moveit_group_ << ": Joint state: " << std::endl << joint_positions_);
      }


      calcTrajectory(); // call RML */

      for (unsigned int i = 0; i < joint_cmds_.size(); i++) {
          state_error_.position[i] = desired_joint_state_.position[i] - joint_positions_(i);
          state_error_.velocity[i] = desired_joint_state_.velocity[i] - (*joint_handles_ptr_)[i].getVelocity();
          state_error_.acceleration[i] = desired_joint_state_.acceleration[i] - 0;
      }
      joint_if_adapter_.updateCommand(time, period, desired_joint_state_, state_error_);
  }

  Transform getTipPose() {
      for (unsigned int i = 0; i < joint_positions_.size(); i++) {
          joint_positions_(i) = (*joint_handles_ptr_)[i].getPosition();
      }
      inv_kin_controller_.updateJointState(joint_positions_);

      Eigen::Affine3d pose;
      inv_kin_controller_.getTipTransform(pose);
      Transform transform;
      transform.rotation = pose.rotation();
      transform.translation = pose.translation();
      return transform;
  }


private:
  bool setUpRML(ros::NodeHandle& controller_nh) {
      rml_.reset(new ReflexxesAPI(joint_handles_ptr_->size(), 0.001)); // TODO check sampling size
      rml_in_.reset(new RMLPositionInputParameters(joint_handles_ptr_->size()));
      rml_out_.reset(new RMLPositionOutputParameters(joint_handles_ptr_->size()));

      // max values
      urdf::Model urdf_model;
      std::string robot_description;
      controller_nh.getParam("/robot_description", robot_description);
      if (!urdf_model.initString(robot_description)) {
          ROS_ERROR_STREAM("Failed to get robot_description from parameter server.");
          return false;
      }

      for (unsigned int i; i < joint_handles_ptr_->size(); i++) {
          boost::shared_ptr<const urdf::Joint> joint = urdf_model.getJoint((*joint_handles_ptr_)[i].getName());
          if (!joint) {
              ROS_ERROR_STREAM("Couldn't find joint " << (*joint_handles_ptr_)[i].getName() << " in urdf model.");
              return false;
          }
          rml_in_->MaxVelocityVector->VecData[i] = joint->limits->velocity;
          rml_in_->MaxAccelerationVector->VecData[i]  = 10; // TODO set legit value
          rml_in_->MaxJerkVector->VecData[i] = 10; // TODO set legit value
          rml_in_->SelectionVector->VecData[i] = true;
      }
      if (rml_in_->CheckForValidity()) {
          ROS_INFO_STREAM("RML INPUT valid");
          return true;
      } else {
          ROS_ERROR_STREAM("RML INPUT invalid");
          return false;
      }

  }

  bool calcTrajectory() {
      // set up input
      for (unsigned int i = 0; i < joint_handles_ptr_->size(); i++) {
          // set target
          rml_in_->TargetPositionVector->VecData[i] = joint_cmds_(i);
          rml_in_->TargetVelocityVector->VecData[i] = 0.0;
          // set current state
          rml_in_->CurrentPositionVector->VecData[i] = joint_positions_(i);
          rml_in_->CurrentVelocityVector->VecData[i] = (*joint_handles_ptr_)[i].getVelocity();
          rml_in_->CurrentAccelerationVector->VecData[i] = 0.0;
      }

      // calc trajectory
      int result_value = rml_->RMLPosition(*rml_in_.get(),rml_out_.get(),rml_flags_);
      if (result_value < 0) {
          ROS_ERROR_STREAM("An error occured while calculating trajectory: " << result_value);
          return false;
      }
      // write output to desired_joint_state
      for (unsigned int i = 0; i < joint_handles_ptr_->size(); i++) {
          desired_joint_state_.position[i] = rml_out_->NewPositionVector->VecData[i];
          desired_joint_state_.velocity[i] = rml_out_->NewVelocityVector->VecData[i];
          desired_joint_state_.acceleration[i] = rml_out_->NewAccelerationVector->VecData[i];
      }
      return true;
  }
  std::string moveit_group_;

  std::vector<hardware_interface::VigirAtlasJointHandle>* joint_handles_ptr_;
  compliant_controller::InvKinController inv_kin_controller_;

  //pre-allocated variables
  JointState desired_joint_state_;
  JointState state_error_;
  VectorNd joint_positions_;
  VectorNd joint_cmds_;

  // reflexxes
  boost::shared_ptr<ReflexxesAPI> rml_;
  boost::shared_ptr<RMLPositionInputParameters> rml_in_;
  boost::shared_ptr<RMLPositionOutputParameters> rml_out_;
  RMLPositionFlags rml_flags_;

  // vigir atlas joint interface adapter
  ::HardwareInterfaceAdapter<hardware_interface::VigirAtlasJointInterface, JointState> joint_if_adapter_;
};
}
#endif
