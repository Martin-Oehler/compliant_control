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
                     const State&         desired_state,
                     const State&         state_error) {}
//private:
//    compliant_controller::CartForceController cart_force_controller_;
//    compliant_controller::CartVelController cart_vel_controller_;
};

template <>
class HardwareInterfaceAdapter<hardware_interface::VelocityJointInterface, compliant_controller::CartState> {
public:
    bool init(std::vector<std::string> segment_names, std::vector<hardware_interface::JointHandle>& joint_handles, ros::NodeHandle& controller_nh) {
        joint_handles_ptr_ = &joint_handles;
        if (!cart_force_controller_.init(controller_nh, segment_names[0], segment_names[segment_names.size()-1], 10, 1, 0, 0.001)) {
            return false;
        }
        if (!cart_vel_controller_.init(controller_nh, segment_names[0], segment_names[segment_names.size()-1])) {
            return false;
        }
    }

    void starting(const ros::Time& time) {}
    void stopping(const ros::Time& time) {}

    void updateCommand(const ros::Time&     time,
                       const ros::Duration& period,
                       const compliant_controller::CartState&         desired_state,
                       const compliant_controller::CartState&         state_error) {
        // update joint state
        compliant_controller::VectorNd joint_positions(joint_handles_ptr_->size());
        compliant_controller::VectorNd joint_velocities(joint_handles_ptr_->size());
        for (unsigned int i = 0; i < joint_positions.size(); i++) {
            joint_positions(i) = (*joint_handles_ptr_)[i].getPosition();
            joint_velocities(i) = (*joint_handles_ptr_)[i].getVelocity();
        }
        cart_force_controller_.updateJointState(joint_positions, joint_velocities);
        cart_vel_controller_.updatePosition(joint_positions);

        // calculate correction direction in cartesian space
        compliant_controller::Vector6d twist;
        KDL::Twist twist_kdl;
        cart_force_controller_.calcCorrectionVector(desired_state.position, desired_state.velocity, twist);
        compliant_controller::ConversionHelper::eigenToKdl(twist, twist_kdl);

        // transform cartesian vector to joint velocities
        compliant_controller::VectorNd velocities(joint_handles_ptr_->size());
        cart_vel_controller_.update(twist_kdl, velocities);

        for (unsigned int i = 0; i < joint_handles_ptr_->size(); i++) {
            (*joint_handles_ptr_)[i].setCommand(velocities(i));
        }
    }

private:
    std::vector<hardware_interface::JointHandle>* joint_handles_ptr_;
    compliant_controller::CartForceController cart_force_controller_;
    compliant_controller::CartVelController cart_vel_controller_;
};


#endif
