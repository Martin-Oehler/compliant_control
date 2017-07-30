///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2014, TU Darmstadt
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of TU Darmstadt. nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//////////////////////////////////////////////////////////////////////////////

/// \author Martin Oehler, Stefan Kohlbrecher

#ifndef COMPLIANT_CONTROLLER_H
#define COMPLIANT_CONTROLLER_H

// C++ standard
#include <cassert>
#include <iterator>
#include <stdexcept>
#include <string>

// Boost
#include <boost/shared_ptr.hpp>
#include <boost/scoped_ptr.hpp>

// ROS
#include <ros/node_handle.h>

// realtime_tools
#include <realtime_tools/realtime_box.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

// ros_controls
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/force_torque_sensor_interface.h>

// compliant control
#include <compliant_ros_controller/hardware_interface_adapter.h>
#include <compliant_ros_controller/compliance/AdmittanceController.h>
#include <compliant_ros_controller/compliance/AdmittanceParamManager.h>

#include <hardware_interface/internal/demangle_symbol.h>

// interactive marker
#include <geometry_msgs/PoseStamped.h>

#include <compliant_ros_controller/ConversionHelper.h>
#include <robot_transforms/robot_transforms.h>

namespace compliant_controller
{

/**
 * \brief Controller for compliant control
 *
 * \tparam HardwareInterface Controller hardware interface. Currently \p hardware_interface::PositionJointInterface,
 * \p hardware_interface::VelocityJointInterface, and \p hardware_interface::EffortJointInterface are supported 
 * out-of-the-box.
 */
template <class HardwareInterface>
class CompliantController : public controller_interface::ControllerBase
{
public:

  CompliantController();

  /** \name Non Real-Time Safe Functions
   *\{*/
  bool init(HardwareInterface* hw, ros::NodeHandle& controller_nh);
  /*\}*/

  /** \name Real-Time Safe Functions
   *\{*/
  /** \brief Holds the current position. */
  void starting(const ros::Time& time);

  /** \brief Cancels the active action goal, if any. */
  void stopping(const ros::Time& time);

  void update(const ros::Time& time, const ros::Duration& period);
  /*\}*/

  /** \name Non Real-Time Safe Functions
   *\{*/

  // These two functions have to be implemented as we derive from ControllerBase
  std::string getHardwareInterfaceType() const;
  bool initRequest(hardware_interface::RobotHW* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh,
                           ClaimedResources& claimed_resources);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  void poseCmdUpdate(const geometry_msgs::PoseStampedConstPtr& pose_ptr);

  Vector6d readFTSensor();
  std::string getLeafNamespace(const ros::NodeHandle& nh);
  std::vector<std::string> getStrings(const ros::NodeHandle& nh, const std::string& param_name);
  bool getVector(const ros::NodeHandle& nh, const std::string& param_name, Vector6d& vector);
  Vector6d inertia_;
  Vector6d damping_;
  Vector6d stiffness_;
  AdmittanceController admittance_controller_;
  AdmittanceParamManager admittance_param_manager_;

  struct TimeData
  {
    TimeData() : time(0.0), period(0.0), uptime(0.0) {}

    ros::Time     time;   ///< Time of last update cycle
    ros::Duration period; ///< Period of last update cycle
    ros::Time     uptime; ///< Controller uptime. Set to zero at every restart.
  };

  typedef HardwareInterfaceAdapter<HardwareInterface, CartState> HwIfaceAdapter;
  typedef typename HardwareInterface::ResourceHandleType JointHandle;

  bool                      verbose_;            ///< Hard coded verbose flag to help in debugging
  std::string               name_;               ///< Controller name.
  std::vector<JointHandle>  joints_;             ///< Handles to controlled joints.
  std::vector<std::string>  joint_names_;        ///< Controlled joint names.
  std::vector<std::string>  segment_names_;      ///< Controlled segments (links)
  std::string base_link_;

  CartState state_cmd_;        ///< virtual set-point
  CartState desired_state_;    ///< compliant set-point

  HwIfaceAdapter            hw_iface_adapter_;   ///< Adapts desired end-effector state to HW interface.

  realtime_tools::RealtimeBuffer<TimeData> time_data_;

  // ROS API
  ros::NodeHandle    controller_nh_;
  ros::Subscriber    pose_sub_;

  // additional hardware interfaces
  hardware_interface::ForceTorqueSensorHandle force_torque_sensor_handle_;
  bool ft_interface_found_;

  // Forward kinematics
  robot_tools::RobotTransforms transforms_;

};

} // namespace

#include <compliant_ros_controller/compliant_controller_impl.h>

#endif // header guard
