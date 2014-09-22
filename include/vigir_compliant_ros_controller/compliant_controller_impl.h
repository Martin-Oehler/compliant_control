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

#ifndef COMPLIANT_CONTROLLER_IMPL_H
#define COMPLIANT_CONTROLLER_IMPL_H


namespace compliant_controller {

template <class SegmentImpl, class HardwareInterface>
CompliantController<SegmentImpl, HardwareInterface>::
CompliantController()
  : verbose_(false) // Set to true during debugging
{}

template <class SegmentImpl, class HardwareInterface>
inline void CompliantController<SegmentImpl, HardwareInterface>::
starting(const ros::Time& time) {
  // Update time data
  TimeData time_data;
  time_data.time   = time;
  time_data.uptime = ros::Time(0.0);
  time_data_.initRT(time_data);

  desired_state_ = Vector6d::Zero();
  current_state_ = Vector6d::Zero();
  state_error_ = Vector6d::Zero();

  // Initialize last state update time
  //last_state_publish_time_ = time_data.uptime;

  // Hardware interface adapter
  hw_iface_adapter_.starting(time_data.uptime);
}

template <class SegmentImpl, class HardwareInterface>
inline void CompliantController<SegmentImpl, HardwareInterface>::
stopping(const ros::Time& time) {
    //hw_iface_adapter.stopping(time_data_.uptime);
}

template <class SegmentImpl, class HardwareInterface>
bool CompliantController<SegmentImpl, HardwareInterface>::
init(HardwareInterface* hw, ros::NodeHandle&   root_nh, ros::NodeHandle&   controller_nh) {
  // Cache controller node handle
  controller_nh_ = controller_nh;

  // Controller name
  name_ = getLeafNamespace(controller_nh_);

  double update_step = 0.001;

  // controlled joints
  joint_names_ = getStrings(controller_nh_, "joints");
  if (joint_names_.empty()) {
      ROS_ERROR("No joint names set for controller.");
      return false;
  }
  size_t n_joints = joint_names_.size();
  joints_.resize(n_joints);
  for (unsigned int i = 0; i < n_joints; i++) {
      try {
          joints_[i] = hw->getHandle(joint_names_[i]);
      } catch (hardware_interface::HardwareInterfaceException e) {
          ROS_ERROR_STREAM("Couldn't find handle for " << joint_names_[i] << ". " << e.what() << std::endl);
          return false;
      }
  }

  // hardware interface adapter
  hw_iface_adapter_.init(joints_, controller_nh_);

  // ROS API subscribed topics
  // TODO subscribe to interactive marker topic

  return true;
}

template <class SegmentImpl, class HardwareInterface>
void CompliantController<SegmentImpl, HardwareInterface>::
update(const ros::Time& time, const ros::Duration& period){

  // Update time data
  TimeData time_data;
  time_data.time   = time;                                     // Cache current time
  time_data.period = period;                                   // Cache current control period
  time_data.uptime = time_data_.readFromRT()->uptime + period; // Update controller uptime
  time_data_.writeFromNonRT(time_data); // TODO: Grrr, we need a lock-free data structure here!



  //Write desired_state and state_error to hardware interface adapter
  // TODO read desired state from interactive marker topic
  hw_iface_adapter_.updateCommand(time_data.uptime, time_data.period,
                                  desired_state_, state_error_);

  // Publish state
  //publishState(time_data.uptime);

}

template <class SegmentImpl, class HardwareInterface>
std::string CompliantController<SegmentImpl, HardwareInterface>::
getHardwareInterfaceType() const
{
  return "This_controller_derives_from_ControllerBase_so_cannot_return_a_proper_single_interface_type";
}

template <class SegmentImpl, class HardwareInterface>
bool CompliantController<SegmentImpl, HardwareInterface>::
initRequest(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle &controller_nh,
                         std::set<std::string>& claimed_resources)
{
  // check if construction finished cleanly
  if (state_ != CONSTRUCTED){
    ROS_ERROR("Cannot initialize this controller because it failed to be constructed");
    return false;
  }

  // get a pointer to the hardware interface
  HardwareInterface* hw = robot_hw->get<HardwareInterface>();
  if (!hw)
  {
      ROS_ERROR_STREAM("This controller requires a hardware interface of type " << hardware_interface::internal::demangledTypeName<HardwareInterface>() << ".");
      return false;
  }


  //We have access to the full hw interface here and thus can grab multiple components of it

  //Get pointer to joint state interface
  // I don't think we actually need this, since we get the joint handles via the command interface.
  //hardware_interface::JointStateInterface* joint_state_interface = robot_hw->get<hardware_interface::JointStateInterface>();

//  if (!joint_state_interface){
//    ROS_ERROR("Unable to retrieve JointStateInterface for compliant controller!");
//    return false;
//  }
  // @TODO: Initialize own joint state representation so it points to the pointers given
  // in joint_state_interface´s JointHandles
  // A lot of code from JointTrajectoryController´s init() can be re-used (e.g. copied)
  // for this.

  // get pointer to force torque sensor interface
  hardware_interface::ForceTorqueSensorInterface * force_torque_sensor_interface = robot_hw->get<hardware_interface::ForceTorqueSensorInterface >();
  if (!force_torque_sensor_interface){
    ROS_ERROR("Unable to retrieve ForceTorqueSensorInterface for compliant controller!");
    return false;
  }
  // Get the name of the FT sensor to use from the parameter server
  std::string ft_sensor_name = "ft_sensor";
  controller_nh.getParam("ft_sensor_name", ft_sensor_name);
  // Query the interface for the selected FT sensor
  try {
      force_torque_sensor_handle_ = force_torque_sensor_interface->getHandle(ft_sensor_name);
  } catch (hardware_interface::HardwareInterfaceException e) {
      ROS_ERROR_STREAM("Couldn't get handle for f/t sensor: " << ft_sensor_name << ". " << e.what());
      return false;
  }

  // init controller
  hw->clearClaims();
  if (!init(hw, root_nh, controller_nh))
  {
    ROS_ERROR("Failed to initialize the controller");
    return false;
  }
  claimed_resources = hw->getClaims();
  hw->clearClaims();


  // @TODO: Could make below nicer by using getLeafNamespace as in joint_trajectory_controller
  ROS_INFO("Using force torque sensor: %s for compliant controller %s", ft_sensor_name.c_str(), controller_nh_.getNamespace().c_str());

  state_ = INITIALIZED;
  return true;
}

template <class SegmentImpl, class HardwareInterface>
std::string CompliantController<SegmentImpl, HardwareInterface>::
getLeafNamespace(const ros::NodeHandle& nh) {
  const std::string complete_ns = nh.getNamespace();
  std::size_t id   = complete_ns.find_last_of("/");
  return complete_ns.substr(id + 1);
}

template <class SegmentImpl, class HardwareInterface>
std::vector<std::string> CompliantController<SegmentImpl, HardwareInterface>::
getStrings(const ros::NodeHandle& nh, const std::string& param_name) {
  using namespace XmlRpc;
  XmlRpcValue xml_array;
  if (!nh.getParam(param_name, xml_array))
  {
    ROS_ERROR_STREAM("Could not find '" << param_name << "' parameter (namespace: " << nh.getNamespace() << ").");
    return std::vector<std::string>();
  }
  if (xml_array.getType() != XmlRpcValue::TypeArray)
  {
    ROS_ERROR_STREAM("The '" << param_name << "' parameter is not an array (namespace: " <<
                     nh.getNamespace() << ").");
    return std::vector<std::string>();
  }

  std::vector<std::string> out;
  for (int i = 0; i < xml_array.size(); ++i)
  {
    if (xml_array[i].getType() != XmlRpcValue::TypeString)
    {
      ROS_ERROR_STREAM("The '" << param_name << "' parameter contains a non-string element (namespace: " <<
                       nh.getNamespace() << ").");
      return std::vector<std::string>();
    }
    out.push_back(static_cast<std::string>(xml_array[i]));
  }
  return out;
}


} // namespace

#endif // header guard
