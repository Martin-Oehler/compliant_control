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

#include <kdl/frames.hpp>

namespace compliant_controller {

template <class SegmentImpl, class HardwareInterface>
CompliantController<SegmentImpl, HardwareInterface>::
CompliantController()
  : verbose_(false) // Set to true during debugging
{}

template <class SegmentImpl, class HardwareInterface>
inline void CompliantController<SegmentImpl, HardwareInterface>::
starting(const ros::Time& time) {
    ROS_INFO_STREAM("Starting controller: " << name_);
  // Update time data
  TimeData time_data;
  time_data.time   = time;
  time_data.uptime = ros::Time(0.0);
  time_data_.initRT(time_data);

  state_cmd_.position = Vector6d::Zero();
  state_cmd_.velocity = Vector6d::Zero();
  desired_state_.position = Vector6d::Zero();
  desired_state_.velocity = Vector6d::Zero();
  current_state_.position = Vector6d::Zero();
  current_state_.velocity = Vector6d::Zero();
  state_error_.position = Vector6d::Zero();
  state_error_.velocity = Vector6d::Zero();

  // for testing
  state_cmd_.position(0) = 0.0130977+0.5;
  state_cmd_.position(1) = -0.400299;
  state_cmd_.position(2) = -0.205006;
  state_cmd_.position(3) = 1.33341;
  state_cmd_.position(4) = 0.193883;
  state_cmd_.position(5) = 1.78593+0.5;

  // Initialize last state update time
  //last_state_publish_time_ = time_data.uptime;

  // Hardware interface adapter
  hw_iface_adapter_.starting(time_data.uptime);
}

template <class SegmentImpl, class HardwareInterface>
inline void CompliantController<SegmentImpl, HardwareInterface>::
stopping(const ros::Time& time) {
    ROS_INFO_STREAM("Stopping controller: " << name_);
    //hw_iface_adapter.stopping(time_data_.uptime);
}

template <class SegmentImpl, class HardwareInterface>
bool CompliantController<SegmentImpl, HardwareInterface>::
init(HardwareInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) {
  // Cache controller node handle
  controller_nh_ = controller_nh;

  // Controller name
  name_ = getLeafNamespace(controller_nh_);
  ROS_INFO_STREAM("Initializing controller " << name_);

  // admittance parameters
  // TODO read from parameter server
  double update_step = 0.001;
  inertia_ = 150;
  damping_ = 250;
  stiffness_ = 400;

  // controlled segments
  segment_names_ = getStrings(controller_nh_, "segments");
  if (segment_names_.empty()) {
      ROS_ERROR("No segment names set for controller.");
      return false;
  }

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
  ROS_INFO_STREAM("Found " << n_joints << " joints.");
  std::stringstream joint_list;
  for (unsigned int i = 0; i < n_joints; i++) {
      joint_list << segment_names_[i] << "\t\t(" <<joints_[i].getName() << ")" << std::endl;
  }
  ROS_INFO_STREAM("Controlled segments (joints): " << std::endl << joint_list.str());

  // hardware interface adapter
  if (!hw_iface_adapter_.init(segment_names_, joints_, controller_nh_)) {
      ROS_ERROR("Initializing hardware interface adapter failed.");
      return false;
  }

  // admittance controller
  admittance_controller_.init(inertia_, damping_, stiffness_, update_step);

  // ROS API subscribed topics
  // TODO add topic name to config
  pose_sub_ = root_nh.subscribe("/r_arm_pose", 1, &CompliantController::poseCmdUpdate, this);

  return true;
}

template <class SegmentImpl, class HardwareInterface>
void CompliantController<SegmentImpl, HardwareInterface>::
update(const ros::Time& time, const ros::Duration& period) {

  // Update time data
  TimeData time_data;
  time_data.time   = time;                                     // Cache current time
  time_data.period = period;                                   // Cache current control period
  time_data.uptime = time_data_.readFromRT()->uptime + period; // Update controller uptime
  time_data_.writeFromNonRT(time_data); // TODO: Grrr, we need a lock-free data structure here!

  // TODO read state cmd from interactive marker topic
  admittance_controller_.calcCompliantPosition(state_cmd_.position, readFTSensor(), desired_state_.position, desired_state_.velocity);


  //Write desired_state and state_error to hardware interface adapter
  hw_iface_adapter_.updateCommand(time_data.uptime, time_data.period,
                                  desired_state_, state_error_);

  // Publish state
  //publishState(time_data.uptime);

}

template <class SegmentImpl, class HardwareInterface>
Vector6d CompliantController<SegmentImpl, HardwareInterface>::
readFTSensor() {
    const double* force = force_torque_sensor_handle_.getForce();
    const double* torque = force_torque_sensor_handle_.getTorque();
    Vector6d force_torque;
    for (unsigned int i = 0; i < 3; i++) {
        force_torque(i) = *(force+i);
        force_torque(i+3) = *(torque+i);
    }
    return force_torque;
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
  ROS_INFO_STREAM ("Loading controller with hardware interface: " << hardware_interface::internal::demangledTypeName<HardwareInterface>() << ".");


  //We have access to the full hw interface here and thus can grab multiple components of it

  //Get pointer to joint state interface
  // I don't think we actually need this, since we get the joint handles via the command interface.
  //hardware_interface::JointStateInterface* joint_state_interface = robot_hw->get<hardware_interface::JointStateInterface>();
//  if (!joint_state_interface){
//    ROS_ERROR("Unable to retrieve JointStateInterface for compliant controller!");
//    return false;
//  }

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
  ROS_INFO("Using force torque sensor: %s for compliant controller %s", ft_sensor_name.c_str(), getLeafNamespace(controller_nh_).c_str());

  // init controller
  hw->clearClaims();
  if (!init(hw, root_nh, controller_nh))
  {
    ROS_ERROR("Failed to initialize the controller");
    return false;
  }
  claimed_resources = hw->getClaims();
  hw->clearClaims();

  state_ = INITIALIZED;
  return true;
}

template <class SegmentImpl, class HardwareInterface>
void CompliantController<SegmentImpl, HardwareInterface>::
poseCmdUpdate(const geometry_msgs::PoseStampedConstPtr& pose_ptr) {
    KDL::Frame kdl_pose;
    kdl_pose.p = KDL::Vector(pose_ptr->pose.position.x, pose_ptr->pose.position.y, pose_ptr->pose.position.z);
    kdl_pose.M = KDL::Rotation::Quaternion(pose_ptr->pose.orientation.x, pose_ptr->pose.orientation.y, pose_ptr->pose.orientation.z, pose_ptr->pose.orientation.w);
    ConversionHelper::kdlToEigen(kdl_pose, state_cmd_.position);
    ROS_INFO_STREAM(pose_ptr->pose.position.x << " " << pose_ptr->pose.position.y << " " << pose_ptr->pose.position.z);
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
