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


namespace compliant_controller
{

template <class SegmentImpl, class HardwareInterface>
inline void CompliantController<SegmentImpl, HardwareInterface>::
starting(const ros::Time& time)
{
  // Update time data
  TimeData time_data;
  time_data.time   = time;
  time_data.uptime = ros::Time(0.0);
  time_data_.initRT(time_data);

  // Initialize last state update time
  //last_state_publish_time_ = time_data.uptime;

  // Hardware interface adapter
  hw_iface_adapter_.starting(time_data.uptime);
}

template <class SegmentImpl, class HardwareInterface>
inline void CompliantController<SegmentImpl, HardwareInterface>::
stopping(const ros::Time& time)
{

}

template <class SegmentImpl, class HardwareInterface>
CompliantController<SegmentImpl, HardwareInterface>::
CompliantController()
  : verbose_(false) // Set to true during debugging
{}

template <class SegmentImpl, class HardwareInterface>
bool CompliantController<SegmentImpl, HardwareInterface>::init(HardwareInterface* hw,
                                                                     ros::NodeHandle&   root_nh,
                                                                     ros::NodeHandle&   controller_nh)
{
  // Cache controller node handle
  controller_nh_ = controller_nh;

  // Controller name
  //name_ = getLeafNamespace(controller_nh_);
  
  size_t n_joints = 0;

    // Preeallocate resources
  current_state_    = typename Segment::State(n_joints);
  desired_state_    = typename Segment::State(n_joints);
  state_error_      = typename Segment::State(n_joints);

  return true;
}

template <class SegmentImpl, class HardwareInterface>
void CompliantController<SegmentImpl, HardwareInterface>::
update(const ros::Time& time, const ros::Duration& period)
{
  // Update time data
  TimeData time_data;
  time_data.time   = time;                                     // Cache current time
  time_data.period = period;                                   // Cache current control period
  time_data.uptime = time_data_.readFromRT()->uptime + period; // Update controller uptime
  time_data_.writeFromNonRT(time_data); // TODO: Grrr, we need a lock-free data structure here!


   //Perform control calculation here.


  
  //Write desired_state and state_error to hardware interface adapter
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
initRequest(hardware_interface::RobotHW* hw, ros::NodeHandle& root_nh, ros::NodeHandle &controller_nh,
                         std::set<std::string>& claimed_resources)
{
  //We have access to the full hw interface here and thus can grab multiple components of it

  //Get pointer to joint state interface
  hardware_interface::JointStateInterface* joint_state_interface = hw->get<hardware_interface::JointStateInterface>();

  if (!joint_state_interface){
    ROS_ERROR("Unable to retrieve JointStateInterface for compliant controller!");
    return false;
  }

  // @TODO: Initialize own joint state representation so it points to the pointers given
  // in joint_state_interface´s JointHandles
  // A lot of code from JointTrajectoryController´s init() can be re-used (e.g. copied)
  // for this.

  hardware_interface::ForceTorqueSensorInterface * force_torque_sensor_interface = hw->get<hardware_interface::ForceTorqueSensorInterface >();

  if (!force_torque_sensor_interface){
    ROS_ERROR("Unable to retrieve ForceTorqueSensorInterface for compliant controller!");
    return false;
  }

  // Get the name of the FT sensor to use from the parameter server
  std::string ft_sensor_name = "ft_sensor";
  controller_nh_.getParam("ft_sensor_name", ft_sensor_name);

  // Query the interface for the selected FT sensor
  // @TODO: Properly check if the handle is valid (getHandle() throws exception if not?)
  force_torque_sensor_handle_ = force_torque_sensor_interface->getHandle(ft_sensor_name);


  // At the end, this probably per default calls the controller´s init (which we pre-empted)
  // @TODO Verify this is indeed the case
  //this->init(joint_state_interface, root_nh, controller_nh);
}


} // namespace

#endif // header guard
