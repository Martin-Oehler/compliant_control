# compliant_control

## About
This package implements active compliant control in task space. It utilizes the [ros_control](http://wiki.ros.org/ros_control) framework.
Your robot needs a force(-torque) sensor located at the end-effector for this controller to work.

## Installation
Clone this repository into your catkin workspace and compile it.

## Usage

### Creating a launch file

First you need to create a yaml-file that defines the controller. Following parameters are available:

| Parameter | Description |
|:----|:----|
| *type* | Name of the controller as defined by the controller plugin.(position_controllers/CompliantController, velocity_controllers/CompliantController or effort_controllers/CompliantController) |
| *ft_sensor_name* | Name of the ft-sensor handle |
| *joints* | Joint names that are to be controlled |
| *segments* | Link names connecting the controlled links |
| *moveit_group* | Only needed if type of controller is position. Name of the moveit group that is controlled. Joint names have to match joints listed in this file |
| *inertia* | Inertia of the end-effector. Overridden by dynamic reconfigure. |
| *damping* | Damping of the end-effector. Overridden by dynamic reconfigure. |
| *stiffness* | Stiffness of the end-effector. Overridden by dynamic reconfigure. |
| *cmd_topic_name* | Topic this controller uses to listen for end-effector position commands |


The controller can then be started using: 

```xml
<node name="compliant_controller_spawner" pkg="controller_manager" type="spawner" respawn="false" output="screen" args="[Controller name]" />
```

### Controller usage

The compliant controller uses [dynamic_reconfigure](http://wiki.ros.org/dynamic_reconfigure) to change control parameters on runtime.
Open up rqt and load the dynamic_reconfigure plugin. Open the node that corresponds to the compliant controller.

| Parameter | Description |
|:----|:----|
| *active* | Activate compliant behaviour. Default: false |
| *inertia* | Mode 0: Virtual inertia of the end-effector. Mode 1: Derivative gain on ext force. |
| *damping* | Mode 0: Virtual damping of the end-effector. Mode 1: Integral gain on ext force. |
| *stiffness* | Mode 0: Virtual stiffness of the end-effector. Mode 1: Proportional gain on ext force. |
| *dead_zone_trans* | Dead zone of the ft-sensor force values |
| *dead_zone_rot* | Dead zone of the ft-sensor torque values |
| *speed_limit_trans* | Translational speed limit of the end-effector in m/s |
| *speed_limit_rot* | Rotational speed limit of the end-effector in m/s |
| *mode* | Changes the controller mode. 0 uses a set-point, 1 tries to reduce to force to zero without a set-point. |


