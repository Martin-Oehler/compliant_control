#ifndef INV_KIN_CONTROLLER_H
#define INV_KIN_CONTROLLER_H

#include<ros/ros.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <compliant_ros_controller/CustomTypes.h>

#include <sensor_msgs/JointState.h>

namespace compliant_controller {

    class InvKinController {
    public:
        bool init(std::string group_name);
        bool calcInvKin(const ros::Time& time, const Vector6d& xd, VectorNd& joint_positions);
        bool updateJointState(const VectorNd& q);
        bool getTipTransform(Eigen::Affine3d& tip_transform);

        void activateStatePublishing(ros::NodeHandle& nh);
    private:
        void publishState(const ros::Time &time, const VectorNd& state);
        ros::Publisher joint_state_publisher_;
        bool publish_state_;
        int seq_counter_;
        robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
        robot_model::RobotModelPtr robot_model_;
        robot_state::RobotStatePtr robot_state_;
        const robot_state::JointModelGroup* joint_model_group_;

        std::string tip_frame_;
        std::vector<std::string> joint_names_;

        std::vector<double> q_;
        std::vector<double> solution_;
        std::vector<geometry_msgs::Pose> poses_;
    };
}
#endif
