#ifndef INV_KIN_CONTROLLER_H
#define INV_KIN_CONTROLLER_H

#include<ros/ros.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <vigir_compliant_ros_controller/CustomTypes.h>
//#include <flor_utilities/timing.h>

namespace compliant_controller {

    class InvKinController {
    public:
        bool init(std::string group_name);
        bool calcInvKin(const Vector6d& xd, VectorNd& joint_positions);
        bool updateJointState(const VectorNd& q);
        bool getTipTransform(Eigen::Affine3d& tip_transform);
    private:
        robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
        robot_model::RobotModelPtr robot_model_;
        robot_state::RobotStatePtr robot_state_;
        const robot_state::JointModelGroup* joint_model_group_;

        std::string tip_frame_;
        std::vector<std::string> joint_names_;

        std::vector<double> q_;
        std::vector<double> solution_;
        std::vector<geometry_msgs::Pose> poses_;

      //  boost::shared_ptr<Timing> timer_ptr_;
    };
}
#endif
