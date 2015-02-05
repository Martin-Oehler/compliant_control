#ifndef ADMITTANCE_CONTROLLER_H
#define ADMITTANCE_CONTROLLER_H

#include <compliant_ros_controller/BoundAdmittanceController.h>
#include <compliant_ros_controller/ZeroAdmittanceController.h>
#include <compliant_ros_controller/CustomTypes.h>


#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

namespace compliant_controller {
    class AdmittanceController {
    public:
        AdmittanceController();
        void init(double inertia, double damping, double stiffness);
        void init(const Vector6d& inertia, const Vector6d& damping, const Vector6d& stiffness);
        void starting();
        void stopping();
        // setNoSolutionFoundInLastCycle - tells controller to reset set-point to current position
        void update(const ros::Time& time, const Vector6d&x0, const Vector6d& f_ext, Vector6d& xd, Vector6d& xdotd, double step_size);
        void setLastSetPointFailed();
        void setMode(unsigned int mode);
        void activate(bool active);
        bool isActive();
        // Setters and getters
        void setInertia(double inertia);
        //double getInertia();
        void setDamping(double damping);
        //double getDamping();
        void setStiffness(double stiffness);
        //double getStiffness();
        void setTransDeadZone(double dead_zone);
        void setRotDeadZone(double dead_zone);
        void setTransSpeedLimit(double speed_limit);
        void setRotSpeedLimit(double speed_limit);

        void activateStatePublishing(ros::NodeHandle& nh);
    private:
        void publishCompliantPose(const ros::Time &time, Vector6d& pose);

        BoundAdmittanceController bound_addm_controller_; // mode 0
        ZeroAdmittanceController zero_addm_controller_; // mode 1

        unsigned int mode_;

        ros::Publisher pose_publisher_;
        bool publish_state_;
        int seq_counter_;

    };
}

#endif
