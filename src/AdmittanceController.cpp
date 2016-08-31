#include <compliant_ros_controller/AdmittanceController.h>

namespace compliant_controller {
    AdmittanceController::AdmittanceController()
        : publish_state_(false),
          mode_(0) {
    }

    void AdmittanceController::init(double inertia, double damping, double stiffness) {
        bound_addm_controller_.init(inertia, damping, stiffness);
        zero_addm_controller_.init(inertia, damping, stiffness);
    }

    void AdmittanceController::init(const Vector6d &inertia, const Vector6d &damping, const Vector6d &stiffness) {
        bound_addm_controller_.init(inertia, damping, stiffness);
        zero_addm_controller_.init(inertia, damping, stiffness);
    }

    void AdmittanceController::starting() {
        if (mode_ == 0) {
            bound_addm_controller_.starting();
        }
        else if (mode_ == 1) {
            zero_addm_controller_.starting();
        }
    }

    void AdmittanceController::stopping() {
        if (mode_ == 0) {
            bound_addm_controller_.stopping();
        }
        else if (mode_ == 1) {
            zero_addm_controller_.stopping();
        }
    }

    void AdmittanceController::update(const ros::Time& time, const Vector6d &x0, const Vector6d& f_ext, Vector6d& xd, Vector6d& xdotd, double step_size) {
        if (mode_ == 0) {
            bound_addm_controller_.update(x0, f_ext, xd, xdotd, step_size);
        }
        else if (mode_ == 1) {
            zero_addm_controller_.update(x0, f_ext, xd, xdotd, step_size);
        }

        if (publish_state_) {
            publishCompliantPose(time, xd, frame_);
        }
    }

    void AdmittanceController::setLastSetPointFailed() {
        if (mode_ == 1) {
            zero_addm_controller_.setLastSetPointFailed();
        }
    }

    void AdmittanceController::activateStatePublishing(ros::NodeHandle& nh, std::string frame_id) {
        pose_publisher_ = nh.advertise<geometry_msgs::PoseStamped>("compliant_pose", 1000);
        publish_state_ = true;
        seq_counter_ = 0;
        frame_ = frame_id;
    }

    void AdmittanceController::publishCompliantPose(const ros::Time &time, Vector6d& pose, std::string frame_id) {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp = time;
        pose_stamped.header.seq = seq_counter_; seq_counter_++;
        pose_stamped.header.frame_id = frame_id;

        pose_stamped.pose.position.x = pose(0);
        pose_stamped.pose.position.y = pose(1);
        pose_stamped.pose.position.z = pose(2);

        pose_stamped.pose.orientation.w = 1;
        pose_stamped.pose.orientation.x = 0;
        pose_stamped.pose.orientation.y = 0;
        pose_stamped.pose.orientation.z = 0;

        pose_publisher_.publish(pose_stamped);
    }

    void AdmittanceController::setMode(unsigned int mode) {
        if (mode_ != mode) {
            stopping();
            mode_ = mode;
            starting();
        }
    }


    // Setters and getters
    void AdmittanceController::activate(bool active) {
        bound_addm_controller_.activate(active);
        zero_addm_controller_.activate(active);
    }

    bool AdmittanceController::isActive() {
        return bound_addm_controller_.isActive();
    }

    void AdmittanceController::setInertia(double inertia) {
        bound_addm_controller_.setInertia(inertia);
        zero_addm_controller_.setInertia(inertia);
    }

    void AdmittanceController::setDamping(double damping) {
        bound_addm_controller_.setDamping(damping);
        zero_addm_controller_.setDamping(damping);
    }

    void AdmittanceController::setStiffness(double stiffness) {
        bound_addm_controller_.setStiffness(stiffness);
        zero_addm_controller_.setStiffness(stiffness);
    }

    void AdmittanceController::setTransDeadZone(double dead_zone) {
        bound_addm_controller_.setTransDeadZone(dead_zone);
        zero_addm_controller_.setTransDeadZone(dead_zone);
    }

    void AdmittanceController::setRotDeadZone(double dead_zone) {
        bound_addm_controller_.setRotDeadZone(dead_zone);
        zero_addm_controller_.setRotDeadZone(dead_zone);
    }

    void AdmittanceController::setTransSpeedLimit(double speed_limit) {
        bound_addm_controller_.setTransSpeedLimit(speed_limit);
        zero_addm_controller_.setTransSpeedLimit(speed_limit);
    }

    void AdmittanceController::setRotSpeedLimit(double speed_limit) {
        bound_addm_controller_.setRotSpeedLimit(speed_limit);
        zero_addm_controller_.setRotSpeedLimit(speed_limit);
    }
}
