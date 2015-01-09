#include <vigir_compliant_ros_controller/AdmittanceController.h>

namespace compliant_controller {
    void AdmittanceController::init(double inertia, double damping, double stiffness) {
        for (unsigned int i = 0; i < Md.size(); i++) {
            Md(i) = inertia;
            Dd(i) = damping;
            Kd(i) = stiffness;
        }
        init(Md, Dd, Kd);
        publish_state_ = false;
    }

    void AdmittanceController::init(Vector6d& inertia, Vector6d& damping, Vector6d& stiffness) {
        Md = inertia;
        Dd = damping;
        Kd = stiffness;
        e_ = Eigen::Matrix<double, 12, 1>::Zero();
        dead_zone_ = 0.1;
    }

    Vector6d AdmittanceController::getE1() const {
        return e_.block<6,1>(0,0);
    }

    Vector6d AdmittanceController::getE2() const {
        return e_.block<6,1>(6,0);
    }

    /**
      Calculates the step function using the last step ek and the current external forces f_ext
      */
    Eigen::Matrix<double, 12, 1> AdmittanceController::f(const Vector6d& f_ext) {
        Eigen::Matrix<double, 12, 1> f_out;
        f_out.block<6,1>(0,0) = getE2();
        f_out.block<6,1>(6,0) = Md.asDiagonal().inverse() * (f_ext - Dd.asDiagonal() * getE2() - Kd.asDiagonal() * getE1());
        return f_out;
    }

    void AdmittanceController::starting() {
        e_ = Eigen::Matrix<double, 12, 1>::Zero();
        active_ = false;
    }

    void AdmittanceController::stopping() {

    }

    void AdmittanceController::update(const ros::Time& time, const Vector6d &x0, const Vector6d& f_ext, Vector6d& xd, Vector6d& xdotd, double step_size) {
        if (!active_) {
            e_ = Eigen::Matrix<double, 12, 1>::Zero(); // error = 0 if deactivated
        } else {
            Vector6d f_ext_zeroed;
            // set force zero if value below dead zone threshold
            for (unsigned int i = 0; i < f_ext.size(); i++) {
                if (std::abs(f_ext(i)) < dead_zone_) {
                    f_ext_zeroed(i) = 0;
                } else {
                    f_ext_zeroed(i) = f_ext(i);
                }
            }
            e_ = e_ + step_size * f(f_ext_zeroed);                // e_(k+1) = e_k + h*f(e_k, f_ext)
        }
        xd = x0 + getE1();                        // add the calculated position offset to our virtual set point
        xdotd = getE2();

        if (publish_state_) {
            publishCompliantPose(time, xd);
        }
    }

    void AdmittanceController::activateStatePublishing(ros::NodeHandle& nh) {
        pose_publisher_ = nh.advertise<geometry_msgs::PoseStamped>("compliant_pose", 1000);
        publish_state_ = true;
        seq_counter_ = 0;
    }

    void AdmittanceController::publishCompliantPose(const ros::Time &time, Vector6d& pose) {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp.fromNSec(time.toNSec());
        pose_stamped.header.seq = seq_counter_; seq_counter_++;
        pose_stamped.header.frame_id = "utorso";

        pose_stamped.pose.position.x = pose(0);
        pose_stamped.pose.position.y = pose(1);
        pose_stamped.pose.position.z = pose(2);

        pose_stamped.pose.orientation.w = 1;
        pose_stamped.pose.orientation.x = 0;
        pose_stamped.pose.orientation.y = 0;
        pose_stamped.pose.orientation.z = 0;

        pose_publisher_.publish(pose_stamped);
    }


    // Setters and getters
    void AdmittanceController::activate(bool active) {
        active_ = active;
    }

    bool AdmittanceController::isActive() {
        return active_;
    }

    void AdmittanceController::setInertia(double inertia) {
        Md.setConstant(inertia);
    }

    void AdmittanceController::setDamping(double damping) {
        Dd.setConstant(damping);
    }

    void AdmittanceController::setStiffness(double stiffness) {
        Kd.setConstant(stiffness);
    }

    void AdmittanceController::setDeadZone(double dead_zone) {
        dead_zone_ = dead_zone;
    }
}
