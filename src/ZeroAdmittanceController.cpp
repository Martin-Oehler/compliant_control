#include <vigir_compliant_ros_controller/ZeroAdmittanceController.h>

namespace compliant_controller {
    void ZeroAdmittanceController::init(double inertia, double damping, double stiffness) {
        for (unsigned int i = 0; i < Md.size(); i++) {
            Md(i) = inertia;
            Dd(i) = damping;
            Kd(i) = stiffness;
        }
        init(Md, Dd, Kd);
    }

    void ZeroAdmittanceController::init(const Vector6d &inertia, const Vector6d &damping, const Vector6d &stiffness) {
        Md = inertia;
        Dd = damping;
        Kd = stiffness;
        dead_zone_ = 0.1;
    }

    /**
      Calculates the step function using the last step ek and the current external forces f_ext
      */
    Eigen::Matrix<double, 12, 1> ZeroAdmittanceController::f(const Vector6d& f_ext) {

    }

    void ZeroAdmittanceController::starting() {
        active_ = false;
        first_update_ = true;
        force_integral_ = Vector6d::Zero();
        prev_force_ = Vector6d::Zero();
    }

    void ZeroAdmittanceController::stopping() {

    }

    void ZeroAdmittanceController::update(const Vector6d &x0, const Vector6d& f_ext, Vector6d& xd, Vector6d& xdotd, double step_size) {
        if (first_update_) {
            xd_ = x0;
            first_update_ = false;
        }

        if (!active_) {
            xd = xd_;
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
            // Calculate integral of force
            force_integral_ = force_integral_ + step_size * (Dd.asDiagonal() * f_ext_zeroed);

            // Calculate derivative of force
            Vector6d fdot = (prev_force_ - f_ext_zeroed) / step_size;
            prev_force_ = f_ext_zeroed;
            Vector6d proportional = Kd.asDiagonal() * f_ext_zeroed;
            Vector6d derivative = Md.asDiagonal() * fdot;
            xdotd = proportional + force_integral_ + derivative;

            ROS_INFO_STREAM_THROTTLE(0.5, "Proportional part: " << proportional);
            ROS_INFO_STREAM_THROTTLE(0.5, "Derivative part: " << derivative);
            ROS_INFO_STREAM_THROTTLE(0.5, "Integral part: " << force_integral_);
            xd_ = xd_ + step_size * xdotd;
            xd = xd_;
        }
    }

    // Setters and getters
    void ZeroAdmittanceController::activate(bool active) {
        active_ = active;
    }

    bool ZeroAdmittanceController::isActive() {
        return active_;
    }

    void ZeroAdmittanceController::setInertia(double inertia) {
        Md.setConstant(inertia);
    }

    void ZeroAdmittanceController::setDamping(double damping) {
        Dd.setConstant(damping);
    }

    void ZeroAdmittanceController::setStiffness(double stiffness) {
        Kd.setConstant(stiffness);
    }

    void ZeroAdmittanceController::setDeadZone(double dead_zone) {
        dead_zone_ = dead_zone;
    }
}
