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
        dead_zone_trans_ = 0.1;
        dead_zone_rot_ = 0.1;
        speed_limit_rot_ = 0.1;
        speed_limit_trans_ = 0.1;
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
            double force_length = f_ext.block<3,1>(0,0).squaredNorm();
            if (force_length < dead_zone_trans_) {
                f_ext_zeroed.block<3,1>(0,0) = Vector3d::Zero();
            } else {
                f_ext_zeroed.block<3,1>(0,0) = f_ext.block<3,1>(0,0);
            }
            double torque_length = f_ext.block<3,1>(3,0).squaredNorm();
            if (torque_length < dead_zone_rot_) {
                f_ext_zeroed.block<3,1>(3,0) = Vector3d::Zero();
            } else {
                f_ext_zeroed.block<3,1>(3,0) = f_ext.block<3,1>(3,0);
            }

            // for now, set torques to zero
            for (unsigned int i = 3; i < 6; i++) {
                f_ext_zeroed(i) = 0;
            }

            // Calculate integral of force
            force_integral_ = (force_integral_ + step_size * (Dd.asDiagonal() * f_ext_zeroed)).eval();

            // Calculate derivative of force
            Vector6d fdot = (f_ext_zeroed - prev_force_) / step_size;
            prev_force_ = f_ext_zeroed;
            Vector6d derivative = Md.asDiagonal() * fdot;

            // Calculate derivative part
            Vector6d proportional = Kd.asDiagonal() * f_ext_zeroed;

            // Calculate end-effector speed
            xdotd = proportional + force_integral_ + derivative;

            // enforce speed limit
            double trans_speed = xdotd.block<3,1>(0,0).squaredNorm();
            if (trans_speed > speed_limit_trans_) {
                xdotd.block<3,1>(0,0) = (xdotd.block<3,1>(0,0) * speed_limit_trans_ / trans_speed).eval();
            }
            double rot_speed = xdotd.block<3,1>(3,0).squaredNorm();
            if (rot_speed > speed_limit_rot_) {
                xdotd.block<3,1>(3,0) = (xdotd.block<3,1>(3,0) * speed_limit_rot_ / rot_speed).eval();
            }

//            ROS_INFO_STREAM_THROTTLE(0.5, "Proportional part: " << proportional);
//            ROS_INFO_STREAM_THROTTLE(0.5, "Derivative part: " << derivative);
//            ROS_INFO_STREAM_THROTTLE(0.5, "Integral part: " << force_integral_);
            xd_ = (xd_ + step_size * xdotd).eval();
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

    void ZeroAdmittanceController::setTransDeadZone(double dead_zone) {
        dead_zone_trans_ = dead_zone;
    }

    void ZeroAdmittanceController::setRotDeadZone(double dead_zone) {
        dead_zone_rot_ = dead_zone;
    }

    void ZeroAdmittanceController::setTransSpeedLimit(double speed_limit) {
        speed_limit_trans_ = speed_limit;
    }

    void ZeroAdmittanceController::setRotSpeedLimit(double speed_limit) {
        speed_limit_rot_ = speed_limit;
    }
}
