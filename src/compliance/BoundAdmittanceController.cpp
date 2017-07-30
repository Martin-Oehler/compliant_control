#include <compliant_ros_controller/compliance/BoundAdmittanceController.h>

namespace compliant_controller {
    void BoundAdmittanceController::init(double inertia, double damping, double stiffness) {
        for (unsigned int i = 0; i < Md.size(); i++) {
            Md(i) = inertia;
            Dd(i) = damping;
            Kd(i) = stiffness;
        }
        init(Md, Dd, Kd);
    }

    void BoundAdmittanceController::init(const Vector6d &inertia, const Vector6d &damping, const Vector6d &stiffness) {
        Md = inertia;
        Dd = damping;
        Kd = stiffness;
        e_ = Eigen::Matrix<double, 12, 1>::Zero();
        dead_zone_trans_ = 0.1;
        dead_zone_rot_ = 0.1;
        speed_limit_rot_ = 0.1;
        speed_limit_trans_ = 0.1;
        active_ = false;
    }

    Vector6d BoundAdmittanceController::getE1() const {
        return e_.block<6,1>(0,0);
    }

    Vector6d BoundAdmittanceController::getE2() const {
        return e_.block<6,1>(6,0);
    }

    /**
      Calculates the step function using the last step ek and the current external forces f_ext
      */
    Eigen::Matrix<double, 12, 1> BoundAdmittanceController::f(const Vector6d& f_ext) {
        Eigen::Matrix<double, 12, 1> f_out;
        f_out.block<6,1>(0,0) = getE2();
        f_out.block<6,1>(6,0) = Md.asDiagonal().inverse() * (f_ext - Dd.asDiagonal() * getE2() - Kd.asDiagonal() * getE1());

        return f_out;
    }

    void BoundAdmittanceController::starting() {
        e_ = Eigen::Matrix<double, 12, 1>::Zero();
    }

    void BoundAdmittanceController::stopping() {

    }

    void BoundAdmittanceController::update(const Vector6d &x0, const Vector6d& f_ext, Vector6d& xd, Vector6d& xdotd, double step_size) {
        if (!active_) {
            e_ = Eigen::Matrix<double, 12, 1>::Zero(); // error = 0 if deactivated
        } else {
            Vector6d f_ext_zeroed;
            // set force zero if value below dead zone threshold
            double force_length = f_ext.block<3,1>(0,0).squaredNorm();
            if (force_length < dead_zone_trans_) {
                f_ext_zeroed.block<3,1>(0,0) = Vector3d::Zero();
            } else {
                f_ext_zeroed.block<3,1>(0,0) = (1-dead_zone_trans_ / force_length) * f_ext.block<3,1>(0,0);
            }
            double torque_length = f_ext.block<3,1>(3,0).squaredNorm();
            if (torque_length < dead_zone_rot_) {
                f_ext_zeroed.block<3,1>(3,0) = Vector3d::Zero();
            } else {
                f_ext_zeroed.block<3,1>(3,0) = (1-dead_zone_rot_ / torque_length) * f_ext.block<3,1>(3,0);;
            }

            // for now, set torques to zero
            for (unsigned int i = 3; i < 6; i++) {
                f_ext_zeroed(i) = 0;
            }

            e_ = e_ + step_size * f(f_ext_zeroed);                // e_(k+1) = e_k + h*f(e_k, f_ext)

            // enforce speed limit
            double trans_speed = e_.block<3,1>(6,0).squaredNorm();
            if (trans_speed > speed_limit_trans_) {
                e_.block<3,1>(6,0) = (e_.block<3,1>(6,0) * speed_limit_trans_ / trans_speed).eval();
            }
            double rot_speed = e_.block<3,1>(9,0).squaredNorm();
            if (rot_speed > speed_limit_rot_) {
                e_.block<3,1>(9,0) = (e_.block<3,1>(9,0) * speed_limit_rot_ / rot_speed).eval();
            }
        }
        xd = x0 + getE1();                        // add the calculated position offset to our virtual set point
        xdotd = getE2();
    }

    // Setters and getters
    void BoundAdmittanceController::activate(bool active) {
        active_ = active;
    }

    bool BoundAdmittanceController::isActive() {
        return active_;
    }

    void BoundAdmittanceController::setInertia(double inertia) {
        Md.setConstant(inertia);
    }

    void BoundAdmittanceController::setDamping(double damping) {
        Dd.setConstant(damping);
    }

    void BoundAdmittanceController::setStiffness(double stiffness) {
        Kd.setConstant(stiffness);
    }

    void BoundAdmittanceController::setTransDeadZone(double dead_zone) {
        dead_zone_trans_ = dead_zone;
    }

    void BoundAdmittanceController::setRotDeadZone(double dead_zone) {
        dead_zone_rot_ = dead_zone;
    }

    void BoundAdmittanceController::setTransSpeedLimit(double speed_limit) {
        speed_limit_trans_ = speed_limit;
    }

    void BoundAdmittanceController::setRotSpeedLimit(double speed_limit) {
        speed_limit_rot_ = speed_limit;
    }
}
