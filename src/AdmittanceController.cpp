#include <vigir_compliant_ros_controller/AdmittanceController.h>

namespace compliant_controller {
    void AdmittanceController::init(double inertia, double damping, double stiffness) {
        for (unsigned int i = 0; i < Md.size(); i++) {
            Md(i) = inertia;
            Dd(i) = damping;
            Kd(i) = stiffness;
        }
        init(Md, Dd, Kd);
    }

    void AdmittanceController::init(Vector6d& inertia, Vector6d& damping, Vector6d& stiffness) {
        Md = inertia;
        Dd = damping;
        Kd = stiffness;
        e_ = Eigen::Matrix<double, 12, 1>::Zero();
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
    }

    void AdmittanceController::stopping() {

    }

    void AdmittanceController::update(const Vector6d &x0, const Vector6d& f_ext, Vector6d& xd, Vector6d& xdotd, double step_size) {
        e_ = e_ + step_size * f(f_ext);                // e_(k+1) = e_k + h*f(e_k, f_ext)
        xd = x0 + getE1();                        // add the calculated position offset to our virtual set point
        xdotd = getE2();
    }
}
