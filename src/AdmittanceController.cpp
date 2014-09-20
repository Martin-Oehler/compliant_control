#include <vigir_compliant_ros_controller/AdmittanceController.h>

namespace compliant_controller {
    void AdmittanceController::init(double inertia, double damping, double stiffness, double step_size) {
        for (unsigned int i = 0; i < Md.size(); i++) {
            Md(i) = inertia;
            Dd(i) = damping;
            Kd(i) = stiffness;
        }
        init(Md, Dd, Kd, step_size);
    }

    void AdmittanceController::init(Vector6d& inertia, Vector6d& damping, Vector6d& stiffness, double step_size) {
        e_.resize(12);
        step_size_ = step_size;
        Md = inertia;
        Dd = damping;
        Kd = stiffness;
        for (unsigned int i = 0; i < e_.size(); i++) {
            e_(i) = 0;
        }
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
    VectorNd AdmittanceController::f(const Vector6d& f_ext) {
        VectorNd f_out(12);
        f_out.block<6,1>(0,0) = getE2();
        f_out.block<6,1>(6,0) = Md.asDiagonal().inverse() * (f_ext - Dd.asDiagonal() * getE2() - Kd.asDiagonal() * getE1());
        return f_out;
    }

    void AdmittanceController::calcCompliantPosition(const Vector6d &x0, const Vector6d& f_ext, Vector6d& xd, Vector6d& xdotd) {
        e_ = e_ + step_size_ * f(f_ext);                // e_(k+1) = e_k + h*f(e_k, f_ext)
        xd = x0 + getE1();                        // add the calculated position offset to our virtual set point
        xdotd = getE2();
    }
}
