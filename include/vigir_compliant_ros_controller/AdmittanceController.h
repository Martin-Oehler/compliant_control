#ifndef ADMITTANCE_CONTROLLER_H
#define ADMITTANCE_CONTROLLER_H

#include <vigir_compliant_ros_controller/CustomTypes.h>

namespace compliant_controller {
    class AdmittanceController {
    public:
        void init(double inertia, double damping, double stiffness);
        void init(Vector6d& inertia, Vector6d& damping, Vector6d& stiffness);
        void starting();
        void stopping();
        void update(const Vector6d&x0, const Vector6d& f_ext, Vector6d& xd, Vector6d& xdotd, double step_size);
    private:
        Eigen::Matrix<double, 12, 1> f(const Vector6d &f_ext);
        /**
          Returns the position part
          */
        Vector6d getE1() const;
        /**
          Returns the velocity part
          */
        Vector6d getE2() const;
        /**
          12x1 vector combining position error e (6x1) and velocity error edot (6x1) .
          */
        Eigen::Matrix<double, 12, 1> e_;
        /**
          Admittance parameters
          */
        Vector6d Md;
        Vector6d Dd;
        Vector6d Kd;
    };
}

#endif
