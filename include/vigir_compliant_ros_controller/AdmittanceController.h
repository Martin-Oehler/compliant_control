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
        void activate(bool active);
        bool isActive();
        // Setters and getters
        void setInertia(double inertia);
        //double getInertia();
        void setDamping(double damping);
        //double getDamping();
        void setStiffness(double stiffness);
        //double getStiffness();

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    private:
        bool active_;
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
