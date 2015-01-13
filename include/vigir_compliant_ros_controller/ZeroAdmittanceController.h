#ifndef ZERO_ADMITTANCE_CONTROLLER_H
#define ZERO_ADMITTANCE_CONTROLLER_H

#include <vigir_compliant_ros_controller/CustomTypes.h>

namespace compliant_controller {
    class ZeroAdmittanceController {
    public:
        void init(double inertia, double damping, double stiffness);
        void init(const Vector6d& inertia, const Vector6d& damping, const Vector6d& stiffness);
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
        void setDeadZone(double dead_zone);

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    private:
        bool first_update_;
        bool active_;
        Eigen::Matrix<double, 12, 1> f(const Vector6d &f_ext);
        Vector6d xd_;
        /**
          Admittance parameters
          */
        Vector6d Md;
        Vector6d Dd;
        Vector6d Kd;

        Vector6d force_integral_;
        Vector6d prev_force_;

        double dead_zone_;
    };
}

#endif
