#ifndef ZERO_ADMITTANCE_CONTROLLER_H
#define ZERO_ADMITTANCE_CONTROLLER_H

#include <compliant_ros_controller/CustomTypes.h>
#include <ros/ros.h>

namespace compliant_controller {
    class ZeroAdmittanceController {
    public:
        void init(double inertia, double damping, double stiffness);
        void init(const Vector6d& inertia, const Vector6d& damping, const Vector6d& stiffness);
        void starting();
        void stopping();
        void update(const Vector6d&x0, const Vector6d& f_ext, Vector6d& xd, Vector6d& xdotd, double step_size);

        void setLastSetPointFailed();

        // Configuration
        void activate(bool active);
        bool isActive();
        // Setters and getters
        void setInertia(double inertia);
        //double getInertia();
        void setDamping(double damping);
        //double getDamping();
        void setStiffness(double stiffness);
        //double getStiffness();
        void setTransDeadZone(double dead_zone);
        void setRotDeadZone(double dead_zone);
        void setTransSpeedLimit(double speed_limit);
        void setRotSpeedLimit(double speed_limit);

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    private:
        bool first_update_;
        bool active_;
        Eigen::Matrix<double, 12, 1> f(const Vector6d &f_ext);
        Vector6d xd_;
        Vector6d prev_xd_;
        /**
          Admittance parameters
          */
        Vector6d Md;
        Vector6d Dd;
        Vector6d Kd;

        Vector6d force_integral_;
        Vector6d prev_force_;

        double dead_zone_trans_;
        double dead_zone_rot_;
        double speed_limit_trans_;
        double speed_limit_rot_;

        bool last_setpoint_failed_;
    };
}

#endif
