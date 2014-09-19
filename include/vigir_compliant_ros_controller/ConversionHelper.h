#ifndef CONVERSION_HELPER
#define CONVERSION_HELPER

#include <vigir_compliant_ros_controller/CustomTypes.h>

#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>

namespace control {
    class ConversionHelper {
    public:
        static void kdlToEigen(const KDL::Vector& kdl_vector, Vector3d& eigen_vector);
        static void kdlToEigen(const KDL::Frame& frame, Vector3d& position);
        static void kdlToEigen(const KDL::Frame& frame, Vector6d& pose);
        static void kdlToEigen(const KDL::Jacobian& jacobian, Jacobian& jac_eigen);
        static void kdlToEigen(const KDL::JntArray& jnt_array, VectorNd& vector);
        static void kdlToEigen(const VectorNd& vector, KDL::JntArray& jnt_array);
    };
}
#endif
