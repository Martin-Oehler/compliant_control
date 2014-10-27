#ifndef CONVERSION_HELPER
#define CONVERSION_HELPER

#include <vigir_compliant_ros_controller/CustomTypes.h>

#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>

namespace compliant_controller {
    class ConversionHelper {
    public:
        static void kdlToEigen(const KDL::Vector& kdl_vector, Vector3d& eigen_vector);
        static void kdlToEigen(const KDL::Frame& frame, Vector3d& position);
        static void kdlToEigen(const KDL::Frame& frame, Vector6d& pose);
        static void kdlToEigen(const KDL::Frame& frame, Transform& transform);
        static void kdlToEigen(const KDL::Jacobian& jacobian, Jacobian& jac_eigen);
        static void kdlToEigen(const KDL::JntArray& jnt_array, VectorNd& vector);
        static void kdlToEigen(const KDL::Rotation& rotation, Matrix3d& rotation_eigen);
        static void eigenToKdl(const VectorNd& vector, KDL::JntArray& jnt_array);
        static void eigenToKdl(const Vector6d& vector, KDL::Twist &twist);
        static void eigenToKdl(const Transform& transform, KDL::Frame& frame);
        static void eigenToKdl(const Matrix3d& matrix, KDL::Rotation& rotation);
        static void eigenToKdl(const Vector3d& eigen_vector, KDL::Vector& kdl_vector);
        static void eigenToEigen(const Vector6d& vector, Eigen::Affine3d &affine);
    };
}
#endif
