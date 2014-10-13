#include <vigir_compliant_ros_controller/ConversionHelper.h>

#include <ros/ros.h>

namespace compliant_controller {
    void ConversionHelper::kdlToEigen(const KDL::Frame& frame, Vector6d& pose) {
        for (unsigned int i = 0; i < 3; i++) {
            pose(i) = frame.p(i);
        }
        double roll, pitch, yaw;
        frame.M.GetRPY(roll, pitch, yaw);
        pose(3) = roll;
        pose(4) = pitch;
        pose(5) = yaw;
    }

    void ConversionHelper::kdlToEigen(const KDL::Frame &frame, Vector3d& position) {
        for (unsigned int i = 0; i < 3; i++) {
            position(i) = frame.p(i);
        }
    }

    void ConversionHelper::kdlToEigen(const KDL::Jacobian &jacobian, Jacobian& jac_eigen) {
        jac_eigen = jacobian.data;
    }

    void ConversionHelper::kdlToEigen(const KDL::JntArray &jnt_array, VectorNd& vector) {
        if (vector.size() != jnt_array.rows()) {
            ROS_ERROR("kdlToEigen: jnt_array.rows() doesn't match vector.size().");
            return;
        }
        for (unsigned int i = 0; i < vector.size(); i++) {
            vector(i) = jnt_array(i);
        }
    }
    void ConversionHelper::kdlToEigen(const KDL::Vector& kdl_vector, Vector3d& eigen_vector) {
        for (unsigned int i = 0; i < 3; i++) {
            eigen_vector(i) = kdl_vector(i);
        }
    }

    void ConversionHelper::kdlToEigen(const KDL::Rotation& rotation, Matrix3d& rotation_eigen) {
        for (unsigned int i = 0; i < 3; i++) {
            for (unsigned int j = 0; j < 3; j++) {
                rotation_eigen(i, j) = rotation(i, j);
            }
        }
    }

    void ConversionHelper::eigenToKdl(const Vector6d& vector, KDL::Twist& twist) {
        for (unsigned int i = 0; i < vector.size(); i++) {
            twist(i) = vector(i);
        }
    }

    void ConversionHelper::eigenToKdl(const VectorNd& vector, KDL::JntArray& jnt_array) {
        jnt_array.resize(vector.size());
        for (unsigned int i = 0; i < vector.size(); i++) {
            jnt_array(i) = vector(i);
        }
    }

}
