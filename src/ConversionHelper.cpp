#include <vigir_compliant_ros_controller/ConversionHelper.h>

namespace control {
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
        vector.resize(jnt_array.rows());
        for (unsigned int i = 0; i < vector.size(); i++) {
            vector(i) = jnt_array(i);
        }
    }

    void ConversionHelper::kdlToEigen(const VectorNd& vector, KDL::JntArray& jnt_array) {
        jnt_array.resize(vector.size());
        for (unsigned int i = 0; i < vector.size(); i++) {
            jnt_array(i) = vector(i);
        }
    }

    void ConversionHelper::kdlToEigen(const KDL::Vector& kdl_vector, Vector3d& eigen_vector) {
        for (unsigned int i = 0; i < 3; i++) {
            eigen_vector(i) = kdl_vector(i);
        }
    }
}
