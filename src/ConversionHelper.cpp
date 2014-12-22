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

   void ConversionHelper::kdlToEigen(const KDL::Frame& frame, Transform& transform) {
        kdlToEigen(frame.p, transform.translation);
        kdlToEigen(frame.M, transform.rotation);
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
    void ConversionHelper::eigenToKdl(const Transform& transform, KDL::Frame& frame) {
        eigenToKdl(transform.rotation, frame.M);
        eigenToKdl(transform.translation, frame.p);
    }

    void ConversionHelper::eigenToKdl(const Matrix3d& matrix, KDL::Rotation& rotation) {
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                rotation(i, j) = matrix(i, j);
            }
        }
    }

    void ConversionHelper::eigenToKdl(const Vector3d& eigen_vector, KDL::Vector& kdl_vector) {
        for (int i = 0; i < 3; i++) {
            kdl_vector(i) = eigen_vector(i);
        }
    }

    void ConversionHelper::eigenToEigen(const Vector6d& vector, Eigen::Affine3d& affine) {
        double ca1,cb1,cc1,sa1,sb1,sc1;
        ca1 = cos(vector(5)); sa1 = sin(vector(5));
        cb1 = cos(vector(4)); sb1 = sin(vector(4));
        cc1 = cos(vector(3)); sc1 = sin(vector(3));
        affine.matrix() << ca1*cb1, ca1*sb1*sc1 - sa1*cc1, ca1*sb1*cc1 + sa1*sc1, vector(0),
                           sa1*cb1, sa1*sb1*sc1 + ca1*cc1, sa1*sb1*cc1 - ca1*sc1, vector(1),
                              -sb1,               cb1*sc1,               cb1*cc1, vector(2),
                                 0,                     0,                     0,         1;
    }

  void ConversionHelper::rotToQuatd(const Matrix3d& rot, Quatd& quat) {
      KDL::Rotation kdl_rot;
      for (unsigned int i = 0; i < 3; i++) {
          for (unsigned int j = 0; j < 3; j++) {
              kdl_rot(i, j) = rot(i, j);
          }
      }
      double x, y, z, w;
      kdl_rot.GetQuaternion(x, y, z, w);
      quat.x() = x;
      quat.y() = y;
      quat.z() = z;
      quat.w() = w;
  }

}
