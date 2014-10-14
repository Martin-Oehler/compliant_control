#include <vigir_compliant_ros_controller/CartForceController.h>

#include <kdl_parser/kdl_parser.hpp>
#include <vigir_compliant_ros_controller/ConversionHelper.h>

namespace compliant_controller {
    bool CartForceController::init(const ros::NodeHandle& node, const std::string& root_name, const std::string& tip_name, double kp, double kd, double ki) {
        Vector6d kp_vector;
        Vector6d kd_vector;
        Vector6d ki_vector;
        for (unsigned int i = 0; i < 6; i++) {
            kp_vector(i) = kp;
            kd_vector(i) = kd;
            ki_vector(i) = ki;
        }
        return init(node, root_name, tip_name, kp_vector.asDiagonal(), kd_vector.asDiagonal(), ki_vector.asDiagonal());
    }

    bool CartForceController::init(const ros::NodeHandle &node, const std::string &root_name, const std::string &tip_name, const Matrix6d &Kp, const Matrix6d &Kd, const Matrix6d &Ki) {
        std::string robot_description;

        if (!node.getParam("/robot_description", robot_description)) {
            ROS_ERROR("Failed to get robot description from parameter server.");
            return false;
        }

        tip_name_ = tip_name;
        root_name_ = root_name;

       if (!kdl_parser::treeFromString(robot_description,kdl_tree_)) {
           ROS_ERROR("Error constructing kdl tree from robot_description.");
           return false;
       }

       if (!kdl_tree_.getChain(root_name_,tip_name_,kdl_chain_)) {
           ROS_ERROR("Error constructing kdl chain from kdl tree.");
           return false;
       }

       jnt_to_pose_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
       jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));

       // resize dynamic objects
       q_.resize(kdl_chain_.getNrOfJoints());
       qdot_.resize(kdl_chain_.getNrOfJoints());
       Jtmp_.resize(kdl_chain_.getNrOfJoints());
       qtmp_.resize(kdl_chain_.getNrOfJoints());
       qdottmp_.resize(kdl_chain_.getNrOfJoints());

       Kp_ = Kp;
       Kd_ = Kd;
       Ki_ = Ki;
       integral_ = Vector6d::Zero();

       std::stringstream debug;
       debug << "Initialized Cartesian controller with segments (joint): " << std::endl;
       for (unsigned int i = 0; i < kdl_chain_.getNrOfJoints(); i++) {
           debug << i << ": " << kdl_chain_.getSegment(i).getName() << " (" << kdl_chain_.getSegment(i).getJoint().getName() << ")" << std::endl;
       }
       std::cout << debug.str();
       initialized_ = true;
       return true;
    }

    void CartForceController::updateJointState(const VectorNd& q, const VectorNd& qdot) {
        jac_updated_ = false;
        if (q_.size() != q.size() || qdot_.size() != qdot.size()) {
            ROS_ERROR_STREAM("updateJointState: One of the given vectors q(" << q.size() << "), qdot(" << qdot.size() << ") doesn't match joint number (" << kdl_chain_.getNrOfJoints() << ").");
            return;
        }
        q_ = q;
        qdot_ = qdot;
    }

    bool CartForceController::calcCorrectionVector(const Vector6d& xd, const Vector6d& xdotd, Vector6d& force, double step_size) {
        Vector6d x, xdot;
        ConversionHelper::eigenToKdl(q_,qtmp_);
        if (jnt_to_pose_solver_->JntToCart(qtmp_, xtmp_) < 0) {
            ROS_ERROR("Failed to compute forward kinematics.");
            return false;
        }
        ConversionHelper::kdlToEigen(xtmp_, x);
        if (!jac_updated_) {
            ConversionHelper::eigenToKdl(q_, qtmp_);
            jnt_to_jac_solver_->JntToJac(qtmp_,Jtmp_);
            ConversionHelper::kdlToEigen(Jtmp_, J_);
            jac_updated_ = true;
        }
        xdot = J_ * qdot_;

        // calculate errors
        Vector6d pos_error;
        calcCartError(xd, x, pos_error);
        Vector6d vel_error;
        calcCartError(xdotd, xdot, vel_error);

        // calculate integral
        integral_ = integral_ + step_size*Ki_*pos_error;
        force = Kp_*pos_error + Kd_*vel_error + integral_;

        return true;
    }

    Vector6d CartForceController::calcCorrectionVector(const Vector6d& xd, const Vector6d& xdotd, double step_size) {
        Vector6d force;
        calcCorrectionVector(xd, xdotd, force, step_size);
        return force;
    }

    // not real-time safe
    VectorNd CartForceController::calcTorques(const Vector6d& force) {
        VectorNd torques(kdl_chain_.getNrOfJoints());
        calcTorques(force, torques);
        return torques;
    }

    bool CartForceController::calcTorques(const Vector6d &force, VectorNd &torques) {
        // initialize output to 0 (in case of error)
        for (unsigned int i = 0; i < torques.size(); i++) {
            torques(i) = 0;
        }
        if (torques.size() != kdl_chain_.getNrOfJoints()) {
            ROS_ERROR_STREAM("Size of torque vector ( + " << torques.size() << ") doesn't match number of joints (" << kdl_chain_.getNrOfJoints() << ").");
            return false;
        }
        if (!jac_updated_) {
            ConversionHelper::eigenToKdl(q_, qtmp_);
            jnt_to_jac_solver_->JntToJac(qtmp_,Jtmp_);
            ConversionHelper::kdlToEigen(Jtmp_, J_);
            jac_updated_ = true;
        }

        // calculate integral
        torques = J_.transpose() * force;

        return true;
    }

    Matrix3d CartForceController::rotFromRPY(double roll, double pitch, double yaw) const {
        Matrix3d rot;
        double ca1,cb1,cc1,sa1,sb1,sc1;
        ca1 = cos(yaw); sa1 = sin(yaw);
        cb1 = cos(pitch);sb1 = sin(pitch);
        cc1 = cos(roll);sc1 = sin(roll);
        rot << ca1*cb1, ca1*sb1*sc1 - sa1*cc1, ca1*sb1*cc1 + sa1*sc1,
               sa1*cb1, sa1*sb1*sc1 + ca1*cc1, sa1*sb1*cc1 - ca1*sc1,
               -sb1,    cb1*sc1,               cb1*cc1;
        return rot;
    }

    void CartForceController::calcCartError(const Vector6d& xd, const Vector6d& x, Vector6d& x_err) const {
        x_err.block<3,1>(0,0) = xd.block<3,1>(0,0) - x.block<3,1>(0,0);

        Matrix3d xd_rot = rotFromRPY(xd(3), xd(4), xd(5));
        Matrix3d x_rot = rotFromRPY(x(3), x(4), x(5));
        x_err.block<3,1>(3,0) = -0.5 * (xd_rot.col(0).cross(x_rot.col(0)) + xd_rot.col(1).cross(x_rot.col(1)) + xd_rot.col(2).cross(x_rot.col(2)));
    }

    void CartForceController::getTipPose(KDL::Frame& pose) {
        ConversionHelper::eigenToKdl(q_, qtmp_);
        jnt_to_pose_solver_->JntToCart(qtmp_,pose);
    }
}
