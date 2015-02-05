#include <compliant_ros_controller/JointAdmittanceController.h>

#include <kdl_parser/kdl_parser.hpp>
#include <compliant_ros_controller/ConversionHelper.h>

namespace compliant_controller {
bool JointAdmittanceController::init(ros::NodeHandle& node, std::string root_name, std::string tip_name, VectorNd& inertia, VectorNd& damping, VectorNd& stiffness, double step_size) {
    std::string robot_description;

    if (!node.getParam("robot_description", robot_description)) {
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

    jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));
    jnt_to_pose_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));

    e_.resize(kdl_chain_.getNrOfJoints()*2);
    for (unsigned int i = 0; i < e_.size(); i++) {
        e_(i) = 0;
    }
    inertia_ = inertia;
    damping_ = damping;
    stiffness_ = stiffness;
    step_size_ = step_size;

    std::stringstream debug;
    debug << "Initialized joint admittance controller with segments (joint): " << std::endl;
    for (unsigned int i = 0; i < kdl_chain_.getNrOfJoints(); i++) {
        debug << i << ": " << kdl_chain_.getSegment(i).getName() << " (" << kdl_chain_.getSegment(i).getJoint().getName() << ")" << std::endl;
    }
    std::cout << debug.str();
    initialized_ = true;

    return true;
}

void JointAdmittanceController::updateJointState(VectorNd& q) {
    if (q.size() != kdl_chain_.getNrOfJoints()) {
        ROS_ERROR_THROTTLE(0.5, "State vector size doesn't match chain size.");
        return;
    }
    q_ = q;
}

void JointAdmittanceController::calcCompliantPosition(const VectorNd &q0, const Vector6d &fext, VectorNd& qd_out, VectorNd &qdotd_out) {
    Jacobian J;
    KDL::Jacobian Jtmp(kdl_chain_.getNrOfJoints());
    KDL::JntArray qtmp(kdl_chain_.getNrOfJoints());
    ConversionHelper::eigenToKdl(q_,qtmp);
    jnt_to_jac_solver_->JntToJac(qtmp,Jtmp);
    ConversionHelper::kdlToEigen(Jtmp, J);

    VectorNd tau = J * fext;
    e_ = e_ + step_size_ * f(tau);
    qd_out = q0 + getE1();
    qdotd_out = getE2();
}

VectorNd JointAdmittanceController::f(const VectorNd &tau) const {
    VectorNd f1 = getE2();
    VectorNd f2 = inertia_.asDiagonal().inverse() * (tau - damping_.asDiagonal() * getE2() - stiffness_.asDiagonal() * getE1());
    VectorNd f_out(f1.size() + f2.size());
    unsigned int i;
    for (i = 0; i < f1.size(); i++) {
        f_out(i) = f1(i);
    }
    for (i = 0; i < f2.size(); i++) {
        f_out(i+f1.size()) = f2(i);
    }
    return f_out;
}

Vector3d JointAdmittanceController::getTipPosition(const VectorNd& q) {
    KDL::JntArray q_kdl(q.size());
    for (unsigned int i = 0; i < q.size(); i++) {
        q_kdl(i) = q(i);
    }
    KDL::Frame pose;
    jnt_to_pose_solver_->JntToCart(q_kdl, pose);
    Vector3d position;
    for (unsigned int i = 0; i < 3; i++) {
        position(i) = pose.p(i);
    }
    return position;
}

VectorNd JointAdmittanceController::getE1() const {
    VectorNd e1(kdl_chain_.getNrOfJoints());
    for (unsigned int i = 0; i < e1.size(); i++) {
        e1(i) = e_(i);
    }
    return e1;
}
VectorNd JointAdmittanceController::getE2() const {
    VectorNd e2(kdl_chain_.getNrOfJoints());
    for (unsigned int i = 0; i < e2.size(); i++) {
        e2(i) = e_(i+kdl_chain_.getNrOfJoints());
    }
    return e2;
}

}
