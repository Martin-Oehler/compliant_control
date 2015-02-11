#include <compliant_ros_controller/JointAdmittanceController.h>

#include <kdl_parser/kdl_parser.hpp>
#include <compliant_ros_controller/ConversionHelper.h>

namespace compliant_controller {
bool JointAdmittanceController::init(std::string root_name, std::string tip_name, Vector6d& cart_inertia, Vector6d& cart_damping, Vector6d& cart_stiffness) {
    // Load robot_description
    ros::NodeHandle private_nh("~");
    std::string robot_description;
    if (!private_nh.getParam("/robot_description", robot_description)) {
        ROS_ERROR("Failed to load robot description from parameter server.");
        return false;
    }

    // Create KDL Chain for solvers
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


    // Re-size pre-allocated variables
    e_.resize(kdl_chain_.getNrOfJoints()*2);
    for (unsigned int i = 0; i < e_.size(); i++) {
        e_(i) = 0;
    }
    q_.resize(kdl_chain_.getNrOfJoints());
    e1_.resize(kdl_chain_.getNrOfJoints());
    e2_.resize(kdl_chain_.getNrOfJoints());
    for (unsigned int i = 0; i < q_.size(); i++) {
        e1_(i) = 0;
        e2_(i) = 0;
        q_(i) = 0;
    }
    Jtmp_.resize(kdl_chain_.getNrOfJoints());
    qtmp_.resize(kdl_chain_.getNrOfJoints());

    inertia_ = cart_inertia;
    damping_ = cart_damping;
    stiffness_ = cart_stiffness;

    std::stringstream debug;
    debug << "Initialized joint admittance controller with segments (joint): " << std::endl;
    for (unsigned int i = 0; i < kdl_chain_.getNrOfJoints(); i++) {
        debug << i << ": " << kdl_chain_.getSegment(i).getName() << " (" << kdl_chain_.getSegment(i).getJoint().getName() << ")" << std::endl;
    }
    std::cout << debug.str();
    initialized_ = true;

    return true;
}

bool JointAdmittanceController::init(std::string root_name, std::string tip_name, double cart_inertia, double cart_damping, double cart_stiffness) {
    Vector6d m, d, k;
    m.setConstant(cart_inertia);
    d.setConstant(cart_damping);
    k.setConstant(cart_stiffness);
    return init(root_name, tip_name, m, d, k);
}

void JointAdmittanceController::updateJointState(VectorNd& q) {
    if (q.size() != kdl_chain_.getNrOfJoints()) {
        ROS_ERROR_STREAM_THROTTLE(0.5, "State vector size (" << q.size() << ") doesn't match chain size (" << kdl_chain_.getNrOfJoints() << ").");
        return;
    }
    q_ = q;
}

void JointAdmittanceController::calcCompliantPosition(const VectorNd &q0, const Vector6d &fext, VectorNd& qd_out, VectorNd &qdotd_out, double step_size) {
    Jacobian J;
    ConversionHelper::eigenToKdl(q_,qtmp_);
    jnt_to_jac_solver_->JntToJac(qtmp_,Jtmp_);
    ConversionHelper::kdlToEigen(Jtmp_, J);

    VectorNd tau = J.transpose() * fext;
    e_ = e_ + step_size * f(tau);
    getE1(e1_);
    getE2(e2_);
    qd_out = q0 + e1_;
    qdotd_out = e2_;
}

VectorNd JointAdmittanceController::f(const VectorNd &tau) {
    getE1(e1_);
    getE2(e2_);
    VectorNd f1 = e2_;
    VectorNd f2 = inertia_.asDiagonal().inverse() * (tau - damping_.asDiagonal() * e2_ - stiffness_.asDiagonal() * e1_);
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

void JointAdmittanceController::getE1(VectorNd &e1) {
    for (unsigned int i = 0; i < e1.size(); i++) {
        e1(i) = e_(i);
    }
}
void JointAdmittanceController::getE2(VectorNd &e2) {
    for (unsigned int i = 0; i < e2.size(); i++) {
        e2(i) = e_(i+kdl_chain_.getNrOfJoints());
    }
}

}
