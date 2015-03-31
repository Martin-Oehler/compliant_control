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
       ROS_ERROR_STREAM("Error constructing kdl chain from kdl tree. Root: " << root_name_ << " Tip: " << tip_name_);
       return false;
    }
    jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));
    jnt_to_pose_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));


    // Re-size pre-allocated variables
    e_.setZero(kdl_chain_.getNrOfJoints()*2);
    f_out_.setZero(kdl_chain_.getNrOfJoints()*2);

    q_.setZero(kdl_chain_.getNrOfJoints());
    q0_.setZero(kdl_chain_.getNrOfJoints());
    qd_out_.setZero(kdl_chain_.getNrOfJoints());
    qdotd_out_.setZero(kdl_chain_.getNrOfJoints());
    e1_.setZero(kdl_chain_.getNrOfJoints());
    e2_.setZero(kdl_chain_.getNrOfJoints());
    tau_.setZero(kdl_chain_.getNrOfJoints());
    jnt_inertia_.setZero(kdl_chain_.getNrOfJoints(), kdl_chain_.getNrOfJoints());
    jnt_damping_.setZero(kdl_chain_.getNrOfJoints(), kdl_chain_.getNrOfJoints());
    jnt_stiffness_.setZero(kdl_chain_.getNrOfJoints(), kdl_chain_.getNrOfJoints());

    jac_.resize(Eigen::NoChange, kdl_chain_.getNrOfJoints());
    Jtmp_.resize(kdl_chain_.getNrOfJoints());
    qtmp_.resize(kdl_chain_.getNrOfJoints());

    cart_inertia_ = cart_inertia;
    cart_damping_ = cart_damping;
    cart_stiffness_ = cart_stiffness;

    virtual_force_ = Vector6d::Zero();

    std::stringstream debug;
    debug << "Initialized joint admittance controller with segments (joint): " << std::endl;
    for (unsigned int i = 0; i < kdl_chain_.getNrOfJoints(); i++) {
        debug << i << ": " << kdl_chain_.getSegment(i).getName() << " (" << kdl_chain_.getSegment(i).getJoint().getName() << ")" << std::endl;
    }
    std::cout << debug.str();
    initialized_ = true;

    ros::NodeHandle nh("joint_admittance_controller");
    wrench_sub_ = nh.subscribe("apply_force", 1, &JointAdmittanceController::applyForceCB, this);

    return true;
}

bool JointAdmittanceController::init(std::string root_name, std::string tip_name, double cart_inertia, double cart_damping, double cart_stiffness) {
    Vector6d m, d, k;
    m.setConstant(cart_inertia);
    d.setConstant(cart_damping);
    k.setConstant(cart_stiffness);
    return init(root_name, tip_name, m, d, k);
}

void JointAdmittanceController::starting() {
    e_.setZero();
}

void JointAdmittanceController::stopping() {

}

void JointAdmittanceController::updateJointState(const VectorNd &q) {
    if (q.size() != kdl_chain_.getNrOfJoints()) {
        ROS_ERROR_STREAM_THROTTLE(0.5, "State vector size (" << q.size() << ") doesn't match chain size (" << kdl_chain_.getNrOfJoints() << ").");
        return;
    }
    q_ = q;
}

void JointAdmittanceController::updateJointState(const std::vector<double> &q) {
    if (q.size() != kdl_chain_.getNrOfJoints()) {
        ROS_ERROR_STREAM_THROTTLE(0.5, "State vector size (" << q.size() << ") doesn't match chain size (" << kdl_chain_.getNrOfJoints() << ").");
        return;
    }
    for (unsigned int i = 0; i < q.size(); i++) {
        q_(i) = q[i];
    }
}

void JointAdmittanceController::update(const std::vector<double> q0, const Vector6d& fext, std::vector<double> &qd_out, std::vector<double> &qdotd_out, double step_size) {
    for (unsigned int i = 0; i < q0.size(); i++) {
        q0_(i) = q0[i];
    }
    update(q0_, fext, qd_out_, qdotd_out_, step_size);
    for (unsigned int i = 0; i < q0.size(); i++) {
        qd_out[i] = qd_out_(i);
        qdotd_out[i] = qdotd_out_(i);
    }
}

void JointAdmittanceController::update(const VectorNd &q0, const Vector6d &fext, VectorNd& qd_out, VectorNd &qdotd_out, double step_size) {
    ConversionHelper::eigenToKdl(q_,qtmp_);
    jnt_to_jac_solver_->JntToJac(qtmp_,Jtmp_);
    ConversionHelper::kdlToEigen(Jtmp_, jac_);

    tau_ = jac_.transpose() * (fext + virtual_force_);
    stepFunction();
    e_ = e_ + step_size * f_out_;
    getE1();
    getE2();
    qd_out = q0 + e1_;
    qdotd_out = e2_;
}

void JointAdmittanceController::stepFunction() {
    // Convert Cartesian to joint params
    jnt_inertia_ = jac_.transpose() * cart_inertia_.asDiagonal() * jac_;
    jnt_damping_ = jac_.transpose() * cart_damping_.asDiagonal() * jac_;
    jnt_stiffness_ = jac_.transpose() * cart_stiffness_.asDiagonal() * jac_;
    getE1();
    getE2();
    f_out_.block(0, 0, kdl_chain_.getNrOfJoints(), 1) = e2_;
    //f_out_.block(kdl_chain_.getNrOfJoints(), 0, kdl_chain_.getNrOfJoints(), 1) = jnt_inertia_.inverse() * (tau_ - jnt_damping_ * e2_ - jnt_stiffness_ * e1_);
    f_out_.block(kdl_chain_.getNrOfJoints(), 0, kdl_chain_.getNrOfJoints(), 1) = tau_ - jnt_damping_ * e2_ - jnt_stiffness_ * e1_;
}

Vector3d JointAdmittanceController::getTipPosition(const VectorNd& q) {
    ConversionHelper::eigenToKdl(q,qtmp_);
    KDL::Frame pose;
    jnt_to_pose_solver_->JntToCart(qtmp_, pose);
    Vector3d position;
    for (unsigned int i = 0; i < 3; i++) {
        position(i) = pose.p(i);
    }
    return position;
}

void JointAdmittanceController::getE1() {
    for (unsigned int i = 0; i < e1_.size(); i++) {
        e1_(i) = e_(i);
    }
}
void JointAdmittanceController::getE2() {
    for (unsigned int i = 0; i < e2_.size(); i++) {
        e2_(i) = e_(i+kdl_chain_.getNrOfJoints());
    }
}

void JointAdmittanceController::applyForceCB(const geometry_msgs::WrenchStampedConstPtr& wrench_ptr) {
    virtual_force_(0) = wrench_ptr->wrench.force.x;
    virtual_force_(1) = wrench_ptr->wrench.force.y;
    virtual_force_(2) = wrench_ptr->wrench.force.z;

    virtual_force_(3) = wrench_ptr->wrench.torque.x;
    virtual_force_(4) = wrench_ptr->wrench.torque.y;
    virtual_force_(5) = wrench_ptr->wrench.torque.z;
    ROS_INFO_STREAM("Received virtual force: " << virtual_force_);
}

}
