#include <vigir_compliant_ros_controller/AdmittanceParamManager.h>

namespace compliant_controller {

AdmittanceParamManager::AdmittanceParamManager(AdmittanceController& controller) {
    controller_ = &controller;
}

void AdmittanceParamManager::init(ros::NodeHandle& nh) {
    // Set up dynamic reconfigure for admittance params
    ros::NodeHandle nh_params(nh, "admittance_params/");
    param_reconfig_server_.reset(new AdmittanceDynamicReconfigServer(param_reconfig_mutex_, nh_params));

    param_reconfig_callback_ = boost::bind(&AdmittanceParamManager::dynamicReconfigCB, this, _1, _2);
    param_reconfig_server_->setCallback(param_reconfig_callback_);
}

void AdmittanceParamManager::dynamicReconfigCB(vigir_compliant_ros_controller::VigirAdmittanceParamsConfig &config, uint32_t level) {
    setAdmittanceParams(config.active, config.inertia, config.damping, config.stiffness);
}

void AdmittanceParamManager::updateDynamicReconfig(vigir_compliant_ros_controller::VigirAdmittanceParamsConfig &config) {
    param_reconfig_mutex_.lock();
    param_reconfig_server_->updateConfig(config);
    param_reconfig_mutex_.unlock();
}

void AdmittanceParamManager::setAdmittanceParams(bool active, double inertia, double damping, double stiffness) {
    controller_->activate(active);
    controller_->setInertia(inertia);
    controller_->setDamping(damping);
    controller_->setStiffness(stiffness);

    ROS_INFO_STREAM("Admittance params changed to: " << inertia << ", " << damping << ", " << stiffness << ".");

    vigir_compliant_ros_controller::VigirAdmittanceParamsConfig config;
    config.inertia = inertia;
    config.stiffness = stiffness;
    config.damping = damping;
    config.active = active;
    updateDynamicReconfig(config);
}

}