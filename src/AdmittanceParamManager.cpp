#include <compliant_ros_controller/AdmittanceParamManager.h>

namespace compliant_controller {

AdmittanceParamManager::AdmittanceParamManager(AdmittanceController& controller) {
    controller_ = &controller;
}

/**
 * @brief AdmittanceParamManager::init
 * Initializes this class by starting the reconfig server.
 * @param nh NodeHandle in which the reconfig server is running
 */
void AdmittanceParamManager::init(ros::NodeHandle& nh) {
    // Set up dynamic reconfigure for admittance params
    ros::NodeHandle nh_params(nh, "admittance_params/");
    param_reconfig_server_.reset(new AdmittanceDynamicReconfigServer(param_reconfig_mutex_, nh_params));

    param_reconfig_callback_ = boost::bind(&AdmittanceParamManager::dynamicReconfigCB, this, _1, _2);
    param_reconfig_server_->setCallback(param_reconfig_callback_);
}

/**
 * @brief AdmittanceParamManager::dynamicReconfigCB
 * Callback of reconfig server.
 * @param config Contains the settings made by the user.
 * @param level
 */
void AdmittanceParamManager::dynamicReconfigCB(compliant_ros_controller::AdmittanceParamsConfig &config, uint32_t level) {
    setParams(config);
}

/**
 * @brief AdmittanceParamManager::setAdmittanceParams
 * Sends changes of the admittance parameters to the controller.
 * @param active
 * @param inertia
 * @param damping
 * @param stiffness
 * @param dead_zone
 */
void AdmittanceParamManager::setParams(compliant_ros_controller::AdmittanceParamsConfig &config) {
    controller_->activate(config.active);
    controller_->setInertia(config.inertia);
    controller_->setDamping(config.damping);
    controller_->setStiffness(config.stiffness);
    controller_->setTransDeadZone(config.dead_zone_trans);
    controller_->setRotDeadZone(config.dead_zone_rot);
    controller_->setMode(config.mode);
    controller_->setTransSpeedLimit(config.speed_limit_trans);
    controller_->setRotSpeedLimit(config.speed_limit_rot);

    if (config.active) {
        ROS_INFO_STREAM("Admittance params changed to: Active with "
                        << config.inertia << ", " << config.damping << ", " << config.stiffness
                        << ". Dead Zone: " << config.dead_zone_trans << " | " << config.dead_zone_rot
                        << ", Speed Limit: " << config.speed_limit_trans << " | " << config.speed_limit_rot
                        << " Mode: " << config.mode << ".");
    } else {
        ROS_INFO_STREAM("Admittance params changed to: Not active");
    }
    updateDynamicReconfig(config);
}

/**
 * @brief AdmittanceParamManager::updateDynamicReconfig
 * Sends updates back to the reconfig server.
 * @param config Updated config.
 */
void AdmittanceParamManager::updateDynamicReconfig(compliant_ros_controller::AdmittanceParamsConfig &config) {
    param_reconfig_mutex_.lock();
    param_reconfig_server_->updateConfig(config);
    param_reconfig_mutex_.unlock();
}

}
