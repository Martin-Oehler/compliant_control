#ifndef ADMITTANCE_PARAM_MANAGER_H
#define ADMITTANCE_PARAM_MANAGER_H

#include <ros/ros.h>

#include <vigir_compliant_ros_controller/AdmittanceController.h>
#include <vigir_compliant_ros_controller/VigirAdmittanceParamsConfig.h>
#include <dynamic_reconfigure/server.h>

#include <boost/shared_ptr.hpp>

namespace compliant_controller {
    class AdmittanceParamManager {
    public:
        AdmittanceParamManager(AdmittanceController &controller);
        void init(ros::NodeHandle& nh);
        void dynamicReconfigCB(vigir_compliant_ros_controller::VigirAdmittanceParamsConfig &config, uint32_t level);
        void updateDynamicReconfig(vigir_compliant_ros_controller::VigirAdmittanceParamsConfig &config);
        void setAdmittanceParams(bool active, double inertia, double damping, double stiffness, double dead_zone);
    private:
        AdmittanceController* controller_;

        typedef dynamic_reconfigure::Server<vigir_compliant_ros_controller ::VigirAdmittanceParamsConfig> AdmittanceDynamicReconfigServer;
        boost::shared_ptr<AdmittanceDynamicReconfigServer> param_reconfig_server_;
        AdmittanceDynamicReconfigServer::CallbackType param_reconfig_callback_;

        boost::recursive_mutex param_reconfig_mutex_;
    };
}

#endif
