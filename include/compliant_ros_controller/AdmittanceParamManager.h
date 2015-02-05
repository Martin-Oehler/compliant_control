#ifndef ADMITTANCE_PARAM_MANAGER_H
#define ADMITTANCE_PARAM_MANAGER_H

#include <ros/ros.h>

#include <compliant_ros_controller/AdmittanceController.h>
#include <compliant_ros_controller/AdmittanceParamsConfig.h>
#include <dynamic_reconfigure/server.h>

#include <boost/shared_ptr.hpp>

namespace compliant_controller {
    class AdmittanceParamManager {
    public:
        AdmittanceParamManager(AdmittanceController &controller);
        void init(ros::NodeHandle& nh);
        void dynamicReconfigCB(compliant_ros_controller::AdmittanceParamsConfig &config, uint32_t level);
        void updateDynamicReconfig(compliant_ros_controller::AdmittanceParamsConfig &config);
        void setParams(compliant_ros_controller::AdmittanceParamsConfig &config);
    private:
        AdmittanceController* controller_;

        typedef dynamic_reconfigure::Server<compliant_ros_controller::AdmittanceParamsConfig> AdmittanceDynamicReconfigServer;
        boost::shared_ptr<AdmittanceDynamicReconfigServer> param_reconfig_server_;
        AdmittanceDynamicReconfigServer::CallbackType param_reconfig_callback_;

        boost::recursive_mutex param_reconfig_mutex_;
    };
}

#endif
