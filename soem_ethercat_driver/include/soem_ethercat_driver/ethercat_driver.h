#ifndef SOEM_ETHERCAT_DRIVER_H
#define SOEM_ETHERCAT_DRIVER_H

#include <soem_ethercat_driver/ethercat_slave.h>

namespace soem_ethercat_driver
{
    class EtherCatDriver
  {
  public:
    EtherCatDriver(const std::string &ifname, const std::vector<int> &slave_no, std::string driver_name, const ros::NodeHandle &nh = ros::NodeHandle("~"));
    ~EtherCatDriver();

    bool getEnableState();
    bool getFaultState();
    std::vector<float64> getTorqueSensorState();
    void updateStatus(const ros::TimerEvent &te);
    void updateSensor(const ros::TimerEvent &te);

    size_t getNumberOfSlaves();
    EtherCATSlave *getSlavePtr(size_t n);

    bool enableRobot_cb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &resp);
    bool disableRobot_cb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &resp);
    bool clearFault_cb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &resp);

    bool getErrorCode_cb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &resp);
    bool getWarningCode_cb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &resp);

    void set_feedForward_torque(const std_msgs::Float64MultiArrayConstPtr &msg);
    
  private:
    std::vector<EtherCATSlave *> ethercat_slaves_;
    std::vector<int> slave_no_;

    EtherCATMaster *master_;
    ros::NodeHandle ed_nh_;

    ros::ServiceServer enable_robot_;
    ros::ServiceServer disable_robot_;
    ros::ServiceServer clear_fault_;

    ros::ServiceServer get_error_code_;
    ros::ServiceServer get_warning_code_;
    
    std_msgs::Bool enable_state_msg_;
    std_msgs::Bool fault_state_msg_;
    std_msgs::Float64MultiArray torque_sensor_msg_;

    ros::Subscriber feedForward_sub_;

    ros::Publisher enable_state_pub_;
    ros::Publisher fault_state_pub_;
    ros::Publisher torque_sensor_pub_;

    ros::Duration status_update_period_;
    ros::Duration sensor_update_period_;
    ros::Timer status_timer_;
    ros::Timer sensor_timer_;

  };

}

#endif
