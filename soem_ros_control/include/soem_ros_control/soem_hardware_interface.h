#ifndef SOEM_HW_INTERFACE
#define SOEM_HW_INTERFACE

#include <ros/ros.h>
#include <pthread.h>
#include <time.h>
#include <math.h>
#include <string>

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <sensor_msgs/JointState.h>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/posvel_command_interface.h>
#include <hardware_interface/robot_hw.h>

#include <controller_manager/controller_manager.h>
#include <controller_manager_msgs/ListControllers.h>

#include "soem_ethercat_driver/ethercat_driver.h"

namespace soem_ros_control {

struct AxisInfo {
    soem_ethercat_driver::EtherCATSlave *slave_ptr;
    std::string name;
    double count_rad_factor;
    double count_Nm_factor;
    int32_t encoder_zero;

    double encoder_count;
    double rated_torques;

    double position;
    double velocity;
    double effort;
    double torque_sensor;

    double position_cmd;
    double velocity_cmd;
    double effort_cmd;
};

class soem_hw_interface : public hardware_interface::RobotHW {
public:
    soem_hw_interface(const std::string& ifname, const ros::NodeHandle &nh = ros::NodeHandle("~"));
    ~soem_hw_interface();
    bool prepareSwitch(const std::list<hardware_interface::ControllerInfo> &start_list,
                       const std::list<hardware_interface::ControllerInfo> &stop_list);

    void doSwitch(const std::list<hardware_interface::ControllerInfo> &start_list,
                  const std::list<hardware_interface::ControllerInfo> &stop_list);

    void read_init();
    void read();
    void write();

private:
    soem_ethercat_driver::EtherCatDriver *ethercat_driver_;
    std::vector<AxisInfo> joint_info_;

    hardware_interface::JointStateInterface jnt_state_interface_;
    hardware_interface::PositionJointInterface jnt_position_cmd_interface_;
    hardware_interface::EffortJointInterface jnt_effort_cmd_interface_;

    ros::NodeHandle n_;

    std::vector<bool> pre_switch_flags_;
    std::vector<boost::shared_ptr<boost::mutex>> pre_switch_mutex_ptrs_;

    bool isModuleMoving(int module_num);
    double motion_threshold_;
    bool use_torque_sensor_;
    bool setGroupPosMode(const std::vector<int> &module_no);
    bool setGroupTrqMode(const std::vector<int> &module_no);


    std::vector<int> slave_no_;
    std::vector<std::string> joint_names_;
    std::vector<double> encoder_resolution_;
    std::vector<double> rated_torques_;
    std::vector<int32_t> encoder_zeros_;

};

}

#endif


