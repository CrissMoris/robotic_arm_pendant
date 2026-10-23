/*
Created on Mon Sep 17 10:31:06 2018

@author: Cong Liu

 Software License Agreement (BSD License)

 Copyright (c) 2018, Han's Robot Co., Ltd.
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

  * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
  * Redistributions in binary form must reproduce the above
    copyright notice, this list of conditions and the following
    disclaimer in the documentation and/or other materials provided
    with the distribution.
  * Neither the name of the copyright holders nor the names of its
    contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 POSSIBILITY OF SUCH DAMAGE.
*/
// author: Cong Liu

#include <soem_ethercat_driver/ethercat_driver.h>

namespace soem_ethercat_driver
{
EtherCatDriver::EtherCatDriver(const std::string &ifname, const std::vector<int> &slave_no, std::string driver_name, const ros::NodeHandle &nh)
:ed_nh_(nh, driver_name), slave_no_(slave_no)
{

    // Initialize master_
    master_ = new soem_ethercat_driver::EtherCATMaster(ifname);

    // Initialize ethercat_client_
    ethercat_slaves_.clear();
    ethercat_slaves_.resize(slave_no_.size());
    for (int i = 0; i < slave_no_.size(); i++) {
        ethercat_slaves_[i] = new EtherCATSlave(master_, slave_no_[i]);
    }

    // Initialize ROS service server
    enable_robot_ = ed_nh_.advertiseService("enable_robot", &EtherCatDriver::enableRobot_cb, this);
    disable_robot_ = ed_nh_.advertiseService("disable_robot", &EtherCatDriver::disableRobot_cb, this);
    clear_fault_ = ed_nh_.advertiseService("clear_fault", &EtherCatDriver::clearFault_cb, this);

    get_error_code_ = ed_nh_.advertiseService("get_error_code", &EtherCatDriver::getErrorCode_cb, this);
    get_warning_code_ = ed_nh_.advertiseService("get_warning_code", &EtherCatDriver::getWarningCode_cb, this);

    // Initialize ROS subscriber
    feedForward_sub_ = ed_nh_.subscribe("/feed_forward_torque", 1, &EtherCatDriver::set_feedForward_torque, this);

    // Initialize ROS publisher
    enable_state_pub_ = ed_nh_.advertise<std_msgs::Bool>("enable_state", 1);
    fault_state_pub_ = ed_nh_.advertise<std_msgs::Bool>("fault_state", 1);

    // Initialize ROS torque sensor publisher
    torque_sensor_pub_ = ed_nh_.advertise<std_msgs::Float64MultiArray>("torque_sensor", 1);

    // Initialize ROS timers
    status_update_period_.sec = 0;
    status_update_period_.nsec = 1e+7;
    status_timer_ = ed_nh_.createTimer(status_update_period_, &EtherCatDriver::updateStatus, this);

    sensor_update_period_.sec = 0;
    sensor_update_period_.nsec = 2e+7;
    sensor_timer_ = ed_nh_.createTimer(sensor_update_period_, &EtherCatDriver::updateSensor, this);
    
    status_timer_.start();
    sensor_timer_.start();

}

EtherCatDriver::~EtherCatDriver()
{
    for (int i = 0; i < ethercat_slaves_.size(); i++) {
        if (ethercat_slaves_[i] != NULL) {
            delete ethercat_slaves_[i];
        }
    }
    
    if (master_ != NULL)
    {
        delete master_;
    }
}

// true: enabled; false: disabled
bool EtherCatDriver::getEnableState()
{
    bool enable_flag_tmp = true;
    for (int i = 0; i < ethercat_slaves_.size(); i++) {
        enable_flag_tmp = enable_flag_tmp && ethercat_slaves_[i]->is_enabled();
    }
    return enable_flag_tmp;
}

// true: there is a fault; false: there is no fault
bool EtherCatDriver::getFaultState()
{
    bool fault_flag_tmp = false;
    for (int i = 0; i < ethercat_slaves_.size(); i++) {
        fault_flag_tmp = fault_flag_tmp || ethercat_slaves_[i]->has_fault();
    }
    return fault_flag_tmp;
}

std::vector<float64> EtherCatDriver::getTorqueSensorState()
{
    std::vector<float64> sensor_state(ethercat_slaves_.size());
    int32_t trq_sensor_count;

    for (int i = 0; i < ethercat_slaves_.size(); i++) {
        trq_sensor_count = ethercat_slaves_[i]->get_torque_sensor();
        sensor_state[i] = (-1) * trq_sensor_count / 1000.0;  //mNm to Nm
    }

    return sensor_state;
}

void EtherCatDriver::updateStatus(const ros::TimerEvent& te)
{
    enable_state_msg_.data = getEnableState();
    fault_state_msg_.data = getFaultState();

    enable_state_pub_.publish(enable_state_msg_);
    fault_state_pub_.publish(fault_state_msg_); 
}

void EtherCatDriver::updateSensor(const ros::TimerEvent& te)
{
    torque_sensor_msg_.data = getTorqueSensorState();
    torque_sensor_pub_.publish(torque_sensor_msg_);
}

size_t EtherCatDriver::getNumberOfSlaves()
{
    return ethercat_slaves_.size();
}

EtherCATSlave* EtherCatDriver::getSlavePtr(size_t n)
{
    return ethercat_slaves_[n];
}

bool EtherCatDriver::enableRobot_cb(std_srvs::Trigger::Request &, std_srvs::Trigger::Response &resp)
{
    if (getFaultState()) {
        resp.success = false;
        resp.message = "Please clear fault first";
        return true;
    }

    std::vector<pthread_t> tids;
    tids.resize(ethercat_slaves_.size());

    std::vector<int> threads;
    threads.resize(ethercat_slaves_.size());

    for (int i = 0; i < ethercat_slaves_.size(); i++) {
        threads[i] = pthread_create(&tids[i], NULL, ethercat_slaves_[i]->enable, (void *)ethercat_slaves_[i]);
        usleep(300'000); // Don't release all the brakes at the same time
    }

    for (int i = 0; i < ethercat_slaves_.size(); i++) {
        pthread_join(tids[i], NULL);
    }

    struct timespec before, tick;
    clock_gettime(CLOCK_REALTIME, &before);
    clock_gettime(CLOCK_REALTIME, &tick);
    while (ros::ok())
    {
        if (getEnableState()) {
            resp.success = true;
            resp.message = "Robot is enabled";
            return true;
        }
        if (tick.tv_sec * 1e+9 + tick.tv_nsec - before.tv_sec * 1e+9 - before.tv_nsec >= 1e+9) {  // 20e+9
            resp.success = false;
            resp.message = "Robot is not enabled";
            return true;
        }
        usleep(100000);
        clock_gettime(CLOCK_REALTIME, &tick);
    }
    return true;
}

bool EtherCatDriver::disableRobot_cb(std_srvs::Trigger::Request &, std_srvs::Trigger::Response &resp)
{
    std::vector<pthread_t> tids;
    tids.resize(ethercat_slaves_.size());

    std::vector<int> threads;
    threads.resize(ethercat_slaves_.size());

    for (int i = 0; i < ethercat_slaves_.size(); i++) {
        threads[i] = pthread_create(&tids[i], NULL, ethercat_slaves_[i]->disable, (void *)ethercat_slaves_[i]);
    }

    for (int i = 0; i < ethercat_slaves_.size(); i++) {
        pthread_join(tids[i], NULL);
    }

    struct timespec before, tick;
    clock_gettime(CLOCK_REALTIME, &before);
    clock_gettime(CLOCK_REALTIME, &tick);
    while (ros::ok())
    {
        if (!getEnableState()) {
            resp.success = true;
            resp.message = "Robot is disabled";
            return true;
        }
        if (tick.tv_sec * 1e+9 + tick.tv_nsec - before.tv_sec * 1e+9 - before.tv_nsec >= 1e+9) {  // 20e+9
            resp.success = false;
            resp.message = "Robot is not disabled";
            return true;
        }
        usleep(100000);
        clock_gettime(CLOCK_REALTIME, &tick);
    }
    return true;
}

bool EtherCatDriver::clearFault_cb(std_srvs::Trigger::Request &, std_srvs::Trigger::Response &resp)
{
    for (int i = 0; i < ethercat_slaves_.size(); i++) {
        ethercat_slaves_[i]->clear_fault();
    }

    struct timespec before, tick;
    clock_gettime(CLOCK_REALTIME, &before);
    clock_gettime(CLOCK_REALTIME, &tick);
    while (ros::ok())
    {
        if (!getFaultState()) {
            resp.success = true;
            resp.message = "Faults are cleared";
            return true;
        }
        if (tick.tv_sec * 1e+9 + tick.tv_nsec - before.tv_sec * 1e+9 - before.tv_nsec >= 1e+9) {  // 10e+9
            resp.success = false;
            resp.message = "There are still Faults";
            return true;
        }
        usleep(100000);
        clock_gettime(CLOCK_REALTIME, &tick);
    }
    return true;
}

bool EtherCatDriver::getErrorCode_cb(std_srvs::Trigger::Request &, std_srvs::Trigger::Response &resp)
{
    std::vector<int> error_codes;
    error_codes.resize(ethercat_slaves_.size());

    for (int i = 0; i < ethercat_slaves_.size(); i++) {
        error_codes[i] = ethercat_slaves_[i]->get_error_code();
    }

    resp.success = true;
    resp.message = "";
    for (int i = 0; i < error_codes.size(); i++) {
        std::ostringstream hexCode;
        hexCode << std::hex << error_codes[i];
        
        resp.message += "Slave " + std::to_string(i+1) + ": 0x" + hexCode.str() + " ";
    }
    return true;
} 

bool EtherCatDriver::getWarningCode_cb(std_srvs::Trigger::Request &, std_srvs::Trigger::Response &resp)
{
    std::vector<int> warning_codes;
    warning_codes.resize(ethercat_slaves_.size());

    for (int i = 0; i < ethercat_slaves_.size(); i++) {
        warning_codes[i] = ethercat_slaves_[i]->get_warning_code();
    }

    resp.success = true;
    resp.message = "";
    for (int i = 0; i < warning_codes.size(); i++) {
        std::ostringstream hexCode;
        hexCode << std::hex << warning_codes[i];
        
        resp.message += "Slave " + std::to_string(i+1) + ": 0x" + hexCode.str() + " ";
    }
    return true;
}

void EtherCatDriver::set_feedForward_torque(const std_msgs::Float64MultiArrayConstPtr &msg)
{
    std::vector<float32> rated_torques = {87.0, 87.0, 52.0, 10.0, 10.0, 10.0};

    for (int i = 0; i < ethercat_slaves_.size(); i++) 
    {   
        double torque_cmd = msg->data[i];
        double count_Nm_factor = rated_torques[i] / 1000.0;
        double torque_cmd_count = (-1) * torque_cmd / count_Nm_factor;

        ethercat_slaves_[i]->set_torque_offset(torque_cmd_count);
        // std::cout << "torque_cmd: " << torque_cmd << " | torque_cmd_count: " << torque_cmd_count << std::endl;

    }
    // std::cout << "\n";
}

}
// end namespace

