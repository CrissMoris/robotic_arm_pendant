#include "soem_ros_control/soem_hardware_interface.h"

namespace soem_ros_control
{
soem_hw_interface::soem_hw_interface(const std::string& ifname, const ros::NodeHandle &nh) 
:n_(nh)
{

    // Initialize slave_no_
    if (!n_.getParam("ethercat/slave_no", slave_no_) || slave_no_.size() == 0) {
        ROS_ERROR("slave_no parameter is not set or is empty");
        exit(0);
    }

    // Initialize joint_names_
    if (!n_.getParam("ethercat/joint_names", joint_names_) || joint_names_.size() != slave_no_.size())
    {
        ROS_ERROR("the number of joint names is %lu, it should be %lu", joint_names_.size(), slave_no_.size());
        exit(0);
    }
 
    // Initialize encoder_resolution_
    if (!n_.getParam("ethercat/encoder_resolution", encoder_resolution_) || encoder_resolution_.size() != slave_no_.size())
    {
        ROS_ERROR("the number of axis position factors is %lu, it should be %lu", encoder_resolution_.size(), slave_no_.size());
        exit(0);
    }

    // Initialize rated_torques_
    if (!n_.getParam("ethercat/rated_torques", rated_torques_) || rated_torques_.size() != slave_no_.size())
    {
        ROS_ERROR("the number of rated torques is %lu, it should be %lu", rated_torques_.size(), slave_no_.size());
        exit(0);
    }

    // Initialize encoder_zeros_
     if (!n_.getParam("ethercat/encoder_zeros", encoder_zeros_) ||encoder_zeros_.size() != slave_no_.size())
    {
        ROS_ERROR("the number of encoder_zeros is %lu, it should be %lu", encoder_zeros_.size(), slave_no_.size());
        exit(0);
    }

    // Initialize use_torque_sensor_
    use_torque_sensor_ = nh.param<bool>("use_torque_sensor", false);
    
    if (use_torque_sensor_)
    {
        ROS_INFO("Using torque sensor");
    }
    

    ethercat_driver_ = new soem_ethercat_driver::EtherCatDriver(ifname, slave_no_, "ethercat", nh);
    // Initialize joint_info_
    joint_info_.clear();
    for (size_t j = 0; j < ethercat_driver_->getNumberOfSlaves(); j++)
    {
        AxisInfo joint_info_tmp;
        joint_info_tmp.slave_ptr = ethercat_driver_->getSlavePtr(j);

        joint_info_tmp.name = joint_names_[j];
        joint_info_tmp.encoder_count = encoder_resolution_[j];
        joint_info_tmp.encoder_zero = encoder_zeros_[j];
        joint_info_tmp.rated_torques= rated_torques_[j];

        joint_info_tmp.count_rad_factor = joint_info_tmp.encoder_count / (2 * M_PI);
        joint_info_tmp.count_Nm_factor = joint_info_tmp.rated_torques / 1000.0;
        joint_info_.push_back(joint_info_tmp);
    }

    // Initialize pre_switch_flags_ and pre_switch_mutex_ptrs_
    pre_switch_flags_.resize(joint_info_.size());
    for (int i = 0; i < pre_switch_flags_.size(); i++)
    {
        pre_switch_flags_[i] = false;
    }

    pre_switch_mutex_ptrs_.resize(joint_info_.size());
    for (int i = 0; i < pre_switch_mutex_ptrs_.size(); i++)
    {
        pre_switch_mutex_ptrs_[i] = boost::shared_ptr<boost::mutex>(new boost::mutex);
    }

    // Joint State interface
    for (size_t i = 0; i < joint_info_.size(); i++)
    {
        hardware_interface::JointStateHandle jnt_state_handle_tmp(joint_info_[i].name,
                                                                   &joint_info_[i].position,
                                                                   &joint_info_[i].velocity,
                                                                   &joint_info_[i].effort);
        jnt_state_interface_.registerHandle(jnt_state_handle_tmp);
    }
    registerInterface(&jnt_state_interface_);

    // Position interface
    for (size_t i = 0; i < joint_info_.size(); i++)
    {
        hardware_interface::JointHandle jnt_handle_tmp(jnt_state_interface_.getHandle(joint_info_[i].name),
                                                        &joint_info_[i].position_cmd);
        jnt_position_cmd_interface_.registerHandle(jnt_handle_tmp);
    }
    registerInterface(&jnt_position_cmd_interface_);

    //Effort interface
    for (size_t i = 0; i < joint_info_.size(); i++)
    {
        hardware_interface::JointHandle jnt_handle_tmp(jnt_state_interface_.getHandle(joint_info_[i].name),
                                                        &joint_info_[i].effort_cmd);
        jnt_effort_cmd_interface_.registerHandle(jnt_handle_tmp);
    }
    registerInterface(&jnt_effort_cmd_interface_);

    // Initialize motion_threshold_
    motion_threshold_ = 5e-5;
}

soem_hw_interface::~soem_hw_interface()
{
    if (ethercat_driver_ != NULL)
        delete ethercat_driver_;
}

bool soem_hw_interface::isModuleMoving(int i)
{
    double previous_pos {};
    double last_pos {};
    int32_t count {};

    count = joint_info_[i].slave_ptr->get_position_actual();
    previous_pos = count / joint_info_[i].count_rad_factor;

    usleep(10000);

    count = joint_info_[i].slave_ptr->get_position_actual();
    last_pos = count / joint_info_[i].count_rad_factor;

    if (fabs(last_pos - previous_pos) > motion_threshold_) {
        return true;
    }
    else {
        return false;
    }
}

bool soem_hw_interface::setGroupPosMode(const std::vector<int> &module_no)
{
    for (int i = 0; i < module_no.size(); i++)
    {
        boost::mutex::scoped_lock pre_switch_flags_lock(*pre_switch_mutex_ptrs_[module_no[i]]);
        pre_switch_flags_[module_no[i]] = true;
        pre_switch_flags_lock.unlock();
    }

    usleep(5000);

    for (int i = 0; i < module_no.size(); i++)
    {
        joint_info_[module_no[i]].slave_ptr->set_target_torque(0x0);
    }

    for (int i = 0; i < module_no.size(); i++)
    {
        joint_info_[module_no[i]].slave_ptr->switch_to_position_mode();
    }

    usleep(10000);

    for (int i = 0; i < module_no.size(); i++)
    {
        if (!joint_info_[module_no[i]].slave_ptr->in_position_mode())
        {
            ROS_ERROR("joint[%i]: set position mode failed", module_no[i]);

            for (int k = 0; k < module_no.size(); k++)
            {
                boost::mutex::scoped_lock pre_switch_flags_lock(*pre_switch_mutex_ptrs_[module_no[k]]);
                pre_switch_flags_[module_no[k]] = false;
                pre_switch_flags_lock.unlock();
            }

            return false;
        }
    }

    return true;
}

bool soem_hw_interface::setGroupTrqMode(const std::vector<int> &module_no)
{
    for (int i = 0; i < module_no.size(); i++)
    {
        boost::mutex::scoped_lock pre_switch_flags_lock(*pre_switch_mutex_ptrs_[module_no[i]]);
        pre_switch_flags_[module_no[i]] = true;
        pre_switch_flags_lock.unlock();
    }

    usleep(5000);

    for (int i = 0; i < module_no.size(); i++)
    {
        joint_info_[module_no[i]].slave_ptr->set_target_torque(0x0);
    }

    for (int i = 0; i < module_no.size(); i++)
    {
        joint_info_[module_no[i]].slave_ptr->switch_to_torque_mode();
    }

    usleep(10000);

    bool set_trq_success = true;
    for (int i = 0; i < module_no.size(); i++)
    {
        if (!joint_info_[module_no[i]].slave_ptr->in_torque_mode())
        {
            ROS_ERROR("joint[%i]: set torque mode failed, setting position mode", module_no[i]);
            set_trq_success = false;
            break;
        }
    }

    if (!set_trq_success)
    {
        for (int i = 0; i < module_no.size(); i++)
        {
            joint_info_[module_no[i]].slave_ptr->switch_to_position_mode();
        }

        usleep(10000);

        for (int i = 0; i < module_no.size(); i++)
        {
            if (!joint_info_[module_no[i]].slave_ptr->in_position_mode())
            {
                ROS_ERROR("joint[%i]: set position mode failed too", module_no[i]);
            }
        }

        for (int i = 0; i < module_no.size(); i++)
        {
            boost::mutex::scoped_lock pre_switch_flags_lock(*pre_switch_mutex_ptrs_[module_no[i]]);
            pre_switch_flags_[module_no[i]] = false;
            pre_switch_flags_lock.unlock();
        }

        return false;
    }

    return true;
}

bool soem_hw_interface::prepareSwitch(const std::list<hardware_interface::ControllerInfo> &start_list,
                                       const std::list<hardware_interface::ControllerInfo> &stop_list)
{
    std::list<hardware_interface::ControllerInfo>::const_iterator iter;

    /////////////////////////
    //debug

    std::cout << "----------Stop List["<< stop_list.size() <<"]----------" << std::endl;
    
    for (iter = stop_list.begin(); iter != stop_list.end(); iter++) {
        std::cout << "Name: "<<iter->name << "\nType: " << iter->type << std::endl;
        std::cout << "Claimed Resources[" << iter->claimed_resources.size() << "]: ";

        for(int i = 0; i < iter->claimed_resources.size(); i++) {
            std::cout << "\n\tHardware Interface: "<< iter->claimed_resources[i].hardware_interface;
                        
            std::cout << "\n\tResources["<<iter->claimed_resources[i].resources.size() << "]: {";

            for (std::string const& rsc : iter->claimed_resources[i].resources)
            {
                std::cout << rsc << ",";
            }
             std::cout << "}\n" << std::endl;
        }
    }
    std::cout << "--------------------------------\n" << std::endl;


    std::cout << "----------Start List["<< start_list.size() <<"]----------" << std::endl;
    
    for (iter = start_list.begin(); iter != start_list.end(); iter++) {
        std::cout << "Name: "<<iter->name << "\nType: " << iter->type << std::endl;
        std::cout << "Claimed Resources[" << iter->claimed_resources.size() << "]: ";

        for(int i = 0; i < iter->claimed_resources.size(); i++) {
            std::cout << "\n\tHardware Interface: "<< iter->claimed_resources[i].hardware_interface;
                        
            std::cout << "\n\tResources["<<iter->claimed_resources[i].resources.size() << "]: {";

            for (std::string const& rsc : iter->claimed_resources[i].resources)
            {
                std::cout << rsc << ",";
            }
             std::cout << "}\n" << std::endl;
        }
    }
    std::cout << "--------------------------------\n" << std::endl;

    ////////////////////////
    if(!stop_list.empty())
    {
        for (iter = stop_list.begin(); iter != stop_list.end(); iter++) {
            std::vector<hardware_interface::InterfaceResources> stop_resrcs = iter->claimed_resources;

            for (int i = 0; i < stop_resrcs.size(); i++) {
                std::vector<int> module_no;
                for (int j = 0; j < joint_info_.size(); j++) {
                    if (stop_resrcs[i].resources.find(joint_info_[j].name) != stop_resrcs[i].resources.end()) {
                        if (joint_info_[j].slave_ptr->is_enabled()) {
                            module_no.push_back(j);
                        }
                    }
                }

                std::vector<int> module_no_tmp = module_no;

                if (!setGroupPosMode(module_no_tmp)) {
                    return false;
                }
            }
        }
    }

    if (start_list.empty()) {
        return true;
    }

    for (iter = start_list.begin(); iter != start_list.end(); iter++) {
        std::vector<hardware_interface::InterfaceResources> start_resrcs = iter->claimed_resources;

        for (int i = 0; i < start_resrcs.size(); i++) {
            std::vector<int> module_no;

            for (int j = 0; j < joint_info_.size(); j++) {
                bool axis1_exist = (start_resrcs[i].resources.find(joint_info_[j].name) != start_resrcs[i].resources.end());

                if (axis1_exist) {
                    if (!joint_info_[j].slave_ptr->is_enabled()) {
                        ROS_ERROR("can't start %s, because joint[%i] is not enabled", iter->name.c_str(), j);
                        return false;
                    }

                    if (isModuleMoving(j)) {
                        ROS_ERROR("can't start %s, because joint[%i] is moving", iter->name.c_str(), j);
                        return false;
                    }

                    module_no.push_back(j);
                }
            }

            if (strcmp(start_resrcs[i].hardware_interface.c_str(), "hardware_interface::PositionJointInterface") == 0) {
                std::vector<int> module_no_tmp = module_no;

                if (!setGroupPosMode(module_no_tmp)) {
                    return false;
                }
            }
            else if (strcmp(start_resrcs[i].hardware_interface.c_str(), "hardware_interface::EffortJointInterface") == 0) {
                std::vector<int> module_no_tmp = module_no;
                if (!setGroupTrqMode(module_no_tmp)) {
                    return false;
                }
            }
            else if (!module_no.empty()) {
                ROS_ERROR("Current hardware interface doesn't support %s", start_resrcs[i].hardware_interface.c_str());
                return false;
            }
        }
    }

    return true;
}

void soem_hw_interface::doSwitch(const std::list<hardware_interface::ControllerInfo> &start_list, const std::list<hardware_interface::ControllerInfo> &stop_list)
{
    for (size_t i = 0; i < pre_switch_flags_.size(); i++)
    {
        boost::mutex::scoped_lock pre_switch_flags_lock(*pre_switch_mutex_ptrs_[i]);
        if (pre_switch_flags_[i])
        {
            joint_info_[i].velocity_cmd = 0;
            joint_info_[i].effort_cmd = 0;

            pre_switch_flags_[i] = false;
        }
        pre_switch_flags_lock.unlock();
    }
}

void soem_hw_interface::read_init()
{
    for (size_t i = 0; i < joint_info_.size(); i++)
    {
        int32_t pos_count = joint_info_[i].slave_ptr->get_position_actual();
        double position_tmp_1 = (pos_count - joint_info_[i].encoder_zero) / joint_info_[i].count_rad_factor;
        if (position_tmp_1 >= M_PI)
        {
            joint_info_[i].encoder_zero += joint_info_[i].count_rad_factor * 2 * M_PI;
        }
        else if (position_tmp_1 < -1 * M_PI)
        {
            joint_info_[i].encoder_zero -= joint_info_[i].count_rad_factor * 2 * M_PI;
        }
        joint_info_[i].position = (-1) * (pos_count - joint_info_[i].encoder_zero) / joint_info_[i].count_rad_factor;
    }
}

void soem_hw_interface::read()
{
    for (size_t i = 0; i < joint_info_.size(); i++)
    {
        int32_t pos_count = joint_info_[i].slave_ptr->get_position_actual();
        int32_t vel_count = joint_info_[i].slave_ptr->get_velocity_actual();
        int16_t trq_count = joint_info_[i].slave_ptr->get_torque_actual();
        int32_t pos_count_diff_ = pos_count - joint_info_[i].encoder_zero;
        int32_t trq_sensor_count = joint_info_[i].slave_ptr->get_torque_sensor();

        joint_info_[i].position = (-1) * pos_count_diff_ / joint_info_[i].count_rad_factor;
        joint_info_[i].velocity = (-1) * vel_count / joint_info_[i].count_rad_factor;

        if (use_torque_sensor_)
        {
            joint_info_[i].effort = (-1) * trq_sensor_count / 1000.0; //mNm to Nm
        }
        else
        {   
            joint_info_[i].effort = (-1) * trq_count * joint_info_[i].count_Nm_factor;
        }
    }

}

void soem_hw_interface::write()
{
    for (size_t i = 0; i < joint_info_.size(); i++)
    {
        if (!joint_info_[i].slave_ptr->in_position_mode())
        {
            joint_info_[i].position_cmd = joint_info_[i].position;
        }

        double position_cmd_count = (-1) * joint_info_[i].position_cmd * joint_info_[i].count_rad_factor + joint_info_[i].encoder_zero;

        joint_info_[i].slave_ptr->set_target_position(int32_t(position_cmd_count));

        bool is_preparing_switch;
        boost::mutex::scoped_lock pre_switch_flags_lock(*pre_switch_mutex_ptrs_[i]);
        is_preparing_switch = pre_switch_flags_[i];
        pre_switch_flags_lock.unlock();

        if (!is_preparing_switch)
        {
            double torque_cmd_count = (-1) * joint_info_[i].effort_cmd / joint_info_[i].count_Nm_factor; //fix this
            joint_info_[i].slave_ptr->set_target_torque(int16_t(torque_cmd_count));
        }
    }
}

}


