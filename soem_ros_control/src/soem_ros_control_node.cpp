#include "soem_ros_control/soem_hardware_interface.h"

struct ArgsForThread {
    controller_manager::ControllerManager *manager;
    soem_ros_control::soem_hw_interface *hw_interface;
};

static void timespecInc(struct timespec &tick, int nsec)
{
    constexpr int SEC_2_NSEC = 1e+9;
    tick.tv_nsec += nsec;
    while (tick.tv_nsec >= SEC_2_NSEC) {
        tick.tv_nsec -= SEC_2_NSEC;
        ++tick.tv_sec;
    }
}

void* update_loop(void* threadarg)
{
    ArgsForThread *arg = (ArgsForThread *)threadarg;
    controller_manager::ControllerManager *manager = arg->manager;
    soem_ros_control::soem_hw_interface *interface = arg->hw_interface;
    ros::Duration d(0.001);
    struct timespec tick;
    clock_gettime(CLOCK_REALTIME, &tick);

    // time for checking overrun
    struct timespec before;
    double overrun_time;
    while (ros::ok()) {
        ros::Time this_moment(tick.tv_sec, tick.tv_nsec);
        interface->read();
        manager->update(this_moment, d);
        interface->write();
        timespecInc(tick, d.nsec);

        // check overrun
        clock_gettime(CLOCK_REALTIME, &before);
        overrun_time = (before.tv_sec + double(before.tv_nsec) / 1e+9)
            - (tick.tv_sec + double(tick.tv_nsec) / 1e+9);
        if (overrun_time > 0.0) {
            tick.tv_sec = before.tv_sec;
            tick.tv_nsec = before.tv_nsec;
        }

        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &tick, NULL);
    }
    return nullptr;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "soem_ros_control", ros::init_options::AnonymousName);

    ros::NodeHandle nh("~");
    
    std::string ethernet_name;

    if (!nh.getParam("ifname", ethernet_name) || ethernet_name.size() == 0) {
        ROS_ERROR("ifname parameter is not set or is empty");
        exit(0);
    }

    // soem_ethercat_driver::EtherCatDriver ed(ethernet_name, "ethercat");

    soem_ros_control::soem_hw_interface hw(ethernet_name, nh);
    hw.read_init();

    controller_manager::ControllerManager cm(&hw);
    
    ArgsForThread *thread_arg = new ArgsForThread();
    thread_arg->manager = &cm;
    thread_arg->hw_interface = &hw;
    pthread_t tid;
    pthread_create(&tid, NULL, update_loop, thread_arg);

    ros::Rate r(50);
    while (ros::ok()) {
        ros::spinOnce();
        r.sleep();
    }
}
