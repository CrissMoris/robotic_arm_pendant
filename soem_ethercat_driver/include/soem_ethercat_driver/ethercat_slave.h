#ifndef ETHERCAT_SLAVE_H_H
#define ETHERCAT_SLAVE_H_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_srvs/Trigger.h>
#include <vector>
#include <soem_ethercat_driver/ethercat_master.h>

#include <pthread.h>
#include <time.h>

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

namespace soem_ethercat_driver {

class EtherCATSlave
{
private:

    enum class State402 {
      Unknown, NotReadyToSwitchOn, SwitchOnDisabled, ReadyToSwitchOn,
      SwitchedOn, OperationEnabled, QuickStopActive, FaultReactionActive, Fault,
    };

    struct SwitchCommand {
      static constexpr uint16_t Shutdown = 0x6;
      static constexpr uint16_t SwitchOn = 0x7;
      static constexpr uint16_t DisableVoltage = 0x0;
      static constexpr uint16_t QuickStop = 0x2;
      static constexpr uint16_t DisableOperation = 0x7;
      static constexpr uint16_t EnableOperation = 0xf;
      static constexpr uint16_t FaultReset = 0x80;
    };

    template <typename T>
    struct PdoMember {
      size_t offset;
    };

    static State402 extract_state(uint16_t statusword);

    EtherCATMaster* master_;
    int slave_no_;

    // List of PDO Mappings. Types and byte offsets must match with
    // EtherCATMaster::zeroerr_pdo_config_hook() in elfin_ethercat_master.cpp

    inline static const PdoMember<uint16_t> Statusword {0};
    inline static const PdoMember<int16_t> TorqueActual {2};
    inline static const PdoMember<int32_t> PositionActual {4};
    inline static const PdoMember<int32_t> VelocityActual {8};
    inline static const PdoMember<uint16_t> ErrorCode {12};
    inline static const PdoMember<int32_t> TorqueSensor {14};
    inline static const PdoMember<uint8_t> ModeDisplay {18};

    inline static const PdoMember<int32_t> TargetPosition {0};
    inline static const PdoMember<uint16_t> Controlword {4};
    inline static const PdoMember<int16_t> TargetTorque {6};
    inline static const PdoMember<int16_t> TorqueOffset {8};



public:

    EtherCATSlave(EtherCATMaster* master, int slave_no);
    ~EtherCATSlave() = default;

    template <typename T>
    T read_input(PdoMember<T> member)
    {
      T result;
      master_->read_input(slave_no_, member.offset, &result, sizeof(T));
      return result;
    }

    template <typename T>
    T read_output(PdoMember<T> member)
    {
      T result;
      master_->read_output(slave_no_, member.offset, &result, sizeof(T));
      return result;
    }

    template <typename T>
    void write_output(PdoMember<T> member, T value)
    {
      master_->write_output(slave_no_, member.offset, &value, sizeof(T));
    }

    // Process Image Setters & Getters
    int32_t get_position_actual();
    void set_target_position(int32_t pos_cnt);
    int32_t get_target_position();
    int32_t get_velocity_actual();
    int16_t get_torque_actual();
    int32_t get_torque_sensor();
    void set_target_torque(int16_t trq_cnt);
    void set_torque_offset(int16_t trq_offset);

    bool is_enabled();
    static void* enable(void *threadarg);
    static void* disable(void *threadarg);
    bool has_fault();
    void clear_fault();
    bool in_position_mode();
    bool in_torque_mode();
    void switch_to_position_mode();
    void switch_to_torque_mode();

    uint16_t get_error_code();
    uint32_t get_warning_code();
};

} // namespace soem_ethercat_driver

#endif
