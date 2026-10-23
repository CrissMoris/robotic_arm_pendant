/*
Created on Mon Sep 17 10:02:30 2018

@author: Cong Liu, Burb

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
// author: Cong Liu, Burb

#include <soem_ethercat_driver/ethercat_slave.h>

namespace soem_ethercat_driver
{

EtherCATSlave::State402 EtherCATSlave::extract_state(uint16_t statusword)
{
    switch (statusword & 0x6f) {
        case 0x00: return State402::NotReadyToSwitchOn;
        case 0x40: return State402::SwitchOnDisabled;
        case 0x21: return State402::ReadyToSwitchOn;
        case 0x23: return State402::SwitchedOn;
        case 0x27: return State402::OperationEnabled;
        case 0x07: return State402::QuickStopActive;
        case 0x0f: return State402::FaultReactionActive;
        case 0x08: return State402::Fault;
    }
    return State402::Unknown;
}

EtherCATSlave::EtherCATSlave(EtherCATMaster *master, int slave_no)
: master_(master), slave_no_(slave_no)
{
    write_output(Controlword, SwitchCommand::DisableVoltage);
    switch_to_position_mode(); // CSP Mode
}

int32_t EtherCATSlave::get_position_actual()
{
    return read_input(PositionActual);
}

void EtherCATSlave::set_target_position(int32_t pos_cnt)
{
    write_output(TargetPosition, pos_cnt);
}

int32_t EtherCATSlave::get_target_position()
{
    return read_output(TargetPosition);
}

int32_t EtherCATSlave::get_velocity_actual()
{
    return read_input(VelocityActual);
}

int16_t EtherCATSlave::get_torque_actual()
{
    return read_input(TorqueActual);
}

int32_t EtherCATSlave::get_torque_sensor()
{
    return read_input(TorqueSensor);
}

void EtherCATSlave::set_target_torque(int16_t trq_cnt)
{
    write_output(TargetTorque, trq_cnt);
}

void EtherCATSlave::set_torque_offset(int16_t trq_offset)
{
    write_output(TorqueOffset, trq_offset);
}
bool EtherCATSlave::is_enabled()
{
    return extract_state(read_input(Statusword)) == State402::OperationEnabled;
}

void* EtherCATSlave::enable(void* threadarg)
{
    EtherCATSlave *pthis=(EtherCATSlave *)threadarg;

    if (pthis->has_fault()) {
        pthis->clear_fault();
    }

    if (pthis->has_fault()) {
        ROS_WARN("Slave %i: has fault and clear fault failed", pthis->slave_no_);
        return nullptr;
    }

    if (pthis->is_enabled())    {
        ROS_WARN("Slave %i: already enabled", pthis->slave_no_);
        return nullptr;
    }

    // SwitchedOnDisabled -> ReadyToSwitchOn Transition
    pthis->write_output(TargetPosition, pthis->read_input(PositionActual));
    usleep(100000);
    pthis->write_output(Controlword, SwitchCommand::Shutdown);
    pthis->switch_to_position_mode(); // CSP Mode
    struct timespec before, tick;
    clock_gettime(CLOCK_REALTIME, &before);
    clock_gettime(CLOCK_REALTIME, &tick);
    while(ros::ok())
    {
        if (extract_state(pthis->read_input(Statusword)) == State402::ReadyToSwitchOn)
        {
            break;
        }
        if (tick.tv_sec*1e+9+tick.tv_nsec - before.tv_sec*1e+9 - before.tv_nsec >= 2e+9)
        {
            ROS_WARN("Slave %i: SwitchedOnDisabled -> ReadyToSwitchOn transition failed",
                pthis->slave_no_);
            ROS_WARN("Statusword: 0x%x", pthis->read_input(Statusword));
            return (void *)0;
        }
        usleep(10000);
        clock_gettime(CLOCK_REALTIME, &tick);
    }

    ROS_INFO("Slave %d: Ready to Switch On", pthis->slave_no_);

    // ReadyToSwitchOn -> SwitchedOn Transition
    pthis->write_output(Controlword, SwitchCommand::SwitchOn);
    clock_gettime(CLOCK_REALTIME, &before);
    clock_gettime(CLOCK_REALTIME, &tick);
    while(ros::ok())
    {
        if (extract_state(pthis->read_input(Statusword)) == State402::SwitchedOn) {
            break;
        }
        if (tick.tv_sec*1e+9+tick.tv_nsec - before.tv_sec*1e+9 - before.tv_nsec >= 2e+9) {
            ROS_WARN("Slave %i: ReadyToSwitchOn -> SwitchedOn transition failed",
                pthis->slave_no_);
            ROS_WARN("Statusword: 0x%x", pthis->read_input(Statusword));
            return (void *)0;
        }
        usleep(10000);
        clock_gettime(CLOCK_REALTIME, &tick);
    }
    ROS_INFO("Slave %d: Switched On", pthis->slave_no_);

    // SwitchedOn -> OperationEnabled Transition
    pthis->write_output(Controlword, SwitchCommand::EnableOperation);
    clock_gettime(CLOCK_REALTIME, &before);
    clock_gettime(CLOCK_REALTIME, &tick);
    while(ros::ok())
    {
        if (extract_state(pthis->read_input(Statusword)) == State402::OperationEnabled) {
            break;
        }
        if (tick.tv_sec*1e+9+tick.tv_nsec - before.tv_sec*1e+9 - before.tv_nsec >= 2e+9) {
            ROS_WARN("Slave %i: SwitchedOn -> OperationEnabled transition failed",
                pthis->slave_no_);
            ROS_WARN("Statusword: 0x%x", pthis->read_input(Statusword));
            auto error_code = pthis->read_input(ErrorCode);
            ROS_ERROR("Error code from 0x603f: 0x%x", error_code);
            return nullptr;
        }
        usleep(10000);
        clock_gettime(CLOCK_REALTIME, &tick);
    }
    ROS_INFO("Slave %d: Operation Enabled", pthis->slave_no_);

    pthis->write_output(Controlword, uint16_t(0x1f));
    usleep(100000);
    return nullptr;
}

void* EtherCATSlave::disable(void *threadarg)
{
    EtherCATSlave *pthis = (EtherCATSlave *)threadarg;
    if (pthis->is_enabled()) {
        pthis->write_output(Controlword, SwitchCommand::Shutdown);
    }
    return nullptr;
}

bool EtherCATSlave::has_fault()
{
    return extract_state(read_input(Statusword) & 0x08) == State402::Fault;
}

void EtherCATSlave::clear_fault()
{
    write_output(Controlword, uint16_t(SwitchCommand::Shutdown + SwitchCommand::FaultReset));
    usleep(20000);
    write_output(Controlword, SwitchCommand::Shutdown);
    usleep(20000);
}

bool EtherCATSlave::in_position_mode()
{
    return is_enabled() && (read_input(ModeDisplay) == 0x8);
}

bool EtherCATSlave::in_torque_mode()
{
    return is_enabled() && (read_input(ModeDisplay)) == 0xa;
}

void EtherCATSlave::switch_to_position_mode()
{
    master_->writeSDO<int8_t>(slave_no_, 0x6060, 0x00, int8_t(0x8));
}

void EtherCATSlave::switch_to_torque_mode()
{
     master_->writeSDO<int8_t>(slave_no_, 0x6060, 0x00, int8_t(0xa));
}

uint16_t EtherCATSlave::get_error_code()
{
    return read_input(ErrorCode);
}

uint32_t EtherCATSlave::get_warning_code()
{
    return master_->readSDO<uint32_t>(slave_no_, 0x3b68, 0x00);
}
}

