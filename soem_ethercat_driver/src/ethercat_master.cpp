/*
 * Copyright (C) 2015, Jonathan Meyer
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Tokyo Opensource Robotics Kyokai Association. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// copied from https://github.com/ros-industrial/robotiq/blob/jade-devel/robotiq_ethercat/src/ethercat_master.cpp


#include "soem_ethercat_driver/ethercat_master.h"

#include <unistd.h>
#include <stdio.h>
#include <time.h>
#include <iostream>

#include <boost/ref.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>

namespace
{

static constexpr unsigned THREAD_SLEEP_TIME = 1000; // 1 ms
static constexpr unsigned EC_TIMEOUTMON = 500;
static constexpr int NSEC_PER_SECOND = 1e+9;

void timespecInc(struct timespec &tick, int nsec)
{
    tick.tv_nsec += nsec;
    while (tick.tv_nsec >= NSEC_PER_SECOND) {
        tick.tv_nsec -= NSEC_PER_SECOND;
        tick.tv_sec++;
    }
}

void handleErrors()
{
    // one ore more slaves are not responding
    ec_group[0].docheckstate = FALSE;
    ec_readstate();

    for (int slave = 1; slave <= ec_slavecount; slave++) {
        if ((ec_slave[slave].group == 0) && (ec_slave[slave].state != EC_STATE_OPERATIONAL)) {
            ec_group[0].docheckstate = TRUE;
            if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR)) {
                fprintf(stderr, "ERROR : slave %d is in SAFE_OP + ERROR, attempting ack.\n", slave);
                ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
                ec_writestate(slave);
            }
            else if(ec_slave[slave].state == EC_STATE_SAFE_OP) {
                fprintf(stderr, "WARNING : slave %d is in SAFE_OP, change to OPERATIONAL.\n", slave);
                ec_slave[slave].state = EC_STATE_OPERATIONAL;
                ec_writestate(slave);
            }
            else if(ec_slave[slave].state > 0) {
                if (ec_reconfig_slave(slave, EC_TIMEOUTMON)) {
                    ec_slave[slave].islost = FALSE;
                    printf("MESSAGE : slave %d reconfigured\n",slave);
                }
            }
            else if(!ec_slave[slave].islost) {
                // re-check state
                ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
                if (!ec_slave[slave].state) {
                    ec_slave[slave].islost = TRUE;
                    fprintf(stderr, "ERROR : slave %d lost\n",slave);
                }
            }
        }

        if (ec_slave[slave].islost) {
            if(!ec_slave[slave].state) {
                if (ec_recover_slave(slave, EC_TIMEOUTMON)) {
                    ec_slave[slave].islost = FALSE;
                    printf("MESSAGE : slave %d recovered\n",slave);
                }
            }
            else {
                ec_slave[slave].islost = FALSE;
                printf("MESSAGE : slave %d found\n",slave);
            }
        }
    }
}

void cycleWorker(boost::mutex& mutex, bool& stop_flag)
{
    // 1ms in nanoseconds
    double period = THREAD_SLEEP_TIME * 1000;
    // get current time
    struct timespec tick;
    clock_gettime(CLOCK_REALTIME, &tick);
    timespecInc(tick, period);
    // time for checking overrun
    struct timespec before;
    double overrun_time;

    while (!stop_flag) {
        int expected_wkc = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
        int sent, wkc;

        {
            boost::mutex::scoped_lock lock(mutex);
            sent = ec_send_processdata();
            wkc = ec_receive_processdata(EC_TIMEOUTRET);
        }

        if (wkc < expected_wkc) {
            handleErrors();
        }

        // check overrun
        clock_gettime(CLOCK_REALTIME, &before);
        overrun_time = (before.tv_sec + double(before.tv_nsec) / NSEC_PER_SECOND)
            - (tick.tv_sec + double(tick.tv_nsec)/NSEC_PER_SECOND);
        if (overrun_time > 0.0) {
            tick.tv_sec=before.tv_sec;
            tick.tv_nsec=before.tv_nsec;
        }
        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &tick, NULL);
        timespecInc(tick, period);
    }
}

} // end of anonymous namespace


namespace soem_ethercat_driver {

int EtherCATMaster::zeroerr_pdo_config_hook(uint16_t slave_no)
{

    uint8_t temp = 0;

    uint32_t rpdo_mapping[4] = {
        0x607a'00'20, // target position
        0x6040'00'10, // controlword
        0x6071'00'10, // target torque
        0x60b2'00'10, // torque offset
    };
    temp = 0;
    ec_SDOwrite(slave_no, 0x1600, 0x00, FALSE, sizeof(temp), &temp, EC_TIMEOUTSAFE);
    ec_SDOwrite(slave_no, 0x1600, 0x01, TRUE, sizeof(rpdo_mapping), rpdo_mapping, EC_TIMEOUTSAFE);
    temp = 4;
    ec_SDOwrite(slave_no, 0x1600, 0x00, FALSE, sizeof(temp), &temp, EC_TIMEOUTSAFE);

    uint32_t tpdo_mapping[8] = {
        0x6041'00'10, // statusword
        0x6077'00'10, // actual torque
        0x6064'00'20, // actual position
        0x606c'00'20, // actual velocity
        0x603f'00'10, // error code
        0x3b69'00'20, // torque sensor [mNm]
        0x6061'00'08, // modes of operation display
        0x0000'00'08, // dummy
    };
    temp = 0;
    ec_SDOwrite(slave_no, 0x1a00, 0x00, FALSE, sizeof(temp), &temp, EC_TIMEOUTSAFE);
    ec_SDOwrite(slave_no, 0x1a00, 0x01, TRUE, sizeof(tpdo_mapping), tpdo_mapping, EC_TIMEOUTSAFE);
    temp = 8;
    ec_SDOwrite(slave_no, 0x1a00, 0x00, FALSE, sizeof(temp), &temp, EC_TIMEOUTSAFE);

    return 0;
}

EtherCATMaster::EtherCATMaster(const std::string& ifname)
: ifname_(ifname), stop_flag_(false)
{
    std::memset(iomap_, 0, sizeof(iomap_));

    if (initSoem(ifname_)) {
        cycle_thread_ = boost::thread(cycleWorker, boost::ref(iomap_mutex_), boost::ref(stop_flag_));
    }
    else {
    // construction failed
        throw EtherCatError("Could not initialize SOEM");
    }
}

EtherCATMaster::~EtherCATMaster()
{
    stop_flag_ = true;

    // Request init operational state for all slaves
    ec_slave[0].state = EC_STATE_INIT;

    /* request init state for all slaves */
    ec_writestate(0);

    //stop SOEM, close socket
    ec_close();
    cycle_thread_.join();
}

bool EtherCATMaster::initSoem(const std::string& ifname)
{
    // Copy string contents because SOEM library doesn't
    // practice const correctness
    const static unsigned MAX_BUFF_SIZE = 1024;
    char buffer[MAX_BUFF_SIZE];
    size_t name_size = ifname.size();
    if (name_size > sizeof(buffer) - 1) {
        fprintf(stderr, "Ifname %s exceeds maximum size of %u bytes\n", ifname.c_str(), MAX_BUFF_SIZE);
        return false;
    }
    std::strncpy(buffer, ifname.c_str(), MAX_BUFF_SIZE);

    printf("Initializing etherCAT master\n");

    if (!ec_init(buffer)) {
        fprintf(stderr, "Could not initialize ethercat driver on %s\n", ifname.c_str());
        return false;
    }

    /* find and auto-config slaves */
    if (ec_config_init(FALSE) <= 0) {
        fprintf(stderr, "No slaves are found on %s\n", ifname.c_str());
        return false;
    }

    printf("SOEM found and configured %d slaves\n", ec_slavecount);

    if (ec_statecheck(0, EC_STATE_PRE_OP, EC_TIMEOUTSTATE) != EC_STATE_PRE_OP) {
        fprintf(stderr, "Could not set EC_STATE_PRE_OP\n");
        return false;
    }

    for (int i = 0; i <= ec_slavecount; ++i) {
        ec_slave[i].PO2SOconfig = zeroerr_pdo_config_hook;
    }

    // configure IOMap
    int iomap_size = ec_config_map(iomap_);
    printf("SOEM IOMap size: %d\n", iomap_size);

    // Switching to SAFEOP state...

    ec_slave[0].state = EC_STATE_SAFE_OP;
    ec_writestate(0);
    if (ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE) != EC_STATE_SAFE_OP) {
        fprintf(stderr, "Could not set EC_STATE_SAFE_OP\n");
        return false;
    }

    // configure DC options for every DC capable slave found in the list
    ec_configdc();
    for (int slave = 1; slave <= ec_slavecount; ++slave) {
        // ec_dcsync01(slave, TRUE, 125 * 1000, 0, 0);
        ec_dcsync0(slave, TRUE, 125 * 1000, 0);
    }
    ec_readstate();

    // Swithing to OPERATIONAL state...

    ec_slave[0].state = EC_STATE_OPERATIONAL;
    ec_send_processdata();
    ec_receive_processdata(EC_TIMEOUTRET);

    ec_writestate(0);
    int chk = 4000;
    do {
        ec_send_processdata();
        ec_receive_processdata(EC_TIMEOUTRET);
        ec_statecheck(0, EC_STATE_OPERATIONAL, 750); // 1 ms wait for state check
    } while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));

    if(ec_statecheck(0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE) != EC_STATE_OPERATIONAL) {
        fprintf(stderr, "OPERATIONAL state not set, exiting\n");
        ec_readstate();
        for (int i = 1; i <= ec_slavecount; ++i) {
            fprintf(stderr, "Slave %d AL Status Code: 0x%x -> %s\n", i,
                ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
        }
        return false;
    }

    ec_readstate();
    printf("\nFinished configuration successfully\n");
    return true;
}

void EtherCATMaster::write_output(int slave_no, size_t offset, const void* source, size_t size)
{
    assert(slave_no <= ec_slavecount);
    boost::mutex::scoped_lock lock(iomap_mutex_);
    std::memcpy(&ec_slave[slave_no].outputs[offset], source, size);
}

void EtherCATMaster::read_input(int slave_no, size_t offset, void* target, size_t size)
{
    assert(slave_no <= ec_slavecount);
    boost::mutex::scoped_lock lock(iomap_mutex_);
    std::memcpy(target, &ec_slave[slave_no].inputs[offset], size);
}

void EtherCATMaster::read_output(int slave_no, size_t offset, void* target, size_t size)
{
    assert(slave_no <= ec_slavecount);
    boost::mutex::scoped_lock lock(iomap_mutex_);
    std::memcpy(target, &ec_slave[slave_no].outputs[offset], size);
}
}

