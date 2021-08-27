//*****************************************************
//OW
// (c) olliw, www.olliw.eu, GPL3
// STorM32 mount backend class
// uses 100% MAVlink + storm32.xml
//*****************************************************

#pragma once

#include "AP_Mount.h"
#include "AP_Mount_Backend.h"


#define USE_FIND_GIMBAL_MAX_SEARCH_TIME_MS  0 //set to 0 to disable
#define FIND_GIMBAL_MAX_SEARCH_TIME_MS  300000 //90000 //AP's startup has become quite slow, so give it plenty of time


// that's the main class
class BP_Mount_STorM32_MAVLink : public AP_Mount_Backend
{
public:
    // Constructor
    BP_Mount_STorM32_MAVLink(AP_Mount &frontend, AP_Mount::mount_state &state, uint8_t instance);

    // init - performs any required initialisation for this instance
    void init() override;

    // update mount position - should be called periodically
    void update() override;
    void update_fast() override;

    // has_pan_control - returns true if this mount can control it's pan (required for multicopters)
    bool has_pan_control() const override { return false; }

    // set_mode - sets mount's mode
    void set_mode(enum MAV_MOUNT_MODE mode) override {};

    // send_mount_status - called to allow mounts to send their status to GCS using the MOUNT_STATUS message
    void send_mount_status(mavlink_channel_t chan) override {};

private:
    // internal variables
    bool _initialised;              // true once the driver has been fully initialised

    // internal MAVLink variables
    uint8_t _sysid;                 // system id of gimbal
    uint8_t _compid;                // component id of gimbal
    mavlink_channel_t _chan;        // mavlink channel used to communicate with gimbal

    void send_autopilot_state_for_gimbal_device_to_gimbal(void);

    // internal task variables
    enum TASKENUM {
        TASK_SLOT0 = 0,
        TASK_SLOT1,
        TASK_SLOT2,
        TASK_SLOT3,
    };
    uint32_t _task_time_last;
    uint16_t _task_counter;

};
