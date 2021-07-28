//*****************************************************
//OW
// (c) olliw, www.olliw.eu, GPL3
// STorM32 mount backend class
// uses 100% MAVlink + storm32.xml
//*****************************************************

#pragma once

#include "AP_Mount.h"
#include "AP_Mount_Backend.h"
#include <GCS_MAVLink/include/mavlink/v2.0/checksum.h>
#include <AP_Mount/STorM32_lib.h>


#define USE_FIND_GIMBAL_MAX_SEARCH_TIME_MS  0 //set to 0 to disable
#define FIND_GIMBAL_MAX_SEARCH_TIME_MS  300000 //90000 //AP's startup has become quite slow, so give it plenty of time

#define USE_GIMBAL_ZFLAGS  1


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
    void set_mode(enum MAV_MOUNT_MODE mode) override;

    // send_mount_status - called to allow mounts to send their status to GCS using the MOUNT_STATUS message
    void send_mount_status(mavlink_channel_t chan) override;

    // handle_msg - allows to process messages received from gimbal
    void handle_msg(const mavlink_message_t &msg) override;

    // pre arm checks
    bool pre_arm_checks(void) override;

    // send banner
    void send_banner(void) override;

private:
    // internal variables
    bool _initialised;              // true once the driver has been fully initialised
    bool _armed;                    // true once the gimbal has reached normal operation state
    bool _prearmchecks_ok;          // true when the gimbal stops reporting prearm fail

    // internal MAVLink variables
    uint8_t _sysid;                 // system id of gimbal
    uint8_t _compid;                // component id of gimbal
    mavlink_channel_t _chan;        // mavlink channel used to communicate with gimbal

    void determine_auto_mode(const mavlink_message_t &msg);
    void find_gimbal(void);
    void find_gimbal_oneonly(void);

    // rc channels
    bool is_rc_failsafe(void);
    void send_rc_channels_to_gimbal(void);

    // storm32 mount_status in, mount_status out
    struct {
        float roll_deg;
        float pitch_deg;
        float yaw_deg;
        float yaw_deg_absolute;
    } _status;

    struct {
        float roll_deg;
        float pitch_deg;
        float yaw_deg;
        enum MAV_MOUNT_MODE mode;
        enum MAV_MOUNT_MODE mode_last;
    } _target;

    void set_target_angles(void);
    void send_target_angles_to_gimbal(void);

    void send_mount_status_to_ground(void);
    void send_cmd_do_mount_control_to_gimbal(float roll_deg, float pitch_deg, float yaw_deg, enum MAV_MOUNT_MODE mode);

    // storm32 gimbal protocol
    bool _use_protocolv2;
    bool _use_gimbalmanager;
    bool _sendonly;
    bool _is_active;                // true when autopilot client is active
    struct {
        uint8_t mode;
        uint8_t mode_last;
    } _qshot;

    void set_target_angles_qshot(void);
    void send_target_angles_to_gimbal_v2(void);

    void send_autopilot_state_for_gimbal_device_to_gimbal(void);
    void send_storm32_gimbal_device_control_to_gimbal(float roll_deg, float pitch_deg, float yaw_deg, uint16_t flags);
    void send_storm32_gimbal_manager_control_to_gimbal(float roll_deg, float pitch_deg, float yaw_deg, uint16_t device_flags, uint16_t manager_flags);

    // system time
    uint32_t _send_system_time_last;
    void send_system_time_to_gimbal(void);

    // helper
    void send_to_ground(uint32_t msgid, const char *pkt);

    // internal task variables
    enum TASKENUM {
        TASK_SLOT0 = 0,
        TASK_SLOT1,
        TASK_SLOT2,
        TASK_SLOT3,
        TASK_SLOT4,
    };
    uint32_t _task_time_last;
    uint16_t _task_counter;

    // automatic operation mode detection
    #define AUTOMODE_CNT   10
    enum AUTOMODEENUM {
        AUTOMODE_UNDEFINED = 0,
        AUTOMODE_V1,
        AUTOMODE_GIMBALDEVICE,
        AUTOMODE_GIMBALMANAGER,
    };
    uint8_t _auto_mode;
    uint8_t _auto_mode_cntdown;

    // interface to STorM32_lib
    enum MAV_TUNNEL_PAYLOAD_TYPE_STORM32_ENUM {
        MAV_TUNNEL_PAYLOAD_TYPE_STORM32_CHANNEL1_IN = 200, //MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED0
        MAV_TUNNEL_PAYLOAD_TYPE_STORM32_CHANNEL1_OUT, //MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED1
        MAV_TUNNEL_PAYLOAD_TYPE_STORM32_CHANNEL2_IN, //MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED2
        MAV_TUNNEL_PAYLOAD_TYPE_STORM32_CHANNEL2_OUT, //MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED3
    };

    // STorM32_MAVLink_class
    uint8_t _storm32link_seq;
    void send_cmd_storm32link_v2(void);
    bool is_normal_state(uint16_t state);
    void send_cmd_sethomelocation(void);
    void send_cmd_settargetlocation(void);
    bool _tx_hasspace(const size_t size);
    size_t _write(const uint8_t* buffer, size_t size);
};
