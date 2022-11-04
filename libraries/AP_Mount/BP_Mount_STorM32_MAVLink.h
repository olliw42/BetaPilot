//*****************************************************
//OW
// (c) olliw, www.olliw.eu, GPL3
// STorM32 mount backend class
// uses 100% MAVlink + storm32.xml
//*****************************************************

#pragma once

#include "AP_Mount.h"
#include "AP_Mount_Backend.h"


#define USE_FIND_GIMBAL_MAX_SEARCH_TIME_MS  0 // set to 0 to disable
#define FIND_GIMBAL_MAX_SEARCH_TIME_MS  300000 // AP's startup has become quite slow, so give it plenty of time

#define USE_GIMBAL_ZFLAGS  1

#define GIMBAL_AUTOMODE_TIMEOUT_CNT  10


// that's the main class
class BP_Mount_STorM32_MAVLink : public AP_Mount_Backend
{
public:
    // Constructor
    BP_Mount_STorM32_MAVLink(AP_Mount &frontend, AP_Mount_Params &params, uint8_t instance);

    // init - performs any required initialisation for this instance
    void init() override;

    // update mount position - should be called periodically
    void update() override;

    // used for gimbals that need to read INS data at full rate
    void update_fast() override;

    // return true if healthy
    bool healthy() const override { return true; }

    // returns true if this mount can control its pan (required for multicopters)
    bool has_pan_control() const override { return false; }

    // handle a GLOBAL_POSITION_INT message
    bool handle_global_position_int(uint8_t msg_sysid, const mavlink_global_position_int_t &packet) override;

    // handle GIMBAL_DEVICE_INFORMATION message
    void handle_gimbal_device_information(const mavlink_message_t &msg) override {}

    // handle GIMBAL_DEVICE_ATTITUDE_STATUS message
    void handle_gimbal_device_attitude_status(const mavlink_message_t &msg) override {}

protected:

    // get attitude as a quaternion.  returns true on success
    bool get_attitude_quaternion(Quaternion& att_quat) override { return false; }

public:

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

    MountTarget _angle_rad;

    void set_target_angles(void);

    void send_mount_status_to_ground(void);
    void send_cmd_do_mount_control_to_gimbal(float roll_deg, float pitch_deg, float yaw_deg, enum MAV_MOUNT_MODE mode);

    // STorM32 gimbal protocol
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
    uint8_t landed_state(void);
    void send_to_ground(uint32_t msgid, const char *pkt);

    // internal task variables
    uint16_t _loop_rate_hz;
    uint8_t _decimate_counter_max;
    uint8_t _decimate_counter;

    enum TASKENUM {
        TASK_SLOT0 = 0,
        TASK_SLOT1,
        TASK_SLOT2,
        TASK_SLOT3,
    };
    uint32_t _task_time_last;
    uint16_t _task_counter;

    // automatic gimbal operation mode detection
    enum AUTOMODEENUM {
        AUTOMODE_UNDEFINED = 0,     // we do not yet know
        // AUTOMODE_V1,                // gimbal uses plain old V1 gimbal protocol messages // deprecated
        AUTOMODE_GIMBALDEVICE_V2,   // gimbal is a gimbal protocol v2 gimbal device // TODO
        AUTOMODE_GIMBALDEVICE,      // gimbal is a STorM32 gimbal device
        AUTOMODE_GIMBALMANAGER,     // gimbal is a STorM32 gimbal manager
    };
    uint8_t _auto_mode;
    uint8_t _auto_mode_cntdown;

    // logging
    bool _should_log;
};
