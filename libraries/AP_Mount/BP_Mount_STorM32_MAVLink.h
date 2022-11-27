//*****************************************************
//OW
// (c) olliw, www.olliw.eu, GPL3
// STorM32 mount backend class
// 100% MAVLink + storm32.xml
//*****************************************************
#pragma once

#include "AP_Mount.h"
#include "AP_Mount_Backend.h"


// STorM32 states
enum STORM32STATEENUM {
    STORM32STATE_STARTUP_MOTORS = 0,
    STORM32STATE_STARTUP_SETTLE,
    STORM32STATE_STARTUP_CALIBRATE,
    STORM32STATE_STARTUP_LEVEL,
    STORM32STATE_STARTUP_MOTORDIRDETECT,
    STORM32STATE_STARTUP_RELEVEL,
    STORM32STATE_NORMAL,
    STORM32STATE_STARTUP_FASTLEVEL,
};


class BP_Mount_STorM32_MAVLink : public AP_Mount_Backend
{

public:
    // Constructor
    BP_Mount_STorM32_MAVLink(AP_Mount &frontend, AP_Mount_Params &params, uint8_t instance);

    // init - performs any required initialisation for this instance
    void init() override {}

    // update mount position - should be called periodically
    void update() override;

    // used for gimbals that need to read INS data at full rate
    void update_fast() override {}

    // return true if healthy
    // is called by mount pre_arm_checks() (and nowhere else)
    // which in turn is called in AP_Arming::pre_arm_checks() (and nowhere else)
    // mount prearm checks are tied to camera, and enabled by setting
    // thus Gremsy's method for e.g. checking attitude_status doesn't make much sense
    bool healthy() const override;

    // returns true if this mount can control its pan (required for multicopters)
    // STorM32: work it out later
    // false: copter is rotated in yaw, not the gimbal
    // this is evaluated in
    // - GCS_MAVLINK_Copter::handle_command_mount(), cmd = MAV_CMD_DO_MOUNT_CONTROL
    // - GCS_MAVLINK_Copter::handle_mount_message(), msg = MAVLINK_MSG_ID_MOUNT_CONTROL
    // - Copter Mode::AutoYaw::set_roi()
    // - Copter ModeAuto::do_mount_control()
    bool has_pan_control() const override { return false; }

    // set yaw_lock
    // STorM32: this needs to not set _yaw_lock, as it doesn't (yet) support earth frame
    // it does support yaw lock in vehicle frame
    void set_yaw_lock(bool yaw_lock) override {};

    // handle GIMBAL_DEVICE_INFORMATION message
    void handle_gimbal_device_information(const mavlink_message_t &msg) override;

    // handle GIMBAL_DEVICE_ATTITUDE_STATUS message
    void handle_gimbal_device_attitude_status(const mavlink_message_t &msg) override;

    // send a GIMBAL_DEVICE_ATTITUDE_STATUS message to GCS
    // STorM32: this needs to be empty, because the gimbal device is sending it itself
    // we don't do any of ArduPilot's private mavlink channel nonsense
    void send_gimbal_device_attitude_status(mavlink_channel_t chan) override {}

protected:

    // get attitude as a quaternion.  returns true on success
    // this is used in send_gimbal_device_attitude_status()
    // STorM32: empty it, send_gimbal_device_attitude_status() has been emptied also, so no use
    bool get_attitude_quaternion(Quaternion& att_quat) override { return false; }

private:

    // search for gimbal in GCS_MAVLink routing table
    void find_gimbal(void);

    // request GIMBAL_DEVICE_INFORMATION from gimbal (holds vendor and model name, min/max angles)
    void send_request_gimbal_device_information(void);

    // send GIMBAL_DEVICE_SET_ATTITUDE to gimbal to command gimbal to retract (aka relax)
    void send_gimbal_device_retract(void);

    // send GIMBAL_DEVICE_SET_ATTITUDE to gimbal to control attitude
    // earth_frame should be true if yaw_rad target is an earth frame angle, false if body_frame
    void send_gimbal_device_set_attitude(float roll_rad, float pitch_rad, float yaw_rad, bool earth_frame);

// -- this so far is the interface provided by AP_Mount_Backend and as seen in the Gremsy driver
// -- here now come our methods
public:

    // handle msg - allows to process a msg from a gimbal
    void handle_msg(const mavlink_message_t &msg) override;

    // send banner
    void send_banner(void) override;

private:

    // internal variables

    bool _got_device_info;          // true once gimbal has provided device info
    bool _initialised;              // true once startup procedure has been fully completed
    bool _armed;                    // true once the gimbal has reached normal state
    bool _prearmchecks_ok;          // true when the gimbal stops reporting prearm fail

    // internal MAVLink variables

    uint8_t _sysid;                 // system id of gimbal
    uint8_t _compid;                // component id of gimbal, 0 indicates gimbal not yet found
    mavlink_channel_t _chan;        // mavlink channel used to communicate with gimbal

    // initialization etc

    uint32_t _request_device_info_tlast_ms;
    mavlink_gimbal_device_information_t _device_info;

    // flags

    bool _sendonly;
    bool _should_log;

    // internal task variables

    enum TASKENUM {
        TASK_SLOT0 = 0,
        TASK_SLOT1,
        TASK_SLOT2,
        TASK_SLOT3,
    };

    uint16_t _task_counter;

    // blabla

    bool _prearmcheck_last; // to detect changes
    bool prearmchecks_do(void); // workaround needed since healthy() is const

    uint16_t _device_flags_for_gimbal;
    void update_gimbal_device_flags_for_gimbal(void);

    uint16_t _received_device_flags;
    uint32_t _received_device_failure_flags;
    uint32_t _received_attitude_status_tlast_ms; // time last attitude status was received (used for health reporting)

    void set_and_send_target_angles(void);
    void send_autopilot_state_for_gimbal_device(void);

    uint32_t _send_system_time_tlast_ms;
    void send_system_time(void);

    void send_rc_channels(void);

    uint32_t _send_gimbal_manager_status_tlast_ms;
    void send_gimbal_manager_status_to_all(void);

    // mount_status forwarding

    struct tMountStatus {
        float roll_deg;
        float pitch_deg;
        float yaw_deg;
    };

    void send_mount_status_to_ground(tMountStatus &status);
    void send_to_ground(uint32_t msgid, const char *pkt);
};

