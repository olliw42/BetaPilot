//*****************************************************
//OW
// (c) olliw, www.olliw.eu, GPL3
// STorM32 mount backend class
// pure v2 gimbal protocol
//*****************************************************
#pragma once

#include "AP_Mount_Backend.h"

#if HAL_MOUNT_STORM32_MAVLINK_V2_ENABLED


class AP_Mount_STorM32_MAVLink : public AP_Mount_Backend
{

public:
    // Constructor
    using AP_Mount_Backend::AP_Mount_Backend;

    // init - performs any required initialisation for this instance
    void init() override;

    // update mount position - should be called periodically at 50 Hz
    void update() override;

    // used for gimbals that need to read INS data at full rate
    //not used void update_fast() override {}

    // return true if healthy
    // Is called by mount pre_arm_checks() (and nowhere else), which in turn is called
    // in AP_Arming::pre_arm_checks() (and nowhere else).
    // The mount prearm checks are tied to camera, and enabled by setting ARMING_CHECK_CAMERA.
    bool healthy() const override;

    // return true if this mount accepts roll or pitch targets
    // affects the gimbal manager capability flags send out by AP
    // only used in get_gimbal_manager_capability_flags(), which is only called in
    // send_gimbal_manager_information(), which we overwrite anyhow
    // => irrelevant for now, but we set it to something useful nevertheless
    bool has_roll_control() const override;
    bool has_pitch_control() const override;

    // returns true if this mount can control its pan (required for multicopters)
    // false: copter is rotated in yaw, not the gimbal
    // work it out later
    // This is evaluated in
    // - GCS_MAVLINK_Copter::handle_command_mount(), cmd = MAV_CMD_DO_MOUNT_CONTROL
    // - GCS_MAVLINK_Copter::handle_mount_message(), msg = MAVLINK_MSG_ID_MOUNT_CONTROL
    // - Copter Mode::AutoYaw::set_roi()
    // - Copter ModeAuto::do_mount_control()
    // for now do something simple
    bool has_pan_control() const override;

    ///>> Deal with some AP strangeness:

    // get attitude as a quaternion.  Returns true on success
    // att_quat will be an earth-frame quaternion rotated such that yaw is in body-frame.
    // This is used in
    // - AP_Mount_Backend::send_gimbal_device_attitude_status()
    // - AP_Mount::get_attitude_euler()
    // - AP_Mount_Backend::write_log() via calling AP_Mount::get_attitude_euler()
    // - scripts via calling AP_Mount::get_attitude_euler()
    // AP nonsense: uses inappropriate Euler angles.
    // => set roll to zero, to minimize harm
    bool get_attitude_quaternion(Quaternion &att_quat) override;

    // get angular velocity of mount. Only available on some backends
    // This is used in
    // - AP_Mount_Backend::send_gimbal_device_attitude_status()
    // => not needed, but is supplied since the data may be available
    bool get_angular_velocity(Vector3f& rates) override;

    // set yaw_lock.  If true, the gimbal's yaw target is maintained in earth-frame meaning
    // it will lock onto an earth-frame heading (e.g. North). If false (aka "follow") the gimbal's yaw
    // is maintained in body-frame meaning the gimbal will rotate with the vehicle.
    // This must not set _yaw_lock, since
    // - AP nonsense: mixes up yaw lock and vehicle or earth frame, which are not fundamentally related to each other
    // - STorM32 supports yaw lock in vehicle frame, but doesn't (yet) support earth frame
    // => we can't use yaw lock without massive changes, and need to overwrite it
    // In the backend, _yaw_lock is used as yaw_is_ef. However, gladly, backend never sets
    // it by itself, so setting it to false in this driver will ensure it's false.
    // This is called for AUX_FUNC::MOUNT_LOCK switch setting.
    void set_yaw_lock(bool yaw_lock) override {}

    // send a GIMBAL_DEVICE_ATTITUDE_STATUS message to GCS
    // Is part of stream Extra3. This is nonsense. Streaming is appropriate only if the gimbal is
    // not a gimbal device, but then it should be streamed to all parties except the gimbal, which
    // seems not to be what is done.
    // MissionPlaner "understands" gimbal device attitude status, but doesn't use it for campoint, so we
    // still want to send MOUNT_STATUS.
    // => for as long as MissionPlanner doesn't do mount/gimbal status properly
    //    we use this to stream MOUNT_STATUS
    // We don't do any of ArduPilot's private mavlink channel nonsense.
    void send_gimbal_device_attitude_status(mavlink_channel_t chan) override;

    // send a GIMBAL_MANAGER_INFORMATION message to GCS
    // AP nonsense: does it wrong, just good for its own limits.
    // => we need to overwrite it
    void send_gimbal_manager_information(mavlink_channel_t chan) override;

    // added: return gimbal device id
    uint8_t get_gimbal_device_id() const override;

    // added: return gimbal manager flags used by GIMBAL_MANAGER_STATUS message
    // AP nonsense: does it wrong, just good for its own limits.
    // => we need to overwrite it
    uint32_t get_gimbal_manager_flags() const override;

    // added: handle gimbal manager flags received from gimbal manager messages
    // AP nonsense: does it wrong, just good for its limits.
    // => we need to overwrite it
    bool handle_gimbal_manager_flags(uint32_t flags) override;

    ///<< end of Deal with some AP strangeness

    // handle GIMBAL_DEVICE_INFORMATION message
    // empty => we need to overwrite it
    void handle_gimbal_device_information(const mavlink_message_t &msg) override;

    // handle GIMBAL_DEVICE_ATTITUDE_STATUS message
    // empty => we need to overwrite it
    void handle_gimbal_device_attitude_status(const mavlink_message_t &msg) override;

    // added: handle_message_extra - allows to process a msg from a gimbal
    void handle_message_extra(const mavlink_message_t &msg) override;

    // added: send banner
    void send_banner() override;

private:

    // internal variables

    uint8_t _sysid;                 // sysid of gimbal
    uint8_t _compid;                // component id of gimbal, zero if gimbal not yet discovered
    mavlink_channel_t _chan;        // mavlink channel used to communicate with gimbal
    // Comment: in some drivers the newer _link construct is used, which is however not smart,
    // since for each message it does a binary search to find and check the size. This is not
    // needed with the older _chan construct, which is thus much more efficient cpu wise. The
    // older _chan also allows for an early test for space avoiding potentially lengthy calcs.
    // We hence continue to use the _chan construct.

    bool _got_device_info;          // gimbal discovered, waiting for gimbal provide device info
    bool _initialised;              // true once all init steps have been passed
    bool _got_radio_rc_channels;    // true when a RADIO_RC_CHANNELS message has been received
    bool _startup_rc_failsafe = true;

    // gimbal discovery

    // search for gimbal in GCS_MAVLink routing table
    void find_gimbal();

    uint32_t _request_device_info_tlast_ms;
    mavlink_gimbal_device_information_t _device_info;
    uint16_t _device_version_int;

    // request GIMBAL_DEVICE_INFORMATION, we can get this also if 'old' MOUNT messages are used
    void send_cmd_request_gimbal_device_information();

    // pre-arm and healthy checks
    // some need to be made mutable to get around that healthy() is const

    enum STorM32State {
        STARTUP_MOTORS = 0,
        STARTUP_SETTLE,
        STARTUP_CALIBRATE,
        STARTUP_LEVEL,
        STARTUP_MOTORDIRDETECT,
        STARTUP_RELEVEL,
        NORMAL,
        STARTUP_FASTLEVEL,
    };

    bool _gimbal_armed;             // true once the gimbal has reached normal state
    bool _gimbal_prearmchecks_ok;   // true when the gimbal stops reporting prearm fail in the HEARTBEAT message

    bool _healthy;

    bool _prearmchecks_passed;      // becomes true when all checks were passed once at startup
    uint32_t _prearmcheck_sendtext_tlast_ms;
    bool _checks_last;              // result of last check, to detect toggle from true -> false, false -> true
    uint32_t _checks_tlast_ms;

    bool is_healthy();
    void update_checks();

    // gimbal target & control

    struct {
        uint16_t received_flags;    // obtained from GIMBAL_DEVICE_ATTITUDE_STATUS
        uint32_t received_failure_flags; // obtained from GIMBAL_DEVICE_ATTITUDE_STATUS
        uint32_t received_tlast_ms; // time last GIMBAL_DEVICE_ATTITUDE_STATUS was received (used for health reporting)
    } _device_status;

    uint32_t _flags_from_gimbal_client = UINT32_MAX; // flags received via gimbal manager messages/commands, the UINT32_MAX is important!
    uint16_t _flags_for_gimbal_device;     // flags to be send to gimbal device

    struct {
        float roll;
        float pitch;
        float yaw_bf;
        float delta_yaw;
    } _current_angles = {0.0f, 0.0f, 0.0f, NAN}; // current angles, obtained from MOUNT_STATUS or GIMBAL_DEVICE_ATTITUDE_STATUS, the NAN is important!
    Vector3f _current_omega;        // current angular velocities, obtained from GIMBAL_DEVICE_ATTITUDE_STATUS

    // set the flags for gimbal according to current conditions
    void update_gimbal_device_flags();

    // determines the target angles based on mount mode, does the crucial job of controlling
    void update_target_angles();

    // sends the target angles to gimbal, called in update loop with 12.5 Hz
    void send_target_angles();

    // send GIMBAL_DEVICE_SET_ATTITUDE with latest angle targets to the gimbal to control attitude
    // When the gimbal receives this message, it can assume it is coming from its gimbal manager
    // (as nobody else is allowed to send this to it). STorM32 thus identifies the message's
    // source sysid & compid as its associated gimbal manager's sysid & compid.
    void send_gimbal_device_set_attitude();

    uint32_t _tahrs_healthy_ms;
    void send_autopilot_state_for_gimbal_device();

    // further tasks

    void send_rc_channels();

    uint32_t _send_system_time_tlast_ms;
    void update_send_system_time();

    uint32_t _request_send_banner_ms;
    void update_send_banner();

    struct {
        uint8_t fast;               // counter to determine faster responses
        uint32_t tlast_ms;          // time last GIMBAL_MANAGER_STATUS was send
        uint32_t flags_last;        // to detect changes
        mavlink_control_id_t primary_last;
    } _manager_status;
    void update_send_gimbal_manager_status();

    // task

    enum Task {
        TASK_SLOT0 = 0,
        TASK_SLOT1,
        TASK_SLOT2,
        TASK_SLOT3
    };
    uint8_t _task_counter;

    // logging

    bool _should_log = true;
};


#endif // HAL_MOUNT_STORM32_MAVLINK_V2_ENABLED
