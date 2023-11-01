//*****************************************************
//OW
// (c) olliw, www.olliw.eu, GPL3
// STorM32 mount backend class
//*****************************************************
#pragma once

#include "AP_Mount.h"
#include "AP_Mount_Backend.h"


#define PROTOCOL_AUTO_TIMEOUT_CNT  10


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
    // STorM32: this needs to not set _yaw_lock, as
    // (1) STorM32 doesn't (yet) support earth frame, it does support yaw lock in vehicle frame
    // (2) _yaw_lock and vehicle or earth frame are not fundamentally related to each other
    // in the backend, _yaw_lock is used as yaw_is_ef
    void set_yaw_lock(bool yaw_lock) override { _is_yaw_lock = yaw_lock; }

    // handle GIMBAL_DEVICE_INFORMATION message
    void handle_gimbal_device_information(const mavlink_message_t &msg) override;

    // handle GIMBAL_DEVICE_ATTITUDE_STATUS message
    void handle_gimbal_device_attitude_status(const mavlink_message_t &msg) override;

    // send a GIMBAL_DEVICE_ATTITUDE_STATUS message to GCS
    // STorM32: this needs to be empty, because the gimbal device is sending it itself
    // we don't do any of ArduPilot's private mavlink channel nonsense
    void send_gimbal_device_attitude_status(mavlink_channel_t chan) override {}

    // take a picture.  returns true on success
    // we do not need to do anything since AP_Camera::take_picture() will send a a CMD_LONG:DO_DIGICAM_CONTROL to all components
    // we have modified this such that it is not send if it is CamTrigType::mount (CAM_TRIGG_TYPE = 3)
    bool take_picture() override;

    // start or stop video recording.  returns true on success
    // set start_recording = true to start record, false to stop recording
    bool record_video(bool start_recording) override;

    // set camera zoom step.  returns true on success
    // zoom out = -1, hold = 0, zoom in = 1
//XX ??    bool set_zoom_step(int8_t zoom_step) override { return false; }

    // set photo or video mode
    bool set_cam_mode(bool video_mode) override;

    // 3-way switch mode
    bool set_cam_photo_video(int8_t sw_flag) override;

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
    bool _prearmchecks_ok;          // true when the gimbal stops reporting prearm fail in the HEARTBEAT message
    bool _got_radio_rc_channels;    // true when a RADIO_RC_CHANNELS message has been received

    // internal MAVLink variables

    uint8_t _sysid;                 // system id of gimbal, supposedly equal to our flight controller sysid
    uint8_t _compid;                // component id of gimbal, 0 indicates gimbal not yet found
    mavlink_channel_t _chan;        // mavlink channel used to communicate with gimbal

    // initialization etc

    uint32_t _request_device_info_tlast_ms;
    mavlink_gimbal_device_information_t _device_info;

    // flags

    bool _sendonly;
    bool _send_autopilotstateext;
    bool _should_log;
    bool _use_3way_photo_video;

    enum PROTOCOLENUM {
        PROTOCOL_UNDEFINED = 0,          // we do not yet know
        PROTOCOL_ARDUPILOT_LIKE,         // gimbal uses V2 gimbal device messages
        PROTOCOL_STORM32_GIMBAL_MANAGER, // gimbal is a STorM32 gimbal manager
    };
    uint8_t _protocol;

    // internal task variables

    enum TASKENUM {
        TASK_SLOT0 = 0,
        TASK_SLOT1,
        TASK_SLOT2,
        TASK_SLOT3,
    };

    uint16_t _task_counter;

    // blabla

    uint8_t _protocol_auto_cntdown;
    void determine_protocol(const mavlink_message_t &msg);

    bool _armingchecks_enabled; // true when ARMING_CHECK_ALL or ARMING_CHECK_CAMERA set
    bool _prearmchecks_all_ok; // becomes true when a number of conditions are full filled, track changes
    bool prearmchecks_all_ok(void);
    bool prearmchecks_do(void); // workaround needed since healthy() is const
    void send_prearmchecks_txt(void);

    struct {
        bool updated; // set to true when a new EVENT message is received
        uint32_t enabled_flags;
        uint32_t fail_flags;
        uint32_t fail_flags_last;
        uint32_t sendtext_tlast_ms;
        bool available(void) { return updated && (enabled_flags > 0) && (fail_flags > 0); }
    } _prearmcheck; // for component prearm status handling

    bool _is_yaw_lock;

    void set_and_send_target_angles(void);
    void send_autopilot_state_for_gimbal_device(void);
    void send_autopilot_state_for_gimbal_device_ext(void);

    enum GIMBALTARGETMODEENUM {
        TARGET_MODE_RETRACT = 0,
        TARGET_MODE_NEUTRAL,
        TARGET_MODE_ANGLE,
    };

    struct GimbalTarget {
        float roll;
        float pitch;
        float yaw;
        bool yaw_is_ef;
        uint8_t mode;

        void set(MountTarget &t, uint8_t m) { roll = t.roll; pitch = t.pitch; yaw = t.yaw; yaw_is_ef = t.yaw_is_ef; mode = m; }
        void set(uint8_t m) { roll = pitch = yaw = 0.0f; yaw_is_ef = false; mode = m; }
        void set_angle(MountTarget &t) { set(t, TARGET_MODE_ANGLE); }
        void set_from_vec_deg(const Vector3f &v_deg, uint8_t m) { roll = ToRad(v_deg.x); pitch = ToRad(v_deg.y); yaw = ToRad(v_deg.z); yaw_is_ef = false; mode = m; }
        void get_q_array(float* q_array);
    };

    struct {
        uint16_t received_flags; // obtained from GIMBAL_DEVICE_ATTITUDE_STATUS
        uint32_t received_failure_flags; // obtained from GIMBAL_DEVICE_ATTITUDE_STATUS
        uint32_t received_tlast_ms; // time last GIMBAL_DEVICE_ATTITUDE_STATUS was received (used for health reporting)
        uint16_t flags_for_gimbal;
    } _device;

    struct {
        bool ap_client_is_active; // true when autopilot client is active
        uint16_t flags_for_gimbal;
    } _manager;

    struct {
        uint8_t mode;
        uint8_t mode_last;
    } _qshot;

    bool get_target_angles_mount(GimbalTarget &gtarget, enum MAV_MOUNT_MODE mmode);
    bool get_target_angles_qshot(GimbalTarget &gtarget);
    void update_gimbal_device_flags_mount(enum MAV_MOUNT_MODE mmode);
    void update_gimbal_device_flags_qshot(void);
    void update_gimbal_manager_flags(void);

    void send_gimbal_device_set_attitude(GimbalTarget &gtarget);
    void send_storm32_gimbal_manager_control(GimbalTarget &gtarget);

    uint32_t _send_system_time_tlast_ms;
    void send_system_time(void);

    void send_rc_channels(void);

    // camera

    uint8_t _camera_compid;         // component id of a mavlink camera, 0 indicates camera not yet found

    enum CAMERAMODEENUM {
        CAMERA_MODE_UNDEFINED = 0,  // we do not yet know
        CAMERA_MODE_PHOTO,
        CAMERA_MODE_VIDEO,
    };
    uint8_t _camera_mode;           // current camera mode

    void send_cmd_do_digicam_configure(bool video_mode);
    void send_cmd_do_digicam_control(bool shoot);

    // mount_status forwarding

    struct MountStatus {
        float roll_deg;
        float pitch_deg;
        float yaw_deg;
    };

    void send_mount_status_to_ground(MountStatus &status);
    void send_to_ground(uint32_t msgid, const char *pkt);
};

