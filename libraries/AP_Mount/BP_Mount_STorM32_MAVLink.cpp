//*****************************************************
//OW
// (c) olliw, www.olliw.eu, GPL3
// STorM32 mount backend class
// uses 100% MAVlink + storm32.xml
//*****************************************************

#include <AP_HAL/AP_HAL.h>
#include <RC_Channel/RC_Channel.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_Notify/AP_Notify.h>
#include <AP_RTC/AP_RTC.h>
#include "BP_Mount_STorM32_MAVLink.h"
#include "bp_version.h"

extern const AP_HAL::HAL& hal;


/*
0:  auto mode
2:  old behavior
    sends DO_MOUNT_CONTROL, DO_MOUNT_CONFIGURE for control, sends tunnel for STorM32-Link
4:  like 2, but using new gimbal device messages
    sends GIMBAL_DEVICE_SET_ATTITUDE for control, sends AUTOPILOT_STATE_FOR_GIMBAL for STorM32-Link
    this mode is for testing, not for regular use
1:  for gimbal manager
    mode for when there is a gimbal manager in the system, e.g. on the STorM32 or on the companion
    uses STorM32 gimbal manager messages
8:  only streaming
    only sends out RC_CHANNLES, AUTOPILOT_STATE_FOR_GIMBAL for STorM32-Link
    this mode could in principle be replaced by asking for the streams, but since AP isn't streaming reliably we don't

 in all modes sends MOUNT_STATUS to ground, so that "old" things like MP etc can see the gimbal orientation
 listens to ATTITUDE, MOUNT_STATUS, STORM32_GIMBAL_DEVICE_STATUS to send out MOUNT_STATUS in sync

128: prearming check is enabled
*/
/*
 also sends SYSTEM_TIME to gimbal
*/
/*
TODO:
*/


//******************************************************
// Quaternion & Euler for Gimbal
//******************************************************
// we do not use NED (roll-pitch-yaw) to convert received quaternion to Euler angles and vice versa
// we use pitch-roll-yaw instead
// when the roll angle is zero, both are equivalent, this should be the majority of cases anyhow
// also, for most gimbals pitch-roll-yaw is appropriate
// the issue with NED is the gimbal lock at pitch +-90°, but pitch +-90° is a common operation point for gimbals
// the angles we store in this lib are thus pitch-roll-yaw Euler

class GimbalQuaternion : public Quaternion
{
public:
    // inherit constructors
    using Quaternion::Quaternion;

    // create a quaternion from gimbal Euler angles
    void from_gimbal_euler(float roll, float pitch, float yaw);

    // create gimbal Euler angles from a quaternion
    void to_gimbal_euler(float &roll, float &pitch, float &yaw) const;
};


void GimbalQuaternion::from_gimbal_euler(float roll, float pitch, float yaw)
{
    const float cr2 = cosf(roll*0.5f);
    const float cp2 = cosf(pitch*0.5f);
    const float cy2 = cosf(yaw*0.5f);
    const float sr2 = sinf(roll*0.5f);
    const float sp2 = sinf(pitch*0.5f);
    const float sy2 = sinf(yaw*0.5f);

    q1 = cp2*cr2*cy2 - sp2*sr2*sy2;  // ->  cp2*cy2
    q2 = cp2*sr2*cy2 - sp2*cr2*sy2;  // -> -sp2*sy2
    q3 = sp2*cr2*cy2 + cp2*sr2*sy2;  // ->  sp2*cy2
    q4 = sp2*sr2*cy2 + cp2*cr2*sy2;  // ->  cp2*sy2
}


void GimbalQuaternion::to_gimbal_euler(float &roll, float &pitch, float &yaw) const
{
    pitch = atan2f(2.0f*(q1*q3 - q2*q4), 1.0f - 2.0f*(q2*q2 + q3*q3));  // -R31 / R33 = -(-spcr) / cpcr
    roll = safe_asin(2.0f*(q1*q2 + q3*q4));                             // R32 = sr
    yaw = atan2f(2.0f*(q1*q4 - q2*q3), 1.0f - 2.0f*(q2*q2 + q4*q4));    // -R12 / R22 = -(-crsy) / crcy
}



//******************************************************
// BP_Mount_STorM32_MAVLink, that's the main class
//******************************************************

// constructor
BP_Mount_STorM32_MAVLink::BP_Mount_STorM32_MAVLink(AP_Mount &frontend, AP_Mount::mount_state &state, uint8_t instance) :
    AP_Mount_Backend(frontend, state, instance)
{
    _initialised = false;
    _armed = false;
    _prearmchecks_ok = true; //true means it is not checked

    _sysid = 0;
    _compid = 0;
    _chan = MAVLINK_COMM_0; //this is a dummy, will be set correctly by find_gimbal()

    _task_time_last = 0;
    _task_counter = TASK_SLOT0;
    _send_system_time_last = 0;

    _target.mode_last = MAV_MOUNT_MODE_RC_TARGETING;

    _is_active = true;
    _qshot.mode = MAV_QSHOT_MODE_UNDEFINED; //as soon as it becomes defined, we switch to using qshot instead of mount_mode
    _qshot.mode_last = _qshot.mode;

    if (_state._zflags & 0x80) {
        _prearmchecks_ok = false; //enable prearm checks
    }

    _auto_mode = AUTOMODE_UNDEFINED; // determines operation mode
    _auto_mode_cntdown = AUTOMODE_CNT;

    _use_protocolv2 = false;    //true means mode 1, 4, 8
    _use_gimbalmanager = false; //true means mode 1
    _sendonly = false;          //true means mode 8
#if USE_GIMBAL_ZFLAGS
    if (_state._zflags & 0x02) { //2 set
        _auto_mode = AUTOMODE_V1;
    } else
    if (_state._zflags & 0x0F) { //1, 4, 8 set
        _use_protocolv2 = true;
        _auto_mode = AUTOMODE_GIMBALDEVICE;
        if (_state._zflags & 0x01) { _use_gimbalmanager = true; _auto_mode = AUTOMODE_GIMBALMANAGER; }
        if (_state._zflags & 0x08) _sendonly = true;
    }
#endif
}


//------------------------------------------------------
// BP_Mount_STorM32_MAVLink interface functions, ArduPilot Mount
//------------------------------------------------------

// init - performs any required initialisation for this instance
void BP_Mount_STorM32_MAVLink::init(void)
{
    _initialised = false; //should be false but can't hurt to ensure that

    // set mode to default value set by user via parameter
    set_mode((enum MAV_MOUNT_MODE)_state._default_mode.get());
}


// update mount position - should be called periodically
// this function must be defined in any case
void BP_Mount_STorM32_MAVLink::update()
{
    if (!_initialised) {
        find_gimbal();
        return;
    }

    uint32_t now_ms = AP_HAL::millis();

    if ((now_ms - _send_system_time_last) >= 5000) { //every 5 sec is really plenty
        _send_system_time_last = now_ms;
        send_system_time_to_gimbal();
    }
}


// 400 Hz loop
void BP_Mount_STorM32_MAVLink::update_fast()
{
    if (!_initialised) {
        return;
    }

    // we originally wanted to slow down everything to 100 Hz,
    // so that updates are at 20 Hz, especially for STorM32-Link
    // however, sadly, plane runs at 50 Hz only, so we update at 25 Hz and 12.5 Hz respectively
    // not soo nice
    // not clear what it does for STorM32Link, probably not too bad, maybe even good

    #define PERIOD_US   20000 //20 ms = 50 Hz

    uint32_t now_us = AP_HAL::micros();
    if ((now_us - _task_time_last) >= PERIOD_US) {
        // _task_time_last = now_us;
        // this gives MUCH higher precision!!!:
        _task_time_last += PERIOD_US;
        if ((now_us - _task_time_last) > 5000) _task_time_last = now_us; //we got out of sync, so get back in sync

        switch (_task_counter) {
            case TASK_SLOT0:
            case TASK_SLOT2:
                if (_use_protocolv2) {
                    send_autopilot_state_for_gimbal_device_to_gimbal();
                } else {
                    send_cmd_storm32link_v2(); //2.3ms
                }
                break;

            case TASK_SLOT1:
                if (_sendonly) break; //don't send any control messages
                if (_use_protocolv2) {
                    if (_qshot.mode == MAV_QSHOT_MODE_UNDEFINED) {
                        set_target_angles();
                    } else {
                        set_target_angles_qshot();
                    }
                    send_target_angles_to_gimbal_v2();
                } else {
                    set_target_angles();
                    send_target_angles_to_gimbal();
                }
                break;

            case TASK_SLOT3:
                send_rc_channels_to_gimbal();
                break;
        }

        _task_counter++;
        if (_task_counter > TASK_SLOT3) _task_counter = 0;
    }
}


// set_mode - sets mount's mode
void BP_Mount_STorM32_MAVLink::set_mode(enum MAV_MOUNT_MODE mode)
{
    if (!_initialised) {
        return;
    }

    _state._mode = mode;
}


// send_mount_status - called to allow mounts to send their status to GCS using the MOUNT_STATUS message
void BP_Mount_STorM32_MAVLink::send_mount_status(mavlink_channel_t chan)
{
    // it doesn't matter if not _initalised, it will then send out zeros

    // this is called by GCS_MAVLINK_Copter::try_send_message() which in turn is called by the streamer
    // so, it is send out with 2Hz, by the SR0_EXTRA3 setting, which per default is 2Hz only

    //mavlink_msg_mount_status_send(chan, 0, 0, _status.pitch_deg*100.0f, _status.roll_deg*100.0f, _status.yaw_deg*100.0f);
}


// handle_msg - allows to process messages received from gimbal
void BP_Mount_STorM32_MAVLink::handle_msg(const mavlink_message_t &msg)
{
    if (!_initialised) {
        if (!_auto_mode) determine_auto_mode(msg);
        return;
    }

    //TODO: should we check here msg.sysid if it is for us? sounds good, but could also cause issues, so not for now
    //TODO: if we capture a COMMAND_LONG here, what happens with COMMAND_ACK outside of here??
    // comment: we do not bother with sending/handling CMD_ACK momentarily
    // this is dirty, should go to GCS_MAVLINK, GCS, etc., but we don't want to pollute and infect, hence here in dirty ways

    // listen to the qshot commands and messages to track changes in qshot mode
    // this may come from anywhere
    switch (msg.msgid) {
        case MAVLINK_MSG_ID_COMMAND_LONG: { //76
            if (!_use_protocolv2) break;
            mavlink_command_long_t payload;
            mavlink_msg_command_long_decode( &msg, &payload );
            switch (payload.command) {
                case MAV_CMD_QSHOT_DO_CONFIGURE: //62020
                    uint8_t new_mode = payload.param1;
                    if (new_mode == MAV_QSHOT_MODE_UNDEFINED) _qshot.mode = MAV_QSHOT_MODE_UNDEFINED;
                    if (new_mode != _qshot.mode) {
                        _qshot.mode = UINT8_MAX; //mode change requested, so put it into hold, must be acknowledged by qshot status
                    }
                break;
            }
            }break;

        case MAVLINK_MSG_ID_QSHOT_STATUS: { //62020
            if (!_use_protocolv2) break;
            mavlink_qshot_status_t payload;
            mavlink_msg_qshot_status_decode( &msg, &payload );
            _qshot.mode = payload.mode;
            }break;
    }

    if (msg.sysid != _sysid) { //this msg is not from our system
        return;
    }

    // we use STORM32_GIMBAL_MANGER_STATUS to detect the activity of the autopilot client
    switch (msg.msgid) {
        case MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_STATUS: { //62011
            if (!_use_protocolv2) break;
            mavlink_storm32_gimbal_manager_status_t payload;
            mavlink_msg_storm32_gimbal_manager_status_decode( &msg, &payload );
            if (payload.gimbal_id != _compid) break; //not for our gimbal device
            _is_active = (payload.supervisor != MAV_STORM32_GIMBAL_MANAGER_CLIENT_NONE) && //a client is supervisor
                         (payload.manager_flags & MAV_STORM32_GIMBAL_MANAGER_FLAGS_CLIENT_AUTOPILOT_ACTIVE); //and autopilot is active
            }break;
    }

    if (msg.compid != _compid) { //this msg is not from our gimbal
        return;
    }

    bool send_mountstatus = false;

    switch (msg.msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT: {
            mavlink_heartbeat_t payload;
            mavlink_msg_heartbeat_decode( &msg, &payload );
            _armed = is_normal_state(payload.custom_mode & 0xFF);
            if( !(payload.custom_mode & 0x80000000) ){ //we don't follow all changes, but just toggle it to true once
                _prearmchecks_ok = true;
            }
            }break;

        case MAVLINK_MSG_ID_ATTITUDE: { //30
            mavlink_attitude_t payload;
            mavlink_msg_attitude_decode( &msg, &payload );
            _status.pitch_deg = degrees(payload.pitch);
            _status.roll_deg = degrees(payload.roll);
            _status.yaw_deg = degrees(payload.yaw);
            _status.yaw_deg_absolute = NAN;
            send_mountstatus = true;
            }break;

        case MAVLINK_MSG_ID_MOUNT_STATUS: { //158
            mavlink_mount_status_t payload;
            mavlink_msg_mount_status_decode( &msg, &payload );
            if (payload.target_system) break; //trigger sending of MOUNT_STATUS to ground only if target_sysid = 0
            _status.pitch_deg = (float)payload.pointing_a * 0.01f;
            _status.roll_deg = (float)payload.pointing_b * 0.01f;
            _status.yaw_deg = (float)payload.pointing_c * 0.01f;
            _status.yaw_deg_absolute = NAN;
            send_mountstatus = true;
            }break;

        case MAVLINK_MSG_ID_STORM32_GIMBAL_DEVICE_STATUS: { //62001
            if (!_use_protocolv2) break;
            mavlink_storm32_gimbal_device_status_t payload;
            mavlink_msg_storm32_gimbal_device_status_decode( &msg, &payload );
            if (payload.target_system) break; //trigger sending of MOUNT_STATUS to ground only if target_sysid = 0
            float roll_rad, pitch_rad, yaw_rad;
            GimbalQuaternion quat(payload.q[0], payload.q[1], payload.q[2], payload.q[3]);
            quat.to_gimbal_euler(roll_rad, pitch_rad, yaw_rad);
            _status.roll_deg = degrees(roll_rad);
            _status.pitch_deg = degrees(pitch_rad);
            _status.yaw_deg = degrees(yaw_rad);
            _status.yaw_deg_absolute = degrees(payload.yaw_absolute);
            send_mountstatus = true;
            }break;
    }

    // forward a MOUNT_STATUS message to ground, this is only to make MissionPlanner and alike happy
    // do it only for the primary gimbal
    if (send_mountstatus && is_primary()) {
        send_mount_status_to_ground();
    }
}


bool BP_Mount_STorM32_MAVLink::pre_arm_checks(void)
{
    return _prearmchecks_ok;
}


void BP_Mount_STorM32_MAVLink::send_banner(void)
{
    if (_initialised) {
        char s[50];
        s[0] = '\0';
        if (is_primary()) strcpy(s, ", is primary");
        gcs().send_text(MAV_SEVERITY_INFO, "MNT%u: gimbal at %u%s", _instance+1, _compid, s); // %u vs %d ???
        //gcs().send_text(MAV_SEVERITY_INFO, "auto mode: %u", _auto_mode);
    } else {
        gcs().send_text(MAV_SEVERITY_INFO, "MNT%u: no gimbal yet", _instance+1);
    }
}


bool BP_Mount_STorM32_MAVLink::is_rc_failsafe(void)
{
    const RC_Channel *roll_ch = rc().channel(_state._roll_rc_in - 1);
    const RC_Channel *tilt_ch = rc().channel(_state._tilt_rc_in - 1);
    const RC_Channel *pan_ch = rc().channel(_state._pan_rc_in - 1);

    if ((roll_ch != nullptr) && (roll_ch->get_radio_in() < 700)) return true;
    if ((tilt_ch != nullptr) && (tilt_ch->get_radio_in() < 700)) return true;
    if ((pan_ch != nullptr) && (pan_ch->get_radio_in() < 700)) return true;

    return false;
}


//------------------------------------------------------
// private functions, protocol v1
//------------------------------------------------------

void BP_Mount_STorM32_MAVLink::set_target_angles(void)
{
    bool set_target = false;

    enum MAV_MOUNT_MODE mount_mode = get_mode();

    switch (mount_mode) {
        // move mount to a "retracted" position.  To-Do: remove support and replace with a relaxed mode?
        case MAV_MOUNT_MODE_RETRACT:
            {
                // this is not really needed as it's ignored in send_target_angles_to_gimbal() and recenter is triggered, but hey, can't harm
                const Vector3f &target = _state._retract_angles.get();
                _angle_ef_target_rad.x = radians(target.x);
                _angle_ef_target_rad.y = radians(target.y);
                _angle_ef_target_rad.z = radians(target.z);
                set_target = true;
            }
            break;

        // move mount to a neutral position, typically pointing forward
        case MAV_MOUNT_MODE_NEUTRAL:
            {
                // this is not really needed as it's ignored in send_target_angles_to_gimbal() and recenter is triggered, but hey, can't harm
                const Vector3f &target = _state._neutral_angles.get();
                _angle_ef_target_rad.x = radians(target.x);
                _angle_ef_target_rad.y = radians(target.y);
                _angle_ef_target_rad.z = radians(target.z);
                set_target = true;
            }
            break;

        // point to the angles given by a mavlink message
        case MAV_MOUNT_MODE_MAVLINK_TARGETING:
            // do nothing because earth-frame angle targets (i.e. _angle_ef_target_rad) should have already been set by a MOUNT_CONTROL message from GCS
            // NO!!: clear yaw since if has_pan == false the copter will yaw, so we must not forward it to the gimbal
            if (!has_pan_control()) {
                const Vector3f &target = _state._neutral_angles.get();
                _angle_ef_target_rad.z = radians(target.z); //clear yaw
            }
            set_target = true;
            break;

        // RC radio manual angle control, but with stabilization
        case MAV_MOUNT_MODE_RC_TARGETING:
            // update targets using pilot's rc inputs
            update_targets_from_rc();
            if (is_rc_failsafe()) {
                _angle_ef_target_rad.y = _angle_ef_target_rad.x = _angle_ef_target_rad.z = 0.0f;
            }
            set_target = true;
            break;

        // point mount to a GPS point given by the mission planner
        case MAV_MOUNT_MODE_GPS_POINT:
            if (calc_angle_to_roi_target(_angle_ef_target_rad, true, true, true)) {
                set_target = true;
            }
            break;

        case MAV_MOUNT_MODE_HOME_LOCATION:
            // constantly update the home location:
            if (!AP::ahrs().home_is_set()) {
                break;
            }
            _state._roi_target = AP::ahrs().get_home();
            _state._roi_target_set = true;
            if (calc_angle_to_roi_target(_angle_ef_target_rad, true, true, true)) {
                set_target = true;
            }
            break;

        case MAV_MOUNT_MODE_SYSID_TARGET:
            if (calc_angle_to_sysid_target(_angle_ef_target_rad, true, true, true)) {
                set_target = true;
            }
            break;

        default:
            // we do not know this mode so do nothing
            return;
    }

    // set target angles, to communicate to send functions
    // I think this is not needed, as it just duplicates _angle_ef_target_rad, but hey, doesn't harm
    if (set_target) {
        _target.roll_deg = degrees(_angle_ef_target_rad.x);
        _target.pitch_deg = degrees(_angle_ef_target_rad.y);
        _target.yaw_deg = degrees(_angle_ef_target_rad.z);
    }
    _target.mode = mount_mode; //we always set the mode
}


// is called by task loop at 25 Hz
void BP_Mount_STorM32_MAVLink::send_target_angles_to_gimbal(void)
{
    if (_target.mode <= MAV_MOUNT_MODE_NEUTRAL) { //RETRACT and NEUTRAL
        // only do it once, i.e., when mode has just changed
        if (_target.mode_last != _target.mode) {
            _target.mode_last = _target.mode;
            send_cmd_do_mount_control_to_gimbal(_target.roll_deg, _target.pitch_deg, _target.yaw_deg, _target.mode);
        }
        return;
    }

    // update to current mode, to avoid repeated actions on some mount mode changes
    _target.mode_last = _target.mode;

    send_cmd_do_mount_control_to_gimbal(_target.roll_deg, _target.pitch_deg, _target.yaw_deg, _target.mode);
}


//------------------------------------------------------
// private functions, storm32 gimbal protocol
//------------------------------------------------------

void BP_Mount_STorM32_MAVLink::set_target_angles_qshot(void)
{
    switch (_qshot.mode) {
        case MAV_QSHOT_MODE_UNDEFINED:
            return;

        case MAV_QSHOT_MODE_GIMBAL_RETRACT: {
            // we don't really have to do anything since handled by flags, but hey, why not clear it
            const Vector3f &target = _state._retract_angles.get();
            _angle_ef_target_rad.x = radians(target.x);
            _angle_ef_target_rad.y = radians(target.y);
            _angle_ef_target_rad.z = radians(target.z);
            }break;

        case MAV_QSHOT_MODE_GIMBAL_NEUTRAL: {
            // we don't really have to do anything since handled by flags, but hey, why not clear it
            const Vector3f &target = _state._neutral_angles.get();
            _angle_ef_target_rad.x = radians(target.x);
            _angle_ef_target_rad.y = radians(target.y);
            _angle_ef_target_rad.z = radians(target.z);
            }break;

        case MAV_QSHOT_MODE_GIMBAL_RC_CONTROL:
            update_targets_from_rc();
            if (is_rc_failsafe()) {
                _angle_ef_target_rad.y = _angle_ef_target_rad.x = _angle_ef_target_rad.z = 0.0f;
            }
            break;

        case MAV_QSHOT_MODE_POI_TARGETING:
            if (!calc_angle_to_roi_target(_angle_ef_target_rad, true, true, true)) return;
            break;

        case MAV_QSHOT_MODE_SYSID_TARGETING:
            if (!calc_angle_to_sysid_target(_angle_ef_target_rad, true, true, true)) return;
            break;

/* not yet existing
        case MAV_QSHOT_MODE_HOME_TARGETING:*/
        case 9:
            if (!AP::ahrs().home_is_set()) return;
            _state._roi_target = AP::ahrs().get_home();
            _state._roi_target_set = true;
            if (!calc_angle_to_roi_target(_angle_ef_target_rad, true, true, true)) return;
            break;

        default:
            // in all other modes we don't do nothing, i.e. just send out something
            // it is the job of the supervisor to get things right by setting the activity
            if (!has_pan_control()) {
                const Vector3f &target = _state._neutral_angles.get();
                _angle_ef_target_rad.z = radians(target.z); //clear yaw
            }
            break;
    }

    _target.roll_deg = degrees(_angle_ef_target_rad.x);
    _target.pitch_deg = degrees(_angle_ef_target_rad.y);
    _target.yaw_deg = degrees(_angle_ef_target_rad.z);
}


// is called by task loop at 25 Hz
// finally sends out target angles
// assumes that old set_target_angles() is called

void BP_Mount_STorM32_MAVLink::send_target_angles_to_gimbal_v2(void)
{
uint16_t gimbaldevice_flags, gimbalmanager_flags;

    // we set these flags here!!
    gimbaldevice_flags = MAV_STORM32_GIMBAL_DEVICE_FLAGS_ROLL_LOCK | MAV_STORM32_GIMBAL_DEVICE_FLAGS_PITCH_LOCK;

    if (_qshot.mode == MAV_QSHOT_MODE_UNDEFINED) {

        if (_target.mode == MAV_MOUNT_MODE_RETRACT) {
            gimbaldevice_flags = MAV_STORM32_GIMBAL_DEVICE_FLAGS_RETRACT;
        }
        if (_target.mode == MAV_MOUNT_MODE_NEUTRAL) {
            gimbaldevice_flags = MAV_STORM32_GIMBAL_DEVICE_FLAGS_NEUTRAL;
        }

    } else {

        if (_qshot.mode == MAV_QSHOT_MODE_GIMBAL_RETRACT) {
            gimbaldevice_flags = MAV_STORM32_GIMBAL_DEVICE_FLAGS_RETRACT;
        }
        if (_qshot.mode == MAV_QSHOT_MODE_GIMBAL_NEUTRAL) {
            gimbaldevice_flags = MAV_STORM32_GIMBAL_DEVICE_FLAGS_NEUTRAL;
        }

    }
    _qshot.mode_last = _qshot.mode;

    if (_qshot.mode == UINT8_MAX) return; //is in hold, don't send

    if (_use_gimbalmanager) {
        // when not active, don't send, this reduces traffic
        if (!_is_active) return;

        // we play it simple and do not attempt to claim supervision nor activity
        // we thus leave this to other components, e.g. a gcs, to set this
        gimbalmanager_flags = MAV_STORM32_GIMBAL_MANAGER_FLAGS_NONE;

        //TODO: we could claim supervisor and activity if the qshot mode suggests so
        // what is then about missions ???? should qshots include a mission mode??

        send_storm32_gimbal_manager_control_to_gimbal(_target.roll_deg, _target.pitch_deg, _target.yaw_deg, gimbaldevice_flags, gimbalmanager_flags);
    } else {
        send_storm32_gimbal_device_control_to_gimbal(_target.roll_deg, _target.pitch_deg, _target.yaw_deg, gimbaldevice_flags);
    }
}


//------------------------------------------------------
// discovery functions
//------------------------------------------------------

// handle_msg - allows to process messages received from gimbal
void BP_Mount_STorM32_MAVLink::determine_auto_mode(const mavlink_message_t &msg)
{
    if (msg.sysid != _sysid || msg.compid != _compid) { //this msg is not from our gimbal
        return;
    }

    switch (msg.msgid) {
        case MAVLINK_MSG_ID_ATTITUDE: { //30
            _auto_mode = AUTOMODE_V1;
            _auto_mode_cntdown = AUTOMODE_CNT;
            _use_protocolv2 = false;
            _use_gimbalmanager = false;
            }break;

        case MAVLINK_MSG_ID_MOUNT_STATUS: { //158
            _auto_mode = AUTOMODE_V1;
            _auto_mode_cntdown = AUTOMODE_CNT;
            _use_protocolv2 = false;
            _use_gimbalmanager = false;
            }break;

        case MAVLINK_MSG_ID_STORM32_GIMBAL_DEVICE_STATUS: { //62001
            if (_auto_mode == AUTOMODE_GIMBALMANAGER) break;
            if (_auto_mode_cntdown) _auto_mode_cntdown--;
            if (!_auto_mode_cntdown) {
                _auto_mode = AUTOMODE_GIMBALDEVICE;
                _use_protocolv2 = true;
                _use_gimbalmanager = false;
            }
            }break;

        case MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_STATUS: { //62011
            _auto_mode = AUTOMODE_GIMBALMANAGER;
            _auto_mode_cntdown = AUTOMODE_CNT;
            _use_protocolv2 = true;
            _use_gimbalmanager = true;
            }break;
   }
}


// is periodically called for as long as _initialised = false
// that's the old method, we use it as fallback
void BP_Mount_STorM32_MAVLink::find_gimbal_oneonly(void)
{
#if USE_FIND_GIMBAL_MAX_SEARCH_TIME_MS
    uint32_t now_ms = AP_HAL::millis();

    if (now_ms > FIND_GIMBAL_MAX_SEARCH_TIME_MS) {
        _initialised = false; //should be already false, but it can't hurt to ensure that
        return;
    }
#else
    const AP_Notify &notify = AP::notify();
    if (notify.flags.armed) {
        return; //do not search if armed, this implies we are going to fly soon
    }
#endif

    //TODO: should we double check that gimbal sysid == autopilot sysid?
    // yes, we should, but we don't bother, and consider it user error LOL

    // find_by_mavtype()  finds a gimbal and also sets _sysid, _compid, _chan
    if (GCS_MAVLINK::find_by_mavtype(MAV_TYPE_GIMBAL, _sysid, _compid, _chan)) {
        if (!_auto_mode) return;
        _initialised = true;
        send_banner();
    }

    //proposal:
    // change this function to allow an index, like find_by_mavtype(index, ....)
    // we then can call it repeatedly until it returns false, whereby increasing index as 0,1,...
    // we then can define that the first mavlink mount is that with lowest ID, and so on
}


void BP_Mount_STorM32_MAVLink::find_gimbal(void)
{
    // we only have one instance at max
#if AP_MOUNT_MAX_INSTANCES <= 1
    find_gimbal_oneonly();
#else

    // we allow several instances, but do have only one
    if (num_instances() <= 1) {
        find_gimbal_oneonly();
        return;
    }

    // so, we have several instances

#if USE_FIND_GIMBAL_MAX_SEARCH_TIME_MS
    uint32_t now_ms = AP_HAL::millis();

    if (now_ms > FIND_GIMBAL_MAX_SEARCH_TIME_MS) {
        _initialised = false; //should be already false, but it can't hurt to ensure that
        return;
    }
#else
    const AP_Notify &notify = AP::notify();
    if (notify.flags.armed) {
        return; //do not search if armed, this implies we are going to fly soon
    }
#endif

    //TODO: should we double check that gimbal sysid == autopilot sysid?
    // yes, we should, but we don't bother, and consider it user error LOL

    // we expect that instance 0 has compid = GIMBAL, instance 1 has compid = GIMBAL2, instance 2 has compid = GIMBAL3, ...
    uint8_t mycompid = (_instance == 0) ? MAV_COMP_ID_GIMBAL : MAV_COMP_ID_GIMBAL2 + (_instance-1);

    // search routed gimbals
    if (GCS_MAVLINK::find_by_mavtype_and_compid(MAV_TYPE_GIMBAL, mycompid, _sysid, _chan)) {
        _compid = mycompid;
        if (!_auto_mode) return;
        _initialised = true;
        send_banner();
        return;
    }
#endif
}


//------------------------------------------------------
// MAVLink send functions
//------------------------------------------------------

// forward a MOUNT_STATUS message to ground, this is only to make MissionPlanner and alike happy
void BP_Mount_STorM32_MAVLink::send_mount_status_to_ground(void)
{
    // space is checked by send_to_ground()

    mavlink_mount_status_t msg = {
        pointing_a : (int32_t)(_status.pitch_deg*100.0f),
        pointing_b : (int32_t)(_status.roll_deg*100.0f),
        pointing_c : (int32_t)(_status.yaw_deg*100.0f),
        target_system : 0,
        target_component : 0 };

    send_to_ground(MAVLINK_MSG_ID_MOUNT_STATUS, (const char*)&msg);
}


void BP_Mount_STorM32_MAVLink::send_cmd_do_mount_control_to_gimbal(float roll_deg, float pitch_deg, float yaw_deg, enum MAV_MOUNT_MODE mode)
{
    if (!HAVE_PAYLOAD_SPACE(_chan, COMMAND_LONG)) {
        return;
    }

    // convert from ArduPilot to STorM32 convention
    // this need correction p:-1,r:+1,y:-1
    pitch_deg = -pitch_deg;
    yaw_deg = -yaw_deg;

    // this is send out with _sysid, _compid as target
    // so it can be differentiated by the STorM32 controller from routed commands/messages by looking at source and target
    mavlink_msg_command_long_send(
        _chan,
        _sysid,
        _compid,
        MAV_CMD_DO_MOUNT_CONTROL,
        0,        //confirmation of zero means this is the first time this message has been sent
        pitch_deg,
        roll_deg,
        yaw_deg,
        0, 0, 0,  //param4 ~ param6 unused
        mode);
}


void BP_Mount_STorM32_MAVLink::send_rc_channels_to_gimbal(void)
{
    if (!HAVE_PAYLOAD_SPACE(_chan, RC_CHANNELS)) {
        return;
    }

    // rc().channel(ch)->get_radio_in() or RC_Channels::get_radio_in(ch) and so on
    // is not the same as hal.rcin->read(), since radio_in can be set by override
    #define RCIN(ch_index)  hal.rcin->read(ch_index)

    mavlink_msg_rc_channels_send(
        _chan,
        AP_HAL::millis(),
        16,
        RCIN(0), RCIN(1), RCIN(2), RCIN(3), RCIN(4), RCIN(5), RCIN(6), RCIN(7),
        RCIN(8), RCIN(9), RCIN(10), RCIN(11), RCIN(12), RCIN(13), RCIN(14), RCIN(15),
        0, 0,
        0);
}


void BP_Mount_STorM32_MAVLink::send_system_time_to_gimbal(void)
{
    if (!HAVE_PAYLOAD_SPACE(_chan, SYSTEM_TIME)) {
        return;
    }

    uint64_t time_unix = 0;
    AP::rtc().get_utc_usec(time_unix); //may fail, leaving time_unix at 0

    if (!time_unix) return; //no unix time available, so no reason to send

    mavlink_msg_system_time_send(
        _chan,
        time_unix,
        AP_HAL::millis());
}


void BP_Mount_STorM32_MAVLink::send_autopilot_state_for_gimbal_device_to_gimbal(void)
{
    if (!HAVE_PAYLOAD_SPACE(_chan, AUTOPILOT_STATE_FOR_GIMBAL_DEVICE)) {
        return;
    }

    const AP_AHRS_NavEKF &ahrs = AP::ahrs_navekf();
    const AP_GPS &gps = AP::gps();
    const AP_Notify &notify = AP::notify();

    nav_filter_status nav_status;
    ahrs.get_filter_status(nav_status);

    uint8_t status = STORM32LINK_FCSTATUS_ISARDUPILOT;
    if (ahrs.healthy()) { status |= STORM32LINK_FCSTATUS_AP_AHRSHEALTHY; }
    if (ahrs.initialised()) { status |= STORM32LINK_FCSTATUS_AP_AHRSINITIALIZED; }
    if (nav_status.flags.horiz_vel) { status |= STORM32LINK_FCSTATUS_AP_NAVHORIZVEL; }
    if (gps.status() >= AP_GPS::GPS_OK_FIX_3D) { status |= STORM32LINK_FCSTATUS_AP_GPS3DFIX; }
    if (notify.flags.armed) { status |= STORM32LINK_FCSTATUS_AP_ARMED; }

    Quaternion quat;
    quat.from_rotation_matrix(ahrs.get_rotation_body_to_ned());
    float q[4] = { quat.q1, quat.q2, quat.q3, quat.q4 }; //TODO: figure out how to do it correctly

    Vector3f vel;
    if (!ahrs.get_velocity_NED(vel)) { vel.x = vel.y = vel.z = 0.0f; } //it returns a bool, so it's a good idea to consider it

    float yawrate = NAN; //0.0f;

/* estimator status
no support by ArduPilot whatsoever
*/
    uint16_t _estimator_status = 0;
    if (status & STORM32LINK_FCSTATUS_AP_AHRSHEALTHY) _estimator_status |= ESTIMATOR_ATTITUDE;
    if (status & STORM32LINK_FCSTATUS_AP_AHRSINITIALIZED) _estimator_status |= ESTIMATOR_VELOCITY_VERT;

/* landed state
GCS_Common.cpp: virtual MAV_LANDED_STATE landed_state() const { return MAV_LANDED_STATE_UNDEFINED; }
Copter has it: GCS_MAVLINK_Copter::landed_state()
Plane does NOT have it ????
but it is protected, so we can't use it, need to redo it anyways
we can identify this be MAV_LANDED_STATE_UNDEFINED as value
we probably want to also take into account the arming state to mock something up
ugly as we will have vehicle dependency here
*/
    uint8_t _landed_state = MAV_LANDED_STATE_UNDEFINED;

    mavlink_msg_autopilot_state_for_gimbal_device_send(
        _chan,
        _sysid, _compid,
        AP_HAL::micros64(),
        q,
        0, //uint32_t q_estimated_delay_us,
        vel.x, vel.y, vel.z,
        0, //uint32_t v_estimated_delay_us,
        yawrate,
        _estimator_status, _landed_state);
        //uint64_t time_boot_us, const float *q, uint32_t q_estimated_delay_us,
        //float vx, float vy, float vz, uint32_t v_estimated_delay_us,
        //float feed_forward_angular_velocity_z, uint16_t estimator_status, uint8_t landed_state)
}


// the interface is similar to that of do_mount_control, to make it simpler for the moment, so we internally convert
void BP_Mount_STorM32_MAVLink::send_storm32_gimbal_device_control_to_gimbal(float roll_deg, float pitch_deg, float yaw_deg, uint16_t flags)
{
    if (!HAVE_PAYLOAD_SPACE(_chan, STORM32_GIMBAL_DEVICE_CONTROL)) {
        return;
    }

    GimbalQuaternion quat;
    quat.from_gimbal_euler( radians(roll_deg), radians(pitch_deg), radians(yaw_deg) );
    float q[4];
    q[0] = quat.q1;
    q[1] = quat.q2;
    q[2] = quat.q3;
    q[3] = quat.q4;

    mavlink_msg_storm32_gimbal_device_control_send(
        _chan,
        _sysid, _compid,
        flags,
        q,
        NAN, NAN, NAN);
        //uint16_t flags,
        //const float *q,
        //float angular_velocity_x, float angular_velocity_y, float angular_velocity_z)
}


// the interface is similar to that of do_mount_control, to make it simpler for the moment, so we internally convert
void BP_Mount_STorM32_MAVLink::send_storm32_gimbal_manager_control_to_gimbal(float roll_deg, float pitch_deg, float yaw_deg, uint16_t device_flags, uint16_t manager_flags)
{
    if (!HAVE_PAYLOAD_SPACE(_chan, STORM32_GIMBAL_MANAGER_CONTROL)) {
        return;
    }

    GimbalQuaternion quat;
    quat.from_gimbal_euler( radians(roll_deg), radians(pitch_deg), radians(yaw_deg) );
    float q[4];
    q[0] = quat.q1;
    q[1] = quat.q2;
    q[2] = quat.q3;
    q[3] = quat.q4;

    mavlink_msg_storm32_gimbal_manager_control_send(
        _chan,
        _sysid, _compid,
        _compid, //gimbal_id
        MAV_STORM32_GIMBAL_MANAGER_CLIENT_AUTOPILOT,
        device_flags, manager_flags,
        q,
        NAN, NAN, NAN);
        //uint8_t gimbal_id, uint8_t client, uint16_t device_flags, uint16_t manager_flags,
        //const float *q, float angular_velocity_x, float angular_velocity_y, float angular_velocity_z)
}


//------------------------------------------------------
// MAVLink send helper
//------------------------------------------------------

// this is essentially GCS::send_to_active_channels(uint32_t msgid, const char *pkt)
// but exempts the gimbal channel
//TODO: Is dodgy as it assumes that ONLY the gimbal is on the link !!
//      We actually only need to send to the ground, i.e., the gcs-es (but there could be more than one)
//      This is what this achieves for gcs-ap-g or gcs-ap-cc-g topologies, but not for others
void BP_Mount_STorM32_MAVLink::send_to_ground(uint32_t msgid, const char *pkt)
{
    const mavlink_msg_entry_t* entry = mavlink_get_msg_entry(msgid);

    if (entry == nullptr) {
        return;
    }

    for (uint8_t i=0; i<gcs().num_gcs(); i++) {
        GCS_MAVLINK &c = *gcs().chan(i);

        if (c.get_chan() == _chan) continue; //the gimbal is on this channel

        if (!c.is_active()) continue;
        if (entry->max_msg_len + GCS_MAVLINK::packet_overhead_chan(c.get_chan()) > c.get_uart()->txspace()) {
            continue; //no room on this channel
        }
        c.send_message(pkt, entry);
    }
}


//------------------------------------------------------
// interfaces to STorM32_MAVLink_class
//------------------------------------------------------

bool BP_Mount_STorM32_MAVLink::_tx_hasspace(const size_t size)
{
    if (_use_protocolv2) {
        return false;
    }

    // gladly, we can put it into one tunnel message
    // so check if it fits into the tunnel payload, and if there is space

    if ( size > MAVLINK_MSG_TUNNEL_FIELD_PAYLOAD_LEN ) return false;

    return HAVE_PAYLOAD_SPACE(_chan, TUNNEL);
}


size_t BP_Mount_STorM32_MAVLink::_write(const uint8_t* buffer, size_t size)
{
    if (_use_protocolv2) {
        return 0;
    }

    // the buffer holds the STorM32 command, which is now wrapped into a MAVLink tunnel message
    // copy it to a new buffer of proper size, and clear it with zeros
    // gladly, it fits completely into one tunnel message, so we get away with this simple method

    uint8_t payload[MAVLINK_MSG_TUNNEL_FIELD_PAYLOAD_LEN+1];
    memset(payload, 0, sizeof(payload));

    if( size > MAVLINK_MSG_TUNNEL_FIELD_PAYLOAD_LEN ) size = MAVLINK_MSG_TUNNEL_FIELD_PAYLOAD_LEN; //should not happen, but play it safe
    memcpy(payload, buffer, size);

    mavlink_msg_tunnel_send(
            _chan,
            _sysid,
            _compid,
            MAV_TUNNEL_PAYLOAD_TYPE_STORM32_CHANNEL2_IN,
            size,
            payload);

    return PAYLOAD_SIZE(_chan, TUNNEL); //we don't know the actual written length, so use this as dummy
}


bool BP_Mount_STorM32_MAVLink::is_normal_state(uint16_t state)
{
    return storm32_is_normalstate(state) ? true : false;
}


void BP_Mount_STorM32_MAVLink::send_cmd_storm32link_v2(void)
{
#if AP_AHRS_NAVEKF_AVAILABLE

    if (!_tx_hasspace(sizeof(tSTorM32LinkV2))) {
        return;
    }

    //from tests, 2018-02-10/11, I concluded
    // ahrs.healthy():     indicates Q is OK, ca. 15 secs, Q is doing a square dance before, so must wait for this
    // ahrs.initialised(): indicates vz is OK (vx,vy ate OK very early), ca. 30-35 secs
    // ekf_filter_status().flags.vert_vel: ca. 60-XXs, few secs after position_ok() and ca 30-XXs after GPS fix
    //                                     don't know what it really indicates

    //ahrs.get_filter_status(), I think, works only because AP_AHRS_TYPE = AP_AHRS_NavEKF
    // AP_AHRS doesn't have a get_filter_status() method
    // AP_InertialNav_NavEKF:get_filter_status() calls _ahrs_ekf.get_filter_status(status)
    // so I think s = copter.letmeget_ekf_filter_status() and ahrs.get_filter_status(s) should be identical !
    // in a test flight a check for equal never triggered => I assume these are indeed identical !

    // AP_Notify::instance()->flags.initialising: if at all when it is only very briefly at startup true
    // AP_Notify::instance()->armed seems to be identical to copter.letmeget_motors_armed() !!!
    // => copter.letmeget_motors_armed() can be avoided

    const AP_AHRS_NavEKF &ahrs = AP::ahrs_navekf();
    const AP_GPS &gps = AP::gps();
    const AP_Notify &notify = AP::notify();

    nav_filter_status nav_status;
    ahrs.get_filter_status(nav_status);

    uint8_t status = STORM32LINK_FCSTATUS_ISARDUPILOT;
    if (ahrs.healthy()) { status |= STORM32LINK_FCSTATUS_AP_AHRSHEALTHY; }
    if (ahrs.initialised()) { status |= STORM32LINK_FCSTATUS_AP_AHRSINITIALIZED; }
    if (nav_status.flags.horiz_vel) { status |= STORM32LINK_FCSTATUS_AP_NAVHORIZVEL; }
    if (gps.status() >= AP_GPS::GPS_OK_FIX_3D) { status |= STORM32LINK_FCSTATUS_AP_GPS3DFIX; }
    if (notify.flags.armed) { status |= STORM32LINK_FCSTATUS_AP_ARMED; }

    Quaternion quat;
    quat.from_rotation_matrix(ahrs.get_rotation_body_to_ned());

    Vector3f vel;
    // ahrs.get_velocity_NED(vel) returns a bool, so it's a good idea to consider it
    if (!ahrs.get_velocity_NED(vel)) { vel.x = vel.y = vel.z = 0.0f; }

    int16_t yawrate = 0;

    tSTorM32LinkV2 t;
    t.seq = _storm32link_seq; _storm32link_seq++; //this is not really used
    t.status = status;
    t.spare = 0;
    t.yawratecmd = yawrate;
    t.q0 = quat.q1;
    t.q1 = quat.q2;
    t.q2 = quat.q3;
    t.q3 = quat.q4;
    t.vx = vel.x;
    t.vy = vel.y;
    t.vz = vel.z;
    storm32_finalize_STorM32LinkV2(&t);

    _write( (uint8_t*)(&t), sizeof(tSTorM32LinkV2) );

#endif
}


void BP_Mount_STorM32_MAVLink::send_cmd_sethomelocation(void)
{
    if (!_tx_hasspace(sizeof(tSTorM32CmdSetHomeTargetLocation))) {
        return;
    }

    uint16_t status = 0; //= LOCATION_INVALID
    struct Location location = {};

    const AP_AHRS &ahrs = AP::ahrs();

    if (ahrs.get_position(location)) {
        status = 0x0001; //= LOCATION_VALID
    }

    tSTorM32CmdSetHomeTargetLocation t;
    t.latitude = location.lat;
    t.longitude = location.lng;
    t.altitude = location.alt;
    t.status = status;
    storm32_finalize_CmdSetHomeLocation(&t);

    _write( (uint8_t*)(&t), sizeof(tSTorM32CmdSetHomeTargetLocation) );
}


void BP_Mount_STorM32_MAVLink::send_cmd_settargetlocation(void)
{
    if (!_tx_hasspace(sizeof(tSTorM32CmdSetHomeTargetLocation))) {
        return;
    }

    uint16_t status = 0; //= LOCATION_INVALID
    struct Location location = {};

    tSTorM32CmdSetHomeTargetLocation t;
    t.latitude = location.lat;
    t.longitude = location.lng;
    t.altitude = location.alt;
    t.status = status;
    storm32_finalize_CmdSetTargetLocation(&t);

    _write( (uint8_t*)(&t), sizeof(tSTorM32CmdSetHomeTargetLocation) );
}







//about sending to other components

// we also could use send_message(msgid, pkt), but sadly it isn't doing a size check

/* what about send_to_components(msgid, pkt, pkt_len), does call routing class
mavlink_command_long_t cmd_msg{};
cmd_msg.command = MAV_CMD_POWER_OFF_INITIATED;
cmd_msg.param1 = i+1;
GCS_MAVLINK::send_to_components(MAVLINK_MSG_ID_COMMAND_LONG, (char*)&cmd_msg, sizeof(cmd_msg));
*/

// this doesn't send it to the GCS!!!
//GCS_MAVLINK::send_to_components(MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS, (char*)&msg, sizeof(msg));

/* like void GCS::send_to_active_channels(uint32_t msgid, const char *pkt) but without active channels check
    const mavlink_msg_entry_t *entry = mavlink_get_msg_entry(msgid);
    if (entry == nullptr) return;
    for (uint8_t i=0; i<num_gcs(); i++) {
        GCS_MAVLINK &c = *chan(i);
        if (entry->max_msg_len + c.packet_overhead() > c.get_uart()->txspace()) continue; // no room on this channel
        c.send_message(pkt, entry);
    }
*/
//WHAT exactly is a active channel ????

//send_to_channels(MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS, (const char*)&msg);

//tested it: sends to all, GCS and gimbal
//gcs().send_to_active_channels(MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS, (const char*)&msg);




// I believe the three alternatives essentially call the same underlying function
// but the first method is preferred since it deactivates the normal streaming, i.e. is really 1to1

//alternative ??
// const mavlink_button_change_t packet{
//        time_boot_ms: AP_HAL::millis(),
//        last_change_ms: uint32_t(last_change_time_ms),
//        state: last_mask
// };
// gcs().send_to_active_channels(MAVLINK_MSG_ID_BUTTON_CHANGE, (const char *)&packet);
//?? does this also send to the gimbal??? YES, it does

//gcs().send_to_active_channels(MAVLINK_MSG_ID_MOUNT_STATUS, (const char*)&msg);

/* // the "same" without is_active check, but is dangerous as there is no size check
const mavlink_msg_entry_t *entry = mavlink_get_msg_entry(MAVLINK_MSG_ID_MOUNT_STATUS);
if (entry != nullptr) {
for (uint8_t i=0; i<gcs().num_gcs(); i++) gcs().chan(i)->send_message((const char*)&packet, entry);
} */

//alternative ??
// GCS_MAVLINK::try_send_message(MSG_MOUNT_STATUS);
//gcs().send_message(MSG_MOUNT_STATUS);

//alternative ??
// GCS_MAVLINK::send_mount_status();
//for (uint8_t i=0; i<gcs().num_gcs(); i++) gcs().chan(i)->send_mount_status();
