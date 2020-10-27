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
#include <AP_RTC/AP_RTC.h>
#include "BP_Mount_STorM32_MAVLink.h"

extern const AP_HAL::HAL& hal;


/*
0:  old behavior
    sends DO_MOUNT_CONTROL, DO_MOUNT_CONFIGURE for control, sends tunnel for STorM32-Link
8:  like 0, but using new gimbal device messages
    sends GIMBAL_DEVICE_SET_ATTITUDE for control, sends AUTOPILOT_STATE_FOR_GIMBAL for STorM32-Link
    this mode is for testing, not for regular use
1:  for gimbal manager
    mode for when there is a gimbal manager in the system, e.g. on the STorM32 or on the companion
    uses new gimbal manager messages
    listens to DO_MOUNT_CONTROL, DO_MOUNT_CONFIGURE for the mode, RC_MAVLINK corresponds to quickshots etc.
    is not standard conform
2:  only streaming
    only sends out RC_CHANNLES, AUTOPILOT_STATE_FOR_GIMBAL for STorM32-Link
    this mode could in principle be replaced by asking for the streams, but since AP isn't streaming reliably we don't

 in all modes sends MOUNT_STATUS, so that "old" things like MP etc can see the gimbal orientation

128: prearming is always true
*/
/*
 also sends SYSTEM_TIME to gimbal
*/
/*
TODO:
- we probably want to slow the emission of of

*/


//******************************************************
// Quaternion & Euler for Gimbal
//******************************************************
// we do not use NED (roll-pitch-yaw) to convert received quaternion to Euler angles and vice versa
// we use pitch-roll-yaw instead
// when the roll angle is zero, both are equivalent, this should be the majority of cases anyhow
// also, for most gimbals pitch-roll-yaw is appropriate
// the issue with NED is the gimbal lock at pitch +-90�, but pitch +-90� is a common operation point for gimbals
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

    if (_state._zflags & 0x80) {
        _prearmchecks_ok = false; //enable prearm checks
    }

    _use_protocolv2 = false;    //true means mode 1, 2, 8
    _for_gimbalmanager = false; //true means mode 1
    _sendonly = false;          //true means mode 2
#if USE_GIMBAL_ZFLAGS
    if (_state._zflags & 0x0F) {
        _use_protocolv2 = true;
        if (_state._zflags & 0x01) _for_gimbalmanager = true;
        if (_state._zflags & 0x02) _sendonly = true;
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

    //we originally wanted to slow down everything to 100 Hz,
    // so that updates are at 20 Hz, especially for STorM32-Link
    // however, sadly, plane runs at 50 Hz only, so we update at 25 Hz and 12.5 Hz respectively
    // not soo nice
    // not clear what it does for STorM32Link, probably not too bad, maybe even good

    #define PERIOD_US   20000 //20 ms = 50 Hz

    uint32_t now_us = AP_HAL::micros();
    if ((now_us - _task_time_last) >= PERIOD_US) {
        //_task_time_last = now_us;
        //this gives MUCH higher precision!!!:
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
                if (_sendonly) break;
                if (_use_protocolv2) {
                    set_target_angles();
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
    //it doesn't matter if not _initalised, it will then send out zeros

    //this is called by GCS_MAVLINK_Copter::try_send_message() which in turn is called by the streamer
    // so, it is send out with 2Hz, by the SR0_EXTRA3 setting, which per default is 2Hz only

    //mavlink_msg_mount_status_send(chan, 0, 0, _status.pitch_deg*100.0f, _status.roll_deg*100.0f, _status.yaw_deg*100.0f);
}


// handle_msg - allows to process messages received from gimbal
void BP_Mount_STorM32_MAVLink::handle_msg(const mavlink_message_t &msg)
{
    if (!_initialised) {
        return;
    }

    //TODO: should we check here msg.sysid if it is for us? sounds good, but could also cause issues, so not for now
    //TODO: if we capture a COMMAND_LONG here, what happens with COMMAND_ACK outside of here??
    //comment: we do not bother with sending/handling CMD_ACK momentarily
    // this is dirty, should go to GCS_MAVLINK, GCS, etc., but we don't want to pollute and infect, hence here in dirty ways

    if ((msg.sysid != _sysid) || (msg.compid != _compid)) { //this msg is not from our gimbal
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

    if (send_mountstatus) {
        //forward to other links, to make MissionPlanner and alike happy
        send_mount_status_to_channels();
    }
}


bool BP_Mount_STorM32_MAVLink::pre_arm_checks(void)
{
    return _prearmchecks_ok;
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
                //this is not really needed as it's ignored in send_target_angles_to_gimbal() and recenter is triggered, but hey, can't harm
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
                //this is not really needed as it's ignored in send_target_angles_to_gimbal() and recenter is triggered, but hey, can't harm
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
            if (AP::gps().status() >= AP_GPS::GPS_OK_FIX_2D) {
                calc_angle_to_location(_state._roi_target, _angle_ef_target_rad, true, true);
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


// is called by task loop at 20 Hz
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
// private functions, storm32 gimbal protocol v2
//------------------------------------------------------

// is called by task loop at 20 Hz
// finally sends out target angles
// assumes that old set_target_angles() is called

void BP_Mount_STorM32_MAVLink::send_target_angles_to_gimbal_v2(void)
{
uint16_t gimbaldevice_flags = MAV_STORM32_GIMBAL_DEVICE_FLAGS_ROLL_LOCK | MAV_STORM32_GIMBAL_DEVICE_FLAGS_PITCH_LOCK;

    if (_target.mode == MAV_MOUNT_MODE_RETRACT) {
        gimbaldevice_flags = MAV_STORM32_GIMBAL_DEVICE_FLAGS_RETRACT;
    }
    if (_target.mode == MAV_MOUNT_MODE_NEUTRAL) {
        gimbaldevice_flags = MAV_STORM32_GIMBAL_DEVICE_FLAGS_NEUTRAL;
    }

    if (_for_gimbalmanager) {
        uint16_t gimbalmanager_flags;
        //we do not attempt to claim supervision nor activity, we thus leave it to other components, e.g. a gcs, to set this
        gimbalmanager_flags = 0;
        send_storm32_gimbal_manager_control_to_gimbal(_target.roll_deg, _target.pitch_deg, _target.yaw_deg, gimbaldevice_flags, gimbalmanager_flags);
    } else {
        send_storm32_gimbal_device_control_to_gimbal(_target.roll_deg, _target.pitch_deg, _target.yaw_deg, gimbaldevice_flags);
    }
}


//------------------------------------------------------
// MAVLink send functions
//------------------------------------------------------

void BP_Mount_STorM32_MAVLink::send_mount_status_to_channels(void)
{
    // space is checked by send_to_channels()

    mavlink_mount_status_t msg = {
        pointing_a : (int32_t)(_status.pitch_deg*100.0f),
        pointing_b : (int32_t)(_status.roll_deg*100.0f),
        pointing_c : (int32_t)(_status.yaw_deg*100.0f),
        target_system : 0,
        target_component : 0 };

    send_to_channels(MAVLINK_MSG_ID_MOUNT_STATUS, (const char*)&msg, true);
}


void BP_Mount_STorM32_MAVLink::send_cmd_do_mount_control_to_gimbal(float roll_deg, float pitch_deg, float yaw_deg, enum MAV_MOUNT_MODE mode)
{
    if (!HAVE_PAYLOAD_SPACE(_chan, COMMAND_LONG)) {
        return;
    }

    //convert from ArduPilot to STorM32 convention
    // this need correction p:-1,r:+1,y:-1
    pitch_deg = -pitch_deg;
    yaw_deg = -yaw_deg;

    //this is send out with _sysid, _compid as target
    // so it can be differentiated by the STorM32 controller from routed commands/messages by looking at source and target
    mavlink_msg_command_long_send(
        _chan,
        _sysid,
        _compid,
        MAV_CMD_DO_MOUNT_CONTROL,
        0,        // confirmation of zero means this is the first time this message has been sent
        pitch_deg,
        roll_deg,
        yaw_deg,
        0, 0, 0,  // param4 ~ param6 unused
        mode);
}


void BP_Mount_STorM32_MAVLink::send_rc_channels_to_gimbal(void)
{
    if (!HAVE_PAYLOAD_SPACE(_chan, RC_CHANNELS)) {
        return;
    }

    //rc().channel(ch)->get_radio_in() or RC_Channels::get_radio_in(ch) and so on
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
    AP::rtc().get_utc_usec(time_unix); // may fail, leaving time_unix at 0

    if (!time_unix) return; // no unix time available, so no reason to send

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
    float q[4] = { quat.q1, quat.q2, quat.q3, quat.q4 }; //figure out how to do it correctly

    Vector3f vel;
    // ahrs.get_velocity_NED(vel) returns a bool, so it's a good idea to consider it
    if (!ahrs.get_velocity_NED(vel)) { vel.x = vel.y = vel.z = 0.0f; }

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
        0, // uint32_t q_estimated_delay_us,
        vel.x, vel.y, vel.z,
        0, // uint32_t v_estimated_delay_us,
        yawrate,
        _estimator_status, _landed_state);
        //uint64_t time_boot_us, const float *q, uint32_t q_estimated_delay_us,
        //float vx, float vy, float vz, uint32_t v_estimated_delay_us,
        //float feed_forward_angular_velocity_z, uint16_t estimator_status, uint8_t landed_state)
}


//the interface is similar to that of do_mount_control, to make it simpler for the moment, so we internally convert
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


//the interface is similar to that of do_mount_control, to make it simpler for the moment, so we internally convert
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
        _compid, //gimbal_device_id
        MAV_STORM32_GIMBAL_MANAGER_CLIENT_AUTOPILOT,
        device_flags, manager_flags,
        q,
        NAN, NAN, NAN);
        //uint8_t gimbal_device_id, uint8_t client, uint16_t device_flags, uint16_t manager_flags,
        //const float *q, float angular_velocity_x, float angular_velocity_y, float angular_velocity_z)
}


//------------------------------------------------------
// MAVLink send helper
//------------------------------------------------------

// this is essentially GCS::send_to_active_channels(uint32_t msgid, const char *pkt)
// but allows us to excempt the gimbal channel
void BP_Mount_STorM32_MAVLink::send_to_channels(uint32_t msgid, const char *pkt, bool except_gimbal)
{
    const mavlink_msg_entry_t* entry = mavlink_get_msg_entry(msgid);

    if (entry == nullptr) {
        return;
    }

    for (uint8_t i=0; i<gcs().num_gcs(); i++) {
        GCS_MAVLINK &c = *gcs().chan(i);

        if (except_gimbal && (c.get_chan() == _chan)) continue;

        if (!c.is_active()) continue;
        if (entry->max_msg_len + GCS_MAVLINK::packet_overhead_chan(c.get_chan()) > c.get_uart()->txspace()) {
            continue; // no room on this channel
        }
        c.send_message(pkt, entry);
    }
}


//------------------------------------------------------
// discovery functions
//------------------------------------------------------

void BP_Mount_STorM32_MAVLink::find_gimbal(void)
{
#if FIND_GIMBAL_MAX_SEARCH_TIME_MS
    uint32_t now_ms = AP_HAL::millis();

    if (now_ms > FIND_GIMBAL_MAX_SEARCH_TIME_MS) {
        _initialised = false; //should be already false, but it can't hurt to ensure that
        return;
    }
#else
    const AP_Notify &notify = AP::notify();
    if (notify.flags.armed) {
        return; //do not search if armed
    }
#endif

    //TODO: this I think only allows one MAVLink gimbal
    if (GCS_MAVLINK::find_by_mavtype(MAV_TYPE_GIMBAL, _sysid, _compid, _chan)) {
        _initialised = true;
    }

    //proposal:
    // change this function to allow an index, like find_by_mavtype(index, ....)
    // we then can call it repeatedly until it returns false, whereby increasing index as 0,1,...
    // we then can define that the first mavlink mount is that with lowest ID, and so on
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

    //the buffer holds the STorM32 command, which is now wrapped into a MAVLink tunnel message
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
