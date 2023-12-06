//*****************************************************
//OW
// (c) olliw, www.olliw.eu, GPL3
// STorM32 mount backend class
//*****************************************************

#include "BP_Mount_STorM32_MAVLink.h"

#if HAL_MOUNT_BP_STORM32_MAVLINK_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Common/AP_Common.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_RTC/AP_RTC.h>
#include <AP_Notify/AP_Notify.h>
#include <AP_Logger/AP_Logger.h>
#include <RC_Channel/RC_Channel.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;


//******************************************************
// Quaternion & Euler for Gimbal
//******************************************************
// For most gimbals it is inappropriate to use NED (roll-pitch-yaw) to convert received
// quaternion to Euler angles and vice versa. For most gimbals pitch-roll-yaw is
// appropriate. When the roll angle is zero, both are equivalent, which should be the
// majority of cases. The issue with NED is the gimbal lock at pitch +-90 deg, but pitch
// +-90 deg is a common operation point for gimbals.
// The angles in this driver are thus pitch-roll-yaw Euler.

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
// BP_Mount_STorM32_MAVLink, main class
//******************************************************

// for reasons I really don't understand, calling them as log methods didn't work
// the MTC got mixed up with MTH, i.e., no MTC0 was there but a MTH0 with the MTC params...
// so done as macro in-place, which works fine
// units:
// { '-', "" }, // no units e.g. Pi, or a string
// { 'd', "deg" },           // of the angular variety, -180 to 180
// { 'n', "m/s" }, // metres per second
// { 's', "s" }, // seconds
// { 'E', "rad/s" },         // radians per second
// { 'k', "deg/s" },         // degrees per second. Degrees are NOT SI, but is some situations more user-friendly than radians
// scales:
// { '-', 0 },
// { 'F', 1e-6 },
// data types:
// B   : uint8_t
// H   : uint16_t
// I   : uint32_t
// f   : float

#define BP_LOG(m,h,...) if(_should_log){char logn[5] = m; logn[3] += _instance; AP::logger().Write(logn, h, AP_HAL::micros64(), __VA_ARGS__);}


// log incoming GIMBAL_DEVICE_ATTITUDE_STATUS
#define BP_LOG_MTS_ATTITUDESTATUS_HEADER \
        "TimeUS,Roll,Pitch,Yaw,dYaw,YawAhrs,Flags,FailFlags", \
        "sddddd--", \
        "F-------", \
        "QfffffHH"

// log outgoing GIMBAL_DEVICE_SET_ATTITUDE, STORM32_GIMBAL_MANAGER_CONTROL
#define BP_LOG_MTC_GIMBALCONTROL_HEADER \
        "TimeUS,Type,Roll,Pitch,Yaw,GDFlags,GMFlags,TMode,QMode", \
        "s-ddd----", \
        "F--------", \
        "QBfffHHBB"

// log outgoing AUTOPILOT_STATE_FOR_GIMBAL_DEVICE
#define BP_LOG_MTL_AUTOPILOTSTATE_HEADER \
        "TimeUS,q1,q2,q3,q4,vx,vy,vz,wz,YawRate,Est,Land,NavEst,NEst2,dtUS", \
        "s----nnnkk----s", \
        "F-------------F", \
        "QfffffffffHBHHI"

// log outgoing AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_EXT
#define BP_LOG_MTLE_AUTOPILOTSTATEEXT_HEADER \
        "TimeUS,windx,windy,corrangle,vh,wh,gh,ah", \
        "snnddddd", \
        "F-------", \
        "Qfffffff"


//******************************************************
// BP_Mount_STorM32_MAVLink, main class
//******************************************************

void BP_Mount_STorM32_MAVLink::init()
{
    AP_Mount_Backend::init();

    _mode = MAV_MOUNT_MODE_RC_TARGETING; // irrelevant, will be later set to default in frontend init()

    _flags_from_manager = UINT32_MAX; // the UINT32_MAX is important!

    _current_angles = {0.0f, 0.0f, 0.0f, NAN}; // the NAN is important!

    _should_log = true; // for now do always log
}


// called by all vehicles with 50 Hz, using the scheduler
// several vehicles do not support fast_update(), so let's go with this
// priority of update() not very high, so no idea how reliable that is, may be not so good
// STorM32-Link wants 25 Hz, so we update at 25 Hz and 12.5 Hz respectively
void BP_Mount_STorM32_MAVLink::update()
{
    update_target_angles(); // update at 50 Hz (RC_TARGETING handling assumes this)

    switch (_task_counter) {
        case TASK_SLOT0:
        case TASK_SLOT2:
            if (_compid) { // we send it as soon as we have found the gimbal
                send_autopilot_state_for_gimbal_device();
            }
            break;

        case TASK_SLOT1:
            if (_initialised) { // we do it when the startup sequence has been fully completed
                send_target_angles();
            }
            break;

        case TASK_SLOT3:
            if (_compid) { // we send it as soon as we have found the gimbal
                if (!_got_radio_rc_channels) { // don't send it if we have seen RADIO_RC_CHANNELS messages
                    send_rc_channels();
                }
            }
            break;
    }

    _task_counter++;
    if (_task_counter > TASK_SLOT3) _task_counter = TASK_SLOT0;

    update_send_banner();

    if (!_initialised) {
        find_gimbal();
        return;
    }

    uint32_t tnow_ms = AP_HAL::millis();

    if ((tnow_ms - _checks_tlast_ms) >= 1000) {
        _checks_tlast_ms = tnow_ms;
        update_checks();
    }

    if ((tnow_ms - _send_system_time_tlast_ms) >= 5000) { // every 5 sec is really plenty
        _send_system_time_tlast_ms = tnow_ms;
        send_system_time();
    }
}


//------------------------------------------------------
// Mode handling and targeting functions
//------------------------------------------------------

// flags coming from gimbal manager messages and commands
// Called from AP_Mount's gimbal manager handlers, but only if source is in control.
// The front-end calls set_angle_target() and/or set_rate_target() depending
// on the info in the gimbal manager messages.
// Return false to skip this angle/rate processing.
// AP's gimbal manager should never ask to do things which the gimbal device
// can't do. Isn't so but we know well our gimbal device.
bool BP_Mount_STorM32_MAVLink::handle_gimbal_manager_flags(uint32_t flags)
{
    // check flags for change to RETRACT
    if (flags & GIMBAL_MANAGER_FLAGS_RETRACT) {
        set_mode(MAV_MOUNT_MODE_RETRACT);
    } else
    // check flags for change to NEUTRAL
    if (flags & GIMBAL_MANAGER_FLAGS_NEUTRAL) {
        set_mode(MAV_MOUNT_MODE_NEUTRAL);
    }

    _flags_from_manager = flags;

    update_gimbal_device_flags();

    // driver currently does not support yaw LOCK
    // front-end is digesting GIMBAL_MANAGER_FLAGS_YAW_LOCK to determine yaw_is_earth_frame
    // we could make it to modify the flag, but for moment let's be happy.
    if (flags & GIMBAL_MANAGER_FLAGS_YAW_LOCK) {
        return false; // don't accept angle/rate setting
    }

    // STorM32 expects one of them to be set, otherwise it rejects
    if (!(flags & GIMBAL_MANAGER_FLAGS_YAW_IN_VEHICLE_FRAME) && !(flags & GIMBAL_MANAGER_FLAGS_YAW_IN_EARTH_FRAME)) {
        return false; // don't accept angle/rate setting
    }

    // STorM32 currently only supports GIMBAL_MANAGER_FLAGS_YAW_IN_VEHICLE_FRAME
    if (flags & GIMBAL_MANAGER_FLAGS_YAW_IN_EARTH_FRAME) {
        return false; // don't accept angle/rate setting
    }

    return true; // accept angle/rate setting
}


void BP_Mount_STorM32_MAVLink::update_gimbal_device_flags()
{
    _flags_for_gimbal = 0;

    switch (get_mode()) {
        case MAV_MOUNT_MODE_RETRACT:
            _flags_for_gimbal |= GIMBAL_DEVICE_FLAGS_RETRACT;
            break;
        case MAV_MOUNT_MODE_NEUTRAL:
            _flags_for_gimbal |= GIMBAL_DEVICE_FLAGS_NEUTRAL;
            break;
        default:
            break;
    }

    // account for gimbal manager flags
    if (_flags_from_manager != UINT32_MAX) {
        if (_flags_from_manager & GIMBAL_MANAGER_FLAGS_RC_EXCLUSIVE) _flags_for_gimbal |= GIMBAL_DEVICE_FLAGS_RC_EXCLUSIVE;
        if (_flags_from_manager & GIMBAL_MANAGER_FLAGS_RC_MIXED) _flags_for_gimbal |= GIMBAL_DEVICE_FLAGS_RC_MIXED;
    } else {
        // for as long as no gimbal manager message has been sent to the fc, enable rc mixed.
        // Should avoid user confusion when choosing the gimbal device operation mode.
        _flags_for_gimbal |= GIMBAL_DEVICE_FLAGS_RC_MIXED;
    }

    // driver currently does not support pitch,roll follow, only pitch,roll lock
    _flags_for_gimbal |= GIMBAL_DEVICE_FLAGS_ROLL_LOCK | GIMBAL_DEVICE_FLAGS_PITCH_LOCK;

    // driver currently does not support yaw lock, only yaw follow
//    if (_is_yaw_lock) _flags_for_gimbal |= GIMBAL_DEVICE_FLAGS_YAW_LOCK;

    // set either YAW_IN_VEHICLE_FRAME or YAW_IN_EARTH_FRAME, to indicate new message format, STorM32 will reject otherwise
    _flags_for_gimbal |= GIMBAL_DEVICE_FLAGS_YAW_IN_VEHICLE_FRAME;
}


void BP_Mount_STorM32_MAVLink::send_target_angles()
{
    // just send stupidly at 12.5 Hz, no check if get_target_angles() made a change

    update_gimbal_device_flags();

    if (mnt_target.target_type == MountTargetType::RATE) {
        // we ignore it. We may think to just send angle_rad, but if yaw is earth frame
        // this could result in pretty strange behavior. So better ignore.
        // Should happen only in MAV_MOUNT_MODE_RC_TARGETING, so no need to test for this
        // explicitly.
        return;
    }

    if (_protocol == Protocol::GIMBAL_DEVICE) {
        send_gimbal_device_set_attitude();
    } else {
        send_cmd_do_mount_control();
    }
}


// update_angle_target_from_rate() assumes a 50hz update rate!
// TODO: one should allow angles outside of +-PI, to go shortest path in case of turn around
void BP_Mount_STorM32_MAVLink::update_target_angles()
{
    // update based on mount mode
    switch ((uint8_t)get_mode()) {

        // move mount to a "retracted" position
        // -> ANGLE
        case MAV_MOUNT_MODE_RETRACT: {
            const Vector3f &target = _params.retract_angles.get();
            mnt_target.target_type = MountTargetType::ANGLE;
            mnt_target.angle_rad.set(target*DEG_TO_RAD, false);
            mnt_target_loc_valid = false;
            break;
        }

        // move mount to a neutral position, typically pointing forward
        // -> ANGLE
        case MAV_MOUNT_MODE_NEUTRAL: {
            const Vector3f &target = _params.neutral_angles.get();
            mnt_target.target_type = MountTargetType::ANGLE;
            mnt_target.angle_rad.set(target*DEG_TO_RAD, false);
            mnt_target_loc_valid = false;
            break;
        }

        // point to the angles given by a mavlink message or mission command
        // -> ANGLE
        case MAV_MOUNT_MODE_MAVLINK_TARGETING:
            // mnt_target should have already been populated by set_angle_target() or set_rate_target().
            // SToRM32 doesn't support rate, so update target angle from rate if necessary.
            if (mnt_target.target_type == MountTargetType::RATE) {
                update_angle_target_from_rate(mnt_target.rate_rads, mnt_target.angle_rad);
                mnt_target_loc_valid = false;
            }
            break;

        // RC radio manual angle control, but with stabilization from the AHRS
        // update targets using pilot's RC inputs
        // -> can be RATE, will be ignored
        case MAV_MOUNT_MODE_RC_TARGETING: {
            MountTarget rc_target;
            get_rc_target(mnt_target.target_type, rc_target);
            switch (mnt_target.target_type) {
            case MountTargetType::ANGLE:
                mnt_target.angle_rad = rc_target;
                break;
            case MountTargetType::RATE:
                mnt_target.rate_rads = rc_target;
                break;
            }
            mnt_target_loc_valid = false;
            break;
        }

        // point mount to a GPS point given by the mission planner
        // ATTENTION: angle_rad.yaw_is_ef = true
        // -> ANGLE
        case MAV_MOUNT_MODE_GPS_POINT:
            if (get_angle_target_to_roi(mnt_target.angle_rad)) {
                mnt_target.target_type = MountTargetType::ANGLE;
                mnt_target_loc_valid = true;
                mnt_target_loc = _roi_target;
            }
            break;

        // point mount to Home location
        // ATTENTION: angle_rad.yaw_is_ef = true
        // -> ANGLE
        case MAV_MOUNT_MODE_HOME_LOCATION:
            if (get_angle_target_to_home(mnt_target.angle_rad)) {
                mnt_target.target_type = MountTargetType::ANGLE;
                mnt_target_loc_valid = true;
                mnt_target_loc = AP::ahrs().get_home();
            }
            break;

        // point mount to another vehicle
        // ATTENTION: angle_rad.yaw_is_ef = true
        // -> ANGLE
        case MAV_MOUNT_MODE_SYSID_TARGET:
            if (get_angle_target_to_sysid(mnt_target.angle_rad)) {
                mnt_target.target_type = MountTargetType::ANGLE;
                mnt_target_loc_valid = true;
                mnt_target_loc = _target_sysid_location;
            }
            break;

        default:
            // we do not know this mode so do nothing
            break;
    }

    if (_script_angles.control) {
        // point mount to where a script wants it to point
        // -> ANGLE
        mnt_target.target_type = MountTargetType::ANGLE;
        mnt_target.angle_rad.roll = _script_angles.roll;
        mnt_target.angle_rad.pitch = _script_angles.pitch;
        mnt_target.angle_rad.yaw = _script_angles.yaw_bf;
        mnt_target.angle_rad.yaw_is_ef = false;
        mnt_target_loc_valid = false;
    }

    // account for range limits
    // only do the yaw axis (should be done by STorM32 supervisor, but doesn't hurt)

    if (_got_device_info) return;
    if (_device_info.cap_flags & GIMBAL_DEVICE_CAP_FLAGS_SUPPORTS_INFINITE_YAW) return;
    if (isnan(_device_info.yaw_min) || isnan(_device_info.yaw_max)) return;

    // I believe currently angles are inside +-PI, no wrapPI needed
    // TODO: if copter, copter might yaw by what the gimbal can't do

    if (mnt_target.angle_rad.yaw < _device_info.yaw_min) mnt_target.angle_rad.yaw = _device_info.yaw_min;
    if (mnt_target.angle_rad.yaw > _device_info.yaw_max) mnt_target.angle_rad.yaw = _device_info.yaw_max;
}


//------------------------------------------------------
// Gimbal and protocol discovery
//------------------------------------------------------

void BP_Mount_STorM32_MAVLink::find_gimbal()
{
    // search for gimbal only until vehicle is armed
    if (hal.util->get_soft_armed()) {
        return;
    }

    uint32_t tnow_ms = AP_HAL::millis();

    // search for gimbal in routing table
    if (!_compid) {
        // we expect that instance 0 has compid = MAV_COMP_ID_GIMBAL, instance 1 has compid = MAV_COMP_ID_GIMBAL2, etc
        uint8_t compid = (_instance == 0) ? MAV_COMP_ID_GIMBAL : MAV_COMP_ID_GIMBAL2 + (_instance - 1);
        bool found = GCS_MAVLINK::find_by_mavtype_and_compid(MAV_TYPE_GIMBAL, compid, _sysid, _chan);
        if (!found || (_sysid != mavlink_system.sysid)) {
            // have not yet found a gimbal so return
            return;
        }

        _compid = compid;
        _request_device_info_tlast_ms = (tnow_ms < 900) ? 0 : tnow_ms - 900; // start sending requests in 100 ms
    }

    // request GIMBAL_DEVICE_INFORMATION
    // STorM32 gives this also for mount mode
    if (!_got_device_info) {
        if (tnow_ms - _request_device_info_tlast_ms > 1000) {
            _request_device_info_tlast_ms = tnow_ms;
            send_cmd_request_gimbal_device_information();
        }
        return;
    }

    // we don't know yet what we should do
    if (_protocol == Protocol::UNDEFINED) {
        return;
    }

    _initialised = true;
}


void BP_Mount_STorM32_MAVLink::determine_protocol(const mavlink_message_t &msg)
{
    if (msg.sysid != _sysid || msg.compid != _compid) { // this msg is not from our gimbal
        return;
    }

    switch (msg.msgid) {
        case MAVLINK_MSG_ID_MOUNT_STATUS:
            _protocol = Protocol::MOUNT;
            break;
            case MAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS:
            _protocol = Protocol::GIMBAL_DEVICE;
            break;
    }
}


//------------------------------------------------------
// Gimbal control flags
//------------------------------------------------------

bool BP_Mount_STorM32_MAVLink::has_roll_control() const
{
    if (_protocol == Protocol::GIMBAL_DEVICE) {
        return (_device_info.cap_flags & GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_AXIS);
    }
    if (_protocol == Protocol::MOUNT) {
        return (_params.roll_angle_min < _params.roll_angle_max);
    }
    return false;
}


bool BP_Mount_STorM32_MAVLink::has_pitch_control() const
{
    if (_protocol == Protocol::GIMBAL_DEVICE) {
        return (_device_info.cap_flags & GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_AXIS);
    }
    if (_protocol == Protocol::MOUNT) {
        return (_params.pitch_angle_min < _params.pitch_angle_max);
    }
    return false;
}


bool BP_Mount_STorM32_MAVLink::has_pan_control() const
{
    if (_protocol == Protocol::GIMBAL_DEVICE) {
        return (_device_info.cap_flags & GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_AXIS);
    }
    if (_protocol == Protocol::MOUNT) {
        return yaw_range_valid();
    }
    return false;
}


//------------------------------------------------------
// Gimbal attitude and rate
//------------------------------------------------------

bool BP_Mount_STorM32_MAVLink::get_attitude_quaternion(Quaternion &att_quat)
{
    if (!_initialised) {
        return false;
    }

    // we set roll to zero since wrong Euler's
    att_quat.from_euler(0.0f, _current_angles.pitch, _current_angles.yaw_bf);

    return true;
}


bool BP_Mount_STorM32_MAVLink::get_angular_velocity(Vector3f& rates)
{
    if (!_initialised) {
        return false;
    }

    if (isnan(_current_omega.x) || isnan(_current_omega.y) || isnan(_current_omega.z)) return false;

    rates.x = _current_omega.x;
    rates.y = _current_omega.y;
    rates.z = _current_omega.z;

    return true;
}


//------------------------------------------------------
// MAVLink handle functions
//------------------------------------------------------

void BP_Mount_STorM32_MAVLink::handle_gimbal_device_information(const mavlink_message_t &msg)
{
    // this msg is not from our gimbal
    if (msg.sysid != _sysid || msg.compid != _compid) {
        return;
    }

    mavlink_msg_gimbal_device_information_decode(&msg, &_device_info);

    // we could check here for sanity of _device_info.gimbal_device_id, but let's just be happy

    // correct parameters from gimbal information
    if (!isnan(_device_info.roll_min) && !isnan(_device_info.roll_max)) {
        if (degrees(_device_info.roll_min) > _params.roll_angle_min) _params.roll_angle_min.set(degrees(_device_info.roll_min));
        if (degrees(_device_info.roll_max) < _params.roll_angle_max) _params.roll_angle_max.set(degrees(_device_info.roll_max));
    }
    if (!isnan(_device_info.pitch_min) && !isnan(_device_info.pitch_max)) {
        if (degrees(_device_info.pitch_min) > _params.pitch_angle_min) _params.pitch_angle_min.set(degrees(_device_info.pitch_min));
        if (degrees(_device_info.pitch_max) < _params.pitch_angle_max) _params.pitch_angle_max.set(degrees(_device_info.pitch_max));
    }
    if (!isnan(_device_info.yaw_min) && !isnan(_device_info.yaw_max)) {
        if (degrees(_device_info.yaw_min) > _params.yaw_angle_min) _params.yaw_angle_min.set(degrees(_device_info.yaw_min));
        if (degrees(_device_info.yaw_max) < _params.yaw_angle_max) _params.yaw_angle_max.set(degrees(_device_info.yaw_max));
    }

    // mark it as having been found
    _got_device_info = true;

    // display gimbal info to user
    send_banner();
}


void BP_Mount_STorM32_MAVLink::handle_gimbal_device_attitude_status(const mavlink_message_t &msg)
{
    if (_protocol != Protocol::GIMBAL_DEVICE) {
        return;
    }

    // this msg is not from our gimbal
    if (msg.sysid != _sysid || msg.compid != _compid) {
        return;
    }

    mavlink_gimbal_device_attitude_status_t payload;
    mavlink_msg_gimbal_device_attitude_status_decode(&msg, &payload);

    // we could check here for sanity of _device_info.gimbal_device_id, but let's just be happy

    // get relevant data

    _device_status.received_flags = payload.flags;
    // TODO: handle case when received device_flags are not equal to those we send, set with update_gimbal_device_flags()

    _device_status.received_failure_flags = payload.failure_flags;

    // used for health check
    _device_status.received_tlast_ms = AP_HAL::millis();

    // Euler angles
    GimbalQuaternion q(payload.q[0], payload.q[1], payload.q[2], payload.q[3]);
    q.to_gimbal_euler(_current_angles.roll, _current_angles.pitch, _current_angles.yaw_bf);

    _current_angles.delta_yaw = payload.delta_yaw;

    _current_omega.x = payload.angular_velocity_x;
    _current_omega.y = payload.angular_velocity_y;
    _current_omega.z = payload.angular_velocity_z;

    // logging

    BP_LOG("MTS0", BP_LOG_MTS_ATTITUDESTATUS_HEADER,
        degrees(_current_angles.roll),
        degrees(_current_angles.pitch),
        degrees(_current_angles.yaw_bf),
        degrees(_current_angles.delta_yaw),
        degrees(AP::ahrs().yaw),
        payload.flags,
        payload.failure_flags);
}


void BP_Mount_STorM32_MAVLink::handle_message_extra(const mavlink_message_t &msg)
{
    if (_protocol == Protocol::UNDEFINED) { // implies !_initialised && _compid
        determine_protocol(msg);
        return;
    }

    // listen to RADIO_RC_CHANNELS messages to stop sending RC_CHANNELS
    if (msg.msgid == MAVLINK_MSG_ID_RADIO_RC_CHANNELS) { // 60045
        _got_radio_rc_channels = true;
    }

    // this msg is not from our gimbal
    if (msg.sysid != _sysid || msg.compid != _compid) {
        return;
    }

    switch (msg.msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT: {
            mavlink_heartbeat_t payload;
            mavlink_msg_heartbeat_decode(&msg, &payload);
            uint8_t storm32_state = (payload.custom_mode & 0xFF);
            _gimbal_armed = ((storm32_state == STorM32State::NORMAL) || (storm32_state == STorM32State::STARTUP_FASTLEVEL));
            if ((payload.custom_mode & 0x80000000) == 0) { // don't follow all changes, but just toggle it to true once
                _gimbal_prearmchecks_ok = true;
            }
            _gimbal_error_flags = (payload.custom_mode & 0x00FFFF00) >> 8;
            break; }

        case MAVLINK_MSG_ID_MOUNT_STATUS: {
            if (_protocol != Protocol::MOUNT) break;
            _mount_status.received_tlast_ms = AP_HAL::millis(); // used for health check
            mavlink_mount_status_t payload;
            mavlink_msg_mount_status_decode(&msg, &payload);
            _current_angles.pitch = radians((float)payload.pointing_a * 0.01f);
            _current_angles.roll = radians((float)payload.pointing_b * 0.01f);
            _current_angles.yaw_bf = radians((float)payload.pointing_c * 0.01f);
            _current_angles.delta_yaw = NAN;
            _current_omega = { NAN, NAN, NAN };
            break; }
    }
}


//------------------------------------------------------
// MAVLink send functions
//------------------------------------------------------

void BP_Mount_STorM32_MAVLink::send_cmd_request_gimbal_device_information()
{
    if (!HAVE_PAYLOAD_SPACE(_chan, COMMAND_LONG)) {
        return;
    }

    mavlink_msg_command_long_send(
        _chan,
        _sysid, _compid,
        MAV_CMD_REQUEST_MESSAGE,                    // command
        0,                                          // confirmation
        MAVLINK_MSG_ID_GIMBAL_DEVICE_INFORMATION,   // param1
        0, 0, 0, 0, 0, 0                            // param2 .. param7
        );
}


// called by send_target_angles()
void BP_Mount_STorM32_MAVLink::send_cmd_do_mount_control()
{
    if (!HAVE_PAYLOAD_SPACE(_chan, COMMAND_LONG)) {
        return;
    }

    // send command_long command containing a do_mount_control command
    // Note: pitch and yaw are reversed
    // ATTENTION: uses get_bf_yaw() to ensure body frame, which uses ahrs.yaw, not delta_yaw!!!
    mavlink_msg_command_long_send(
        _chan,
        _sysid, _compid,
        MAV_CMD_DO_MOUNT_CONTROL,                       // command
        0,                                              // confirmation
        -degrees(mnt_target.angle_rad.pitch),           // param1
        degrees(mnt_target.angle_rad.roll),             // param2
        -degrees(mnt_target.angle_rad.get_bf_yaw()),    // param3
        0, 0, 0,                                        // param4 .. param6
        get_mode()                                      // param7
        );
}


// called by send_target_angles()
// _flags_for_gimbal were just updated, so are correct for sure
void BP_Mount_STorM32_MAVLink::send_gimbal_device_set_attitude()
{
    if (!HAVE_PAYLOAD_SPACE(_chan, GIMBAL_DEVICE_SET_ATTITUDE)) {
        return;
    }

    // convert Euler angles to quaternion
    float target_yaw_bf;
    if (mnt_target.angle_rad.yaw_is_ef) {
        if (isnan(_current_angles.delta_yaw)) { // we don't have a valid yaw_ef
            target_yaw_bf = mnt_target.angle_rad.get_bf_yaw();
        } else {
            // TODO: handle turn around
            target_yaw_bf = wrap_PI(mnt_target.angle_rad.yaw - _current_angles.delta_yaw);
        }
    } else {
        target_yaw_bf = mnt_target.angle_rad.yaw;
    }

    GimbalQuaternion q;
    q.from_euler(mnt_target.angle_rad.roll, mnt_target.angle_rad.pitch, target_yaw_bf);

    float qa[4] = {q.q1, q.q2, q.q3, q.q4};

    mavlink_msg_gimbal_device_set_attitude_send(
        _chan,
        _sysid, _compid,
        _flags_for_gimbal,          // gimbal device flags
        qa,                         // attitude as a quaternion
        NAN, NAN, NAN               // angular velocities
        );


    BP_LOG("MTC0", BP_LOG_MTC_GIMBALCONTROL_HEADER,
        (uint8_t)1,                 // Type, GIMBAL_DEVICE_SET_ATTITUDE
        degrees(mnt_target.angle_rad.roll), degrees(mnt_target.angle_rad.pitch), degrees(target_yaw_bf), // Roll, Pitch, Yaw
        (uint16_t)_flags_for_gimbal, (uint16_t)0, // GDFlags, GMFlags
        (uint8_t)mnt_target.target_type, // TMode
        (uint8_t)0);                // QMode
}


// ensure that MAV_LANDED_STATE enum hasn't been extended
static_assert((uint8_t)MAV_LANDED_STATE_ENUM_END == 5, "MAV_LANDED_STATE enum has changed");

enum LandedStateThisWouldBeGreatToHave {
    MAV_LANDED_STATE_PREPARING_FOR_TAKEOFF = 5,
};


void BP_Mount_STorM32_MAVLink::send_autopilot_state_for_gimbal_device()
{
    if (!HAVE_PAYLOAD_SPACE(_chan, AUTOPILOT_STATE_FOR_GIMBAL_DEVICE)) {
        return;
    }

    const AP_AHRS &ahrs = AP::ahrs();

    // get vehicle attitude
    Quaternion q;
    if (!ahrs.get_quaternion(q)) { // it returns a bool, so it's a good idea to consider it
        q.q1 = q.q2 = q.q3 = q.q4 = NAN;
    }

    // get vehicle velocity
    // comment in AP_AHRS.cpp says "Must only be called if have_inertial_nav() is true", but probably not worth checking
    Vector3f vel;
    if (!ahrs.get_velocity_NED(vel)) { // it returns a bool, so it's a good idea to consider it
        vel.x = vel.y = vel.z = 0.0f; // or NAN ?
    }

    // get vehicle angular velocity z
    float angular_velocity_z = ahrs.get_yaw_rate_earth();

    // get commanded yaw rate
    // see https://github.com/ArduPilot/ardupilot/issues/22564
    float yawrate = NAN;
    const AP_Vehicle *vehicle = AP::vehicle();
    Vector3f rate_ef_targets;
    if ((vehicle != nullptr) && vehicle->get_rate_ef_targets(rate_ef_targets)) {
        yawrate = rate_ef_targets.z;
    }

    // determine estimator status
/*
TODO: how do notify.flags.armed and hal.util->get_soft_armed() compare against each other, also across vehicles?
old:
    bool ahrs_nav_status_horiz_vel = false;
    nav_filter_status nav_status;
    if (ahrs.get_filter_status(nav_status) && nav_status.flags.horiz_vel) {
        ahrs_nav_status_horiz_vel = true;
    }

    uint8_t status = 0;
    if (ahrs.healthy()) { status |= STORM32LINK_FCSTATUS_AP_AHRSHEALTHY; }
    if (ahrs.initialised()) { status |= STORM32LINK_FCSTATUS_AP_AHRSINITIALIZED; }
    if (ahrs_nav_status_horiz_vel) { status |= STORM32LINK_FCSTATUS_AP_NAVHORIZVEL; }
    if (gps.status() >= AP_GPS::GPS_OK_FIX_3D) { status |= STORM32LINK_FCSTATUS_AP_GPS3DFIX; }
    if (notify.flags.armed) { status |= STORM32LINK_FCSTATUS_AP_ARMED; }
    //if (hal.util->get_soft_armed()) { status |= STORM32LINK_FCSTATUS_AP_ARMED; }
    // for copter this is !ap.land_complete, for plane this is new_is_flying
    if (notify.flags.flying) { status |= STORM32LINK_FCSTATUS_AP_ISFLYING; }

mavlink flags                   nav_filter_status
ESTIMATOR_ATTITUDE              attitude           : 1; // 0 - true if attitude estimate is valid
ESTIMATOR_VELOCITY_HORIZ        horiz_vel          : 1; // 1 - true if horizontal velocity estimate is valid
ESTIMATOR_VELOCITY_VERT         vert_vel           : 1; // 2 - true if the vertical velocity estimate is valid
ESTIMATOR_POS_HORIZ_REL         horiz_pos_rel      : 1; // 3 - true if the relative horizontal position estimate is valid
ESTIMATOR_POS_HORIZ_ABS         horiz_pos_abs      : 1; // 4 - true if the absolute horizontal position estimate is valid
ESTIMATOR_POS_VERT_ABS          vert_pos           : 1; // 5 - true if the vertical position estimate is valid
ESTIMATOR_POS_VERT_AGL          terrain_alt        : 1; // 6 - true if the terrain height estimate is valid
ESTIMATOR_CONST_POS_MODE        const_pos_mode     : 1; // 7 - true if we are in const position mode
ESTIMATOR_PRED_POS_HORIZ_REL    pred_horiz_pos_rel : 1; // 8 - true if filter expects it can produce a good relative horizontal position estimate - used before takeoff
ESTIMATOR_PRED_POS_HORIZ_ABS    pred_horiz_pos_abs : 1; // 9 - true if filter expects it can produce a good absolute horizontal position estimate - used before takeoff

ESTIMATOR_GPS_GLITCH            takeoff_detected   : 1; // 10 - true if optical flow takeoff has been detected
ESTIMATOR_ACCEL_ERROR           takeoff            : 1; // 11 - true if filter is compensating for baro errors during takeoff
                                touchdown          : 1; // 12 - true if filter is compensating for baro errors during touchdown
                                using_gps          : 1; // 13 - true if we are using GPS position
                                gps_glitching      : 1; // 14 - true if GPS glitching is affecting navigation accuracy
                                gps_quality_good   : 1; // 15 - true if we can use GPS for navigation
                                initalized         : 1; // 16 - true if the EKF has ever been healthy
                                rejecting_airspeed : 1; // 17 - true if we are rejecting airspeed data
                                dead_reckoning     : 1; // 18 - true if we are dead reckoning (e.g. no position or velocity source)

estimator status:
     STorM32 only listens to ESTIMATOR_ATTITUDE and ESTIMATOR_VELOCITY_VERT
     ahrs.healthy() becomes true during flip of quaternion => no-go
     ahrs.initialised() is simply set after AP_AHRS_NAVEKF_SETTLE_TIME_MS 20000 !!
                        it becomes true some secs after ESTIMATOR_ATTITUDE|ESTIMATOR_VELOCITY_VERT are set
     ESTIMATOR_ATTITUDE can be set during flip of quaternion => no-go
     ESTIMATOR_VELOCITY_VERT are set briefly after the quaternion flip ??Q: really, couldn't it also be raised in the flip?
     ESTIMATOR_VELOCITY_VERT are set however even if there is no gps or alike! I find it hard to trust the data
     => we fake it so:
     for ESTIMATOR_ATTITUDE we delay the flag being raised by few secs
     for ESTIMATOR_VELOCITY_VERT we check for gps like in AP_AHRS_DCM::groundspeed_vector()
     btw STorM32 does also check for non-zero velocities for AHRSFix
*/

    uint16_t nav_estimator_status = 0;
    uint16_t nav_estimator_status2 = 0;

    const uint32_t ESTIMATOR_MASK = (
            ESTIMATOR_ATTITUDE |
            ESTIMATOR_VELOCITY_HORIZ | ESTIMATOR_VELOCITY_VERT |
            ESTIMATOR_POS_HORIZ_REL | ESTIMATOR_POS_HORIZ_ABS |
            ESTIMATOR_POS_VERT_ABS | ESTIMATOR_POS_VERT_AGL |
            ESTIMATOR_CONST_POS_MODE |
            ESTIMATOR_PRED_POS_HORIZ_REL | ESTIMATOR_PRED_POS_HORIZ_ABS);

    nav_filter_status nav_status;
    if (ahrs.get_filter_status(nav_status)) {
        nav_estimator_status = (uint16_t)(nav_status.value & ESTIMATOR_MASK);
        nav_estimator_status2 = (uint16_t)(nav_status.value >> 10);
    }

    uint16_t estimator_status = 0;

    const bool ahrs_healthy = ahrs.healthy(); // it's a bit costly
    if (!_tahrs_healthy_ms && ahrs_healthy) {
        _tahrs_healthy_ms = AP_HAL::millis();
    }

    if (ahrs_healthy && (nav_estimator_status & ESTIMATOR_ATTITUDE) && ((AP_HAL::millis() - _tahrs_healthy_ms) > 3000)) {
        estimator_status |= ESTIMATOR_ATTITUDE; // -> QFix
        if (ahrs.initialised() && (nav_estimator_status & ESTIMATOR_VELOCITY_VERT) && (AP::gps().status() >= AP_GPS::GPS_OK_FIX_2D)) {
            estimator_status |= ESTIMATOR_VELOCITY_VERT; // -> AHRSFix
        }
    }

    // determine landed state
/*
landed state:
     GCS_Common.cpp: virtual MAV_LANDED_STATE landed_state() const { return MAV_LANDED_STATE_UNDEFINED; }
     But protected so we can't access it. Hence either (1) move to public, (2) add public getter to gcs class,
     or (3) add it to vehicle. Latter is most work but nicest, IMHO.
     Copter has it: GCS_MAVLINK_Copter::landed_state(), yields ON_GROUND, TAKEOFF, IN_AIR, LANDING
     Plane has it: GCS_MAVLINK_Plane::landed_state(), only yields ON_GROUND or IN_AIR
     Blimp also has it, blimp not relevant for us
     we want to also take into account the arming state to mock something up
     ugly as we have vehicle dependency, but that's how it is
*/
    uint8_t landed_state = (uint8_t)gcs().get_landed_state(); // AP::vehicle()->get_landed_state();

#if APM_BUILD_TYPE(APM_BUILD_ArduCopter)
    // for copter the landed state is modified such as to reflect the 2 sec pre-take-off period.
    // The code below leads to a PREPARING_FOR_TAKEOFF before takeoff, but also after landing!
    // For the latter one would have to catch that it was flying, but no need to care as
    // the gimbal will do inits when ON_GROUND, and refreshes them when transitioning to PREPARING_FOR_TAKEOFF.
    // It won't do it for other transitions, so e.g. also not for plane.
    if ((landed_state == MAV_LANDED_STATE_ON_GROUND) && AP::notify().flags.armed) {
        landed_state = MAV_LANDED_STATE_PREPARING_FOR_TAKEOFF;
    }
#endif

    // ready to send
    float qa[4] = {q.q1, q.q2, q.q3, q.q4};

    mavlink_msg_autopilot_state_for_gimbal_device_send(
        _chan,
        _sysid, _compid,
        AP_HAL::micros64(),         // uint64_t time_boot_us
        qa,                         // attitude quaternion
        0,                          // uint32_t q_estimated_delay_us,
        vel.x, vel.y, vel.z,        // angular velocity vx, vy, vz
        0,                          // uint32_t v_estimated_delay_us,
        yawrate,                    // float feed_forward_angular_velocity_z
        estimator_status, landed_state,
        angular_velocity_z          // float angular_velocity_z
        );

    static uint32_t tlast_us = 0;
    uint32_t t_us = AP_HAL::micros();
    uint32_t dt_us = t_us - tlast_us;
    tlast_us = t_us;

    BP_LOG("MTL0", BP_LOG_MTL_AUTOPILOTSTATE_HEADER,
        q[0],q[1],q[2],q[3],
        vel.x, vel.y, vel.z,
        degrees(angular_velocity_z),
        degrees(yawrate),
        estimator_status,
        landed_state,
        nav_estimator_status,
        nav_estimator_status2,
        dt_us);
}


void BP_Mount_STorM32_MAVLink::send_system_time()
{
    if (!HAVE_PAYLOAD_SPACE(_chan, SYSTEM_TIME)) {
        return;
    }

    uint64_t time_unix = 0;
    AP::rtc().get_utc_usec(time_unix); // may fail, leaving time_unix at 0

    if (!time_unix) return; // no unix time available, so no reason to send

    mavlink_msg_system_time_send(
        _chan,
        time_unix,          // uint64_t time_unix_usec
        AP_HAL::millis()    // uint32_t time_boot_ms
        );
}


void BP_Mount_STorM32_MAVLink::send_rc_channels()
{
    if (!HAVE_PAYLOAD_SPACE(_chan, RC_CHANNELS)) {
        return;
    }

    // rc().channel(ch)->get_radio_in(), RC_Channels::get_radio_in(ch), and so on
    // these are not the same as hal.rcin->read(), since radio_in can be set by override
    // so we use hal.rcin->read()

    #define RCHALIN(ch_index)  hal.rcin->read(ch_index)

    mavlink_msg_rc_channels_send(
        _chan,
        AP_HAL::millis(),   // uint32_t time_boot_ms
        16,                 // uint8_t chancount, STorM32 won't handle more than 16 anyhow
        RCHALIN(0), RCHALIN(1), RCHALIN(2), RCHALIN(3), RCHALIN(4), RCHALIN(5), RCHALIN(6), RCHALIN(7), // uint16_t chan1_raw ..
        RCHALIN(8), RCHALIN(9), RCHALIN(10), RCHALIN(11), RCHALIN(12), RCHALIN(13), RCHALIN(14), RCHALIN(15), // .. chan16_raw
        0, 0,               // uint16_t chan17_raw .. chan18_raw
        255                 // uint8_t rssi, 255: invalid/unknown
        );
}


void BP_Mount_STorM32_MAVLink::send_banner()
{
    // postpone sending by few seconds, to avoid multiple sends
    _request_send_banner_ms = AP_HAL::millis();
}


void BP_Mount_STorM32_MAVLink::update_send_banner()
{
    if (!_request_send_banner_ms) return; // no request

    uint32_t tnow_ms = AP_HAL::millis();
    if ((tnow_ms - _request_send_banner_ms) < 3500) return; // not yet time to send
    _request_send_banner_ms = 0;

    if (_got_device_info) {
        // we have lots of info
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "MNT%u: gimbal at %u", _instance+1, _compid);

        // convert firmware version to STorM32 convention
        char c = (_device_info.firmware_version & 0x00FF0000) >> 16;
        if (c == '\0') c = ' '; else c += 'a' - 1;

        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "MNT%u: %s v%u.%u%c",
                _instance + 1,
                _device_info.model_name,
                (unsigned)(_device_info.firmware_version & 0x000000FF),
                (unsigned)((_device_info.firmware_version & 0x0000FF00) >> 8),
                c
                );

        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "MNT%u: prearm checks %s", _instance+1, (_prearmchecks_passed) ? "passed" : "fail");

    } else
    if (_compid) {
        // we have some info
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "MNT%u: gimbal at %u", _instance+1, _compid);
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "MNT%u: prearm checks %s", _instance+1, (_prearmchecks_passed) ? "passed" : "fail");

    } else {
        // we don't know yet anything
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "MNT%u: no gimbal yet", _instance+1);
    }

//    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "MNT%u: protocol %u", _instance+1, _protocol);
}


//------------------------------------------------------
// MAVLink gimbal manager send functions
//------------------------------------------------------

// send a GIMBAL_MANAGER_INFORMATION message to GCS
void BP_Mount_STorM32_MAVLink::send_gimbal_manager_information(mavlink_channel_t chan)
{
    // space already checked by streamer

    // The request to send this message should be NACKed for as long as
    // the gimbal device was not found or its device info was not received.
    // Not done currently, so as workaround don't send if not yet available.
    // Should make third parties to repeat requests.

    if (!_got_device_info) return;

    // There are few specific gimbal manager capability flags, which are not used.
    // So one simply can carry forward the cap_flags received from the gimbal.
    // The STorM32 gimbal has these capabilities:
    // - GIMBAL_DEVICE_CAP_FLAGS_HAS_RETRACT | GIMBAL_DEVICE_CAP_FLAGS_HAS_NEUTRAL
    // - GIMBAL_DEVICE_CAP_FLAGS_HAS_xx_AXIS | GIMBAL_DEVICE_CAP_FLAGS_HAS_xx_FOLLOW |
    //   GIMBAL_DEVICE_CAP_FLAGS_HAS_xx_LOCK for each enabled axis
    // - GIMBAL_DEVICE_CAP_FLAGS_SUPPORTS_INFINITE_YAW if turn around enabled
    // - GIMBAL_DEVICE_CAP_FLAGS_HAS_RC_INPUTS
    // - !GIMBAL_DEVICE_CAP_FLAGS_SUPPORTS_YAW_IN_EARTH_FRAME is not currently supported by STorM32

    uint32_t cap_flags = _device_info.cap_flags;

    // This driver does not support all capabilities, so we erase them.
    // ATTENTION: This can mean that the gimbal device and gimbal manager capability flags
    // may be different, and any third party which mistakenly thinks it can use those from
    // the gimbal device messages may get confused !

    cap_flags &=~ (GIMBAL_MANAGER_CAP_FLAGS_HAS_ROLL_FOLLOW |
                   GIMBAL_MANAGER_CAP_FLAGS_HAS_PITCH_FOLLOW |
                   GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_LOCK |
                   GIMBAL_MANAGER_CAP_FLAGS_SUPPORTS_YAW_IN_EARTH_FRAME);

    mavlink_msg_gimbal_manager_information_send(
        chan,
        AP_HAL::millis(),           // autopilot system time
        cap_flags,                  // bitmap of gimbal manager capability flags
        _compid,                    // gimbal device id
        _device_info.roll_min,      // roll_min in radians
        _device_info.roll_max,      // roll_max in radians
        _device_info.pitch_min,     // pitch_min in radians
        _device_info.pitch_max,     // pitch_max in radians
        _device_info.yaw_min,       // yaw_min in radians
        _device_info.yaw_max        // yaw_max in radians
        );
}


// return gimbal manager flags. Used by GIMBAL_MANAGER_STATUS message.
uint32_t BP_Mount_STorM32_MAVLink::get_gimbal_manager_flags() const
{
    // There are currently no specific gimbal manager flags. So one simply
    // can carry forward the _flags received from the gimbal.

    // ATTENTION: This driver does not support all capabilities, but this
    // should never be a problem since any third party should strictly adhere
    // to the capability flags obtained from the gimbal manager !

    return _device_status.received_flags;
}


//------------------------------------------------------
// Prearm & healthy functions
//------------------------------------------------------

const uint32_t FAILURE_FLAGS =
        GIMBAL_DEVICE_ERROR_FLAGS_ENCODER_ERROR |
        GIMBAL_DEVICE_ERROR_FLAGS_POWER_ERROR |
        GIMBAL_DEVICE_ERROR_FLAGS_MOTOR_ERROR |
        GIMBAL_DEVICE_ERROR_FLAGS_SOFTWARE_ERROR |
        GIMBAL_DEVICE_ERROR_FLAGS_COMMS_ERROR;


bool BP_Mount_STorM32_MAVLink::has_failures(char* s)
{
    uint32_t failure_flags = (_protocol == Protocol::GIMBAL_DEVICE) ? _device_status.received_failure_flags : _gimbal_error_flags;

    s[0] = '\0';
    if ((failure_flags & FAILURE_FLAGS) > 0) {
        if (failure_flags & GIMBAL_DEVICE_ERROR_FLAGS_MOTOR_ERROR) strcat(s, "mot,");
        if (failure_flags & GIMBAL_DEVICE_ERROR_FLAGS_ENCODER_ERROR) strcat(s, "enc,");
        if (failure_flags & GIMBAL_DEVICE_ERROR_FLAGS_POWER_ERROR) strcat(s, "volt,");
        if (s[0] != '\0') {
            s[strlen(s)-1] = '\0';
        } else {
            strcpy(s, "err flags");
        }
        return true;
    }
    return false;
}


bool BP_Mount_STorM32_MAVLink::is_healthy()
{
    if (_protocol == Protocol::GIMBAL_DEVICE) {
        // unhealthy if attitude status is not received within the last second
        if ((AP_HAL::millis() - _device_status.received_tlast_ms) > 1000) {
            return false;
        }

        // check failure flags
        // We should also check for GIMBAL_DEVICE_ERROR_FLAGS_NO_MANAGER
        // which means that gimbal did not got GIMBAL_DEVICE_SET_ATTITUDE messages.
//        if ((_device_status.received_failure_flags & (FAILURE_FLAGS | GIMBAL_DEVICE_ERROR_FLAGS_NO_MANAGER)) > 0) {
        if ((_device_status.received_failure_flags & FAILURE_FLAGS) > 0) {
            return false;
        }
    } else {
        // unhealthy if mount status is not received within the last second
        if ((AP_HAL::millis() - _mount_status.received_tlast_ms) > 1000) {
            return false;
        }

        // check failure flags // are set since v2.68b
        if ((_gimbal_error_flags & FAILURE_FLAGS) > 0) {
            return false;
        }
    }

    return true;
}


// is called with 1 Hz from update loop
void BP_Mount_STorM32_MAVLink::update_checks()
{
char txt[255];

    if (_armingchecks_running) _armingchecks_running--; // count down

    if (_prearmchecks_passed && AP::notify().flags.armed &&
        !AP::arming().option_enabled(AP_Arming::Option::DISABLE_PREARM_DISPLAY)) {
        bool checks = is_healthy();

        if (_checks_last && !checks) { // checks went from true to false
            if (has_failures(txt)) {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "MNT%u: failures: %s", _instance+1, txt);
            } else {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "MNT%u: failure: gimbal lost", _instance+1);
            }
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "MNT%u: not healthy", _instance+1);
        }
        if (!_checks_last && checks) { // checks went from false to true
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "MNT%u: healthy again", _instance+1);
        }
        _checks_last = checks;
    }

    // don't do anything further if arming mechanism is not enabled
    if (!_armingchecks_running) {
        return;
        _healthy = true;
    }

    // do these only at startup, until prearm checks have been passed once
    if (!_prearmchecks_passed) {

        if ((AP_HAL::millis() - _prearmcheck_sendtext_tlast_ms) > 30000) { // we haven't send it for a long while
            _prearmcheck_sendtext_tlast_ms = AP_HAL::millis();
            if (!_initialised || !_gimbal_prearmchecks_ok || !_gimbal_armed) {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "MNT%u: prearm checks FAIL: not armed", _instance+1);
            } else
            if (has_failures(txt)) {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "MNT%u: prearm checks FAIL: %s", _instance+1, txt);
            }
        }

        // unhealthy until gimbal has fully passed the startup sequence
        // _initialised:            -> gimbal found (HB received,_compid != 0)
        //                          -> device info obtained (_got_device_info = true)
        //                          -> status message received, protocol set (_protocol != PROTOCOL_UNDEFINED)
        // _gimbal_prearmchecks_ok: -> gimbal HB reported gimbal's prearmchecks ok
        // _gimbal_armed:           -> gimbal HB reported gimbal is in normal state
        if (!_initialised || !_gimbal_prearmchecks_ok || !_gimbal_armed) {
            _healthy = false;
            return;
        }
    }

    // do these continuously
    // - check failures
    // - check connection to gimbal
    bool checks = is_healthy();

    if (_prearmchecks_passed) { // we are past prearm checks
        if (_checks_last && !checks) { // checks went from true to false
            if (has_failures(txt)) {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "MNT%u: prearm checks FAIL: %s", _instance+1, txt);
            } else {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "MNT%u: prearm checks FAIL: gimbal lost", _instance+1);
            }
        }
        if (!_checks_last && checks) { // checks went from false to true
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "MNT%u: prearm checks passed", _instance+1);
        }
        _checks_last = checks;
        _healthy = checks;
        return;
    }

    _checks_last = checks;

    if (!checks) {
        _healthy = false;
        return;
    }

    // if we get this far in prearm state, then mount is healthy

    // if we got this far the first time we inform the gcs
    if (!_prearmchecks_passed) {
        _prearmchecks_passed = true;
         GCS_SEND_TEXT(MAV_SEVERITY_INFO, "MNT%u: prearm checks passed", _instance+1);
    }

    _healthy = true;
}


// return true if healthy
// this is called when ARMING_CHECK_ALL or ARMING_CHECK_CAMERA is set, and if not armed, else not
// is called with 1 Hz
bool BP_Mount_STorM32_MAVLink::healthy() const
{
    _armingchecks_running = 2; // to signal that ArduPilot arming check mechanism is active

    return _healthy;
}


//------------------------------------------------------
// Scripting accessors & bindings
//------------------------------------------------------

// return target location if available
// returns true if a target location is available and fills in target_loc argument
bool BP_Mount_STorM32_MAVLink::get_location_target(Location &_target_loc)
{
    if (mnt_target_loc_valid) {
        _target_loc = mnt_target_loc;
        return true;
    }
    return false;
}


// update mount's actual angles (to be called by script communicating with mount)
void BP_Mount_STorM32_MAVLink::set_attitude_euler(float roll_deg, float pitch_deg, float yaw_bf_deg)
{
    _script_angles.roll = radians(roll_deg);
    _script_angles.pitch = radians(pitch_deg);
    _script_angles.yaw_bf = radians(yaw_bf_deg);
}


bool BP_Mount_STorM32_MAVLink::take_control()
{
    _script_angles.control = true;
    return true; //we assume only one script trying this, so KIS
}


bool BP_Mount_STorM32_MAVLink::give_control()
{
    _script_angles.control = false;
    return true; //we assume only one script trying this, so KIS
}


//------------------------------------------------------
// Camera
//------------------------------------------------------

bool BP_Mount_STorM32_MAVLink::take_picture()
{
    if (_camera_mode == CameraMode::UNDEFINED) {
        _camera_mode = CameraMode::PHOTO;
        send_cmd_do_digicam_configure(false);
    }

    if (_camera_mode != CameraMode::PHOTO) return false;

    send_cmd_do_digicam_control(true);

//    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "cam take pic");

    return true;
}


bool BP_Mount_STorM32_MAVLink::record_video(bool start_recording)
{
    if (_camera_mode == CameraMode::UNDEFINED) {
        _camera_mode = CameraMode::VIDEO;
        send_cmd_do_digicam_configure(true);
    }

    if (_camera_mode != CameraMode::VIDEO) return false;

    send_cmd_do_digicam_control(start_recording);

//    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "cam rec video %u", start_recording);

    return true;
}


bool BP_Mount_STorM32_MAVLink::cam_set_mode(bool video_mode)
{
    _camera_mode = (video_mode) ? CameraMode::VIDEO : CameraMode::PHOTO;
    send_cmd_do_digicam_configure(video_mode);

//    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "cam set mode %u", video_mode);

    return true;
}


bool BP_Mount_STorM32_MAVLink::cam_do_photo_video_mode(PhotoVideoMode photo_video_mode)
{
    if (photo_video_mode == PhotoVideoMode::VIDEO_START) {

        if (_camera_mode != CameraMode::VIDEO) {
            _camera_mode = CameraMode::VIDEO;
            send_cmd_do_digicam_configure(true); // set video mode
        }

        send_cmd_do_digicam_control(true); // start video
        _camera_is_recording = true;

    } else
    if (photo_video_mode == PhotoVideoMode::PHOTO_TAKE_PIC) {

        if (_camera_is_recording) {
            send_cmd_do_digicam_control(false);
            _camera_is_recording = false;
        }

        if (_camera_mode != CameraMode::PHOTO) {
            _camera_mode = CameraMode::PHOTO;
            send_cmd_do_digicam_configure(false); // set photo mode
        }
        send_cmd_do_digicam_control(true); // take picture

    } else {

        if (_camera_mode == CameraMode::VIDEO) {
            send_cmd_do_digicam_control(false);
            _camera_is_recording = false;
        }

    }

    return true;
}


void BP_Mount_STorM32_MAVLink::send_cmd_do_digicam_configure(bool video_mode)
{
    if (!HAVE_PAYLOAD_SPACE(_chan, COMMAND_LONG)) {
        return;
    }

    float param1 = (video_mode) ? 1 : 0;

    mavlink_msg_command_long_send(
        _chan,
        _sysid, _compid,
        MAV_CMD_DO_DIGICAM_CONFIGURE,   // command
        0,                              // confirmation
        param1, 0, 0, 0, 0, 0, 0        // param1 .. param7
        );

//    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "cam digi config %u", video_mode);
}


void BP_Mount_STorM32_MAVLink::send_cmd_do_digicam_control(bool shoot)
{
    if (!HAVE_PAYLOAD_SPACE(_chan, COMMAND_LONG)) {
        return;
    }

    float param5 = (shoot) ? 1 : 0;

    mavlink_msg_command_long_send(
        _chan,
        _sysid, _compid,
        MAV_CMD_DO_DIGICAM_CONTROL,     // command
        0,                              // confirmation
        0, 0, 0, 0, param5, 0, 0        // param1 .. param7
        );

//    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "cam digi cntrl %u", shoot);
}


//------------------------------------------------------
// MAVLink mount status forwarding
//------------------------------------------------------

// send a MOUNT_STATUS message to GCS, this is only to make MissionPlanner and alike happy
void BP_Mount_STorM32_MAVLink::send_gimbal_device_attitude_status(mavlink_channel_t chan)
{
    // space already checked by streamer
    // has checked for space of GIMBAL_DEVICE_ATTITUDE_STATUS, but MOUNT_STATUS is (much) smaller, so no issue

    if (_compid != MAV_COMP_ID_GIMBAL) { // do it only for the 1st gimbal
        return;
    }

    mavlink_msg_mount_status_send(
        chan,
        0,          // uint8_t target_system
        0,          // uint8_t target_component
        (int32_t)(degrees(_current_angles.pitch) * 100.0f),     // int32_t pointing_a
        (int32_t)(degrees(_current_angles.roll) * 100.0f),      // int32_t pointing_b
        (int32_t)(degrees(_current_angles.yaw_bf) * 100.0f),    // int32_t pointing_c
        get_mode()  // uint8_t mount_mode
        );
}


#endif // HAL_MOUNT_BP_STORM32_MAVLINK_ENABLED



/*
STorM32-Link tests

tests V4.3.2-rc1
logging starts only ca 8-12 secs after power up
ahrs.healthy() becomes true during "flip" of quaternion !!
ahrs.initialised() is simply set after AP_AHRS_NAVEKF_SETTLE_TIME_MS 20000 !!

                            231 167 831 895
1:   ATTITUDE               1   1   1   1
2:   VELOCITY_HORIZ         1   1   1   1
4:   VELOCITY_VERT          1   1   1   1
8:   POS_HORIZ_REL          0   0   1   1
16:  POS_HORIZ_ABS          0   0   1   1
32:  POS_VERT_ABS           1   1   1   1
64:  POS_VERT_AGL           1   0   0   1
128: CONST_POS_MODE         1   1   0   0
256: PRED_POS_HORIZ_REL     -   -   1   1
512: PRED_POS_HORIZ_ABS     -   -   1   1

test #1 (2022-12-17 10-28-00.bin):
t           Est NavEst
            0   0
26.0s       1   0       happens during "flip" of quaternion !!
26.8s       1   231     ATTITUDE|VELOCITY_HORIZ|VELOCITY_VERT|POS_VERT_ABS + more
31.3s       5   231
47.4s       5   167
62.6s       5   831     ATTITUDE|VELOCITY_HORIZ|VELOCITY_VERT|POS_VERT_ABS + POS_HORIZ_REL|POS_HORIZ_ABS
90.4s       5   895     happens briefly after landed went from 5 to 3

test #2 (2022-12-17 11-24-40.bin):
t           Est NavEst
            0   0
23.2s       1   0
24.0s       1   167
28.6s       5   167
60.0s       5   831
78.4s       5   895     happens briefly after landed went from 5 to 3

test #3 (2022-12-17 11-28-07.bin):
t           Est NavEst
            0   0
24.2s       1   0
25.0s       1   167
29.5s       5   167
61.4s       5   831
75.8s       5   895     happens briefly after landed went from 5 to 3

2.Jan.2023
the quaternion flip can also be seen in MP in e.g. yaw, when one connects quickly, EKF dialog looks ok then
*/

/*
landed state:
 this is not nice, but kind of the best we can currently do
 copter & plane do support it, but has it private, and in GCS_MAVLINK
 so we end up redoing it in vehicle dependent way, which however gives us also the chance to do it better

copter's landed state
 copter.ap.land_complete <-> MAV_LANDED_STATE_ON_GROUND
 copter.flightmode->is_landing() <-> MAV_LANDED_STATE_LANDING
 copter.flightmode->is_taking_off() <-> MAV_LANDED_STATE_TAKEOFF
 else <-> MAV_LANDED_STATE_IN_AIR

from tests 2021-08-28, in loiter with takeoff/land button, I conclude
 get_landed_state():
 1 = MAV_LANDED_STATE_ON_GROUND  until motors ramp up
 3 = MAV_LANDED_STATE_TAKEOFF  for a moment of gaining height
 2 = MAV_LANDED_STATE_IN_AIR  during flight
 4 = MAV_LANDED_STATE_LANDING  while landing
 1 = MAV_LANDED_STATE_ON_GROUND  after landing
 seems to exactly do what it is supposed to do, but doesn't reflect 2 sec pre-takeoff
 SL status:
 143 = 0x8F until ca 4 sec before take off
 207 = 0xCF = ARMED for ca 4 sec until take off
 239 = 0xEF = 0x20 + ARMED at take off and in flight
 143 = 0x8F after landing
 this thus allows to catch the 4 sec pre-takeoff period
 ARMING_DELAY_SEC 2.0f, MOT_SAFE_TIME 1.0f per default
 with taking-off & landing in loiter with sticks, we get the same behavior at takeoff,
 but when landing the state MAV_LANDED_STATE_LANDING is not there, makes sense as we just drop to ground
*/

