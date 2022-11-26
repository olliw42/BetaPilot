//*****************************************************
//OW
// (c) olliw, www.olliw.eu, GPL3
// STorM32 mount backend class
// 100% MAVLink + storm32.xml
//*****************************************************

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Common/AP_Common.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_RTC/AP_RTC.h>
#include <AP_Notify/AP_Notify.h>
#include <RC_Channel/RC_Channel.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include "BP_Mount_STorM32_MAVLink.h"

extern const AP_HAL::HAL& hal;

//*****************************************************
/*
This is a simplified, heavily reworked version of what's in BetaPilot 4.2.3.
Adopts the gimbal changes in 4.3 as much as possible, and follows the Gremsy & SToRM32 drivers.
Limitations of the Gremsy driver are:
- captures & resends messages instead of relying on routing
- yaw lock and vehicle/earth frame are strictly tight together
- capabilities (i.e. limited capabilities) are not respected

STorM32 gimbal manager not yet implemented
*/
//*****************************************************


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

BP_Mount_STorM32_MAVLink::BP_Mount_STorM32_MAVLink(AP_Mount &frontend, AP_Mount_Params &params, uint8_t instance) :
    AP_Mount_Backend(frontend, params, instance)
{
    _got_device_info = false;
    _initialised = false;
    _armed = false;
    _prearmchecks_ok = false;

    _sysid = 0;
    _compid = 0; // gimbal not yet discovered
    _chan = MAVLINK_COMM_0; // this is a dummy, will be set correctly by find_gimbal()

    _prearmcheck_last = false;

    _received_device_flags = 0;
    _received_device_failure_flags = 0;

    _yaw_lock = false; // STorM32 doesn't currently support earth frame, so we need to ensure this is false

    _mode = MAV_MOUNT_MODE_RC_TARGETING;

    _sendonly = false;
}


// called by all vehicles with 50 Hz, using the scheduler
// several vehicles do not support fast_update(), so let's go with this
// priority of update() not very high, so no idea how reliable that is, may be not so good
// we would have wanted updates with 20 Hz, especially for STorM32-Link
// however, since it's 50 Hz,  we update at 25 Hz and 12.5 Hz respectively
// not soo nice, but best we can do
// not clear what it means for STorM32Link, probably not too bad, maybe even good
void BP_Mount_STorM32_MAVLink::update()
{
    switch (_task_counter) {
        case TASK_SLOT0:
        case TASK_SLOT2:
            if (_compid) { // we send it as soon as we have found the gimbal
                send_autopilot_state_for_gimbal_device();
            }
            break;

        case TASK_SLOT1:
            if (_sendonly) break; // don't do any control messages
            if (_initialised) { // we do it when the startup sequence has been fully completed
                set_and_send_target_angles();
            }
            break;

        case TASK_SLOT3:
            if (_compid) { // we send it as soon as we have found the gimbal
                send_rc_channels();
            }
            break;
    }

    _task_counter++;
    if (_task_counter > TASK_SLOT3) _task_counter = TASK_SLOT0;


    if (!_initialised) {
        find_gimbal();
        return;
    }

    uint32_t tnow_ms = AP_HAL::millis();

    if ((tnow_ms - _send_system_time_tlast_ms) >= 5000) { // every 5 sec is really plenty
        _send_system_time_tlast_ms = tnow_ms;
        send_system_time();
    }
}


void BP_Mount_STorM32_MAVLink::set_and_send_target_angles(void)
{
    MountTarget target_rad {};

    // update based on mount mode
    switch (get_mode()) {

        // move mount to a "retracted" position.  We disable motors
        case MAV_MOUNT_MODE_RETRACT:
            // handled below
            send_gimbal_device_retract();
            break;

        // move mount to a neutral position, typically pointing forward
        case MAV_MOUNT_MODE_NEUTRAL: {
            const Vector3f &target_deg = _params.neutral_angles.get();
            send_gimbal_device_set_attitude(ToRad(target_deg.x), ToRad(target_deg.y), ToRad(target_deg.z), false);
            }
            break;

        // use angle or rate targets provided by a mavlink message or mission command
        case MAV_MOUNT_MODE_MAVLINK_TARGETING:
            switch (mavt_target.target_type) {
            case MountTargetType::ANGLE:
                send_gimbal_device_set_attitude(mavt_target.angle_rad.roll, mavt_target.angle_rad.pitch, mavt_target.angle_rad.yaw, mavt_target.angle_rad.yaw_is_ef);
                break;
            case MountTargetType::RATE:
                // we do not yet support rate, so do it SToRM32 driver like
                // send_gimbal_device_set_rate(mavt_target.rate_rads.roll, mavt_target.rate_rads.pitch, mavt_target.rate_rads.yaw, mavt_target.rate_rads.yaw_is_ef);
                MountTarget rate_target_rad {};
                update_angle_target_from_rate(mavt_target.rate_rads, rate_target_rad);
                send_gimbal_device_set_attitude(rate_target_rad.roll, rate_target_rad.pitch, rate_target_rad.yaw, rate_target_rad.yaw_is_ef);
                break;
            }
            break;

        // RC radio manual angle control, but with stabilization from the AHRS
        // update targets using pilot's rc inputs
        case MAV_MOUNT_MODE_RC_TARGETING: {
            if (get_rc_rate_target(target_rad)) {
                // we do not yet support rate, so do it SToRM32 driver like
                // send_gimbal_device_set_rate(target_rad.roll, target_rad.pitch, target_rad.yaw, target_rad.yaw_is_ef);
                MountTarget rate_target_rad {};
                update_angle_target_from_rate(target_rad, rate_target_rad);
                send_gimbal_device_set_attitude(rate_target_rad.roll, rate_target_rad.pitch, rate_target_rad.yaw, rate_target_rad.yaw_is_ef);
            } else
            if (get_rc_angle_target(target_rad)) {
                send_gimbal_device_set_attitude(target_rad.roll, target_rad.pitch, target_rad.yaw, target_rad.yaw_is_ef);
            }
            break;
        }

        // point mount to a GPS point given by the mission planner
        case MAV_MOUNT_MODE_GPS_POINT: {
            if (get_angle_target_to_roi(target_rad)) {
                send_gimbal_device_set_attitude(target_rad.roll, target_rad.pitch, target_rad.yaw, target_rad.yaw_is_ef);
            }
            break;
        }

        // point mount to home
        case MAV_MOUNT_MODE_HOME_LOCATION: {
            if (get_angle_target_to_home(target_rad)) {
                send_gimbal_device_set_attitude(target_rad.roll, target_rad.pitch, target_rad.yaw, target_rad.yaw_is_ef);
            }
            break;
        }

        case MAV_MOUNT_MODE_SYSID_TARGET: {
            if (get_angle_target_to_sysid(target_rad)) {
                send_gimbal_device_set_attitude(target_rad.roll, target_rad.pitch, target_rad.yaw, target_rad.yaw_is_ef);
            }
            break;
        }

        default:
            // unknown mode so do nothing
            break;
    }
}


// return true if healthy
bool BP_Mount_STorM32_MAVLink::healthy() const
{
    return const_cast<BP_Mount_STorM32_MAVLink*>(this)->prearmchecks_do(); // yes, ugly, but I have not overdesigned the backend
}


void BP_Mount_STorM32_MAVLink::find_gimbal(void)
{
    uint32_t tnow_ms = AP_HAL::millis();

    // search for gimbal for given time or until armed
#if USE_FIND_GIMBAL_MAX_SEARCH_TIME_MS
    if (tnow_ms > FIND_GIMBAL_MAX_SEARCH_TIME_MS) {
        return;
    }
#endif
    if (hal.util->get_soft_armed()) {
        return;
    }

    // search for gimbal in routing table
    if (!_compid) {
        // we expect that instance 0 has compid = MAV_COMP_ID_GIMBAL, instance 1 has compid = MAV_COMP_ID_GIMBAL2, etc
        uint8_t compid = (_instance == 0) ? MAV_COMP_ID_GIMBAL : MAV_COMP_ID_GIMBAL2 + (_instance - 1);
        if (GCS_MAVLINK::find_by_mavtype_and_compid(MAV_TYPE_GIMBAL, compid, _sysid, _chan)) {
            _compid = compid;
            _request_device_info_tlast_ms = (tnow_ms < 900) ? 0 : tnow_ms - 900; // start sending requests in 100 ms
        } else {
            // have not yet found a gimbal so return
            return;
        }
    }

    // request GIMBAL_DEVICE_INFORMATION
    if (!_got_device_info) {
        if (tnow_ms - _request_device_info_tlast_ms > 1000) {
            _request_device_info_tlast_ms = tnow_ms;
            send_request_gimbal_device_information();
        }
        return;
    }

    _initialised = true;
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

    // set parameter defaults from gimbal information
    // Q: why default ?? why not actual value ?? I don't understand this
    if (!isnan(_device_info.roll_min)) _params.roll_angle_min.set_default(degrees(_device_info.roll_min));
    if (!isnan(_device_info.roll_max)) _params.roll_angle_max.set_default(degrees(_device_info.roll_max));
    if (!isnan(_device_info.pitch_min)) _params.pitch_angle_min.set_default(degrees(_device_info.pitch_min));
    if (!isnan(_device_info.pitch_max)) _params.pitch_angle_max.set_default(degrees(_device_info.pitch_max));
    if (!isnan(_device_info.yaw_min)) _params.yaw_angle_min.set_default(degrees(_device_info.yaw_min));
    if (!isnan(_device_info.yaw_max)) _params.yaw_angle_max.set_default(degrees(_device_info.yaw_max));

    // mark it as having been found
    _got_device_info = true;

    // display gimbal info to user
    send_banner();
}


void BP_Mount_STorM32_MAVLink::handle_gimbal_device_attitude_status(const mavlink_message_t &msg)
{
    // this msg is not from our gimbal
    if (msg.sysid != _sysid || msg.compid != _compid) {
        return;
    }

    // get relevant data
    mavlink_gimbal_device_attitude_status_t payload;
    mavlink_msg_gimbal_device_attitude_status_decode(&msg, &payload);

    _received_device_flags = payload.flags;
    _received_device_failure_flags = payload.failure_flags;

    // used for health check
    _received_attitude_status_tlast_ms = AP_HAL::millis();

    // forward to ground as MOUNT_STATUS message
    if (payload.target_system) { // trigger sending of MOUNT_STATUS to ground only if target_sysid = 0
        return;
    }
    if (!is_primary()) {
        return;
    }

    tMountStatus status;
    float roll_rad, pitch_rad, yaw_rad; //, delta_yaw_deg;

    GimbalQuaternion quat(payload.q[0], payload.q[1], payload.q[2], payload.q[3]);
    quat.to_gimbal_euler(roll_rad, pitch_rad, yaw_rad);
    status.roll_deg = degrees(roll_rad);
    status.pitch_deg = degrees(pitch_rad);
    status.yaw_deg = degrees(yaw_rad);
    //delta_yaw_deg = degrees(payload.delta_yaw);

    send_mount_status_to_ground(status);
}


void BP_Mount_STorM32_MAVLink::handle_msg(const mavlink_message_t &msg)
{
    // this msg is not from our gimbal
    if (msg.sysid != _sysid || msg.compid != _compid) {
        return;
    }

    switch (msg.msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT: {
            mavlink_heartbeat_t payload;
            mavlink_msg_heartbeat_decode(&msg, &payload);
            uint8_t storm32_state = (payload.custom_mode & 0xFF);
            _armed = ((storm32_state == STORM32STATE_NORMAL) || (storm32_state == STORM32STATE_STARTUP_FASTLEVEL));
            if (!(payload.custom_mode & 0x80000000)) { // we don't follow all changes, but just toggle it to true once
                _prearmchecks_ok = true;
            }
            }break;
    }
}


//------------------------------------------------------
// MAVLink send functions I
//------------------------------------------------------

void BP_Mount_STorM32_MAVLink::send_request_gimbal_device_information(void)
{
    if (!HAVE_PAYLOAD_SPACE(_chan, COMMAND_LONG)) {
        return;
    }

    mavlink_msg_command_long_send(
        _chan,
        _sysid, _compid,
        MAV_CMD_REQUEST_MESSAGE,
        0, MAVLINK_MSG_ID_GIMBAL_DEVICE_INFORMATION, 0, 0, 0, 0, 0, 0);
}


void BP_Mount_STorM32_MAVLink::send_gimbal_device_retract(void)
{
    if (!HAVE_PAYLOAD_SPACE(_chan, GIMBAL_DEVICE_SET_ATTITUDE)) {
        return;
    }

    // send command_long command containing a do_mount_control command
    const float quat_array[4] = {NAN, NAN, NAN, NAN};

    mavlink_msg_gimbal_device_set_attitude_send(
        _chan,
        _sysid, _compid,
        GIMBAL_DEVICE_FLAGS_RETRACT,    // gimbal device flags
        quat_array,                     // attitude as a quaternion
        NAN, NAN, NAN);                 // angular velocities
}


// earth_frame should be true if yaw_rad target is in earth frame angle, false if body_frame
void BP_Mount_STorM32_MAVLink::send_gimbal_device_set_attitude(float roll_rad, float pitch_rad, float yaw_rad, bool earth_frame)
{
    if (!HAVE_PAYLOAD_SPACE(_chan, GIMBAL_DEVICE_SET_ATTITUDE)) {
        return;
    }

    // prepare flags
    const uint16_t flags = GIMBAL_DEVICE_FLAGS_ROLL_LOCK | GIMBAL_DEVICE_FLAGS_PITCH_LOCK;

    // convert euler angles to quaternion
    Quaternion q;
    q.from_euler(roll_rad, pitch_rad, yaw_rad);
    const float quat_array[4] = {q.q1, q.q2, q.q3, q.q4};

    // send command_long command containing a do_mount_control command
    mavlink_msg_gimbal_device_set_attitude_send(
        _chan,
        _sysid, _compid,
        flags,          // gimbal device flags
        quat_array,     // attitude as a quaternion
        NAN, NAN, NAN); // angular velocities
}


//------------------------------------------------------
// MAVLink send functions Auxiliary
//------------------------------------------------------

//landed state:
// this is not nice, but kind of the best we can currently do
// plane does not support landed state at all, so we do have vehicle dependency here
// copter does support it, but has it private, and in GCS_MAVLINK
// so we end up redoing it in vehicle dependent way, which however gives us also the chance to do it better
//
// 26.05.2022:
// Plane4.2 does now provide a basic landed_state(), returning IN_AIR when flying and ON_GROUND else
// is this useful to us?
//
//copter's landed state
// copter.ap.land_complete <-> MAV_LANDED_STATE_ON_GROUND
// copter.flightmode->is_landing() <-> MAV_LANDED_STATE_LANDING
// copter.flightmode->is_taking_off() <-> MAV_LANDED_STATE_TAKEOFF
// else <-> MAV_LANDED_STATE_IN_AIR
//
//from tests 2021-08-28, in loiter with takeoff/land button, I conclude
// get_landed_state():
// 1 = MAV_LANDED_STATE_ON_GROUND  until motors ramp up
// 3 = MAV_LANDED_STATE_TAKEOFF  for a moment of gaining height
// 2 = MAV_LANDED_STATE_IN_AIR  during flight
// 4 = MAV_LANDED_STATE_LANDING  while landing
// 1 = MAV_LANDED_STATE_ON_GROUND  after landing
// seems to exactly do what it is supposed to do, but doesn't reflect 2 sec pre-takeoff
// SL status:
// 143 = 0x8F until ca 4 sec before take off
// 207 = 0xCF = ARMED for ca 4 sec until take off
// 239 = 0xEF = 0x20 + ARMED at take off and in flight
// 143 = 0x8F after landing
// this thus allows to catch the 4 sec pre-takeoff period
// ARMING_DELAY_SEC 2.0f, MOT_SAFE_TIME 1.0f per default
//with taking-off & landing in loiter with sticks, we get the same behavior at takeoff,
//but when landing the state 4 = MAV_LANDED_STATE_LANDING is not there, makes sense as we just drop to ground

enum THISWOULDBEGREATTOHAVE {
    MAV_LANDED_STATE_PREPARING_FOR_TAKEOFF = 5,
};


void BP_Mount_STorM32_MAVLink::send_autopilot_state_for_gimbal_device(void)
{
    if (!HAVE_PAYLOAD_SPACE(_chan, AUTOPILOT_STATE_FOR_GIMBAL_DEVICE)) {
        return;
    }

    const AP_AHRS &ahrs = AP::ahrs();
    const AP_Vehicle *vehicle = AP::vehicle();
    const AP_Notify &notify = AP::notify();

    Quaternion quat;
//    quat.from_rotation_matrix(ahrs.get_rotation_body_to_ned()); // this gives DCM???
    if (!ahrs.get_quaternion(quat)) { // it returns a bool, so it's a good idea to consider it
        quat.q1 = quat.q2 = quat.q3 = quat.q4 = NAN;
    }
    float q[4] = { quat.q1, quat.q2, quat.q3, quat.q4 };

    Vector3f vel;
    if (!ahrs.get_velocity_NED(vel)) { // it returns a bool, so it's a good idea to consider it
        vel.x = vel.y = vel.z = 0.0f; // or NAN ???
    }

    float angular_velocity_z = NAN;

    float yawrate = NAN;
    Vector3f rate_bf_targets;
    if ((vehicle != nullptr) && vehicle->get_rate_bf_targets(rate_bf_targets)) {
        yawrate = rate_bf_targets.z;
    }

/* estimator status
no support by ArduPilot whatsoever
TODO: how do notify.flags.armed and hal.util->get_soft_armed() compare against each other, also across vehicles?
*/
/*
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
*/

    uint16_t estimator_status = 0;
    if (ahrs.healthy()) estimator_status |= ESTIMATOR_ATTITUDE;
    if (ahrs.initialised()) estimator_status |= ESTIMATOR_VELOCITY_VERT;

/*
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
*/
    uint16_t nav_estimator_status = 0;

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
    }

    // we fake this for the moment to be able to log and investigate
    estimator_status |= (nav_estimator_status << 4);

/* landed state
GCS_Common.cpp: virtual MAV_LANDED_STATE landed_state() const { return MAV_LANDED_STATE_UNDEFINED; }
Plane does NOT have it ????
Copter has it: GCS_MAVLINK_Copter::landed_state()
but is protected, so we needed to mock it up
we can identify this be MAV_LANDED_STATE_UNDEFINED as value
we probably want to also take into account the arming state to mock something up
ugly as we will have vehicle dependency here
*/
    uint8_t landed_state = gcs().get_landed_state();

#if APM_BUILD_TYPE(APM_BUILD_ArduCopter)
// for copter we modify the landed states so as to reflect the 2 sec pre-take-off period
// the way it is done leads to a PREPARING_FOR_TAKEOFF before takeoff, but also after landing!
// we somehow would have to catch that it was flying before to suppress it
// but we don't care, the gimbal will do inits when ON_GROUND, and apply them when transitioned to PREPARING_FOR_TAKEOFF
    if (landed_state == MAV_LANDED_STATE_ON_GROUND && notify.flags.armed) landed_state = MAV_LANDED_STATE_PREPARING_FOR_TAKEOFF;
#endif

    mavlink_msg_autopilot_state_for_gimbal_device_send(
        _chan,
        _sysid, _compid,
        AP_HAL::micros64(),
        q,
        0, // uint32_t q_estimated_delay_us,
        vel.x, vel.y, vel.z,
        0, // uint32_t v_estimated_delay_us,
        yawrate,
        estimator_status, landed_state,
        angular_velocity_z);
}


void BP_Mount_STorM32_MAVLink::send_system_time(void)
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


void BP_Mount_STorM32_MAVLink::send_rc_channels(void)
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
        AP_HAL::millis(),
        16,
        RCHALIN(0), RCHALIN(1), RCHALIN(2), RCHALIN(3), RCHALIN(4), RCHALIN(5), RCHALIN(6), RCHALIN(7),
        RCHALIN(8), RCHALIN(9), RCHALIN(10), RCHALIN(11), RCHALIN(12), RCHALIN(13), RCHALIN(14), RCHALIN(15),
        0, 0,
        0);
}


void BP_Mount_STorM32_MAVLink::send_banner(void)
{
    if (_got_device_info) {
        // we have lots of info

        gcs().send_text(MAV_SEVERITY_INFO, "MNT%u: gimbal at %u%s",
                _instance + 1,
                _compid,
                (is_primary()) ? ", is primary" : ""); // %u vs %d ???

        gcs().send_text(MAV_SEVERITY_INFO, "MNT%u: %s %s fw:%u.%u.%u.%u",
                _instance + 1,
                _device_info.vendor_name,
                _device_info.model_name,
                (unsigned)(_device_info.firmware_version & 0x000000FF),
                (unsigned)((_device_info.firmware_version & 0x0000FF00) >> 8),
                (unsigned)((_device_info.firmware_version & 0x00FF0000) >> 16),
                (unsigned)((_device_info.firmware_version & 0xFF000000) >> 24));

    } else
    if (_compid) {
        // we have some info

        gcs().send_text(MAV_SEVERITY_INFO, "MNT%u: gimbal at %u%s", _instance+1, _compid, (is_primary())?", is primary":""); // %u vs %d ???

    } else {
        // we don't know yet anything

        gcs().send_text(MAV_SEVERITY_INFO, "MNT%u: no gimbal yet", _instance+1);
    }
}


//------------------------------------------------------
// helper
//------------------------------------------------------

bool BP_Mount_STorM32_MAVLink::prearmchecks_do(void)
{
    // unhealthy until gimbal has fully passed the startup sequence
    if (!_initialised || !_prearmchecks_ok || !_armed) {
        _prearmcheck_last = false;
        return false;
    }

    // unhealthy if attitude status is NOT received within the last second
    if (AP_HAL::millis() - _received_attitude_status_tlast_ms > 1000) {
        _prearmcheck_last = false;
        return false;
    }

    // check failure flags
    // we also check for GIMBAL_DEVICE_ERROR_FLAGS_NO_MANAGER, it essentially only means that STorM32 got GIMBAL_DEVICE_SET_ATTITUDE messages
    const uint32_t FAILURE_FLAGS =
            GIMBAL_DEVICE_ERROR_FLAGS_ENCODER_ERROR |
            GIMBAL_DEVICE_ERROR_FLAGS_POWER_ERROR |
            GIMBAL_DEVICE_ERROR_FLAGS_MOTOR_ERROR |
            GIMBAL_DEVICE_ERROR_FLAGS_SOFTWARE_ERROR |
            GIMBAL_DEVICE_ERROR_FLAGS_COMMS_ERROR |
            GIMBAL_DEVICE_ERROR_FLAGS_NO_MANAGER;

    if ((_received_device_failure_flags & FAILURE_FLAGS) > 0) {
        _prearmcheck_last = false;
        return false;
    }

    // if we get this far the mount is healthy

    if (!_prearmcheck_last) {
        _prearmcheck_last = true;
        gcs().send_text(MAV_SEVERITY_INFO, "PreArm: MNT%u: prearm checks passed", _instance+1);
    }

    return true;
}


//------------------------------------------------------
// MAVLink MOUNT_STATUS forwarding
//------------------------------------------------------

// forward a MOUNT_STATUS message to ground, this is only to make MissionPlanner and alike happy
void BP_Mount_STorM32_MAVLink::send_mount_status_to_ground(tMountStatus &status)
{
    // space is checked by send_to_ground()

    mavlink_mount_status_t msg = {
        pointing_a : (int32_t)(status.pitch_deg*100.0f),
        pointing_b : (int32_t)(status.roll_deg*100.0f),
        pointing_c : (int32_t)(status.yaw_deg*100.0f),
        target_system : 0,
        target_component : 0 };

    send_to_ground(MAVLINK_MSG_ID_MOUNT_STATUS, (const char*)&msg);
}


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

        if (c.get_chan() == _chan) continue; // the gimbal is on this channel
//        if (GCS_MAVLINK::mavtype_is_on_channel(MAV_TYPE_GIMBAL, c.get_chan())) continue;
//        if (!GCS_MAVLINK::mavtype_is_on_channel(MAV_TYPE_GCS, c.get_chan())) continue;

        if (!c.is_active()) continue;
        if (entry->max_msg_len + GCS_MAVLINK::packet_overhead_chan(c.get_chan()) > c.get_uart()->txspace()) {
            continue; // no space on this channel
        }
        c.send_message(pkt, entry);
    }
}


