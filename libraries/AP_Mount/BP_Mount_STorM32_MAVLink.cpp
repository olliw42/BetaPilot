//*****************************************************
//OW
// (c) olliw, www.olliw.eu, GPL3
// STorM32 mount backend class
//*****************************************************

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

two modes/protocols of operation are supported
1. ArduPilot like, largely mimics Gremsy gimbal driver = PROTOCOL_ARDUPILOT_LIKE
   we could mimic a super primitive v2 gimbal manager, primary is always autopilot, secondary always our GCS
   - only sends GIMBAL_MANAGER_STATUS
   - don' handle MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE
   - handling of request is needed
   - GIMBAL_MANAGER_SET_PITCHYAW, MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW ? ArduPilot's handling needs to be corrected, also doesn't respect pan
2. STorM32 is gimbal manger, using STorM32 gimbal manager messages = PROTOCOL_STORM32_GIMBAL_MANAGER
*/
//*****************************************************

/*
 ZFLAGS
 0:   protocol is auto determined
 1:   protocol forced to PROTOCOL_ARDUPILOT_LIKE
 2:   protocol forced to PROTOCOL_STORM32_GIMBAL_MANAGER
 8:   only streaming
      only sends out RC_CHANNLES, AUTOPILOT_STATE_FOR_GIMBAL for STorM32-Link
      this mode could in principle be replaced by asking for the streams, but since AP isn't streaming reliably we don't
 16:  do not send AUTOPILOT_STATE_FOR_GIMBAL_EXT
 64:  do not use 3way photo-video switch mode for 'Camera Mode Toggle' aux function
 128: do not log

 in all modes sends MOUNT_STATUS to ground, so that "old" things like MP etc can see the gimbal orientation
 listens to STORM32_GIMBAL_DEVICE_STATUS to send out MOUNT_STATUS in sync
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
// STorM32 states
//******************************************************
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

    _armingchecks_enabled = false;
    _prearmchecks_all_ok = false;
    _prearmcheck.updated = false;
    _prearmcheck.enabled_flags = 0;
    _prearmcheck.fail_flags = 0;
    _prearmcheck.fail_flags_last = UINT32_MAX;

    _device.received_flags = 0;
    _device.received_failure_flags = 0;

    _yaw_lock = false; // STorM32 doesn't currently support earth frame, so we need to ensure this is false
    _is_yaw_lock = false;

    _mode = MAV_MOUNT_MODE_RC_TARGETING;

    _sendonly = false;
    _should_log = true;
    _got_radio_rc_channels = false; // disable sending rc channels when RADIO_RC_CHANNELS messages are detected
    _send_autopilotstateext = true;
    _use_3way_photo_video = true;

    _protocol = PROTOCOL_UNDEFINED;
    _protocol_auto_cntdown = PROTOCOL_AUTO_TIMEOUT_CNT;
    _qshot.mode = MAV_QSHOT_MODE_UNDEFINED;
    _qshot.mode_last = MAV_QSHOT_MODE_UNDEFINED;

    if (_params.zflags & 0x01) { // 1 is set
        _protocol = PROTOCOL_ARDUPILOT_LIKE;
    } else
    if (_params.zflags & 0x02) { // 2 is set
        _protocol = PROTOCOL_STORM32_GIMBAL_MANAGER;
    }
    if (_params.zflags & 0x08) _sendonly = true;
    // we currently always do it if (_params.zflags & 0x10) _send_autopilotstateext = false;
    if (_params.zflags & 0x40) _use_3way_photo_video = !_use_3way_photo_video;
    if (_params.zflags & 0x80) _should_log = false;

    _camera_compid = 0; // camera not yet discovered
    _camera_mode = CAMERA_MODE_UNDEFINED;
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
                if (_send_autopilotstateext) send_autopilot_state_for_gimbal_device_ext();
            }
            break;

        case TASK_SLOT1:
            if (_sendonly) break; // don't do any control messages
            if (_initialised) { // we do it when the startup sequence has been fully completed
                set_and_send_target_angles();  // GRRRRRR this is used to determine hasmanager!!
            }
            break;

        case TASK_SLOT3:
            if (_compid) { // we send it as soon as we have found the gimbal
                if (!_got_radio_rc_channels) send_rc_channels();
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
    GimbalTarget gtarget;

    if (_qshot.mode == MAV_QSHOT_MODE_UNDEFINED) {
        enum MAV_MOUNT_MODE mntmode = get_mode();
        if (!get_target_angles_mount(gtarget, mntmode)) return; // don't send
        update_gimbal_device_flags_mount(mntmode);
    } else {
        if (!get_target_angles_qshot(gtarget)) return; // don't send
        update_gimbal_device_flags_qshot();
    }

    _qshot.mode_last = _qshot.mode;
    if (_qshot.mode == UINT8_MAX) return; // is in hold, don't send

    if (_protocol == PROTOCOL_ARDUPILOT_LIKE) {
        send_gimbal_device_set_attitude(gtarget);
    } else
    if (_protocol == PROTOCOL_STORM32_GIMBAL_MANAGER) {
        // only send when autopilot client is active, this reduces traffic
        if (_manager.ap_client_is_active) {
            update_gimbal_manager_flags();
            send_storm32_gimbal_manager_control(gtarget);
        }
    }
}


//------------------------------------------------------
// V2 GIMBAL DEVICE, ArduPilot like
//------------------------------------------------------

bool BP_Mount_STorM32_MAVLink::get_target_angles_mount(GimbalTarget &gtarget, enum MAV_MOUNT_MODE mntmode)
{
    MountTarget mtarget_rad = {};

    // update based on mount mode
    switch (mntmode) {

        // move mount to a "retracted" position.  We disable motors
        case MAV_MOUNT_MODE_RETRACT:
            gtarget.set(TARGET_MODE_RETRACT);
            return true;

        // move mount to a neutral position, typically pointing forward
        case MAV_MOUNT_MODE_NEUTRAL: {
            const Vector3f &vec_deg = _params.neutral_angles.get();
            gtarget.set_from_vec_deg(vec_deg, TARGET_MODE_NEUTRAL);
            return true;
        }

        // use angle or rate targets provided by a mavlink message or mission command
        case MAV_MOUNT_MODE_MAVLINK_TARGETING:
            switch (mnt_target.target_type) {
            case MountTargetType::ANGLE:
                gtarget.set_angle(mnt_target.angle_rad);
                return true;
            case MountTargetType::RATE:
                // we do not yet support rate, so do it SToRM32 driver like
                MountTarget rate_mtarget_rad {};
                update_angle_target_from_rate(mnt_target.rate_rads, rate_mtarget_rad);
                gtarget.set_angle(rate_mtarget_rad);
                return true;
            }
            break;

        // RC radio manual angle control, but with stabilization from the AHRS
        // update targets using pilot's RC inputs
        case MAV_MOUNT_MODE_RC_TARGETING:
/* XX ??
            if (get_rc_rate_target(mtarget_rad)) {
                // we do not yet support rate, so do it SToRM32 driver like
                MountTarget rate_mtarget_rad {};
                update_angle_target_from_rate(mtarget_rad, rate_mtarget_rad);
                gtarget.set_angle(rate_mtarget_rad);
                return true;
            } else
            if (get_rc_angle_target(mtarget_rad)) {
                gtarget.set_angle(mtarget_rad);
                return true;
            }
*/
            break;

        // point mount to a GPS point given by the mission planner
        case MAV_MOUNT_MODE_GPS_POINT:
            if (get_angle_target_to_roi(mtarget_rad)) {
                gtarget.set_angle(mtarget_rad);
                return true;
            }
            break;

        // point mount to home
        case MAV_MOUNT_MODE_HOME_LOCATION:
            if (get_angle_target_to_home(mtarget_rad)) {
                gtarget.set_angle(mtarget_rad);
                return true;
            }
            break;

        case MAV_MOUNT_MODE_SYSID_TARGET:
            if (get_angle_target_to_sysid(mtarget_rad)) {
                gtarget.set_angle(mtarget_rad);
                return true;
            }
            break;

        default:
            // unknown mode so do nothing
            break;
    }

    return false;
}


void BP_Mount_STorM32_MAVLink::update_gimbal_device_flags_mount(enum MAV_MOUNT_MODE mntmode)
{
    _device.flags_for_gimbal = 0;

    switch (mntmode) {
        case MAV_MOUNT_MODE_RETRACT:
            _device.flags_for_gimbal |= GIMBAL_DEVICE_FLAGS_RETRACT;
            break;
        case MAV_MOUNT_MODE_NEUTRAL:
            _device.flags_for_gimbal |= GIMBAL_DEVICE_FLAGS_NEUTRAL;
            break;
        default:
            break;
    }

    _device.flags_for_gimbal |= GIMBAL_DEVICE_FLAGS_ROLL_LOCK | GIMBAL_DEVICE_FLAGS_PITCH_LOCK;
    if (_is_yaw_lock) _device.flags_for_gimbal |= GIMBAL_DEVICE_FLAGS_YAW_LOCK;

    // set either YAW_IN_VEHICLE_FRAME or YAW_IN_EARTH_FRAME, to indicate new message format, STorM32 will reject otherwise
    _device.flags_for_gimbal |= GIMBAL_DEVICE_FLAGS_YAW_IN_VEHICLE_FRAME;
}


//------------------------------------------------------
// STORM32 GIMBAL MANAGER, QShot
//------------------------------------------------------

bool BP_Mount_STorM32_MAVLink::get_target_angles_qshot(GimbalTarget &gtarget)
{
    switch (_qshot.mode) {
        case MAV_QSHOT_MODE_UNDEFINED:
            return false;

        case MAV_QSHOT_MODE_GIMBAL_RETRACT:
            return get_target_angles_mount(gtarget, MAV_MOUNT_MODE_RETRACT);

        case MAV_QSHOT_MODE_GIMBAL_NEUTRAL:
            return get_target_angles_mount(gtarget, MAV_MOUNT_MODE_NEUTRAL);

        case MAV_QSHOT_MODE_GIMBAL_RC_CONTROL:
            return get_target_angles_mount(gtarget, MAV_MOUNT_MODE_RC_TARGETING);

        case MAV_QSHOT_MODE_POI_TARGETING:
            return get_target_angles_mount(gtarget, MAV_MOUNT_MODE_GPS_POINT);

        case MAV_QSHOT_MODE_HOME_TARGETING:
            return get_target_angles_mount(gtarget, MAV_MOUNT_MODE_HOME_LOCATION);

        case MAV_QSHOT_MODE_SYSID_TARGETING:
            return get_target_angles_mount(gtarget, MAV_MOUNT_MODE_SYSID_TARGET);

        default:
            // in all other modes we don't do nothing, i.e. just send out something
            // it is the job of the supervisor to get things right by setting the activity
            return get_target_angles_mount(gtarget, MAV_MOUNT_MODE_MAVLINK_TARGETING);
    }

    return false;
}


void BP_Mount_STorM32_MAVLink::update_gimbal_device_flags_qshot(void)
{
    switch (_qshot.mode) {
        case MAV_QSHOT_MODE_GIMBAL_RETRACT:
            update_gimbal_device_flags_mount(MAV_MOUNT_MODE_RETRACT);
            break;

        case MAV_QSHOT_MODE_GIMBAL_NEUTRAL:
            update_gimbal_device_flags_mount(MAV_MOUNT_MODE_NEUTRAL);
            break;

        default:
            // currently, anything not retract/neutral is good
            update_gimbal_device_flags_mount(MAV_MOUNT_MODE_ENUM_END);
            break;
    }
}


void BP_Mount_STorM32_MAVLink::update_gimbal_manager_flags(void)
{
    // we play it simple and do not attempt to claim supervision nor activity
    // we thus leave this to other components, e.g. a gcs, to set this
    _manager.flags_for_gimbal = MAV_STORM32_GIMBAL_MANAGER_FLAGS_NONE;

    //TODO: we could claim supervisor and activity if the qshot mode suggests so
    // what is then about missions ???? should qshots include a mission mode??
    // what about scripts?
    // it's not bad as it is
}


//------------------------------------------------------
// Health, Prearm, find
//------------------------------------------------------

// return true if healthy
bool BP_Mount_STorM32_MAVLink::healthy() const
{
    return const_cast<BP_Mount_STorM32_MAVLink*>(this)->prearmchecks_do(); // yes, ugly, but I have not overdesigned the backend
}


void BP_Mount_STorM32_MAVLink::find_gimbal(void)
{
    // search for gimbal until armed
    if (hal.util->get_soft_armed()) {
        return;
    }

    uint32_t tnow_ms = AP_HAL::millis();

    // search for gimbal in routing table

    if (!_compid) {
        // we expect that instance 0 has compid = MAV_COMP_ID_GIMBAL, instance 1 has compid = MAV_COMP_ID_GIMBAL2, etc
        uint8_t compid = (_instance == 0) ? MAV_COMP_ID_GIMBAL : MAV_COMP_ID_GIMBAL2 + (_instance - 1);
        if (GCS_MAVLINK::find_by_mavtype_and_compid(MAV_TYPE_GIMBAL, compid, _sysid, _chan) && (_sysid == mavlink_system.sysid)) {
            _compid = compid;
            _request_device_info_tlast_ms = (tnow_ms < 900) ? 0 : tnow_ms - 900; // start sending requests in 100 ms
        } else {
            // have not yet found a gimbal so return
            return;
        }
    }
/*

    // search for a mavlink enabled gimbal
    if (_link == nullptr) {
        // we expect that instance 0 has compid = MAV_COMP_ID_GIMBAL, instance 1 has compid = MAV_COMP_ID_GIMBAL2, etc
        uint8_t compid = (_instance == 0) ? MAV_COMP_ID_GIMBAL : MAV_COMP_ID_GIMBAL2 + (_instance - 1);
        _link = GCS_MAVLINK::find_by_mavtype_and_compid(MAV_TYPE_GIMBAL, compid, _sysid);
        if (_link == nullptr ||  && (_sysid != mavlink_system.sysid)) {
            // have not yet found a gimbal so return
            return;
        }

        _compid = compid;
        _request_device_info_tlast_ms = (tnow_ms < 900) ? 0 : tnow_ms - 900; // start sending requests in 100 ms
    }
*/

    // request GIMBAL_DEVICE_INFORMATION
    if (!_got_device_info) {
        if (tnow_ms - _request_device_info_tlast_ms > 1000) {
            _request_device_info_tlast_ms = tnow_ms;
            send_request_gimbal_device_information();
        }
        return;
    }

    // we don't know yet what we should do
    if (_protocol == PROTOCOL_UNDEFINED) {
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
        case MAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS:
            if (_protocol == PROTOCOL_STORM32_GIMBAL_MANAGER) break; // do not allow switching from gimbal manager mode to gimbal device mode
            if (_protocol_auto_cntdown) _protocol_auto_cntdown--; // delay switching to gimbal device mode, to give gimbal manager messages a chance
            if (!_protocol_auto_cntdown) {
                _protocol = PROTOCOL_ARDUPILOT_LIKE;
            }
            break;

        case MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_STATUS: // 60011
            _protocol = PROTOCOL_STORM32_GIMBAL_MANAGER;
            _protocol_auto_cntdown = PROTOCOL_AUTO_TIMEOUT_CNT;
            break;
    }
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
/*
    if (!isnan(_device_info.roll_min)) _params.roll_angle_min.set_default(degrees(_device_info.roll_min));
    if (!isnan(_device_info.roll_max)) _params.roll_angle_max.set_default(degrees(_device_info.roll_max));
    if (!isnan(_device_info.pitch_min)) _params.pitch_angle_min.set_default(degrees(_device_info.pitch_min));
    if (!isnan(_device_info.pitch_max)) _params.pitch_angle_max.set_default(degrees(_device_info.pitch_max));
    if (!isnan(_device_info.yaw_min)) _params.yaw_angle_min.set_default(degrees(_device_info.yaw_min));
    if (!isnan(_device_info.yaw_max)) _params.yaw_angle_max.set_default(degrees(_device_info.yaw_max)); */

    if (!isnan(_device_info.roll_min)) _params.roll_angle_min.set(degrees(_device_info.roll_min));
    if (!isnan(_device_info.roll_max)) _params.roll_angle_max.set(degrees(_device_info.roll_max));
    if (!isnan(_device_info.pitch_min)) _params.pitch_angle_min.set(degrees(_device_info.pitch_min));
    if (!isnan(_device_info.pitch_max)) _params.pitch_angle_max.set(degrees(_device_info.pitch_max));
    if (!isnan(_device_info.yaw_min)) _params.yaw_angle_min.set(degrees(_device_info.yaw_min));
    if (!isnan(_device_info.yaw_max)) _params.yaw_angle_max.set(degrees(_device_info.yaw_max));

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

    _device.received_flags = payload.flags;
    // TODO: handle case when received device_flags are not equal to those we send, set with update_gimbal_device_flags_for_gimbal()

    _device.received_failure_flags = payload.failure_flags;

    // used for health check
    _device.received_tlast_ms = AP_HAL::millis();

    // logging
    float roll_rad, pitch_rad, yaw_rad;
    GimbalQuaternion quat(payload.q[0], payload.q[1], payload.q[2], payload.q[3]);
    quat.to_gimbal_euler(roll_rad, pitch_rad, yaw_rad);

    BP_LOG("MTS0", BP_LOG_MTS_ATTITUDESTATUS_HEADER,
        degrees(roll_rad),
        degrees(pitch_rad),
        degrees(yaw_rad),
        degrees(payload.delta_yaw),
        degrees(AP::ahrs().yaw),
        payload.flags,
        payload.failure_flags);

    // forward to ground as MOUNT_STATUS message
    if (payload.target_system) { // trigger sending of MOUNT_STATUS to ground only if target_sysid = 0
        return;
    }
    if (!is_primary()) {
        return;
    }

    MountStatus status = {
        .roll_deg = degrees(roll_rad),
        .pitch_deg = degrees(pitch_rad),
        .yaw_deg = degrees(yaw_rad) };

    send_mount_status_to_ground(status); // MissionPlaner now "understands" gimbal device attitude status, but doesn't use it for campoint, so we still need to send
}


void BP_Mount_STorM32_MAVLink::handle_msg(const mavlink_message_t &msg)
{
    if (!_initialised && _compid) {
        if (_protocol == PROTOCOL_UNDEFINED) determine_protocol(msg);
        return;
    }

    switch (msg.msgid) {
        // listen to qshot commands and messages to track changes in qshot mode
        // these may come from anywhere
        case MAVLINK_MSG_ID_COMMAND_LONG: { // 76
            mavlink_command_long_t payload;
            mavlink_msg_command_long_decode(&msg, &payload);
            switch (payload.command) {
                case MAV_CMD_QSHOT_DO_CONFIGURE: // 60020
                    uint8_t new_mode = payload.param1;
                    if (new_mode == MAV_QSHOT_MODE_UNDEFINED) {
                        _qshot.mode = MAV_QSHOT_MODE_UNDEFINED;
                    }
                    if (new_mode != _qshot.mode) {
                        _qshot.mode = UINT8_MAX; // mode change requested, so put it into hold, must be acknowledged by qshot status
                    }
                break;
            }
            }break;

        case MAVLINK_MSG_ID_QSHOT_STATUS: { // 60020
            mavlink_qshot_status_t payload;
            mavlink_msg_qshot_status_decode(&msg, &payload);
            _qshot.mode = payload.mode; // also sets it if it was put on hold in the above
            }break;

        // listen to RADIO_RC_CHANNELS messages to stop sending RC_CHANNELS
        case MAVLINK_MSG_ID_RADIO_RC_CHANNELS: { // 60045
            _got_radio_rc_channels = true;
            }break;
    }

    // this msg is not from our system
    if (msg.sysid != _sysid) {
        return;
    }

    // search for a MAVLink camera
    // we are somewhat overly strict in that we require both the comp_id and the mav_type to be camera
    if (!_camera_compid && (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) &&
            (msg.compid >= MAV_COMP_ID_CAMERA) && (msg.compid <= MAV_COMP_ID_CAMERA6)) {
        mavlink_heartbeat_t payload;
        mavlink_msg_heartbeat_decode(&msg, &payload);
        if ((payload.autopilot == MAV_AUTOPILOT_INVALID) && (payload.type == MAV_TYPE_CAMERA)) {
            _camera_compid = msg.compid;
        }
    }

    // listen to STORM32_GIMBAL_MANGER_STATUS to detect activity of the autopilot client
    switch (msg.msgid) {
        case MAVLINK_MSG_ID_STORM32_GIMBAL_MANAGER_STATUS: { // 60011
            mavlink_storm32_gimbal_manager_status_t payload;
            mavlink_msg_storm32_gimbal_manager_status_decode(&msg, &payload);
            if (payload.gimbal_id != _compid) break; // not for our gimbal device
            _manager.ap_client_is_active =
                    (payload.supervisor != MAV_STORM32_GIMBAL_MANAGER_CLIENT_NONE) && // a client is supervisor
                    (payload.manager_flags & MAV_STORM32_GIMBAL_MANAGER_FLAGS_CLIENT_AUTOPILOT_ACTIVE); // and autopilot is active
            }break;
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
            _armed = ((storm32_state == STORM32STATE_NORMAL) || (storm32_state == STORM32STATE_STARTUP_FASTLEVEL));
            if ((payload.custom_mode & 0x80000000) == 0) { // we don't follow all changes, but just toggle it to true once
                _prearmchecks_ok = true;
            }
            if (_armingchecks_enabled && !_prearmchecks_all_ok && prearmchecks_all_ok()) {
                _prearmchecks_all_ok = true;
                gcs().send_text(MAV_SEVERITY_INFO, "MNT%u: prearm checks passed", _instance+1);
            }
            }break;

        #define MAVGIMBAL_EVENT_ID(x)  ((uint32_t)154 << 24) + (uint32_t)(x)

        case MAVLINK_MSG_ID_EVENT: {
            mavlink_event_t payload;
            mavlink_msg_event_decode(&msg, &payload);
            if (payload.id != MAVGIMBAL_EVENT_ID(0)) break; // not our event id
            struct PACKED tPrearmCheckEventArgument {
                uint32_t enabled_flags;
                uint32_t fail_flags;
            };
            tPrearmCheckEventArgument* p = (tPrearmCheckEventArgument*)payload.arguments;
            _prearmcheck.enabled_flags = p->enabled_flags;
            _prearmcheck.fail_flags = p->fail_flags & p->enabled_flags; // only keep enabled flags
            _prearmcheck.updated = true;
//          gcs().send_text(MAV_SEVERITY_INFO, "Event: %d %d", (int)_prearmcheck.enabled_flags, (int)_prearmcheck.fail_flags);
            if (_prearmcheck.fail_flags != _prearmcheck.fail_flags_last) {
                _prearmcheck.fail_flags_last = _prearmcheck.fail_flags;
                _prearmcheck.sendtext_tlast_ms = 0; // a change occurred, so force send immediately
            }
            }break;
    }
}


//------------------------------------------------------
// MAVLink send functions
//------------------------------------------------------

void BP_Mount_STorM32_MAVLink::send_request_gimbal_device_information(void)
{
    if (!HAVE_PAYLOAD_SPACE(_chan, COMMAND_LONG)) {
        return;
    }

    mavlink_msg_command_long_send(
        _chan,
        _sysid, _compid,
        MAV_CMD_REQUEST_MESSAGE, 0,
        MAVLINK_MSG_ID_GIMBAL_DEVICE_INFORMATION, 0, 0, 0, 0, 0, 0);
}


void BP_Mount_STorM32_MAVLink::GimbalTarget::get_q_array(float* q_array)
{
    switch (mode) {
        case TARGET_MODE_NEUTRAL:
        case TARGET_MODE_ANGLE: {
            Quaternion q;
            q.from_euler(roll, pitch, yaw);
            q_array[0] = q.q1; q_array[1] = q.q2; q_array[2] = q.q3; q_array[3] = q.q4;
            break;
        }
        default:
            q_array[0] = q_array[1] = q_array[2] = q_array[3] = NAN;
    }
}


void BP_Mount_STorM32_MAVLink::send_gimbal_device_set_attitude(GimbalTarget &gtarget)
{
    if (!HAVE_PAYLOAD_SPACE(_chan, GIMBAL_DEVICE_SET_ATTITUDE)) {
        return;
    }

    float q_array[4];
    gtarget.get_q_array(q_array);

    mavlink_msg_gimbal_device_set_attitude_send(
        _chan,
        _sysid, _compid,
        _device.flags_for_gimbal, // gimbal device flags
        q_array,                  // attitude as a quaternion
        NAN, NAN, NAN);           // angular velocities

    BP_LOG("MTC0", BP_LOG_MTC_GIMBALCONTROL_HEADER,
        (uint8_t)1, // GIMBAL_DEVICE_SET_ATTITUDE
        degrees(gtarget.roll), degrees(gtarget.pitch), degrees(gtarget.yaw),
        _device.flags_for_gimbal, (uint16_t)0,
        gtarget.mode,
        (uint8_t)0);
}


void BP_Mount_STorM32_MAVLink::send_storm32_gimbal_manager_control(GimbalTarget &gtarget)
{
    if (!HAVE_PAYLOAD_SPACE(_chan, STORM32_GIMBAL_MANAGER_CONTROL)) {
        return;
    }

    float q_array[4];
    gtarget.get_q_array(q_array);

    mavlink_msg_storm32_gimbal_manager_control_send(
        _chan,
        _sysid, _compid,
        _compid, // gimbal_id
        MAV_STORM32_GIMBAL_MANAGER_CLIENT_AUTOPILOT,
        _device.flags_for_gimbal, _manager.flags_for_gimbal,
        q_array,
        NAN, NAN, NAN); // float angular_velocity_x, float angular_velocity_y, float angular_velocity_z

    BP_LOG("MTC0", BP_LOG_MTC_GIMBALCONTROL_HEADER,
        (uint8_t)2, // STORM32_GIMBAL_MANAGER_CONTROL
        degrees(gtarget.roll), degrees(gtarget.pitch), degrees(gtarget.yaw),
        _device.flags_for_gimbal, _manager.flags_for_gimbal,
        gtarget.mode,
        _qshot.mode);
}


void BP_Mount_STorM32_MAVLink::send_autopilot_state_for_gimbal_device_ext(void)
{
    if (!HAVE_PAYLOAD_SPACE(_chan, AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_EXT)) {
        return;
    }

    const AP_AHRS &ahrs = AP::ahrs();

#if APM_BUILD_TYPE(APM_BUILD_ArduCopter)
    Vector3f airspeed_vec_bf;
    if (!ahrs.airspeed_vector_true(airspeed_vec_bf)) {
        // if we don't have an airspeed estimate then we don't have a valid wind estimate on copters
        return;
    }
#endif

/*
 https://github.com/ArduPilot/ardupilot/issues/6571
 v_ground = v_air + v_wind // note direction of wind
 there are soo many different accessors, estimates, etc
 vfr_hud_airspeed(),
 ahrs.groundspeed()
 ahrs.groundspeed_vector()
 ahrs.yaw, ahrs.yaw_sensor
 ahrs.airspeed_estimate(airspeed)
 ahrs.airspeed_estimate_true(airspeed)
 ahrs.airspeed_vector_true(airspeed_vec_bf) // gives it in BF, not NED!
 AP::gps().ground_speed()
 AP_Airspeed::get_singleton()->get_airspeed()
 ????
 this gives some good insight!?
 ahrs.groundspeed_vector() -> search for AP_AHRS_DCM::groundspeed_vector(void)
 if airspeed_estimate_true(airspeed) then calculates it as gndVelADS = airspeed_vector + wind2d
 else if gotGPS then gndVelGPS = Vector2f(cosf(cog), sinf(cog)) * AP::gps().ground_speed()
 => AP does indeed do vg = va + vw
 => shows how ground speed is estimated
 question: how does this relate to ahrs.get_velocity_NED(vground)?
 ahrs.airspeed_estimate(airspeed)
 does true_airspeed_vec = nav_vel - wind_vel
 this suggests that nav_vel is a good ground speed
 also suggests that airspeed, wind are in NED too
*/
    Vector3f wind;
    wind = ahrs.wind_estimate();

    float vehicle_heading = ahrs.yaw;
    float wind_heading = atan2f(-wind.y, -wind.x);
    float ground_heading = NAN;
    float air_heading = NAN;

    float correction_angle = NAN;
    Vector3f vground;
    if (ahrs.get_velocity_NED(vground)) { // ??? is this correct, really ground speed ???
        Vector3f vair = vground - wind;
        float vg2 = vground.x * vground.x + vground.y * vground.y;
        float va2 = vair.x * vair.x + vair.y * vair.y;
        float vgva = vground.x * vair.x + vground.y * vair.y;
        correction_angle = acosf(vgva / sqrtf(vg2 * va2));
        if ((vground.x * vair.y - vground.y * vair.x) < 0.0f) correction_angle = -correction_angle;

        ground_heading = atan2f(vground.y, vground.x);
        air_heading = atan2f(vair.y, vair.x);
    }
/* we currently don't send, just log, we need to work it out what we want to do
    mavlink_msg_autopilot_state_for_gimbal_device_ext_send(
        _chan,
        _sysid, _compid,
        AP_HAL::micros64(),
        wind.x, wind.y, correction_angle); */

    BP_LOG("MTLE", BP_LOG_MTLE_AUTOPILOTSTATEEXT_HEADER,
        wind.x, wind.y, degrees(correction_angle),
        degrees(vehicle_heading), degrees(wind_heading), degrees(ground_heading), degrees(air_heading));
}


enum THISWOULDBEGREATTOHAVE {
    MAV_LANDED_STATE_PREPARING_FOR_TAKEOFF = 5,
};


void BP_Mount_STorM32_MAVLink::send_autopilot_state_for_gimbal_device(void)
{
    if (!HAVE_PAYLOAD_SPACE(_chan, AUTOPILOT_STATE_FOR_GIMBAL_DEVICE)) {
        return;
    }

    const AP_AHRS &ahrs = AP::ahrs();

    Quaternion quat;
    if (!ahrs.get_quaternion(quat)) { // it returns a bool, so it's a good idea to consider it
        quat.q1 = quat.q2 = quat.q3 = quat.q4 = NAN;
    }
    float q[4] = { quat.q1, quat.q2, quat.q3, quat.q4 };

    // comment in AP_AHRS.cpp says "Must only be called if have_inertial_nav() is true", but probably not worth checking
    Vector3f vel;
    if (!ahrs.get_velocity_NED(vel)) { // it returns a bool, so it's a good idea to consider it
        vel.x = vel.y = vel.z = 0.0f; // or NAN ???
    }

    float angular_velocity_z = ahrs.get_yaw_rate_earth(); // NAN;

    float yawrate = NAN;
    // see https://github.com/ArduPilot/ardupilot/issues/22564
    const AP_Vehicle *vehicle = AP::vehicle();
    Vector3f rate_ef_targets;
    if ((vehicle != nullptr) && vehicle->get_rate_ef_targets(rate_ef_targets)) {
        yawrate = rate_ef_targets.z;
    }

// TODO: how do notify.flags.armed and hal.util->get_soft_armed() compare against each other, also across vehicles?
/* old:
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
/*
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

/* estimator status
 btw STorM32 only listens to ESTIMATOR_ATTITUDE and ESTIMATOR_VELOCITY_VERT
 ahrs.healthy() becomes true during flip of quaternion => no-go
 ahrs.initialised() is simply set after AP_AHRS_NAVEKF_SETTLE_TIME_MS 20000 !!
                    it becomes true some secs after ESTIMATOR_ATTITUDE|ESTIMATOR_VELOCITY_VERT are set
 ESTIMATOR_ATTITUDE can be set during flip of quaternion => no-go
 ESTIMATOR_VELOCITY_VERT are set briefly after the quaternion flip ??Q: really, couldn't it also be raised in the flip?
 ESTIMATOR_VELOCITY_VERT are set however even if there is no gps or alike! I find it hard to trust the data
 => we fake it so:
 for ESTIMATOR_ATTITUDE we delay the flag being raised by few secs
 for ESTIMATOR_VELOCITY_VERT we check for gps like in AP_AHRS_DCM::groundspeed_vector(void)
 btw STorM32 does also check for non-zero velocities for AHRSFix
*/
    uint16_t estimator_status = 0;
    static uint32_t tahrs_healthy_ms = 0;
    const bool ahrs_healthy = ahrs.healthy(); // it's a bit costly
    if (!tahrs_healthy_ms && ahrs_healthy) tahrs_healthy_ms = AP_HAL::millis();
    if (ahrs_healthy && (nav_estimator_status & ESTIMATOR_ATTITUDE) && ((AP_HAL::millis() - tahrs_healthy_ms) > 3000)) {
        estimator_status |= ESTIMATOR_ATTITUDE; // -> QFix
        if (ahrs.initialised() && (nav_estimator_status & ESTIMATOR_VELOCITY_VERT) && (AP::gps().status() >= AP_GPS::GPS_OK_FIX_2D)) {
            estimator_status |= ESTIMATOR_VELOCITY_VERT; // -> AHRSFix
        }
    }

/* landed state
 GCS_Common.cpp: virtual MAV_LANDED_STATE landed_state() const { return MAV_LANDED_STATE_UNDEFINED; }
 Copter has it: GCS_MAVLINK_Copter::landed_state(), yields ON_GROUND, TAKEOFF, IN_AIR, LANDING
 Plane has it: GCS_MAVLINK_Plane::landed_state(), only yields ON_GROUND or IN_AIR
 Blimp also has it, blimp not relevant for us
 but is protected, so we needed to mock it up
 we probably want to also take into account the arming state to mock something up
 ugly as we will have vehicle dependency here
*/
    uint8_t landed_state = gcs().get_landed_state();

#if APM_BUILD_TYPE(APM_BUILD_ArduCopter)
// for copter we modify the landed states so as to reflect the 2 sec pre-take-off period
// the way it is done leads to a PREPARING_FOR_TAKEOFF before takeoff, but also after landing!
// we somehow would have to catch that it was flying before to suppress it
// but we don't care, the gimbal will do inits when ON_GROUND, and apply them when transitioned to PREPARING_FOR_TAKEOFF
// it won't do it for other transitions, so e.g. also not for plane
    const AP_Notify &notify = AP::notify();
    if ((landed_state == MAV_LANDED_STATE_ON_GROUND) && notify.flags.armed) landed_state = MAV_LANDED_STATE_PREPARING_FOR_TAKEOFF;
#endif

    static uint32_t tlast_us = 0;
    uint32_t t_us = AP_HAL::micros();
    uint32_t dt_us = t_us - tlast_us;
    tlast_us = t_us;

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
        gcs().send_text(MAV_SEVERITY_INFO, "MNT%u: gimbal at %u%s", _instance+1, _compid, (is_primary()) ? ", is primary" : "");

        // we can convert the firmware version to STorM32 convention
        char c = (_device_info.firmware_version & 0x00FF0000) >> 16;
        if (c == '\0') c = ' '; else c += 'a' - 1;

        gcs().send_text(MAV_SEVERITY_INFO, "MNT%u: %s %s v%u.%u%c",
                _instance + 1,
                _device_info.vendor_name,
                _device_info.model_name,
                (unsigned)(_device_info.firmware_version & 0x000000FF),
                (unsigned)((_device_info.firmware_version & 0x0000FF00) >> 8),
                c
                );

        gcs().send_text(MAV_SEVERITY_INFO, "MNT%u: prearm checks %s", _instance+1, (_prearmchecks_all_ok) ? "passed" : "fail");

    } else
    if (_compid) {
        // we have some info
        gcs().send_text(MAV_SEVERITY_INFO, "MNT%u: gimbal at %u%s", _instance+1, _compid, (is_primary()) ? ", is primary" : "");
        gcs().send_text(MAV_SEVERITY_INFO, "MNT%u: prearm checks %s", _instance+1, (_prearmchecks_all_ok) ? "passed" : "fail");

    } else {
        // we don't know yet anything
        gcs().send_text(MAV_SEVERITY_INFO, "MNT%u: no gimbal yet", _instance+1);
    }
}


//------------------------------------------------------
// helper
//------------------------------------------------------
const uint32_t FAILURE_FLAGS =
        GIMBAL_DEVICE_ERROR_FLAGS_ENCODER_ERROR |
        GIMBAL_DEVICE_ERROR_FLAGS_POWER_ERROR |
        GIMBAL_DEVICE_ERROR_FLAGS_MOTOR_ERROR |
        GIMBAL_DEVICE_ERROR_FLAGS_SOFTWARE_ERROR |
        GIMBAL_DEVICE_ERROR_FLAGS_COMMS_ERROR;
        // GIMBAL_DEVICE_ERROR_FLAGS_NO_MANAGER;


bool BP_Mount_STorM32_MAVLink::prearmchecks_all_ok(void)
{
    return (_prearmchecks_ok && _armed && ((_device.received_failure_flags & FAILURE_FLAGS) == 0));
}


void BP_Mount_STorM32_MAVLink::send_prearmchecks_txt(void)
{
    if (_prearmcheck.available() && !_prearmchecks_ok) { // we got a new message, and it is still not ok
        _prearmcheck.updated = false;

        char txt[255];
        strcpy(txt, "");
        if (_prearmcheck.fail_flags & MAV_STORM32_GIMBAL_PREARM_FLAGS_IS_NORMAL) strcat(txt, "arm,");
        if (_prearmcheck.fail_flags & MAV_STORM32_GIMBAL_PREARM_FLAGS_IMUS_WORKING) strcat(txt, "imu,");
        if (_prearmcheck.fail_flags & MAV_STORM32_GIMBAL_PREARM_FLAGS_MOTORS_WORKING) strcat(txt, "mot,");
        if (_prearmcheck.fail_flags & MAV_STORM32_GIMBAL_PREARM_FLAGS_ENCODERS_WORKING) strcat(txt, "enc,");
        if (_prearmcheck.fail_flags & MAV_STORM32_GIMBAL_PREARM_FLAGS_VOLTAGE_OK) strcat(txt, "volt,");
        if (_prearmcheck.fail_flags & MAV_STORM32_GIMBAL_PREARM_FLAGS_VIRTUALCHANNELS_RECEIVING) strcat(txt, "chan,");
        if (_prearmcheck.fail_flags & MAV_STORM32_GIMBAL_PREARM_FLAGS_MAVLINK_RECEIVING) strcat(txt, "mav,");
        if (_prearmcheck.fail_flags & MAV_STORM32_GIMBAL_PREARM_FLAGS_STORM32LINK_QFIX) strcat(txt, "qfix,");
        if (_prearmcheck.fail_flags & MAV_STORM32_GIMBAL_PREARM_FLAGS_STORM32LINK_WORKING) strcat(txt, "stl,");
        if (_prearmcheck.fail_flags & MAV_STORM32_GIMBAL_PREARM_FLAGS_CAMERA_CONNECTED) strcat(txt, "cam,");
        if (_prearmcheck.fail_flags & MAV_STORM32_GIMBAL_PREARM_FLAGS_AUX0_LOW) strcat(txt, "aux0,");
        if (_prearmcheck.fail_flags & MAV_STORM32_GIMBAL_PREARM_FLAGS_AUX1_LOW) strcat(txt, "aux1,");
        if (txt[0] != '\0') {
            txt[strlen(txt)-1] = '\0';
        } else {
            strcpy(txt, "unknown");
        }
        gcs().send_text(MAV_SEVERITY_INFO, "MNT%u: prearm checks FAIL: %s", _instance+1, txt);
    } else
    if (!_initialised || !_prearmchecks_ok || !_armed) {
        gcs().send_text(MAV_SEVERITY_INFO, "MNT%u: prearm checks FAIL: arm", _instance+1);
    } else
    if (_prearmchecks_ok && (_device.received_failure_flags & FAILURE_FLAGS)) {
        char txt[255];
        strcpy(txt, "");
        if (_device.received_failure_flags & GIMBAL_DEVICE_ERROR_FLAGS_MOTOR_ERROR) strcat(txt, "mot,");
        if (_device.received_failure_flags & GIMBAL_DEVICE_ERROR_FLAGS_ENCODER_ERROR) strcat(txt, "enc,");
        if (_device.received_failure_flags & GIMBAL_DEVICE_ERROR_FLAGS_POWER_ERROR) strcat(txt, "volt,");
        if (txt[0] != '\0') {
            txt[strlen(txt)-1] = '\0';
            gcs().send_text(MAV_SEVERITY_INFO, "MNT%u: prearm checks FAIL: %s", _instance+1, txt);
        } else {
            gcs().send_text(MAV_SEVERITY_INFO, "MNT%u: prearm checks FAILURE FLAGS", _instance+1);
        }
    }
}


// this is called when ARMING_CHECK_ALL or ARMING_CHECK_CAMERA is set
bool BP_Mount_STorM32_MAVLink::prearmchecks_do(void)
{
    _armingchecks_enabled = true;

    if ((AP_HAL::millis() - _prearmcheck.sendtext_tlast_ms) > 30000) { // we haven't send it for a long time
        _prearmcheck.sendtext_tlast_ms = AP_HAL::millis();
        send_prearmchecks_txt();
    }

    if (!_prearmchecks_all_ok && prearmchecks_all_ok()) { // has just changed
        _prearmchecks_all_ok = true;
        gcs().send_text(MAV_SEVERITY_INFO, "MNT%u: prearm checks passed", _instance+1);
    }

    // unhealthy until gimbal has fully passed the startup sequence
    if (!_initialised || !_prearmchecks_ok || !_armed) {
        return false;
    }

    // unhealthy if attitude status is NOT received within the last second
    if ((AP_HAL::millis() - _device.received_tlast_ms) > 1000) {
        return false;
    }

    // check failure flags
    // we also check for GIMBAL_DEVICE_ERROR_FLAGS_NO_MANAGER, it essentially only means that STorM32 got GIMBAL_DEVICE_SET_ATTITUDE messages
    if ((_device.received_failure_flags & FAILURE_FLAGS) > 0) {
        return false;
    }

    // if we get this far the mount is healthy
    return true;
}


//------------------------------------------------------
// Camera
//------------------------------------------------------

bool BP_Mount_STorM32_MAVLink::take_picture()
{
    if (_use_3way_photo_video) return false;

    if (_camera_mode == CAMERA_MODE_UNDEFINED) {
        _camera_mode = CAMERA_MODE_PHOTO;
        send_cmd_do_digicam_configure(false);
    }

    if (_camera_mode != CAMERA_MODE_PHOTO) return false;

    send_cmd_do_digicam_control(true);

//    gcs().send_text(MAV_SEVERITY_INFO, "cam take pic");

    return true;
}


bool BP_Mount_STorM32_MAVLink::record_video(bool start_recording)
{
    if (_use_3way_photo_video) return false;

    if (_camera_mode == CAMERA_MODE_UNDEFINED) {
        _camera_mode = CAMERA_MODE_VIDEO;
        send_cmd_do_digicam_configure(true);
    }

    if (_camera_mode != CAMERA_MODE_VIDEO) return false;

    send_cmd_do_digicam_control(start_recording);

//    gcs().send_text(MAV_SEVERITY_INFO, "cam rec video %u", start_recording);

    return true;
}


bool BP_Mount_STorM32_MAVLink::set_cam_mode(bool video_mode)
{
    if (_use_3way_photo_video) return false;

    _camera_mode = (video_mode) ? CAMERA_MODE_VIDEO : CAMERA_MODE_PHOTO;
    send_cmd_do_digicam_configure(video_mode);

//    gcs().send_text(MAV_SEVERITY_INFO, "cam set mode %u", video_mode);

    return true;
}


bool BP_Mount_STorM32_MAVLink::set_cam_photo_video(int8_t sw_flag)
{
    if (!_use_3way_photo_video) return false;

    if (sw_flag > 0) {
        if (_camera_mode != CAMERA_MODE_VIDEO) {
            _camera_mode = CAMERA_MODE_VIDEO;
            send_cmd_do_digicam_configure(true);
        }
        send_cmd_do_digicam_control(true);
    } else
    if (sw_flag < 0) {
        if (_camera_mode != CAMERA_MODE_PHOTO) {
            _camera_mode = CAMERA_MODE_PHOTO;
            send_cmd_do_digicam_configure(false);
        }
        send_cmd_do_digicam_control(true);
    } else {
        if (_camera_mode == CAMERA_MODE_VIDEO) {
            send_cmd_do_digicam_control(false);
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
        MAV_CMD_DO_DIGICAM_CONFIGURE, 0,
        param1, 0, 0, 0, 0, 0, 0);

//    gcs().send_text(MAV_SEVERITY_INFO, "cam digi config %u", video_mode);
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
        MAV_CMD_DO_DIGICAM_CONTROL, 0,
        0, 0, 0, 0, param5, 0, 0);

//    gcs().send_text(MAV_SEVERITY_INFO, "cam digi cntrl %u", shoot);
}


//------------------------------------------------------
// MAVLink MOUNT_STATUS forwarding
//------------------------------------------------------

// forward a MOUNT_STATUS message to ground, this is only to make MissionPlanner and alike happy
void BP_Mount_STorM32_MAVLink::send_mount_status_to_ground(MountStatus &status)
{
    // space is checked by send_to_ground()

    mavlink_mount_status_t msg = {
        pointing_a : (int32_t)(status.pitch_deg * 100.0f),
        pointing_b : (int32_t)(status.roll_deg * 100.0f),
        pointing_c : (int32_t)(status.yaw_deg * 100.0f),
        target_system : 0,
        target_component : 0 };

    send_to_ground(MAVLINK_MSG_ID_MOUNT_STATUS, (const char*)&msg);
}


// this is essentially GCS::send_to_active_channels(uint32_t msgid, const char *pkt)
// but exempts the gimbal channel
//TODO: Is dodgy as it assumes that ONLY the gimbal is on the link !!
//      We actually only need to send to the ground, i.e., to the gcs-es (there could be more than one)
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

        if (!c.is_active()) {
            continue;
        }
        if (!c.is_active()) {
            continue;
        }
        // size checks done by this method:
        c.send_message(pkt, entry);
    }
}



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
