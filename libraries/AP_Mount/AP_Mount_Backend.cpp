#include "AP_Mount_config.h"

#if HAL_MOUNT_ENABLED

#include "AP_Mount_Backend.h"

#include <AP_AHRS/AP_AHRS.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Terrain/AP_Terrain.h>

extern const AP_HAL::HAL& hal;

#define AP_MOUNT_UPDATE_DT 0.02     // update rate in seconds.  update() should be called at this rate
#define AP_MOUNT_POI_REQUEST_TIMEOUT_MS 30000   // POI calculations continue to be updated for this many seconds after last request
#define AP_MOUNT_POI_RESULT_TIMEOUT_MS  3000    // POI calculations valid for 3 seconds
#define AP_MOUNT_POI_DIST_M_MAX         10000   // POI calculations limit of 10,000m (10km)

// Default init function for every mount
void AP_Mount_Backend::init()
{
    // setting default target sysid from parameters
    _target_sysid = _params.sysid_default.get();

#if AP_MOUNT_POI_TO_LATLONALT_ENABLED
    // create a calculation thread for poi.
    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_Mount_Backend::calculate_poi, void),
                                      "mount_calc_poi",
                                      8192, AP_HAL::Scheduler::PRIORITY_IO, -1)) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Mount: failed to start POI thread");
    }
#endif
}

// set device id of this instance, for MNTx_DEVID parameter
void AP_Mount_Backend::set_dev_id(uint32_t id)
{
    _params.dev_id.set_and_save(int32_t(id));
}

// return true if this mount accepts roll targets
bool AP_Mount_Backend::has_roll_control() const
{
    return (_params.roll_angle_min < _params.roll_angle_max);
}

// return true if this mount accepts pitch targets
bool AP_Mount_Backend::has_pitch_control() const
{
    return (_params.pitch_angle_min < _params.pitch_angle_max);
}

bool AP_Mount_Backend::valid_mode(MAV_MOUNT_MODE mode) const
{
    switch (mode) {
    case MAV_MOUNT_MODE_RETRACT...MAV_MOUNT_MODE_HOME_LOCATION:
        return true;
    case MAV_MOUNT_MODE_ENUM_END:
        return false;
    }
    return false;
}

bool AP_Mount_Backend::set_mode(MAV_MOUNT_MODE mode)
{
    if (!valid_mode(mode)) {
        return false;
    }
    _mode = mode;
    return true;
}

// called when mount mode is RC-targetting, updates the mnt_target object from RC inputs:
void AP_Mount_Backend::update_mnt_target_from_rc_target()
{
    if (rc().in_rc_failsafe()) {
        if (option_set(Options::NEUTRAL_ON_RC_FS)) {
            mnt_target.angle_rad.set(_params.neutral_angles.get() * DEG_TO_RAD, false);
            mnt_target.target_type = MountTargetType::ANGLE;
            return;
        }
    }

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
}

// set angle target in degrees
// roll and pitch are in earth-frame
// yaw_is_earth_frame (aka yaw_lock) should be true if yaw angle is earth-frame, false if body-frame
void AP_Mount_Backend::set_angle_target(float roll_deg, float pitch_deg, float yaw_deg, bool yaw_is_earth_frame)
{
    // enforce angle limits
    roll_deg = constrain_float(roll_deg, _params.roll_angle_min, _params.roll_angle_max);
    pitch_deg = constrain_float(pitch_deg, _params.pitch_angle_min, _params.pitch_angle_max);
    if (!yaw_is_earth_frame) {
        // only limit yaw if in body-frame.  earth-frame yaw limiting is backend specific
        // custom wrap code (instead of wrap_180) to better handle yaw of <= -180
        if (yaw_deg > 180) {
            yaw_deg -= 360;
        }
        yaw_deg = constrain_float(yaw_deg, _params.yaw_angle_min, _params.yaw_angle_max);
    }

    // set angle targets
    mnt_target.target_type = MountTargetType::ANGLE;
    mnt_target.angle_rad.roll = radians(roll_deg);
    mnt_target.angle_rad.pitch = radians(pitch_deg);
    mnt_target.angle_rad.yaw = radians(yaw_deg);
    mnt_target.angle_rad.yaw_is_ef = yaw_is_earth_frame;

    // set the mode to mavlink targeting
    set_mode(MAV_MOUNT_MODE_MAVLINK_TARGETING);

    // optionally set RC_TARGETING yaw lock state
    if (option_set(Options::RCTARGETING_LOCK_FROM_PREVMODE)) {
        set_yaw_lock(yaw_is_earth_frame);
    }
}

// sets rate target in deg/s
// yaw_lock should be true if the yaw rate is earth-frame, false if body-frame (e.g. rotates with body of vehicle)
void AP_Mount_Backend::set_rate_target(float roll_degs, float pitch_degs, float yaw_degs, bool yaw_is_earth_frame)
{
    // set rate targets
    mnt_target.target_type = MountTargetType::RATE;
    mnt_target.rate_rads.roll = radians(roll_degs);
    mnt_target.rate_rads.pitch = radians(pitch_degs);
    mnt_target.rate_rads.yaw = radians(yaw_degs);
    mnt_target.rate_rads.yaw_is_ef = yaw_is_earth_frame;

    // set the mode to mavlink targeting
    set_mode(MAV_MOUNT_MODE_MAVLINK_TARGETING);

    // optionally set RC_TARGETING yaw lock state
    if (option_set(Options::RCTARGETING_LOCK_FROM_PREVMODE)) {
        set_yaw_lock(yaw_is_earth_frame);
    }
}

// set_roi_target - sets target location that mount should attempt to point towards
void AP_Mount_Backend::set_roi_target(const Location &target_loc)
{
    // set the target gps location
    _roi_target = target_loc;
    _roi_target_set = true;

    // set the mode to GPS tracking mode
    set_mode(MAV_MOUNT_MODE_GPS_POINT);

    // optionally set RC_TARGETING yaw lock state
    if (option_set(Options::RCTARGETING_LOCK_FROM_PREVMODE)) {
        set_yaw_lock(true);
    }
}

// clear_roi_target - clears target location that mount should attempt to point towards
void AP_Mount_Backend::clear_roi_target()
{
    // clear the target GPS location
    _roi_target_set = false;

    // reset the mode if in GPS tracking mode
    if (get_mode() == MAV_MOUNT_MODE_GPS_POINT) {
        MAV_MOUNT_MODE default_mode = (MAV_MOUNT_MODE)_params.default_mode.get();
        set_mode(default_mode);
    }
}

// set_sys_target - sets system that mount should attempt to point towards
void AP_Mount_Backend::set_target_sysid(uint8_t sysid)
{
    _target_sysid = sysid;

    // set the mode to sysid tracking mode
    set_mode(MAV_MOUNT_MODE_SYSID_TARGET);

    // optionally set RC_TARGETING yaw lock state
    if (option_set(Options::RCTARGETING_LOCK_FROM_PREVMODE)) {
        set_yaw_lock(true);
    }
}

#if HAL_GCS_ENABLED
// send a GIMBAL_DEVICE_ATTITUDE_STATUS message to GCS
void AP_Mount_Backend::send_gimbal_device_attitude_status(mavlink_channel_t chan)
{
    if (suppress_heartbeat()) {
        // block heartbeat from transmitting to the GCS
        GCS_MAVLINK::disable_channel_routing(chan);
    }

    Quaternion att_quat;
    if (!get_attitude_quaternion(att_quat)) {
        return;
    }
    Vector3f ang_velocity { nanf(""), nanf(""), nanf("") };
    IGNORE_RETURN(get_angular_velocity(ang_velocity));

    // construct quaternion array
    const float quat_array[4] = {att_quat.q1, att_quat.q2, att_quat.q3, att_quat.q4};

    mavlink_msg_gimbal_device_attitude_status_send(chan,
                                                   0,   // target system
                                                   0,   // target component
                                                   AP_HAL::millis(),    // autopilot system time
                                                   get_gimbal_device_flags(),
                                                   quat_array,    // attitude expressed as quaternion
                                                   ang_velocity.x,    // roll axis angular velocity (NaN for unknown)
                                                   ang_velocity.y,    // pitch axis angular velocity (NaN for unknown)
                                                   ang_velocity.z,    // yaw axis angular velocity (NaN for unknown)
                                                   0,                                           // failure flags (not supported)
                                                   std::numeric_limits<double>::quiet_NaN(),    // delta_yaw (NaN for unknonw)
                                                   std::numeric_limits<double>::quiet_NaN(),    // delta_yaw_velocity (NaN for unknonw)
//OW
//                                                   _instance + 1);  // gimbal_device_id
                                                   get_gimbal_device_id());  // gimbal_device_id
//OWEND
}
#endif

// return gimbal manager capability flags used by GIMBAL_MANAGER_INFORMATION message
uint32_t AP_Mount_Backend::get_gimbal_manager_capability_flags() const
{
    uint32_t cap_flags = GIMBAL_MANAGER_CAP_FLAGS_HAS_RETRACT |
                         GIMBAL_MANAGER_CAP_FLAGS_HAS_NEUTRAL |
                         GIMBAL_MANAGER_CAP_FLAGS_HAS_RC_INPUTS |
                         GIMBAL_MANAGER_CAP_FLAGS_CAN_POINT_LOCATION_LOCAL |
                         GIMBAL_MANAGER_CAP_FLAGS_CAN_POINT_LOCATION_GLOBAL;

    // roll control
    if (has_roll_control()) {
        cap_flags |= GIMBAL_MANAGER_CAP_FLAGS_HAS_ROLL_AXIS |
                     GIMBAL_MANAGER_CAP_FLAGS_HAS_ROLL_FOLLOW |
                     GIMBAL_MANAGER_CAP_FLAGS_HAS_ROLL_LOCK;
    }

    // pitch control
    if (has_pitch_control()) {
        cap_flags |= GIMBAL_MANAGER_CAP_FLAGS_HAS_PITCH_AXIS |
                     GIMBAL_MANAGER_CAP_FLAGS_HAS_PITCH_FOLLOW |
                     GIMBAL_MANAGER_CAP_FLAGS_HAS_PITCH_LOCK;
    }

    // yaw control
    if (has_pan_control()) {
        cap_flags |= GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_AXIS |
                     GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_FOLLOW |
                     GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_LOCK;
    }

    return cap_flags;
}

// send a GIMBAL_MANAGER_INFORMATION message to GCS
void AP_Mount_Backend::send_gimbal_manager_information(mavlink_channel_t chan)
{
    mavlink_msg_gimbal_manager_information_send(chan,
                                                AP_HAL::millis(),                       // autopilot system time
                                                get_gimbal_manager_capability_flags(),  // bitmap of gimbal manager capability flags
//OW
//                                                _instance + 1,                          // gimbal device id
                                                get_gimbal_device_id(),                 // gimbal device id
//OWEND
                                                radians(_params.roll_angle_min),        // roll_min in radians
                                                radians(_params.roll_angle_max),        // roll_max in radians
                                                radians(_params.pitch_angle_min),       // pitch_min in radians
                                                radians(_params.pitch_angle_max),       // pitch_max in radians
                                                radians(_params.yaw_angle_min),         // yaw_min in radians
                                                radians(_params.yaw_angle_max));        // yaw_max in radians
}

//OW
uint8_t AP_Mount_Backend::get_gimbal_device_id() const
{
    return _instance + 1;
}

bool AP_Mount_Backend::is_in_control(uint8_t sysid, uint8_t compid, uint8_t gimbal_device_id)
{
    if (gimbal_device_id != 0 && gimbal_device_id != get_gimbal_device_id()) {
        return false;
    }

    // allow the source to claim control if nobody is in control
    if (mavlink_control_id.sysid == 0 && mavlink_control_id.compid == 0) {
        mavlink_control_id.sysid = sysid;
        mavlink_control_id.compid = compid;
        return true;
    }

    return (mavlink_control_id.sysid == sysid && mavlink_control_id.compid == compid);
}

// return gimbal manager flags used by GIMBAL_MANAGER_STATUS message
uint32_t AP_Mount_Backend::get_gimbal_manager_flags() const
{
    uint32_t flags = GIMBAL_MANAGER_FLAGS_ROLL_LOCK | GIMBAL_MANAGER_FLAGS_PITCH_LOCK;
    if (_yaw_lock) {
        flags |= GIMBAL_MANAGER_FLAGS_YAW_LOCK;
    }
    return flags;
}

// set gimbal manager flags, called from frontend's gimbal manager handlers
bool AP_Mount_Backend::handle_gimbal_manager_flags(uint32_t flags)
{
    // check flags for change to RETRACT
    if ((flags & GIMBAL_MANAGER_FLAGS_RETRACT) > 0) {
        set_mode(MAV_MOUNT_MODE_RETRACT);
        return false;
    } else
    // check flags for change to NEUTRAL
    if ((flags & GIMBAL_MANAGER_FLAGS_NEUTRAL) > 0) {
        set_mode(MAV_MOUNT_MODE_NEUTRAL);
        return false;
    }
    return true;
}

void AP_Mount_Backend::handle_gimbal_manager_set_pitchyaw(const mavlink_gimbal_manager_set_pitchyaw_t &packet)
{
    const uint32_t flags = packet.flags;

    if (!handle_gimbal_manager_flags(flags)) {
        return;
    }

    // Do not allow both angle and rate to be specified at the same time
    if (!isnan(packet.pitch) && !isnan(packet.yaw) && !isnan(packet.pitch_rate) && !isnan(packet.yaw_rate)) {
        return;
    }

    // pitch and yaw from packet are in radians
    if (!isnan(packet.pitch) && !isnan(packet.yaw)) {
        const float pitch_angle_deg = degrees(packet.pitch);
        const float yaw_angle_deg = degrees(packet.yaw);
        set_angle_target(0, pitch_angle_deg, yaw_angle_deg, flags & GIMBAL_MANAGER_FLAGS_YAW_LOCK);
        return;
    }

    // pitch_rate and yaw_rate from packet are in rad/s
    if (!isnan(packet.pitch_rate) && !isnan(packet.yaw_rate)) {
        const float pitch_rate_degs = degrees(packet.pitch_rate);
        const float yaw_rate_degs = degrees(packet.yaw_rate);
        set_rate_target(0, pitch_rate_degs, yaw_rate_degs, flags & GIMBAL_MANAGER_FLAGS_YAW_LOCK);
    }
}

void AP_Mount_Backend::handle_gimbal_manager_set_attitude(const mavlink_gimbal_manager_set_attitude_t &packet)
{
    const uint32_t flags = packet.flags;

    if (!handle_gimbal_manager_flags(flags)) {
        return;
    }

    const Quaternion att_quat{packet.q};
    const Vector3f att_rate_degs {
        packet.angular_velocity_x,
        packet.angular_velocity_y,
        packet.angular_velocity_y
    };

    // Do not allow both quaternion and angular velocity to be specified at the same time:
    if (!att_quat.is_nan() && !att_rate_degs.is_nan()) {
        return;
    }

    if (!att_quat.is_nan()) {
        // convert quaternion to euler angles
        Vector3f attitude;
        att_quat.to_euler(attitude);  // attitude is in radians here
        attitude *= RAD_TO_DEG;  // convert to degrees

        set_angle_target(attitude.x, attitude.y, attitude.z, flags & GIMBAL_MANAGER_FLAGS_YAW_LOCK);
        return;
    }

    if (!att_rate_degs.is_nan()) {
        const float roll_rate_degs = degrees(packet.angular_velocity_x);
        const float pitch_rate_degs = degrees(packet.angular_velocity_y);
        const float yaw_rate_degs = degrees(packet.angular_velocity_z);
        set_rate_target(roll_rate_degs, pitch_rate_degs, yaw_rate_degs, flags & GIMBAL_MANAGER_FLAGS_YAW_LOCK);
    }
}

MAV_RESULT AP_Mount_Backend::handle_command_do_gimbal_manager_pitchyaw(const mavlink_command_int_t &packet)
{
    const uint32_t flags = packet.x;

    if (!handle_gimbal_manager_flags(flags)) {
        return MAV_RESULT_FAILED;
    }

    // Do not allow both angle and rate to be specified at the same time
    if (!isnan(packet.param1) && !isnan(packet.param2) && !isnan(packet.param3) && !isnan(packet.param4)) {
        return MAV_RESULT_ACCEPTED;
    }

    // param1 : pitch_angle (in degrees)
    // param2 : yaw angle (in degrees)
    const float pitch_angle_deg = packet.param1;
    const float yaw_angle_deg = packet.param2;
    if (!isnan(pitch_angle_deg) && !isnan(yaw_angle_deg)) {
        set_angle_target(0, pitch_angle_deg, yaw_angle_deg, flags & GIMBAL_MANAGER_FLAGS_YAW_LOCK);
        return MAV_RESULT_ACCEPTED;
    }

    // param3 : pitch_rate (in deg/s)
    // param4 : yaw rate (in deg/s)
    const float pitch_rate_degs = packet.param3;
    const float yaw_rate_degs = packet.param4;
    if (!isnan(pitch_rate_degs) && !isnan(yaw_rate_degs)) {
        set_rate_target(0, pitch_rate_degs, yaw_rate_degs, flags & GIMBAL_MANAGER_FLAGS_YAW_LOCK);
        return MAV_RESULT_ACCEPTED;
    }

    return MAV_RESULT_ACCEPTED;
}
//OWEND

// send a GIMBAL_MANAGER_STATUS message to GCS
void AP_Mount_Backend::send_gimbal_manager_status(mavlink_channel_t chan)
{
//OW
/*
    uint32_t flags = GIMBAL_MANAGER_FLAGS_ROLL_LOCK | GIMBAL_MANAGER_FLAGS_PITCH_LOCK;

    if (_yaw_lock) {
        flags |= GIMBAL_MANAGER_FLAGS_YAW_LOCK;
    } */
//OWEND

    mavlink_msg_gimbal_manager_status_send(chan,
                                           AP_HAL::millis(),    // autopilot system time
//OW
//                                           flags,               // bitmap of gimbal manager flags
//                                           _instance + 1,       // gimbal device id
                                           get_gimbal_manager_flags(),  // bitmap of gimbal manager flags
                                           get_gimbal_device_id(),      // gimbal device id
//OWEND
                                           mavlink_control_id.sysid,    // primary control system id
                                           mavlink_control_id.compid,   // primary control component id
                                           0,                           // secondary control system id
                                           0);                          // secondary control component id
}

// handle do_mount_control command.  Returns MAV_RESULT_ACCEPTED on success
MAV_RESULT AP_Mount_Backend::handle_command_do_mount_control(const mavlink_command_int_t &packet)
{
    const MAV_MOUNT_MODE new_mode = (MAV_MOUNT_MODE)packet.z;

    // interpret message fields based on mode
    switch (new_mode) {
    case MAV_MOUNT_MODE_RETRACT:
    case MAV_MOUNT_MODE_NEUTRAL:
    case MAV_MOUNT_MODE_RC_TARGETING:
    case MAV_MOUNT_MODE_HOME_LOCATION:
        // simply set mode
        set_mode(new_mode);
        return MAV_RESULT_ACCEPTED;

    case MAV_MOUNT_MODE_MAVLINK_TARGETING: {
        // set target angles (in degrees) from mavlink message
        const float pitch_deg = packet.param1;  // param1: pitch (earth-frame, degrees)
        const float roll_deg = packet.param2;   // param2: roll (earth-frame, degrees)
        const float yaw_deg = packet.param3;    // param3: yaw (body-frame, degrees)

        // warn if angles are invalid to catch angles sent in centi-degrees
        if ((fabsf(pitch_deg) > 90) || (fabsf(roll_deg) > 180) || (fabsf(yaw_deg) > 360)) {
            send_warning_to_GCS("invalid angle targets");
            return MAV_RESULT_FAILED;
        }

        set_angle_target(packet.param2, packet.param1, packet.param3, false);
        return MAV_RESULT_ACCEPTED;
    }

    case MAV_MOUNT_MODE_GPS_POINT: {
        // set lat, lon, alt position targets from mavlink message

        // warn if lat, lon appear to be in param1,2 instead of param x,y as this indicates
        // sender is relying on a bug in AP-4.2's (and earlier) handling of MAV_CMD_DO_MOUNT_CONTROL
        if (!is_zero(packet.param1) && !is_zero(packet.param2) && packet.x == 0 && packet.y == 0) {
            send_warning_to_GCS("GPS_POINT target invalid");
            return MAV_RESULT_FAILED;
        }

        // param4: altitude in meters
        // x: latitude in degrees * 1E7
        // y: longitude in degrees * 1E7
        const Location target_location {
            packet.x,                       // latitude in degrees * 1E7
            packet.y,                       // longitude in degrees * 1E7
            (int32_t)packet.param4 * 100,   // alt converted from meters to cm
            Location::AltFrame::ABOVE_HOME
        };
        set_roi_target(target_location);
        return MAV_RESULT_ACCEPTED;
    }

    default:
        // invalid mode
        return MAV_RESULT_FAILED;
    }
}

// handle do_gimbal_manager_configure.  Returns MAV_RESULT_ACCEPTED on success
// requires original message in order to extract caller's sysid and compid
MAV_RESULT AP_Mount_Backend::handle_command_do_gimbal_manager_configure(const mavlink_command_int_t &packet, const mavlink_message_t &msg)
{
    // sanity check param1 and param2 values
    if ((packet.param1 < -3) || (packet.param1 > UINT8_MAX) || (packet.param2 < -3) || (packet.param2 > UINT8_MAX)) {
        return MAV_RESULT_FAILED;
    }

    // backup the current values so we can detect a change
    mavlink_control_id_t prev_control_id = mavlink_control_id;

    // convert negative packet1 and packet2 values
    int16_t new_sysid = packet.param1;
    switch (new_sysid) {
        case -1:
            // leave unchanged
            break;
        case -2:
            // set itself in control
            mavlink_control_id.sysid = msg.sysid;
            mavlink_control_id.compid = msg.compid;
            break;
        case -3:
            // remove control if currently in control
            if ((mavlink_control_id.sysid == msg.sysid) && (mavlink_control_id.compid == msg.compid)) {
                mavlink_control_id.sysid = 0;
                mavlink_control_id.compid = 0;
            }
            break;
        default:
            mavlink_control_id.sysid = packet.param1;
            mavlink_control_id.compid = packet.param2;
            break;
    }

    // send gimbal_manager_status if control has changed
    if (prev_control_id != mavlink_control_id) {
        GCS_SEND_MESSAGE(MSG_GIMBAL_MANAGER_STATUS);
    }

    return MAV_RESULT_ACCEPTED;
}

// handle a GLOBAL_POSITION_INT message
bool AP_Mount_Backend::handle_global_position_int(uint8_t msg_sysid, const mavlink_global_position_int_t &packet)
{
    if (_target_sysid != msg_sysid) {
        return false;
    }

    _target_sysid_location.lat = packet.lat;
    _target_sysid_location.lng = packet.lon;
    // global_position_int.alt is *UP*, so is location.
    _target_sysid_location.set_alt_cm(packet.alt*0.1, Location::AltFrame::ABSOLUTE);
    _target_sysid_location_set = true;

    return true;
}

#if HAL_LOGGING_ENABLED
// write mount log packet
void AP_Mount_Backend::write_log(uint64_t timestamp_us)
{
    // return immediately if no yaw estimate
    float ahrs_yaw = AP::ahrs().get_yaw_rad();
    if (isnan(ahrs_yaw)) {
        return;
    }

    const auto nanf = AP::logger().quiet_nanf();

    // get_attitude_quaternion and convert to Euler angles
    float roll = nanf;
    float pitch = nanf;
    float yaw_bf = nanf;
    float yaw_ef = nanf;
    if (_frontend.get_attitude_euler(_instance, roll, pitch, yaw_bf)) {
        yaw_ef = wrap_180(yaw_bf + degrees(ahrs_yaw));
    }

    // get mount's target (desired) angles and convert yaw to earth frame
    float target_roll = nanf;
    float target_pitch = nanf;
    float target_yaw = nanf;
    bool target_yaw_is_ef = false;
    IGNORE_RETURN(get_angle_target(target_roll, target_pitch, target_yaw, target_yaw_is_ef));

    // get rangefinder distance
    float rangefinder_dist = nanf;
    IGNORE_RETURN(get_rangefinder_distance(rangefinder_dist));

    const struct log_Mount pkt {
        LOG_PACKET_HEADER_INIT(static_cast<uint8_t>(LOG_MOUNT_MSG)),
        time_us       : (timestamp_us > 0) ? timestamp_us : AP_HAL::micros64(),
        instance      : _instance,
        desired_roll  : target_roll,
        actual_roll   : roll,
        desired_pitch : target_pitch,
        actual_pitch  : pitch,
        desired_yaw_bf: target_yaw_is_ef ? nanf : target_yaw,
        actual_yaw_bf : yaw_bf,
        desired_yaw_ef: target_yaw_is_ef ? target_yaw : nanf,
        actual_yaw_ef : yaw_ef,
        rangefinder_dist : rangefinder_dist,
    };
    AP::logger().WriteCriticalBlock(&pkt, sizeof(pkt));
}
#endif

#if AP_MOUNT_POI_TO_LATLONALT_ENABLED
// get poi information.  Returns true on success and fills in gimbal attitude, location and poi location
bool AP_Mount_Backend::get_poi(uint8_t instance, Quaternion &quat, Location &loc, Location &poi_loc)
{
    WITH_SEMAPHORE(poi_calculation.sem);

    // record time of request
    const uint32_t now_ms = AP_HAL::millis();
    poi_calculation.poi_request_ms = now_ms;

    // check if poi calculated recently
    if (now_ms - poi_calculation.poi_update_ms > AP_MOUNT_POI_RESULT_TIMEOUT_MS) {
        return false;
    }

    // check attitude is valid
    if (poi_calculation.att_quat.is_nan()) {
        return false;
    }

    quat = poi_calculation.att_quat;
    loc = poi_calculation.loc;
    poi_loc = poi_calculation.poi_loc;
    return true;
}

// calculate the Location that the gimbal is pointing at
void AP_Mount_Backend::calculate_poi()
{
    while (true) {
        // run this loop at 10hz
        hal.scheduler->delay(100);

        // calculate poi if requested within last 30 seconds
        {
            WITH_SEMAPHORE(poi_calculation.sem);
            if ((poi_calculation.poi_request_ms == 0) ||
                (AP_HAL::millis() - poi_calculation.poi_request_ms > AP_MOUNT_POI_REQUEST_TIMEOUT_MS)) {
                continue;
            }
        }

        // get the current location of vehicle
        const AP_AHRS &ahrs = AP::ahrs();
        Location curr_loc;
        if (!ahrs.get_location(curr_loc)) {
            continue;
        }

        // change vehicle alt to AMSL
        curr_loc.change_alt_frame(Location::AltFrame::ABSOLUTE);

        // project forward from vehicle looking for terrain
        // start testing at vehicle's location
        Location test_loc = curr_loc;
        Location prev_test_loc = curr_loc;

        // get terrain altitude (AMSL) at test_loc
        auto terrain = AP_Terrain::get_singleton();
        float terrain_amsl_m;
        if ((terrain == nullptr) || !terrain->height_amsl(test_loc, terrain_amsl_m, true)) {
            continue;
        }

        // retrieve gimbal attitude
        Quaternion quat;
        if (!get_attitude_quaternion(quat)) {
            // gimbal attitude unavailable
            continue;
        }

        // iteratively move test_loc forward until its alt-above-sea-level is below terrain-alt-above-sea-level
        const float dist_increment_m = MAX(terrain->get_grid_spacing(), 10);
        const float mount_pitch_deg = degrees(quat.get_euler_pitch());
        const float mount_yaw_ef_deg = wrap_180(degrees(quat.get_euler_yaw()) + degrees(ahrs.get_yaw_rad()));
        float total_dist_m = 0;
        bool get_terrain_alt_success = true;
        float prev_terrain_amsl_m = terrain_amsl_m;
        while (total_dist_m < AP_MOUNT_POI_DIST_M_MAX && (test_loc.alt * 0.01) > terrain_amsl_m) {
            total_dist_m += dist_increment_m;

            // backup previous test location and terrain amsl
            prev_test_loc = test_loc;
            prev_terrain_amsl_m = terrain_amsl_m;

            // move test location forward
            test_loc.offset_bearing_and_pitch(mount_yaw_ef_deg, mount_pitch_deg, dist_increment_m);

            // get terrain's alt-above-sea-level (at test_loc)
            // fail if terrain alt cannot be retrieved
            if (!terrain->height_amsl(test_loc, terrain_amsl_m, true) || std::isnan(terrain_amsl_m)) {
                get_terrain_alt_success = false;
                continue;
            }
        }

        // if a fail occurred above when getting terrain alt then restart calculations from the beginning
        if (!get_terrain_alt_success) {
            continue;
        }

        if (total_dist_m >= AP_MOUNT_POI_DIST_M_MAX) {
            // unable to find terrain within dist_max
            continue;
        }

        // test location has dropped below terrain
        // interpolate along line between prev_test_loc and test_loc
        float dist_interp_m = linear_interpolate(0, dist_increment_m, 0, prev_test_loc.alt * 0.01 - prev_terrain_amsl_m, test_loc.alt * 0.01 - terrain_amsl_m);
        {
            WITH_SEMAPHORE(poi_calculation.sem);
            poi_calculation.poi_loc = prev_test_loc;
            poi_calculation.poi_loc.offset_bearing_and_pitch(mount_yaw_ef_deg, mount_pitch_deg, dist_interp_m);
            poi_calculation.att_quat = {quat[0], quat[1], quat[2], quat[3]};
            poi_calculation.loc = curr_loc;
            poi_calculation.poi_update_ms = AP_HAL::millis();
        }
    }
}
#endif

// change to RC_TARGETING mode if rc inputs have changed by more than the dead zone
// should be called on every update
void AP_Mount_Backend::set_rctargeting_on_rcinput_change()
{
    // exit immediately if no RC input
    if (!rc().has_valid_input()) {
        return;
    }

    const RC_Channel *roll_ch = rc().find_channel_for_option(_instance == 0 ? RC_Channel::AUX_FUNC::MOUNT1_ROLL : RC_Channel::AUX_FUNC::MOUNT2_ROLL);
    const RC_Channel *pitch_ch = rc().find_channel_for_option(_instance == 0 ? RC_Channel::AUX_FUNC::MOUNT1_PITCH : RC_Channel::AUX_FUNC::MOUNT2_PITCH);
    const RC_Channel *yaw_ch = rc().find_channel_for_option(_instance == 0 ? RC_Channel::AUX_FUNC::MOUNT1_YAW : RC_Channel::AUX_FUNC::MOUNT2_YAW);

    // get rc input
    const int16_t roll_in = (roll_ch == nullptr) ? 0 : roll_ch->get_radio_in();
    const int16_t pitch_in = (pitch_ch == nullptr) ? 0 : pitch_ch->get_radio_in();
    const int16_t yaw_in = (yaw_ch == nullptr) ? 0 : yaw_ch->get_radio_in();

    if (!last_rc_input.initialised) {
            // The first time through, initial RC inputs should be set, but not used
            last_rc_input.initialised = true;
            last_rc_input.roll_in = roll_in;
            last_rc_input.pitch_in = pitch_in;
            last_rc_input.yaw_in = yaw_in;
    }
    // if not in RC_TARGETING or RETRACT modes then check for RC change
    if (get_mode() != MAV_MOUNT_MODE_RC_TARGETING && get_mode() != MAV_MOUNT_MODE_RETRACT) {
        // get dead zones
        const int16_t roll_dz = (roll_ch == nullptr) ? 10 : MAX(roll_ch->get_dead_zone(), 10);
        const int16_t pitch_dz = (pitch_ch == nullptr) ? 10 : MAX(pitch_ch->get_dead_zone(), 10);
        const int16_t yaw_dz = (yaw_ch == nullptr) ? 10 : MAX(yaw_ch->get_dead_zone(), 10);

        // check if RC input has changed by more than the dead zone
        if ((abs(last_rc_input.roll_in - roll_in) > roll_dz) ||
            (abs(last_rc_input.pitch_in - pitch_in) > pitch_dz) ||
            (abs(last_rc_input.yaw_in - yaw_in) > yaw_dz)) {
                set_mode(MAV_MOUNT_MODE_RC_TARGETING);
        }
    }

    // if NOW in RC_TARGETING or RETRACT mode then store last RC input (mode might have changed)
    if (get_mode() == MAV_MOUNT_MODE_RC_TARGETING || get_mode() == MAV_MOUNT_MODE_RETRACT) {
        last_rc_input.roll_in = roll_in;
        last_rc_input.pitch_in = pitch_in;
        last_rc_input.yaw_in = yaw_in;
    }
}

// get pilot input (in the range -1 to +1) received through RC
void AP_Mount_Backend::get_rc_input(float& roll_in, float& pitch_in, float& yaw_in) const
{
    const RC_Channel *roll_ch = rc().find_channel_for_option(_instance == 0 ? RC_Channel::AUX_FUNC::MOUNT1_ROLL : RC_Channel::AUX_FUNC::MOUNT2_ROLL);
    const RC_Channel *pitch_ch = rc().find_channel_for_option(_instance == 0 ? RC_Channel::AUX_FUNC::MOUNT1_PITCH : RC_Channel::AUX_FUNC::MOUNT2_PITCH);
    const RC_Channel *yaw_ch = rc().find_channel_for_option(_instance == 0 ? RC_Channel::AUX_FUNC::MOUNT1_YAW : RC_Channel::AUX_FUNC::MOUNT2_YAW);

    roll_in = 0;
    if ((roll_ch != nullptr) && (roll_ch->get_radio_in() > 0)) {
        roll_in = roll_ch->norm_input_dz();
    }

    pitch_in = 0;
    if ((pitch_ch != nullptr) && (pitch_ch->get_radio_in() > 0)) {
        pitch_in = pitch_ch->norm_input_dz();
    }

    yaw_in = 0;
    if ((yaw_ch != nullptr) && (yaw_ch->get_radio_in() > 0)) {
        yaw_in = yaw_ch->norm_input_dz();
    }
}

// get angle or rate targets from pilot RC
// target_type will be either ANGLE or RATE, rpy will be the target angle in deg or rate in deg/s
void AP_Mount_Backend::get_rc_target(MountTargetType& target_type, MountTarget& target_rpy) const
{
    // get RC input from pilot
    float roll_in, pitch_in, yaw_in;
    get_rc_input(roll_in, pitch_in, yaw_in);

    // yaw frame
    target_rpy.yaw_is_ef = _yaw_lock;

    // if RC_RATE is zero, targets are angle
    if (_params.rc_rate_max <= 0) {
        target_type = MountTargetType::ANGLE;

        // roll angle
        target_rpy.roll = radians(((roll_in + 1.0f) * 0.5f * (_params.roll_angle_max - _params.roll_angle_min) + _params.roll_angle_min));

        // pitch angle
        target_rpy.pitch = radians(((pitch_in + 1.0f) * 0.5f * (_params.pitch_angle_max - _params.pitch_angle_min) + _params.pitch_angle_min));

        // yaw angle
        if (target_rpy.yaw_is_ef) {
            // if yaw is earth-frame pilot yaw input control angle from -180 to +180 deg
            target_rpy.yaw = yaw_in * M_PI;
        } else {
            // yaw target in body frame so apply body frame limits
            target_rpy.yaw = radians(((yaw_in + 1.0f) * 0.5f * (_params.yaw_angle_max - _params.yaw_angle_min) + _params.yaw_angle_min));
        }
        return;
    }

    // calculate rate targets
    target_type = MountTargetType::RATE;
    const float rc_rate_max_rads = radians(_params.rc_rate_max.get());
    target_rpy.roll = roll_in * rc_rate_max_rads;
    target_rpy.pitch = pitch_in * rc_rate_max_rads;
    target_rpy.yaw = yaw_in * rc_rate_max_rads;
}

// get angle targets (in radians) to a Location
// returns true on success, false on failure
bool AP_Mount_Backend::get_angle_target_to_location(const Location &loc, MountTarget& angle_rad) const
{
    // exit immediately if vehicle's location is unavailable
    Location current_loc;
    if (!AP::ahrs().get_location(current_loc)) {
        return false;
    }

    // exit immediate if location is invalid
    if (!loc.initialised()) {
        return false;
    }

    const float GPS_vector_x = Location::diff_longitude(loc.lng, current_loc.lng)*cosf(radians((current_loc.lat + loc.lat) * 0.00000005f)) * 0.01113195f;
    const float GPS_vector_y = (loc.lat - current_loc.lat) * 0.01113195f;
    int32_t target_alt_cm = 0;
    if (!loc.get_alt_cm(Location::AltFrame::ABOVE_HOME, target_alt_cm)) {
        return false;
    }
    int32_t current_alt_cm = 0;
    if (!current_loc.get_alt_cm(Location::AltFrame::ABOVE_HOME, current_alt_cm)) {
        return false;
    }
    float GPS_vector_z = target_alt_cm - current_alt_cm;
    float target_distance = 100.0f*norm(GPS_vector_x, GPS_vector_y);      // Careful , centimeters here locally. Baro/alt is in cm, lat/lon is in meters.

    // calculate roll, pitch, yaw angles
    angle_rad.roll = 0;
    angle_rad.pitch = atan2f(GPS_vector_z, target_distance);
    angle_rad.yaw = atan2f(GPS_vector_x, GPS_vector_y);
    angle_rad.yaw_is_ef = true;

    return true;
}

// get angle targets (in radians) to ROI location
// returns true on success, false on failure
bool AP_Mount_Backend::get_angle_target_to_roi(MountTarget& angle_rad) const
{
    if (!_roi_target_set) {
        return false;
    }
    return get_angle_target_to_location(_roi_target, angle_rad);
}

// return body-frame yaw angle from a mount target
float AP_Mount_Backend::MountTarget::get_bf_yaw() const
{
    if (yaw_is_ef) {
        // convert to body-frame
        return wrap_PI(yaw - AP::ahrs().get_yaw_rad());
    }

    // target is already body-frame
    return yaw;
}

// return earth-frame yaw angle from a mount target
float AP_Mount_Backend::MountTarget::get_ef_yaw() const
{
    if (yaw_is_ef) {
        // target is already earth-frame
        return yaw;
    }

    // convert to earth-frame
    return wrap_PI(yaw + AP::ahrs().get_yaw_rad());
}

// sets roll, pitch, yaw and yaw_is_ef
void AP_Mount_Backend::MountTarget::set(const Vector3f& rpy, bool yaw_is_ef_in)
{
    roll  = rpy.x;
    pitch = rpy.y;
    yaw   = rpy.z;
    yaw_is_ef = yaw_is_ef_in;
}

// update angle targets using a given rate target
// the resulting angle_rad yaw frame will match the rate_rad yaw frame
// assumes a 50hz update rate
void AP_Mount_Backend::update_angle_target_from_rate(const MountTarget& rate_rad, MountTarget& angle_rad) const
{
    // update roll and pitch angles and apply limits
    angle_rad.roll = constrain_float(angle_rad.roll + rate_rad.roll * AP_MOUNT_UPDATE_DT, radians(_params.roll_angle_min), radians(_params.roll_angle_max));
    angle_rad.pitch = constrain_float(angle_rad.pitch + rate_rad.pitch * AP_MOUNT_UPDATE_DT, radians(_params.pitch_angle_min), radians(_params.pitch_angle_max));

    // ensure angle yaw frames matches rate yaw frame
    if (angle_rad.yaw_is_ef != rate_rad.yaw_is_ef) {
        if (rate_rad.yaw_is_ef) {
            angle_rad.yaw = angle_rad.get_ef_yaw();
        } else {
            angle_rad.yaw = angle_rad.get_bf_yaw();
        }
        angle_rad.yaw_is_ef = rate_rad.yaw_is_ef;
    }

    // update yaw angle target
    angle_rad.yaw = angle_rad.yaw + rate_rad.yaw * AP_MOUNT_UPDATE_DT;
    if (angle_rad.yaw_is_ef) {
        // if earth-frame yaw wraps between += 180 degrees
        angle_rad.yaw = wrap_PI(angle_rad.yaw);
    } else {
        // if body-frame constrain yaw to body-frame limits
        angle_rad.yaw = constrain_float(angle_rad.yaw, radians(_params.yaw_angle_min), radians(_params.yaw_angle_max));
    }
}

// helper function to provide GIMBAL_DEVICE_FLAGS for use in GIMBAL_DEVICE_ATTITUDE_STATUS message
uint16_t AP_Mount_Backend::get_gimbal_device_flags() const
{
    // get yaw lock state by mode
    bool yaw_lock_state = false;
    switch (_mode) {
    case MAV_MOUNT_MODE_RETRACT:
    case MAV_MOUNT_MODE_NEUTRAL:
        // these modes always use body-frame yaw (aka follow)
        yaw_lock_state = false;
        break;
    case MAV_MOUNT_MODE_MAVLINK_TARGETING:
        switch (mnt_target.target_type) {
        case MountTargetType::RATE:
            yaw_lock_state = mnt_target.rate_rads.yaw_is_ef;
            break;
        case MountTargetType::ANGLE:
            yaw_lock_state = mnt_target.angle_rad.yaw_is_ef;
            break;
        }
        break;
    case MAV_MOUNT_MODE_RC_TARGETING:
        yaw_lock_state = _yaw_lock;
        break;
    case MAV_MOUNT_MODE_GPS_POINT:
    case MAV_MOUNT_MODE_SYSID_TARGET:
    case MAV_MOUNT_MODE_HOME_LOCATION:
        // these modes always use earth-frame yaw (aka lock)
        yaw_lock_state = true;
        break;
    case MAV_MOUNT_MODE_ENUM_END:
        // unsupported
        yaw_lock_state = false;
        break;
    }

    const uint16_t flags = (get_mode() == MAV_MOUNT_MODE_RETRACT ? GIMBAL_DEVICE_FLAGS_RETRACT : 0) |
                           (get_mode() == MAV_MOUNT_MODE_NEUTRAL ? GIMBAL_DEVICE_FLAGS_NEUTRAL : 0) |
                           GIMBAL_DEVICE_FLAGS_ROLL_LOCK | // roll angle is always earth-frame
                           GIMBAL_DEVICE_FLAGS_PITCH_LOCK| // pitch angle is always earth-frame, yaw_angle is always body-frame
                           GIMBAL_DEVICE_FLAGS_YAW_IN_VEHICLE_FRAME | // yaw angle is always in vehicle-frame
                           (yaw_lock_state ? GIMBAL_DEVICE_FLAGS_YAW_LOCK : 0);
    return flags;
}

// get angle targets (in radians) to home location
// returns true on success, false on failure
bool AP_Mount_Backend::get_angle_target_to_home(MountTarget& angle_rad) const
{
    // exit immediately if home is not set
    if (!AP::ahrs().home_is_set()) {
        return false;
    }
    return get_angle_target_to_location(AP::ahrs().get_home(), angle_rad);
}

// get angle targets (in radians) to a vehicle with sysid of  _target_sysid
// returns true on success, false on failure
bool AP_Mount_Backend::get_angle_target_to_sysid(MountTarget& angle_rad) const
{
    // exit immediately if sysid is not set or no location available
    if (!_target_sysid_location_set) {
        return false;
    }
    if (!_target_sysid) {
        return false;
    }
    return get_angle_target_to_location(_target_sysid_location, angle_rad);
}

// get target rate in deg/sec. returns true on success
bool AP_Mount_Backend::get_rate_target(float& roll_degs, float& pitch_degs, float& yaw_degs, bool& yaw_is_earth_frame)
{
    if (mnt_target.target_type == MountTargetType::RATE) {
        roll_degs = degrees(mnt_target.rate_rads.roll);
        pitch_degs = degrees(mnt_target.rate_rads.pitch);
        yaw_degs = degrees(mnt_target.rate_rads.yaw);
        yaw_is_earth_frame = mnt_target.rate_rads.yaw_is_ef;
        return true;
    }
    return false;
}

// get target angle in deg. returns true on success
bool AP_Mount_Backend::get_angle_target(float& roll_deg, float& pitch_deg, float& yaw_deg, bool& yaw_is_earth_frame)
{
    if (mnt_target.target_type == MountTargetType::ANGLE) {
        roll_deg = degrees(mnt_target.angle_rad.roll);
        pitch_deg = degrees(mnt_target.angle_rad.pitch);
        yaw_deg = degrees(mnt_target.angle_rad.yaw);
        yaw_is_earth_frame = mnt_target.angle_rad.yaw_is_ef;
        return true;
    }
    return false;
}

// sent warning to GCS.  Warnings are throttled to at most once every 30 seconds
void AP_Mount_Backend::send_warning_to_GCS(const char* warning_str)
{
    uint32_t now_ms = AP_HAL::millis();
    if (now_ms - _last_warning_ms < 30000) {
        return;
    }

    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Mount: %s", warning_str);
    _last_warning_ms = now_ms;
}

#endif // HAL_MOUNT_ENABLED
