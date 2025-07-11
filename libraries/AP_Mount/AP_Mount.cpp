#include "AP_Mount_config.h"

#if HAL_MOUNT_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include "AP_Mount.h"

#include "AP_Mount_Backend.h"
#include "AP_Mount_Servo.h"
#include "AP_Mount_SoloGimbal.h"
#include "AP_Mount_Alexmos.h"
#include "AP_Mount_SToRM32.h"
#include "AP_Mount_SToRM32_serial.h"
#include "AP_Mount_Gremsy.h"
#include "AP_Mount_Siyi.h"
#include "AP_Mount_Scripting.h"
#include "AP_Mount_Xacti.h"
#include "AP_Mount_Viewpro.h"
#include "AP_Mount_Topotek.h"
#include "AP_Mount_CADDX.h"
#include "AP_Mount_XFRobot.h"
#include <stdio.h>
#include <AP_Math/location.h>
#include <SRV_Channel/SRV_Channel.h>
#include <AP_Logger/AP_Logger.h>
//OW
#include "AP_Mount_STorM32_MAVLink.h"
//OWEND

const AP_Param::GroupInfo AP_Mount::var_info[] = {

    // @Group: 1
    // @Path: AP_Mount_Params.cpp
    AP_SUBGROUPINFO(_params[0], "1", 43, AP_Mount, AP_Mount_Params),

#if AP_MOUNT_MAX_INSTANCES > 1
    // @Group: 2
    // @Path: AP_Mount_Params.cpp
    AP_SUBGROUPINFO(_params[1], "2", 44, AP_Mount, AP_Mount_Params),
#endif

    AP_GROUPEND
};

AP_Mount::AP_Mount()
{
    if (_singleton != nullptr) {
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        AP_HAL::panic("Mount must be singleton");
#endif
        return;
    }
    _singleton = this;

    AP_Param::setup_object_defaults(this, var_info);
}

// init - detect and initialise all mounts
void AP_Mount::init()
{
    // check init has not been called before
    if (_num_instances != 0) {
        return;
    }

    // perform any required parameter conversion
    convert_params();

    // primary is reset to the first instantiated mount
    bool primary_set = false;

    // keep track of number of serial instances for initialisation
    uint8_t serial_instance = 0;

    // create each instance
    for (uint8_t instance=0; instance<AP_MOUNT_MAX_INSTANCES; instance++) {
        switch (get_mount_type(instance)) {
        case Type::None:
            break;
#if HAL_MOUNT_SERVO_ENABLED
        case Type::Servo:
            _backends[instance] = NEW_NOTHROW AP_Mount_Servo(*this, _params[instance], true, instance);
            _num_instances++;
            break;
#endif
#if HAL_SOLO_GIMBAL_ENABLED
        case Type::SoloGimbal:
            _backends[instance] = NEW_NOTHROW AP_Mount_SoloGimbal(*this, _params[instance], instance);
            _num_instances++;
            break;
#endif // HAL_SOLO_GIMBAL_ENABLED

#if HAL_MOUNT_ALEXMOS_ENABLED
        case Type::Alexmos:
            _backends[instance] = NEW_NOTHROW AP_Mount_Alexmos(*this, _params[instance], instance);
            _num_instances++;
            break;
#endif

#if HAL_MOUNT_STORM32MAVLINK_ENABLED
        // check for SToRM32 mounts using MAVLink protocol
        case Type::SToRM32:
            _backends[instance] = NEW_NOTHROW AP_Mount_SToRM32(*this, _params[instance], instance);
            _num_instances++;
            break;
#endif

#if HAL_MOUNT_STORM32SERIAL_ENABLED
        // check for SToRM32 mounts using serial protocol
        case Type::SToRM32_serial:
            _backends[instance] = NEW_NOTHROW AP_Mount_SToRM32_serial(*this, _params[instance], instance, serial_instance);
            _num_instances++;
            serial_instance++;
            break;
#endif

#if HAL_MOUNT_GREMSY_ENABLED
        // check for Gremsy mounts
        case Type::Gremsy:
            _backends[instance] = NEW_NOTHROW AP_Mount_Gremsy(*this, _params[instance], instance);
            _num_instances++;
            break;
#endif // HAL_MOUNT_GREMSY_ENABLED

#if HAL_MOUNT_SERVO_ENABLED
        // check for BrushlessPWM mounts (uses Servo backend)
        case Type::BrushlessPWM:
            _backends[instance] = NEW_NOTHROW AP_Mount_Servo(*this, _params[instance], false, instance);
            _num_instances++;
            break;
#endif

#if HAL_MOUNT_SIYI_ENABLED
        // check for Siyi gimbal
        case Type::Siyi:
            _backends[instance] = NEW_NOTHROW AP_Mount_Siyi(*this, _params[instance], instance, serial_instance);
            _num_instances++;
            serial_instance++;
            break;
#endif // HAL_MOUNT_SIYI_ENABLED

#if HAL_MOUNT_SCRIPTING_ENABLED
        // check for Scripting gimbal
        case Type::Scripting:
            _backends[instance] = NEW_NOTHROW AP_Mount_Scripting(*this, _params[instance], instance);
            _num_instances++;
            break;
#endif // HAL_MOUNT_SCRIPTING_ENABLED

#if HAL_MOUNT_XACTI_ENABLED
        // check for Xacti gimbal
        case Type::Xacti:
            _backends[instance] = NEW_NOTHROW AP_Mount_Xacti(*this, _params[instance], instance);
            _num_instances++;
            break;
#endif // HAL_MOUNT_XACTI_ENABLED

#if HAL_MOUNT_VIEWPRO_ENABLED
        // check for Xacti gimbal
        case Type::Viewpro:
            _backends[instance] = NEW_NOTHROW AP_Mount_Viewpro(*this, _params[instance], instance, serial_instance);
            _num_instances++;
            serial_instance++;
            break;
#endif // HAL_MOUNT_VIEWPRO_ENABLED

#if HAL_MOUNT_TOPOTEK_ENABLED
        // check for Topotek gimbal
        case Type::Topotek:
            _backends[instance] = NEW_NOTHROW AP_Mount_Topotek(*this, _params[instance], instance, serial_instance);
            _num_instances++;
            serial_instance++;
            break;
#endif // HAL_MOUNT_TOPOTEK_ENABLED

#if HAL_MOUNT_CADDX_ENABLED
        // check for CADDX gimbal
        case Type::CADDX:
            _backends[instance] = NEW_NOTHROW AP_Mount_CADDX(*this, _params[instance], instance, serial_instance);
            _num_instances++;
            serial_instance++;
            break;
#endif // HAL_MOUNT_CADDX_ENABLED

#if HAL_MOUNT_XFROBOT_ENABLED
        // check for XFRobot gimbal
        case Type::XFRobot:
            _backends[instance] = NEW_NOTHROW AP_Mount_XFRobot(*this, _params[instance], instance, serial_instance);
            _num_instances++;
            serial_instance++;
            break;
#endif // HAL_MOUNT_XFROBOT_ENABLED

//OW
#if HAL_MOUNT_STORM32_MAVLINK_V2_ENABLED
        // check for STorM32_MAVLink mounts using MAVLink protocol
        case Type::STorM32_MAVLink:
            _backends[instance] = new AP_Mount_STorM32_MAVLink(*this, _params[instance], instance);
            _num_instances++;
            serial_instance++;
            break;
#endif // HAL_MOUNT_STORM32_MAVLINK_V2_ENABLED
//OWEND
        }

        // init new instance
        if (_backends[instance] != nullptr) {
            if (!primary_set) {
                _primary = instance;
                primary_set = true;
            }
        }
    }

    // init each instance, do it after all instances were created, so that they all know things
    for (uint8_t instance=0; instance<AP_MOUNT_MAX_INSTANCES; instance++) {
        if (_backends[instance] != nullptr) {
            _backends[instance]->init();
            set_mode_to_default(instance);
        }
    }

    (void)serial_instance;
}

// update - give mount opportunity to update servos.  should be called at 10hz or higher
void AP_Mount::update()
{
    // update each instance
    for (uint8_t instance=0; instance<AP_MOUNT_MAX_INSTANCES; instance++) {
        if (_backends[instance] != nullptr) {
            _backends[instance]->update();
        }
    }
}

// used for gimbals that need to read INS data at full rate
void AP_Mount::update_fast()
{
    // update each instance
    for (uint8_t instance=0; instance<AP_MOUNT_MAX_INSTANCES; instance++) {
        if (_backends[instance] != nullptr) {
            _backends[instance]->update_fast();
        }
    }
}

// get_mount_type - returns the type of mount
AP_Mount::Type AP_Mount::get_mount_type(uint8_t instance) const
{
    if (instance >= AP_MOUNT_MAX_INSTANCES) {
        return Type::None;
    }

    return (Type)_params[instance].type.get();
}

// has_pan_control - returns true if the mount has yaw control (required for copters)
bool AP_Mount::has_pan_control(uint8_t instance) const
{
    const auto *backend = get_instance(instance);
    if (backend == nullptr) {
        return false;
    }

    // ask backend if it support pan
    return backend->has_pan_control();
}

// get_mode - returns current mode of mount (i.e. Retracted, Neutral, RC_Targeting, GPS Point)
MAV_MOUNT_MODE AP_Mount::get_mode(uint8_t instance) const
{
    const auto *backend = get_instance(instance);
    if (backend == nullptr) {
        return MAV_MOUNT_MODE_RETRACT;
    }

    // ask backend its mode
    return backend->get_mode();
}

// set_mode_to_default - restores the mode to it's default mode held in the MNTx__DEFLT_MODE parameter
//      this operation requires 60us on a Pixhawk/PX4
void AP_Mount::set_mode_to_default(uint8_t instance)
{
    auto *backend = get_instance(instance);
    if (backend == nullptr) {
        return;
    }
    backend->set_mode((enum MAV_MOUNT_MODE)_params[instance].default_mode.get());
}

// set_mode - sets mount's mode
void AP_Mount::set_mode(uint8_t instance, enum MAV_MOUNT_MODE mode)
{
    auto *backend = get_instance(instance);
    if (backend == nullptr) {
        return;
    }

    // call backend's set_mode
    backend->set_mode(mode);
}

// set yaw_lock used in RC_TARGETING mode.  If true, the gimbal's yaw target is maintained in earth-frame meaning it will lock onto an earth-frame heading (e.g. North)
// If false (aka "follow") the gimbal's yaw is maintained in body-frame meaning it will rotate with the vehicle
void AP_Mount::set_yaw_lock(uint8_t instance, bool yaw_lock)
{
    auto *backend = get_instance(instance);
    if (backend == nullptr) {
        return;
    }

    // call backend's set_yaw_lock
    backend->set_yaw_lock(yaw_lock);
}

// set angle target in degrees
// roll and pitch are in earth-frame
// yaw_is_earth_frame (aka yaw_lock) should be true if yaw angle is earth-frame, false if body-frame
void AP_Mount::set_angle_target(uint8_t instance, float roll_deg, float pitch_deg, float yaw_deg, bool yaw_is_earth_frame)
{
    auto *backend = get_instance(instance);
    if (backend == nullptr) {
        return;
    }

    // send command to backend
    backend->set_angle_target(roll_deg, pitch_deg, yaw_deg, yaw_is_earth_frame);
}

// sets rate target in deg/s
// yaw_lock should be true if the yaw rate is earth-frame, false if body-frame (e.g. rotates with body of vehicle)
void AP_Mount::set_rate_target(uint8_t instance, float roll_degs, float pitch_degs, float yaw_degs, bool yaw_lock)
{
    auto *backend = get_instance(instance);
    if (backend == nullptr) {
        return;
    }

    // send command to backend
    backend->set_rate_target(roll_degs, pitch_degs, yaw_degs, yaw_lock);
}

//OW
/*
MAV_RESULT AP_Mount::handle_command_do_mount_configure(const mavlink_command_int_t &packet)
{
    auto *backend = get_primary();
    if (backend == nullptr) {
        return MAV_RESULT_FAILED;
    }

    backend->set_mode((MAV_MOUNT_MODE)packet.param1);

    return MAV_RESULT_ACCEPTED;
}

MAV_RESULT AP_Mount::handle_command_do_mount_control(const mavlink_command_int_t &packet)
{
    auto *backend = get_primary();
    if (backend == nullptr) {
        return MAV_RESULT_FAILED;
    }

    return backend->handle_command_do_mount_control(packet);
}

MAV_RESULT AP_Mount::handle_command_do_gimbal_manager_pitchyaw(const mavlink_command_int_t &packet)
{
    AP_Mount_Backend *backend;

    // check gimbal device id.  0 is primary, 1 is 1st gimbal, 2 is
    // 2nd gimbal, etc
    const uint8_t instance = packet.z;
    if (instance == 0) {
        backend = get_primary();
    } else {
        backend = get_instance(instance - 1);
    }

    if (backend == nullptr) {
        return MAV_RESULT_FAILED;
    }

    // check flags for change to RETRACT
    const uint32_t flags = packet.x;
    if ((flags & GIMBAL_MANAGER_FLAGS_RETRACT) > 0) {
        backend->set_mode(MAV_MOUNT_MODE_RETRACT);
        return MAV_RESULT_ACCEPTED;
    }
    // check flags for change to NEUTRAL
    if ((flags & GIMBAL_MANAGER_FLAGS_NEUTRAL) > 0) {
        backend->set_mode(MAV_MOUNT_MODE_NEUTRAL);
        return MAV_RESULT_ACCEPTED;
    }

    // param1 : pitch_angle (in degrees)
    // param2 : yaw angle (in degrees)
    const float pitch_angle_deg = packet.param1;
    const float yaw_angle_deg = packet.param2;
    if (!isnan(pitch_angle_deg) && !isnan(yaw_angle_deg)) {
        backend->set_angle_target(0, pitch_angle_deg, yaw_angle_deg, flags & GIMBAL_MANAGER_FLAGS_YAW_LOCK);
        return MAV_RESULT_ACCEPTED;
    }

    // param3 : pitch_rate (in deg/s)
    // param4 : yaw rate (in deg/s)
    const float pitch_rate_degs = packet.param3;
    const float yaw_rate_degs = packet.param4;
    if (!isnan(pitch_rate_degs) && !isnan(yaw_rate_degs)) {
        backend->set_rate_target(0, pitch_rate_degs, yaw_rate_degs, flags & GIMBAL_MANAGER_FLAGS_YAW_LOCK);
        return MAV_RESULT_ACCEPTED;
    }

    // if neither angles nor rates were provided set the RC_TARGETING yaw lock state
    if (isnan(pitch_angle_deg) && isnan(yaw_angle_deg) && isnan(pitch_rate_degs) && isnan(yaw_rate_degs)) {
        backend->set_yaw_lock(flags & GIMBAL_MANAGER_FLAGS_YAW_LOCK);
        return MAV_RESULT_ACCEPTED;
    }

    return MAV_RESULT_FAILED;
}

// handle mav_cmd_do_gimbal_manager_configure for deconflicting different mavlink message senders
MAV_RESULT AP_Mount::handle_command_do_gimbal_manager_configure(const mavlink_command_int_t &packet, const mavlink_message_t &msg)
{
    AP_Mount_Backend *backend;

    // check gimbal device id.  0 is primary, 1 is 1st gimbal, 2 is 2nd gimbal, etc
    const uint8_t instance = packet.z;
    if (instance == 0) {
        backend = get_primary();
    } else {
        backend = get_instance(instance - 1);
    }

    if (backend == nullptr) {
        return MAV_RESULT_FAILED;
    }

    return backend->handle_command_do_gimbal_manager_configure(packet, msg);
}

void AP_Mount::handle_gimbal_manager_set_attitude(const mavlink_message_t &msg)
{
    mavlink_gimbal_manager_set_attitude_t packet;
    mavlink_msg_gimbal_manager_set_attitude_decode(&msg,&packet);

    AP_Mount_Backend *backend;

    // check gimbal device id.  0 is primary, 1 is 1st gimbal, 2 is
    // 2nd gimbal, etc
    const uint8_t instance = packet.gimbal_device_id;
    if (instance == 0) {
        backend = get_primary();
    } else {
        backend = get_instance(instance - 1);
    }

    if (backend == nullptr) {
        return;
    }

    // check flags for change to RETRACT
    const uint32_t flags = packet.flags;
    if ((flags & GIMBAL_MANAGER_FLAGS_RETRACT) > 0) {
        backend->set_mode(MAV_MOUNT_MODE_RETRACT);
        return;
    }

    // check flags for change to NEUTRAL
    if ((flags & GIMBAL_MANAGER_FLAGS_NEUTRAL) > 0) {
        backend->set_mode(MAV_MOUNT_MODE_NEUTRAL);
        return;
    }

    const Quaternion att_quat{packet.q};
    const Vector3f att_rate_degs {
        packet.angular_velocity_x,
        packet.angular_velocity_y,
        packet.angular_velocity_y
    };

    // ensure that we are only demanded to a specific attitude or to
    // achieve a specific rate.  Do not allow both to be specified at
    // the same time:
    if (!att_quat.is_nan() && !att_rate_degs.is_nan()) {
        return;
    }

    if (!att_quat.is_nan()) {
        // convert quaternion to euler angles
        Vector3f attitude;
        att_quat.to_euler(attitude);  // attitude is in radians here
        attitude *= RAD_TO_DEG;  // convert to degrees

        backend->set_angle_target(attitude.x, attitude.y, attitude.z, flags & GIMBAL_MANAGER_FLAGS_YAW_LOCK);
        return;
    }

    {
        const float roll_rate_degs = degrees(packet.angular_velocity_x);
        const float pitch_rate_degs = degrees(packet.angular_velocity_y);
        const float yaw_rate_degs = degrees(packet.angular_velocity_z);
        backend->set_rate_target(roll_rate_degs, pitch_rate_degs, yaw_rate_degs, flags & GIMBAL_MANAGER_FLAGS_YAW_LOCK);
        return;
    }
}

void AP_Mount::handle_gimbal_manager_set_pitchyaw(const mavlink_message_t &msg)
{
    mavlink_gimbal_manager_set_pitchyaw_t packet;
    mavlink_msg_gimbal_manager_set_pitchyaw_decode(&msg,&packet);

    AP_Mount_Backend *backend;

    // check gimbal device id.  0 is primary, 1 is 1st gimbal, 2 is
    // 2nd gimbal, etc
    const uint8_t instance = packet.gimbal_device_id;
    if (instance == 0) {
        backend = get_primary();
    } else {
        backend = get_instance(instance - 1);
    }

    if (backend == nullptr) {
        return;
    }

    // check flags for change to RETRACT
    uint32_t flags = (uint32_t)packet.flags;
    if ((flags & GIMBAL_MANAGER_FLAGS_RETRACT) > 0) {
        backend->set_mode(MAV_MOUNT_MODE_RETRACT);
        return;
    }
    // check flags for change to NEUTRAL
    if ((flags & GIMBAL_MANAGER_FLAGS_NEUTRAL) > 0) {
        backend->set_mode(MAV_MOUNT_MODE_NEUTRAL);
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
        backend->set_angle_target(0, pitch_angle_deg, yaw_angle_deg, flags & GIMBAL_MANAGER_FLAGS_YAW_LOCK);
        return;
    }

    // pitch_rate and yaw_rate from packet are in rad/s
    if (!isnan(packet.pitch_rate) && !isnan(packet.yaw_rate)) {
        const float pitch_rate_degs = degrees(packet.pitch_rate);
        const float yaw_rate_degs = degrees(packet.yaw_rate);
        backend->set_rate_target(0, pitch_rate_degs, yaw_rate_degs, flags & GIMBAL_MANAGER_FLAGS_YAW_LOCK);
        return;
    }

    // if neither angles nor rates were provided set the RC_TARGETING yaw lock state
    if (isnan(packet.pitch) && isnan(packet.yaw) && isnan(packet.pitch_rate) && isnan(packet.yaw_rate)) {
        backend->set_yaw_lock(flags & GIMBAL_MANAGER_FLAGS_YAW_LOCK);
        return;
    }
}

MAV_RESULT AP_Mount::handle_command_do_set_roi_sysid(const mavlink_command_int_t &packet)
{
    set_target_sysid((uint8_t)packet.param1);
    return MAV_RESULT_ACCEPTED;
}

MAV_RESULT AP_Mount::handle_command(const mavlink_command_int_t &packet, const mavlink_message_t &msg)
{
    switch (packet.command) {
    case MAV_CMD_DO_MOUNT_CONFIGURE:
        return handle_command_do_mount_configure(packet);
    case MAV_CMD_DO_MOUNT_CONTROL:
        return handle_command_do_mount_control(packet);
    case MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW:
        return handle_command_do_gimbal_manager_pitchyaw(packet);
    case MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE:
        return handle_command_do_gimbal_manager_configure(packet, msg);
    case MAV_CMD_DO_SET_ROI_SYSID:
        return handle_command_do_set_roi_sysid(packet);
    default:
        return MAV_RESULT_UNSUPPORTED;
    }
}
*/
//OWEND

//OW
uint8_t AP_Mount::get_gimbal_device_id(uint8_t instance) const
{
    auto *backend = get_instance(instance);
    if (backend == nullptr) {
        return 0;
    }
    return backend->get_gimbal_device_id();
}

MAV_RESULT AP_Mount::handle_command_do_mount_configure(const mavlink_command_int_t &packet, const mavlink_message_t &msg)
{
    for (uint8_t instance=0; instance<AP_MOUNT_MAX_INSTANCES; instance++) {
        if (_backends[instance] == nullptr) continue;
        if (_backends[instance]->is_in_control(msg.sysid, msg.compid, 0)) {
            _backends[instance]->set_mode((MAV_MOUNT_MODE)packet.param1);
        }
    }
    return MAV_RESULT_ACCEPTED;
}

MAV_RESULT AP_Mount::handle_command_do_mount_control(const mavlink_command_int_t &packet, const mavlink_message_t &msg)
{
    auto *backend = get_primary();
    if (backend == nullptr) {
        return MAV_RESULT_FAILED;
    }
    if (backend->is_in_control(msg.sysid, msg.compid, 0)) {
        return backend->handle_command_do_mount_control(packet);
    }
    return MAV_RESULT_FAILED;
}

MAV_RESULT AP_Mount::handle_command_do_gimbal_manager_configure(const mavlink_command_int_t &packet, const mavlink_message_t &msg)
{
    for (uint8_t instance=0; instance<AP_MOUNT_MAX_INSTANCES; instance++) {
        if (_backends[instance] == nullptr) continue;
        if ((uint8_t)packet.z == 0 || _backends[instance]->get_gimbal_device_id() == (uint8_t)packet.z) {
            _backends[instance]->handle_command_do_gimbal_manager_configure(packet, msg);
        }
    }
    return MAV_RESULT_ACCEPTED;
}

void AP_Mount::handle_gimbal_manager_set_pitchyaw(const mavlink_message_t &msg)
{
    mavlink_gimbal_manager_set_pitchyaw_t packet;
    mavlink_msg_gimbal_manager_set_pitchyaw_decode(&msg, &packet);

    for (uint8_t instance=0; instance<AP_MOUNT_MAX_INSTANCES; instance++) {
        if (_backends[instance] == nullptr) continue;
        if (_backends[instance]->is_in_control(msg.sysid, msg.compid, packet.gimbal_device_id)) {
            _backends[instance]->handle_gimbal_manager_set_pitchyaw(packet);
        }
    }
}

void AP_Mount::handle_gimbal_manager_set_attitude(const mavlink_message_t &msg)
{
    mavlink_gimbal_manager_set_attitude_t packet;
    mavlink_msg_gimbal_manager_set_attitude_decode(&msg, &packet);

    for (uint8_t instance=0; instance<AP_MOUNT_MAX_INSTANCES; instance++) {
        if (_backends[instance] == nullptr) continue;
        if (_backends[instance]->is_in_control(msg.sysid, msg.compid, packet.gimbal_device_id)) {
            _backends[instance]->handle_gimbal_manager_set_attitude(packet);
        }
    }
}

MAV_RESULT AP_Mount::handle_command_do_gimbal_manager_pitchyaw(const mavlink_command_int_t &packet, const mavlink_message_t &msg)
{
    for (uint8_t instance=0; instance<AP_MOUNT_MAX_INSTANCES; instance++) {
        if (_backends[instance] == nullptr) continue;
        if (_backends[instance]->is_in_control(msg.sysid, msg.compid, packet.z)) {
            _backends[instance]->handle_command_do_gimbal_manager_pitchyaw(packet);
        }
    }
    return MAV_RESULT_ACCEPTED;
}

MAV_RESULT AP_Mount::handle_command_do_set_roi_sysid(const mavlink_command_int_t &packet, const mavlink_message_t &msg)
{
    for (uint8_t instance=0; instance<AP_MOUNT_MAX_INSTANCES; instance++) {
        if (_backends[instance] == nullptr) continue;
        if (_backends[instance]->is_in_control(msg.sysid, msg.compid, (uint8_t)packet.param2)) {
            _backends[instance]->set_target_sysid((uint8_t)packet.param1);
        }
    }
    return MAV_RESULT_ACCEPTED;
}

MAV_RESULT AP_Mount::handle_command(const mavlink_command_int_t &packet, const mavlink_message_t &msg)
{
    switch (packet.command) {
    case MAV_CMD_DO_MOUNT_CONFIGURE:
        return handle_command_do_mount_configure(packet, msg);
    case MAV_CMD_DO_MOUNT_CONTROL:
        return handle_command_do_mount_control(packet, msg);
    case MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW:
        return handle_command_do_gimbal_manager_pitchyaw(packet, msg);
    case MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE:
        return handle_command_do_gimbal_manager_configure(packet, msg);
    case MAV_CMD_DO_SET_ROI_SYSID:
        return handle_command_do_set_roi_sysid(packet, msg);
    default:
        return MAV_RESULT_UNSUPPORTED;
    }
}
//OWEND

/// Change the configuration of the mount
void AP_Mount::handle_global_position_int(const mavlink_message_t &msg)
{
    mavlink_global_position_int_t packet;
    mavlink_msg_global_position_int_decode(&msg, &packet);

    if (!check_latlng(packet.lat, packet.lon)) {
        return;
    }

    for (uint8_t instance=0; instance<AP_MOUNT_MAX_INSTANCES; instance++) {
        if (_backends[instance] != nullptr) {
            _backends[instance]->handle_global_position_int(msg.sysid, packet);
        }
    }
}

#if HAL_GCS_ENABLED
// send a GIMBAL_DEVICE_ATTITUDE_STATUS message to GCS
void AP_Mount::send_gimbal_device_attitude_status(mavlink_channel_t chan)
{
    // call send_gimbal_device_attitude_status for each instance
    for (uint8_t instance=0; instance<AP_MOUNT_MAX_INSTANCES; instance++) {
        if (_backends[instance] != nullptr) {
            _backends[instance]->send_gimbal_device_attitude_status(chan);
        }
    }
}

// send a GIMBAL_MANAGER_INFORMATION message to GCS
void AP_Mount::send_gimbal_manager_information(mavlink_channel_t chan)
{
    // call send_gimbal_device_attitude_status for each instance
    for (uint8_t instance=0; instance<AP_MOUNT_MAX_INSTANCES; instance++) {
        if (_backends[instance] != nullptr) {
            _backends[instance]->send_gimbal_manager_information(chan);
        }
    }
}

// send a GIMBAL_MANAGER_STATUS message to GCS
void AP_Mount::send_gimbal_manager_status(mavlink_channel_t chan)
{
    // call send_gimbal_device_attitude_status for each instance
    for (uint8_t instance=0; instance<AP_MOUNT_MAX_INSTANCES; instance++) {
        if (_backends[instance] != nullptr) {
            _backends[instance]->send_gimbal_manager_status(chan);
        }
    }
}
#endif  // HAL_GCS_ENABLED

#if AP_MOUNT_POI_TO_LATLONALT_ENABLED
// get poi information.  Returns true on success and fills in gimbal attitude, location and poi location
bool AP_Mount::get_poi(uint8_t instance, Quaternion &quat, Location &loc, Location &poi_loc) const
{
    auto *backend = get_instance(instance);
    if (backend == nullptr) {
        return false;
    }
    return backend->get_poi(instance, quat, loc, poi_loc);
}
#endif

// get attitude as a quaternion.  returns true on success.
// att_quat will be an earth-frame quaternion rotated such that
// yaw is in body-frame.
bool AP_Mount::get_attitude_quaternion(uint8_t instance, Quaternion& att_quat)
{
    auto *backend = get_instance(instance);
    if (backend == nullptr) {
        return false;
    }
    return backend->get_attitude_quaternion(att_quat);
}

// get mount's current attitude in euler angles in degrees.  yaw angle is in body-frame
// returns true on success
bool AP_Mount::get_attitude_euler(uint8_t instance, float& roll_deg, float& pitch_deg, float& yaw_bf_deg)
{
    // re-use get_attitude_quaternion and convert to Euler angles
    Quaternion att_quat;
    if (!get_attitude_quaternion(instance, att_quat)) {
        return false;
    }

    float roll_rad, pitch_rad, yaw_rad;
    att_quat.to_euler(roll_rad, pitch_rad, yaw_rad);
    roll_deg = degrees(roll_rad);
    pitch_deg = degrees(pitch_rad);
    yaw_bf_deg = degrees(yaw_rad);
    return true;
}

// run pre-arm check.  returns false on failure and fills in failure_msg
// any failure_msg returned will not include a prefix
bool AP_Mount::pre_arm_checks(char *failure_msg, uint8_t failure_msg_len)
{
    // check type parameters
    for (uint8_t i=0; i<AP_MOUNT_MAX_INSTANCES; i++) {
        if ((get_mount_type(i) != Type::None) && (_backends[i] == nullptr)) {
            strncpy(failure_msg, "check TYPE", failure_msg_len);
            return false;
        }
    }

    // return true if no mount configured
    if (_num_instances == 0) {
        return true;
    }

    // check healthy
    for (uint8_t i=0; i<AP_MOUNT_MAX_INSTANCES; i++) {
        if ((_backends[i] != nullptr) && !_backends[i]->healthy()) {
            strncpy(failure_msg, "not healthy", failure_msg_len);
            return false;
        }
    }

    return true;
}

// get target rate in deg/sec. returns true on success
bool AP_Mount::get_rate_target(uint8_t instance, float& roll_degs, float& pitch_degs, float& yaw_degs, bool& yaw_is_earth_frame)
{
    auto *backend = get_instance(instance);
    if (backend == nullptr) {
        return false;
    }
    return backend->get_rate_target(roll_degs, pitch_degs, yaw_degs, yaw_is_earth_frame);
}

// get target angle in deg. returns true on success
bool AP_Mount::get_angle_target(uint8_t instance, float& roll_deg, float& pitch_deg, float& yaw_deg, bool& yaw_is_earth_frame)
{
    auto *backend = get_instance(instance);
    if (backend == nullptr) {
        return false;
    }
    return backend->get_angle_target(roll_deg, pitch_deg, yaw_deg, yaw_is_earth_frame);
}

// accessors for scripting backends and logging
bool AP_Mount::get_location_target(uint8_t instance, Location& target_loc)
{
    auto *backend = get_instance(instance);
    if (backend == nullptr) {
        return false;
    }
    return backend->get_location_target(target_loc);
}

void AP_Mount::set_attitude_euler(uint8_t instance, float roll_deg, float pitch_deg, float yaw_bf_deg)
{
    auto *backend = get_instance(instance);
    if (backend == nullptr) {
        return;
    }
    backend->set_attitude_euler(roll_deg, pitch_deg, yaw_bf_deg);
}

#if HAL_LOGGING_ENABLED
// write mount log packet for all backends
void AP_Mount::write_log()
{
    // each instance writes log
    for (uint8_t instance=0; instance<AP_MOUNT_MAX_INSTANCES; instance++) {
        if (_backends[instance] != nullptr) {
            _backends[instance]->write_log(0);
        }
    }
}

void AP_Mount::write_log(uint8_t instance, uint64_t timestamp_us)
{
    auto *backend = get_instance(instance);
    if (backend == nullptr) {
        return;
    }
    backend->write_log(timestamp_us);
}
#endif

// point at system ID sysid
void AP_Mount::set_target_sysid(uint8_t instance, uint8_t sysid)
{
    auto *backend = get_instance(instance);
    if (backend == nullptr) {
        return;
    }
    // call instance's set_roi_cmd
    backend->set_target_sysid(sysid);
}

// set_roi_target - sets target location that mount should attempt to point towards
void AP_Mount::set_roi_target(uint8_t instance, const Location &target_loc)
{
    auto *backend = get_instance(instance);
    if (backend == nullptr) {
        return;
    }
    backend->set_roi_target(target_loc);
}

// clear_roi_target - clears target location that mount should attempt to point towards
void AP_Mount::clear_roi_target(uint8_t instance)
{
    auto *backend = get_instance(instance);
    if (backend == nullptr) {
        return;
    }
    backend->clear_roi_target();
}

//
// camera controls for gimbals that include a camera
//

// take a picture.  returns true on success
bool AP_Mount::take_picture(uint8_t instance)
{
    auto *backend = get_instance(instance);
    if (backend == nullptr) {
        return false;
    }
    return backend->take_picture();
}

// start or stop video recording.  returns true on success
// set start_recording = true to start record, false to stop recording
bool AP_Mount::record_video(uint8_t instance, bool start_recording)
{
    auto *backend = get_instance(instance);
    if (backend == nullptr) {
        return false;
    }
    return backend->record_video(start_recording);
}

// set zoom specified as a rate or percentage
bool AP_Mount::set_zoom(uint8_t instance, ZoomType zoom_type, float zoom_value)
{
    auto *backend = get_instance(instance);
    if (backend == nullptr) {
        return false;
    }
    return backend->set_zoom(zoom_type, zoom_value);
}

// set focus specified as rate, percentage or auto
// focus in = -1, focus hold = 0, focus out = 1
SetFocusResult AP_Mount::set_focus(uint8_t instance, FocusType focus_type, float focus_value)
{
    auto *backend = get_instance(instance);
    if (backend == nullptr) {
        return SetFocusResult::FAILED;
    }
    return backend->set_focus(focus_type, focus_value);
}

// set tracking to none, point or rectangle (see TrackingType enum)
// if POINT only p1 is used, if RECTANGLE then p1 is top-left, p2 is bottom-right
// p1,p2 are in range 0 to 1.  0 is left or top, 1 is right or bottom
bool AP_Mount::set_tracking(uint8_t instance, TrackingType tracking_type, const Vector2f& p1, const Vector2f& p2)
{
    auto *backend = get_instance(instance);
    if (backend == nullptr) {
        return false;
    }
    return backend->set_tracking(tracking_type, p1, p2);
}

// set camera lens as a value from 0 to 5
bool AP_Mount::set_lens(uint8_t instance, uint8_t lens)
{
    auto *backend = get_instance(instance);
    if (backend == nullptr) {
        return false;
    }
    return backend->set_lens(lens);
}

#if HAL_MOUNT_SET_CAMERA_SOURCE_ENABLED
// set_camera_source is functionally the same as set_lens except primary and secondary lenses are specified by type
// primary and secondary sources use the AP_Camera::CameraSource enum cast to uint8_t
bool AP_Mount::set_camera_source(uint8_t instance, uint8_t primary_source, uint8_t secondary_source)
{
    auto *backend = get_instance(instance);
    if (backend == nullptr) {
        return false;
    }
    return backend->set_camera_source(primary_source, secondary_source);
}
#endif

// send camera information message to GCS
void AP_Mount::send_camera_information(uint8_t instance, mavlink_channel_t chan) const
{
    auto *backend = get_instance(instance);
    if (backend == nullptr) {
        return;
    }
    backend->send_camera_information(chan);
}

// send camera settings message to GCS
void AP_Mount::send_camera_settings(uint8_t instance, mavlink_channel_t chan) const
{
    auto *backend = get_instance(instance);
    if (backend == nullptr) {
        return;
    }
    backend->send_camera_settings(chan);
}

// send camera capture status message to GCS
void AP_Mount::send_camera_capture_status(uint8_t instance, mavlink_channel_t chan) const
{
    auto *backend = get_instance(instance);
    if (backend == nullptr) {
        return;
    }
    backend->send_camera_capture_status(chan);
}

#if AP_MOUNT_SEND_THERMAL_RANGE_ENABLED
// send camera thermal range message to GCS
void AP_Mount::send_camera_thermal_range(uint8_t instance, mavlink_channel_t chan) const
{
    auto *backend = get_instance(instance);
    if (backend == nullptr) {
        return;
    }
    backend->send_camera_thermal_range(chan);
}
#endif

// change camera settings not normally used by autopilot
// setting values from AP_Camera::Setting enum
bool AP_Mount::change_setting(uint8_t instance, CameraSetting setting, float value)
{
    auto *backend = get_instance(instance);
    if (backend == nullptr) {
        return false;
    }
    return backend->change_setting(setting, value);
}

// get rangefinder distance.  Returns true on success
bool AP_Mount::get_rangefinder_distance(uint8_t instance, float& distance_m) const
{
    auto *backend = get_instance(instance);
    if (backend == nullptr) {
        return false;
    }
    return backend->get_rangefinder_distance(distance_m);
}

// enable/disable rangefinder.  Returns true on success
bool AP_Mount::set_rangefinder_enable(uint8_t instance, bool enable)
{
    auto *backend = get_instance(instance);
    if (backend == nullptr) {
        return false;
    }
    return backend->set_rangefinder_enable(enable);
}

AP_Mount_Backend *AP_Mount::get_primary() const
{
    return get_instance(_primary);
}

AP_Mount_Backend *AP_Mount::get_instance(uint8_t instance) const
{
    if (instance >= ARRAY_SIZE(_backends)) {
        return nullptr;
    }
    return _backends[instance];
}

// pass a GIMBAL_REPORT message to the backend
void AP_Mount::handle_gimbal_report(mavlink_channel_t chan, const mavlink_message_t &msg)
{
    for (uint8_t instance=0; instance<AP_MOUNT_MAX_INSTANCES; instance++) {
        if (_backends[instance] != nullptr) {
            _backends[instance]->handle_gimbal_report(chan, msg);
        }
    }
}

void AP_Mount::handle_message(mavlink_channel_t chan, const mavlink_message_t &msg)
{
//OW
    handle_message_extra(chan, msg);
//OWEND

    switch (msg.msgid) {
    case MAVLINK_MSG_ID_GIMBAL_REPORT:
        handle_gimbal_report(chan, msg);
        break;
    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
        handle_global_position_int(msg);
        break;
    case MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_ATTITUDE:
        handle_gimbal_manager_set_attitude(msg);
        break;
    case MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_PITCHYAW:
        handle_gimbal_manager_set_pitchyaw(msg);
        break;
    case MAVLINK_MSG_ID_GIMBAL_DEVICE_INFORMATION:
        handle_gimbal_device_information(msg);
        break;
    case MAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS:
        handle_gimbal_device_attitude_status(msg);
        break;
    default:
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        AP_HAL::panic("Unhandled mount case");
#endif
        break;
    }
}

// handle PARAM_VALUE
void AP_Mount::handle_param_value(const mavlink_message_t &msg)
{
    for (uint8_t instance=0; instance<AP_MOUNT_MAX_INSTANCES; instance++) {
        if (_backends[instance] != nullptr) {
            _backends[instance]->handle_param_value(msg);
        }
    }
}


// handle GIMBAL_DEVICE_INFORMATION message
void AP_Mount::handle_gimbal_device_information(const mavlink_message_t &msg)
{
    for (uint8_t instance=0; instance<AP_MOUNT_MAX_INSTANCES; instance++) {
        if (_backends[instance] != nullptr) {
            _backends[instance]->handle_gimbal_device_information(msg);
        }
    }
}

// handle GIMBAL_DEVICE_ATTITUDE_STATUS message
void AP_Mount::handle_gimbal_device_attitude_status(const mavlink_message_t &msg)
{
    for (uint8_t instance=0; instance<AP_MOUNT_MAX_INSTANCES; instance++) {
        if (_backends[instance] != nullptr) {
            _backends[instance]->handle_gimbal_device_attitude_status(msg);
        }
    }
}

// perform any required parameter conversion
void AP_Mount::convert_params()
{
    // exit immediately if MNT1_TYPE has already been configured
    if (_params[0].type.configured()) {
        return;
    }

    // below conversions added Sep 2022 ahead of 4.3 release

    // convert MNT_TYPE to MNT1_TYPE
    int8_t mnt_type = 0;
    IGNORE_RETURN(AP_Param::get_param_by_index(this, 19, AP_PARAM_INT8, &mnt_type));
    if (mnt_type == 0) {
        // if the mount was not previously set, no need to perform the upgrade logic
        return;
    } else if (mnt_type > 0) {
        int8_t stab_roll = 0;
        int8_t stab_pitch = 0;
        IGNORE_RETURN(AP_Param::get_param_by_index(this, 4, AP_PARAM_INT8, &stab_roll));
        IGNORE_RETURN(AP_Param::get_param_by_index(this, 5, AP_PARAM_INT8, &stab_pitch));
        if (mnt_type == 1 && stab_roll == 0 && stab_pitch == 0)  {
            // Servo type without stabilization is changed to BrushlessPWM
            // conversion is still done even if HAL_MOUNT_SERVO_ENABLED is false
            mnt_type = 7;  // (int8_t)Type::BrushlessPWM;
        }
        // if the mount was previously set, then we need to save the upgraded mount type
        _params[0].type.set_and_save(mnt_type);
    }

    // convert MNT_JSTICK_SPD to MNT1_RC_RATE
    int8_t jstick_spd = 0;
    if (AP_Param::get_param_by_index(this, 16, AP_PARAM_INT8, &jstick_spd) && (jstick_spd > 0)) {
        _params[0].rc_rate_max.set_and_save(jstick_spd * 0.3);
    }

    // find Mount's top level key
    uint16_t k_param_mount_key;
    if (!AP_Param::find_top_level_key_by_pointer(this, k_param_mount_key)) {
        return;
    }

    // table of mount parameters to be converted without scaling
    static const AP_Param::ConversionInfo mnt_param_conversion_info[] {
        { k_param_mount_key, 0, AP_PARAM_INT8, "MNT1_DEFLT_MODE" },
        { k_param_mount_key, 1, AP_PARAM_VECTOR3F, "MNT1_RETRACT" },
        { k_param_mount_key, 2, AP_PARAM_VECTOR3F, "MNT1_NEUTRAL" },
        { k_param_mount_key, 17, AP_PARAM_FLOAT, "MNT1_LEAD_RLL" },
        { k_param_mount_key, 18, AP_PARAM_FLOAT, "MNT1_LEAD_PTCH" },
    };
    uint8_t table_size = ARRAY_SIZE(mnt_param_conversion_info);
    for (uint8_t i=0; i<table_size; i++) {
        AP_Param::convert_old_parameter(&mnt_param_conversion_info[i], 1.0f);
    }

    // mount parameters conversion from centi-degrees to degrees
    static const AP_Param::ConversionInfo mnt_param_deg_conversion_info[] {
        { k_param_mount_key, 8, AP_PARAM_INT16, "MNT1_ROLL_MIN" },
        { k_param_mount_key, 9, AP_PARAM_INT16, "MNT1_ROLL_MAX" },
        { k_param_mount_key, 11, AP_PARAM_INT16, "MNT1_PITCH_MIN" },
        { k_param_mount_key, 12, AP_PARAM_INT16, "MNT1_PITCH_MAX" },
        { k_param_mount_key, 14, AP_PARAM_INT16, "MNT1_YAW_MIN" },
        { k_param_mount_key, 15, AP_PARAM_INT16, "MNT1_YAW_MAX" },
    };
    table_size = ARRAY_SIZE(mnt_param_deg_conversion_info);
    for (uint8_t i=0; i<table_size; i++) {
        AP_Param::convert_old_parameter(&mnt_param_deg_conversion_info[i], 0.01f);
    }

    // struct and array holding mapping between old param table index and new RCx_OPTION value
    struct MountRCConversionTable {
        uint8_t old_rcin_idx;
        uint16_t new_rc_option;
    };
    const struct MountRCConversionTable mnt_rc_conversion_table[] = {
        {7, 212},   // MTN_RC_IN_ROLL to RCx_OPTION = 212 (MOUNT1_ROLL)
        {10, 213},  // MTN_RC_IN_TILT to RCx_OPTION = 213 (MOUNT1_PITCH)
        {13, 214},  // MTN_RC_IN_PAN to RCx_OPTION = 214 (MOUNT1_YAW)
    };
    for (uint8_t i = 0; i < ARRAY_SIZE(mnt_rc_conversion_table); i++) {
        int8_t mnt_rcin = 0;
        if (AP_Param::get_param_by_index(this, mnt_rc_conversion_table[i].old_rcin_idx, AP_PARAM_INT8, &mnt_rcin) && (mnt_rcin > 0)) {
            // get pointers to the appropriate RCx_OPTION parameter
            char pname[17];
            enum ap_var_type ptype;
            snprintf(pname, sizeof(pname), "RC%u_OPTION", (unsigned)mnt_rcin);
            AP_Int16 *rcx_option = (AP_Int16 *)AP_Param::find(pname, &ptype);
            if ((rcx_option != nullptr) && !rcx_option->configured()) {
                rcx_option->set_and_save(mnt_rc_conversion_table[i].new_rc_option);
            }
        }
    }
}

// singleton instance
AP_Mount *AP_Mount::_singleton;

namespace AP {

AP_Mount *mount()
{
    return AP_Mount::get_singleton();
}

};

//OW
void AP_Mount::handle_message_extra(mavlink_channel_t chan, const mavlink_message_t &msg)
{
    for (uint8_t instance=0; instance<AP_MOUNT_MAX_INSTANCES; instance++) {
        if (_backends[instance] != nullptr) {
            _backends[instance]->handle_message_extra(msg);
        }
    }
}

void AP_Mount::send_banner()
{
    for (uint8_t instance=0; instance<AP_MOUNT_MAX_INSTANCES; instance++) {
        if (_backends[instance] != nullptr) {
            _backends[instance]->send_banner();
        }
    }
}
//OWEND

#endif /* HAL_MOUNT_ENABLED */
