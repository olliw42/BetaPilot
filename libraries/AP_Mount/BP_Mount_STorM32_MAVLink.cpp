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
#include <AP_Logger/AP_Logger.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include "BP_Mount_STorM32_MAVLink.h"
#include "bp_version.h"

extern const AP_HAL::HAL& hal;


//******************************************************
// BP_Mount_STorM32_MAVLink, that's the main class
//******************************************************


// constructor
BP_Mount_STorM32_MAVLink::BP_Mount_STorM32_MAVLink(AP_Mount &frontend, AP_Mount::mount_state &state, uint8_t instance) :
    AP_Mount_Backend(frontend, state, instance)
{
    _initialised = false;

    _sysid = 0;
    _compid = 0;
    _chan = MAVLINK_COMM_0; //this is a dummy, will be set correctly by find_gimbal()

    _task_time_last = 0;
    _task_counter = TASK_SLOT0;
}


//------------------------------------------------------
// BP_Mount_STorM32_MAVLink interface functions, ArduPilot Mount
//------------------------------------------------------

// init - performs any required initialisation for this instance
void BP_Mount_STorM32_MAVLink::init(void)
{
    _initialised = false; //should be false but can't hurt to ensure that
}


// update mount position - should be called periodically
// this function must be defined in any case
void BP_Mount_STorM32_MAVLink::update()
{
    if (!_initialised) {
        find_gimbal();
        return;
    }
}


// 400 Hz loop
void BP_Mount_STorM32_MAVLink::update_fast()
{
    if (!_initialised) {
        return;
    }

    static uint16_t loop_rate_hz = 0;
    static uint8_t decimate_counter_max = 0;
    static uint8_t decimate_counter = 0;
    if (loop_rate_hz != AP::scheduler().get_loop_rate_hz()) { //let's cope with loop rate changes
        loop_rate_hz = AP::scheduler().get_loop_rate_hz(); //only 50 to 2000 Hz allowed, thus can't be less than 50
        decimate_counter_max = (uint16_t)(loop_rate_hz + 24)/50 - 1;
        decimate_counter = 0;
    }
    if (decimate_counter) {
        decimate_counter--; //count down
    } else {
        decimate_counter = decimate_counter_max;

        switch (_task_counter) {
            case TASK_SLOT0:
            case TASK_SLOT2:
                send_autopilot_state_for_gimbal_device_to_gimbal();
                break;
        }

        _task_counter++;
        if (_task_counter > TASK_SLOT3) _task_counter = TASK_SLOT0;
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
        _initialised = true;
    }

    //proposal:
    // change this function to allow an index, like find_by_mavtype(index, ....)
    // we then can call it repeatedly until it returns false, whereby increasing index as 0,1,...
    // we then can define that the first mavlink mount is that with lowest ID, and so on
}


void BP_Mount_STorM32_MAVLink::find_gimbal(void)
{
    find_gimbal_oneonly();
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



