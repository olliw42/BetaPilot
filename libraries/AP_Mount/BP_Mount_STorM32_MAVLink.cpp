//*****************************************************
//OW
// (c) olliw, www.olliw.eu, GPL3
// STorM32 mount backend class
// uses 100% MAVlink + storm32.xml
//*****************************************************

#include <GCS_MAVLink/GCS.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_Notify/AP_Notify.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include "BP_Mount_STorM32_MAVLink.h"


// constructor
BP_Mount_STorM32_MAVLink::BP_Mount_STorM32_MAVLink(AP_Mount &frontend, AP_Mount::mount_state &state, uint8_t instance) :
    AP_Mount_Backend(frontend, state, instance)
{
    _initialised = false;

    _task_time_last = 0;
    _task_counter = TASK_SLOT0;

    // we set this by hand
    _sysid = 1; // g.sysid_this_mav;
    _compid = MAV_COMP_ID_GIMBAL3;
    _chan = MAVLINK_COMM_1;
}


// we set this by hand
#define USE_WINCH_STATUS  1
#define USE_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_FULL  1


// init - performs any required initialisation for this instance
void BP_Mount_STorM32_MAVLink::init(void)
{
    _initialised = true;
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
#if USE_WINCH_STATUS
                send_winch_status_to_gimbal();
#else
                send_autopilot_state_for_gimbal_device_to_gimbal();
#endif
                break;
        }

        _task_counter++;
        if (_task_counter > TASK_SLOT3) _task_counter = TASK_SLOT0;
    }
}


void BP_Mount_STorM32_MAVLink::send_autopilot_state_for_gimbal_device_to_gimbal(void)
{
    if (!HAVE_PAYLOAD_SPACE(_chan, AUTOPILOT_STATE_FOR_GIMBAL_DEVICE)) {
        return;
    }

#if USE_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_FULL
    const AP_AHRS_NavEKF &ahrs = AP::ahrs_navekf();
    const AP_GPS &gps = AP::gps();
    const AP_Notify &notify = AP::notify();

    nav_filter_status nav_status;
    ahrs.get_filter_status(nav_status);

    enum STORM32LINKFCSTATUSAPENUM {
      STORM32LINK_FCSTATUS_AP_AHRSHEALTHY       = 0x01, // => Q ok, ca. 15 secs
      STORM32LINK_FCSTATUS_AP_AHRSINITIALIZED   = 0x02, // => vz ok, ca. 32 secs
      STORM32LINK_FCSTATUS_AP_GPS3DFIX          = 0x04, // ca 60-XXs
      STORM32LINK_FCSTATUS_AP_NAVHORIZVEL       = 0x08, // comes very late, after GPS fix and few secs after position_ok()
      STORM32LINK_FCSTATUS_AP_ARMED             = 0x40, // tells when copter is about to take-off
      STORM32LINK_FCSTATUS_ISARDUPILOT          = 0x80, // permanently set if it's ArduPilot, so STorM32 knows about and can act accordingly
    };

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

    uint16_t _estimator_status = 0;
    if (status & STORM32LINK_FCSTATUS_AP_AHRSHEALTHY) _estimator_status |= ESTIMATOR_ATTITUDE;
    if (status & STORM32LINK_FCSTATUS_AP_AHRSINITIALIZED) _estimator_status |= ESTIMATOR_VELOCITY_VERT;

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

#else

    float q[4] = { 1.0f, 0.0f, 0.0f, 0.0f };

    mavlink_msg_autopilot_state_for_gimbal_device_send(
        _chan,
        _sysid, _compid,
        AP_HAL::micros64(),
        q,
        0, //uint32_t q_estimated_delay_us,
        0.0f, 0.0f, 0.0f,
        0,
        NAN,
        0, 0);

#endif
}


void BP_Mount_STorM32_MAVLink::send_winch_status_to_gimbal(void)
{
    if (!HAVE_PAYLOAD_SPACE(_chan, WINCH_STATUS)) {
        return;
    }

    mavlink_msg_winch_status_send(
        _chan,
        AP_HAL::micros64(),
        0,
        1,
        2,
        3,
        4,
        5,
        6);
}
