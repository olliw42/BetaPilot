#pragma once

#define BETAPILOTVERSION "v059b"
#define DATEOFBASEBRANCH "20231109"

/*
search for //OW to find all changes

2023.11.06: v059
 upgraded to ArduPilot master 4.5.0-dev

2023.03.10: v058
 upgraded to Copter4.3.6 stable
2023.03.10: v057.3
 upgraded to Copter4.3.5-rc1
2023.02.26: v057.2
 make it compile for MatekF405-CAN
 upgraded to Copter4.3.4-rc1
2023.02.11: v057.1
 upgraded to Copter4.3.4-rc1, has get_rate_ef_targets(), does not yet have corrected 1/3 param download
2023.01.20: v056
 upgraded to Copter4.3.3 stable
2023.01.16: v055
 aux functions
 AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_EXT
 revise handling of RADIO_LINL_STATS a bit, add some comments
 do not send rc channels if RADIO_RC_CHANNELS is detected
 put mLRS/RADLIO_LINK_xxx messages under the hood of storm32.xml
 upgraded to Copter4.3.2
 replace COMPONENT_PREARM_STATUS by EVENT msg
 upgraded to Copter4.3.1
 migrate to gimbal device v2
 undo mavtype_is_on_channel()
 undo mavftp per serial port option
 upgraded to Copter4.3.1-rc1
 no mavftp per serial port option, (1U<<13) = 8192
 upgraded to Copter4.3.0
 frsky_passthrough_array support, new param SRx_FRPT added to set stream rate
 radio_rc_channels, radio_link_flow_control, radio_link_stats support added
 upgraded to Copter4.3.0-beta1


ArduCopter specific
- GCS_Mavlink.cpp:          1x RADIOLINK
- version.h:            1x

ArduPlane specific
- ArduPlane.cpp:        1x
- version.h:            1x

Libraries:
- AP_Camera.cpp:        1x
- AP_Camera.h:          1x
- AP_Mount_Backend.cpp: 3x
- AP_Mount_Backend.h:   4x
- AP_Mount_Params.cpp:  1x
- AP_Mount_Params.h:    1x
- AP_Mount.cpp:         16x
- AP_Mount.h:           7x
- AP_RCProtocol.cpp:        4x RADIOLINK
- AP_RCProtocol.h:          4x RADIOLINK
- AP_RSSI.h:                1x RADIOLINK
- GCS_Common.cpp:       2x  3x RADIOLINK
- GCS_MAVLink.h:        2x
- GCS.h:                1x  1x RADIOLINK
- MAVLink_routing.cpp:      1x RADIOLINK
- RC_Channel.cpp:       4x

Additional Files in library:
- bp_version.h
- AP_Mount/BP_Mount_STorM32_MAVLink.cpp
- AP_Mount/BP_Mount_STorM32_MAVLink.h


Effect of SYSID_MYGCS parameter value:

rc_channels_override (all)
manual_contol (copter)
these require sysid = SYSID_MYGCS

accept_packet
only true if sysid = SYSID_MYGCS, if SYSID_ENFORCE = 1 (per default it is 0, so all sysid's are accepted)

failsafe_gcs_check
reseted by heartbeat from SYSID_MYGCS

send_rc_channels(

-------

SET_POSITION_TARGET_GLOBAL_INT  Waiting for 3D fix
EKF_STATUS_REPORT

MSG_LOCATION
MSG_POSITION_TARGET_GLOBAL_INT

WPNAV_SPEED

ACCEL_Z CAPACITY TYPE

RC_CHANNELS_OVERRIDE
RC_CHANNELS
RC_CHANNELS_RAW
RADIO_STATUS
MANUAL_CONTROL

SYSTEM_TIME

STATUSTEXT

ESTIMATOR_STATUS
EXTENDED_SYS_STATE

MAV_CMD_DO_SET_ROI
MAV_CMD_DO_SET_SYSID
MAV_CMD_DO_MOUNT_CONTROL

BUTTON_CHANGE MANUAL_CONTROL

MAV_CMD_DO_SEND_BANNER
EXTENDED_SYS_STATE

FOLLOW
copter.g2.follow.handle_msg(msg);
GCS_MAVLINK_Copter::handle_command_int_packet() -> MAV_CMD_DO_FOLLOW -> copter.g2.follow.set_target_sysid((uint8_t)packet.param1);
GCS_MAVLINK_Copter::handle_command_long_packet()-> MAV_CMD_DO_FOLLOW -> copter.g2.follow.set_target_sysid((uint8_t)packet.param1);

GLOBAL_POSITION_INT
FOLLOW_TARGET

DIGICAM_CONTROL
MOUNT_CONFIGURE
MOUNT_CONTROL
MAV_CMD_SET_CAMERA_MODE, MAV_CMD_SET_CAMERA_ZOOM, MAV_CMD_SET_CAMERA_FOCUS ??
MAV_CMD_IMAGE_START_CAPTURE, MAV_CMD_IMAGE_STOP_CAPTURE, MAV_CMD_VIDEO_START_CAPTURE, MAV_CMD_VIDEO_STOP_CAPTURE

WIND_COV HIGH_LATENCY2 WIND, GPS_RAW_INT VFR_HUD GPS2_RAW OPEN_DRONE_ID_LOCATION

MAV_TYPE_  "GUID"
*/

/*
MissionPlanner
PayloadControl: sends MOUNT_CONTROL messages with target of the autopilot
AuxFunction: sends CMD_LONG with cmd MAV_CMD_DO_AUX_FUNCTION = 218
TriggerCameraHere: sends CMD_LONG with cmd DO_DIGICAM_CONTROL = 203, param5 = 1, with target of the autopilot

*/

/* aux handling
input: RC, script, mavlink
MAV_CMD_DO_AUX_FUNCTION = 218 for the Aux Buttons in MissionPlanner, MissionPlanner sends it with targets 1,1
handle_command_do_aux_function(packet)
MAV_CMD_DO_AUX_FUNCTION_SWITCH_LEVEL is defined in xml but not used

        CAMERA_TRIGGER =       9, // trigger camera servo or relay
        RETRACT_MOUNT1 =      27, // Retract Mount1
        MOUNT_LOCK =         163, // Mount yaw lock vs follow
        CAMERA_REC_VIDEO =   166, // start recording on high, stop recording on low
        CAMERA_ZOOM =        167, // camera zoom high = zoom in, middle = hold, low = zoom out
        CAMERA_MANUAL_FOCUS = 168,// camera manual focus.  high = long shot, middle = stop focus, low = close shot
        CAMERA_AUTO_FOCUS =  169, // camera auto focus

        // inputs from 200 will eventually used to replace RCMAP
        //OW: have no effect currently in RC_Channel::do_aux_function()
        MOUNT1_ROLL =        212, // mount1 roll input
        MOUNT1_PITCH =       213, // mount1 pitch input
        MOUNT1_YAW =         214, // mount1 yaw input
        MOUNT2_ROLL =        215, // mount2 roll input
        MOUNT2_PITCH =       216, // mount3 pitch input
        MOUNT2_YAW =         217, // mount4 yaw input

        // entries from 100-150  are expected to be developer options used for testing
        CAM_MODE_TOGGLE =    102, // Momentary switch to cycle camera modes


AUX_FUNC::CAMERA_TRIGGER:
-> do_aux_function_camera_trigger(ch_flag);
   -> if (ch_flag == AuxSwitchPos::HIGH) camera->take_picture();

AUX_FUNC::CAMERA_REC_VIDEO:
-> do_aux_function_record_video(ch_flag);
   -> camera->record_video(ch_flag == AuxSwitchPos::HIGH);

AUX_FUNC::CAMERA_ZOOM:
-> do_aux_function_camera_zoom(ch_flag);
   -> camera->set_zoom_step(zoom_step);

AUX_FUNC::CAMERA_MANUAL_FOCUS:
-> do_aux_function_camera_manual_focus(ch_flag);
   -> camera->set_manual_focus_step(focus_step);

AUX_FUNC::CAMERA_AUTO_FOCUS:
-> do_aux_function_camera_auto_focus(ch_flag);
   -> if (ch_flag == AuxSwitchPos::HIGH) camera->set_auto_focus();

AUX_FUNC::CAM_MODE_TOGGLE:
->  case AuxSwitchPos::HIGH: camera->cam_mode_toggle();


AUX_FUNC::RETRACT_MOUNT1:
-> case AuxSwitchPos::HIGH: mount->set_mode(0,MAV_MOUNT_MODE_RETRACT);
-> case AuxSwitchPos::LOW:  mount->set_mode_to_default(0);

AUX_FUNC::MOUNT_LOCK:
-> mount->set_yaw_lock(ch_flag == AuxSwitchPos::HIGH);


camera:

AP_Camera::take_picture()
-> trigger_pic()
   -> case CamTrigType::mount: mount->take_picture(0);
-> send to components: MAVLINK_MSG_ID_COMMAND_LONG:MAV_CMD_DO_DIGICAM_CONTROL: param5 = 1;

AP_Camera::record_video(bool start_recording)
-> if (get_trigger_type() == CamTrigType::mount) mount->record_video(0, start_recording);

AP_Camera::set_zoom_step(int8_t zoom_step)
-> if (get_trigger_type() == CamTrigType::mount) mount->set_zoom_step(0, zoom_step);

AP_Camera::set_manual_focus_step(int8_t focus_step)
-> if (get_trigger_type() == CamTrigType::mount) mount->set_manual_focus_step(0, focus_step);

AP_Camera::set_auto_focus()
-> if (get_trigger_type() == CamTrigType::mount) mount->set_auto_focus(0);

AP_Camera::cam_mode_toggle()
-> does NOTHING !!!


*/
