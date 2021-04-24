#pragma once

#define BETAPILOTVERSION "v050"


/*
search for //OW to find all changes
2021.04.18:
 auto mode
 upgraded to Copter4.1.0-beta1

2021.04.05:
 upgraded to Copter4.0.7
_PARAMS

2021.02.05:
 upgraded to Copter4.0.6
 upgraded to storm32.xml v 4. Feb. 2021, 600xx ranges
 dual mount support if enabled

2020.11.25:
 upgraded to Copter4.0.5
 upgraded to storm32.xml v 25. Nov. 2020
 upgraded to storm32_lib.h v 17. Nov. 2020

ArduCopter specific
- compassmot.cpp:       1x
- GCS_Mavlink.cpp:      (+ 2 comments, no change)
- version.h:            1x

Libraries:
- AP_Arming.cpp:        3x
- AP_Arming.h:          1x
- AP_Mount_Backend.cpp: (+ 3 comments, no changes)
- AP_Mount_Backend.h:   1x  (+1 comment, no change)
- AP_Mount.cpp:         7x  (+1 comment, no change)
- AP_Mount.h:           5x
- GCS_Common.cpp:       9x (was 11x ????)
- GCS.h:                1x
- MAVLink_routing.cpp:  1x
- MAVLink_routing.h:    1x

Additional Files in library:
- bp_version.h
- AP_Mount/BP_Mount_STorM32_MAVLink.cpp
- AP_Mount/BP_Mount_STorM32_MAVLink.h
- AP_Mount/STorM32_lib.h

Effect of SYSID_MYGCS parameter value:

rc_channels_override (all)
manual_contol (copter)
these require sysid = SYSID_MYGCS

accept_packet
only true if sysid = SYSID_MYGCS, if SYSID_ENFORCE = 1 (per default it is 0, so all sysid's are accepted)

failsafe_gcs_check
reseted by heartbeat from SYSID_MYGCS

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

SYSTEM_TIME

STATUSTEXT

ESTIMATOR_STATUS
EXTENDED_SYS_STATE

MAV_CMD_DO_SET_ROI
MAV_CMD_DO_MOUNT_CONTROL

BUTTON_CHANGE MANUAL_CONTROL

MAV_CMD_DO_SEND_BANNER


FOLLOW
copter.g2.follow.handle_msg(msg);
GCS_MAVLINK_Copter::handle_command_int_packet() -> MAV_CMD_DO_FOLLOW -> copter.g2.follow.set_target_sysid((uint8_t)packet.param1);
GCS_MAVLINK_Copter::handle_command_long_packet()-> MAV_CMD_DO_FOLLOW -> copter.g2.follow.set_target_sysid((uint8_t)packet.param1);

*/
