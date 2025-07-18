#pragma once

#ifndef FORCE_VERSION_H_INCLUDE
#error version.h should never be included directly. You probably want to include AP_Common/AP_FWVersion.h
#endif

#include "ap_version.h"

#define THISFIRMWARE "ArduCopter V4.7.0-dev"

//OW
#undef THISFIRMWARE
#include "../libraries/bp_version.h"
#define THISFIRMWARE "BetaCopter V4.7.0-dev " BETAPILOTVERSION " " DATEOFBASEBRANCH
//OWEND

// the following line is parsed by the autotest scripts
#define FIRMWARE_VERSION 4,7,0,FIRMWARE_VERSION_TYPE_DEV

#define FW_MAJOR 4
#define FW_MINOR 7
#define FW_PATCH 0
#define FW_TYPE FIRMWARE_VERSION_TYPE_DEV

#include <AP_Common/AP_FWVersionDefine.h>
#include <AP_CheckFirmware/AP_CheckFirmwareDefine.h>
