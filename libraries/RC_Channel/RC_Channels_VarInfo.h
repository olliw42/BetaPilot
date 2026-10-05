#pragma once

#include "RC_Channel_config.h"

#if AP_RC_CHANNEL_ENABLED

#include "RC_Channel.h"


/*
  this header file is expected to be #included by Vehicle subclasses
  of RC_Channels after defining RC_CHANNELS_SUBCLASS and
  RC_CHANNEL_SUBCLASS - for example, Rover defines
  RC_CHANNELS_SUBCLASS to be RC_Channels_Rover in Rover/RC_Channels.cpp, and then includes this header.

  This scheme reduces code duplicate between the Vehicles, and avoids the chance of things getting out of sync.
*/

const AP_Param::GroupInfo RC_Channels::var_info[] = {
    // @Group: 1_
    // @Path: RC_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[0], "1_",  1, RC_CHANNELS_SUBCLASS, RC_CHANNEL_SUBCLASS),

    // @Group: 2_
    // @Path: RC_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[1], "2_",  2, RC_CHANNELS_SUBCLASS, RC_CHANNEL_SUBCLASS),

    // @Group: 3_
    // @Path: RC_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[2], "3_",  3, RC_CHANNELS_SUBCLASS, RC_CHANNEL_SUBCLASS),

    // @Group: 4_
    // @Path: RC_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[3], "4_",  4, RC_CHANNELS_SUBCLASS, RC_CHANNEL_SUBCLASS),

    // @Group: 5_
    // @Path: RC_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[4], "5_",  5, RC_CHANNELS_SUBCLASS, RC_CHANNEL_SUBCLASS),

    // @Group: 6_
    // @Path: RC_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[5], "6_",  6, RC_CHANNELS_SUBCLASS, RC_CHANNEL_SUBCLASS),

    // @Group: 7_
    // @Path: RC_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[6], "7_",  7, RC_CHANNELS_SUBCLASS, RC_CHANNEL_SUBCLASS),

    // @Group: 8_
    // @Path: RC_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[7], "8_",  8, RC_CHANNELS_SUBCLASS, RC_CHANNEL_SUBCLASS),

    // @Group: 9_
    // @Path: RC_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[8], "9_",  9, RC_CHANNELS_SUBCLASS, RC_CHANNEL_SUBCLASS),

    // @Group: 10_
    // @Path: RC_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[9], "10_", 10, RC_CHANNELS_SUBCLASS, RC_CHANNEL_SUBCLASS),

    // @Group: 11_
    // @Path: RC_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[10], "11_", 11, RC_CHANNELS_SUBCLASS, RC_CHANNEL_SUBCLASS),

    // @Group: 12_
    // @Path: RC_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[11], "12_", 12, RC_CHANNELS_SUBCLASS, RC_CHANNEL_SUBCLASS),

    // @Group: 13_
    // @Path: RC_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[12], "13_", 13, RC_CHANNELS_SUBCLASS, RC_CHANNEL_SUBCLASS),

    // @Group: 14_
    // @Path: RC_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[13], "14_", 14, RC_CHANNELS_SUBCLASS, RC_CHANNEL_SUBCLASS),

    // @Group: 15_
    // @Path: RC_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[14], "15_", 15, RC_CHANNELS_SUBCLASS, RC_CHANNEL_SUBCLASS),

    // @Group: 16_
    // @Path: RC_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[15], "16_", 16, RC_CHANNELS_SUBCLASS, RC_CHANNEL_SUBCLASS),

    // @Param: _OVERRIDE_TIME
    // @DisplayName: RC override timeout
    // @Description: Timeout after which RC overrides will no longer be used, and RC input will resume, 0 will disable RC overrides, -1 will never timeout, and continue using overrides until they are disabled
    // @User: Advanced
    // @Range: 0.0 120.0
    // @Units: s
    AP_GROUPINFO("_OVERRIDE_TIME", 32, RC_CHANNELS_SUBCLASS, _override_timeout, 3.0),

    // @Param: _OPTIONS
    // @DisplayName: RC options
    // @Description: RC input options
    // @User: Advanced
    // @Bitmask: 0:Ignore RC Receiver, 1:Ignore MAVLink Overrides, 2:Ignore Receiver Failsafe bit but allow other RC failsafes if setup, 3:FPort Pad, 4:Log RC input bytes, 5:Arming check throttle for 0 input, 6:Skip the arming check for neutral Roll/Pitch/Yaw sticks, 7:Allow Switch reverse, 8:Use passthrough for CRSF telemetry, 9:Suppress CRSF mode/rate message for ELRS systems,10:Enable multiple receiver support, 11:Use Link Quality for RSSI with CRSF, 12:Annotate CRSF flight mode with * on disarm, 13: Use 420kbaud for ELRS protocol, 14: Clear MAVLink overrides on any stick input
    AP_GROUPINFO("_OPTIONS", 33, RC_CHANNELS_SUBCLASS, _options, (uint32_t)RC_Channels::Option::ARMING_CHECK_THROTTLE),

    // _PROTOCOLS copied to AP_Periph/Parameters.cpp
    // @Param: _PROTOCOLS
    // @DisplayName: RC protocols enabled
    // @Description: Bitmask of enabled RC protocols. Allows narrowing the protocol detection to only specific types of RC receivers which can avoid issues with incorrect detection. Set to 1 to enable all protocols.
    // @User: Advanced
    // @Bitmask: 0:All,1:PPM,2:IBUS,3:SBUS,4:SBUS_NI,5:DSM,6:SUMD,7:SRXL,8:SRXL2,9:CRSF,10:ST24,11:FPORT,12:FPORT2,13:FastSBUS,14:DroneCAN,15:Ghost,16:MAVRadio,18:SITL UDP
    AP_GROUPINFO("_PROTOCOLS", 34, RC_CHANNELS_SUBCLASS, _protocols, 1),

    // @Param: _FS_TIMEOUT
    // @DisplayName: RC Failsafe timeout
    // @Description: RC failsafe will trigger this many seconds after loss of RC
    // @User: Standard
    // @Range: 0.1 10.0
    // @Units: s
    AP_GROUPINFO("_FS_TIMEOUT", 35, RC_CHANNELS_SUBCLASS, _fs_timeout, 1.0),

//OW
#if NUM_RC_CHANNELS > 16
    // @Param: HI_RC17_OPTION
    // @CopyFieldsFrom: RC1_OPTION
    // @DisplayName: RC17 input option
    // @Description: Function assigned to this RC channel
    // @SortValues: AlphabeticalZeroAtTop
    // @User: Standard
    AP_GROUPINFO("HI_RC17_OPTION", 36, RC_CHANNELS_SUBCLASS, obj_channels[16].option, 0),
#endif
#if NUM_RC_CHANNELS > 17
    // @Param: HI_RC18_OPTION
    // @CopyFieldsFrom: RC1_OPTION
    // @DisplayName: RC18 input option
    // @Description: Function assigned to this RC channel
    // @SortValues: AlphabeticalZeroAtTop
    // @User: Standard
    AP_GROUPINFO("HI_RC18_OPTION", 37, RC_CHANNELS_SUBCLASS, obj_channels[17].option, 0),
#endif
#if NUM_RC_CHANNELS > 18
    // @Param: HI_RC19_OPTION
    // @CopyFieldsFrom: RC1_OPTION
    // @DisplayName: RC19 input option
    // @Description: Function assigned to this RC channel
    // @SortValues: AlphabeticalZeroAtTop
    // @User: Standard
    AP_GROUPINFO("HI_RC19_OPTION", 38, RC_CHANNELS_SUBCLASS, obj_channels[18].option, 0),
#endif
#if NUM_RC_CHANNELS > 19
    // @Param: HI_RC20_OPTION
    // @CopyFieldsFrom: RC1_OPTION
    // @DisplayName: RC20 input option
    // @Description: Function assigned to this RC channel
    // @SortValues: AlphabeticalZeroAtTop
    // @User: Standard
    AP_GROUPINFO("HI_RC20_OPTION", 39, RC_CHANNELS_SUBCLASS, obj_channels[19].option, 0),
#endif
#if NUM_RC_CHANNELS > 20
    // @Param: HI_RC21_OPTION
    // @CopyFieldsFrom: RC1_OPTION
    // @DisplayName: RC21 input option
    // @Description: Function assigned to this RC channel
    // @SortValues: AlphabeticalZeroAtTop
    // @User: Standard
    AP_GROUPINFO("HI_RC21_OPTION", 40, RC_CHANNELS_SUBCLASS, obj_channels[20].option, 0),
#endif
#if NUM_RC_CHANNELS > 21
    // @Param: HI_RC22_OPTION
    // @CopyFieldsFrom: RC1_OPTION
    // @DisplayName: RC22 input option
    // @Description: Function assigned to this RC channel
    // @SortValues: AlphabeticalZeroAtTop
    // @User: Standard
    AP_GROUPINFO("HI_RC22_OPTION", 41, RC_CHANNELS_SUBCLASS, obj_channels[21].option, 0),
#endif
#if NUM_RC_CHANNELS > 22
    // @Param: HI_RC23_OPTION
    // @CopyFieldsFrom: RC1_OPTION
    // @DisplayName: RC23 input option
    // @Description: Function assigned to this RC channel
    // @SortValues: AlphabeticalZeroAtTop
    // @User: Standard
    AP_GROUPINFO("HI_RC23_OPTION", 42, RC_CHANNELS_SUBCLASS, obj_channels[22].option, 0),
#endif
#if NUM_RC_CHANNELS > 23
    // @Param: HI_RC24_OPTION
    // @CopyFieldsFrom: RC1_OPTION
    // @DisplayName: RC24 input option
    // @Description: Function assigned to this RC channel
    // @SortValues: AlphabeticalZeroAtTop
    // @User: Standard
    AP_GROUPINFO("HI_RC24_OPTION", 43, RC_CHANNELS_SUBCLASS, obj_channels[23].option, 0),
#endif
#if NUM_RC_CHANNELS > 24
    // @Param: HI_RC25_OPTION
    // @CopyFieldsFrom: RC1_OPTION
    // @DisplayName: RC25 input option
    // @Description: Function assigned to this RC channel
    // @SortValues: AlphabeticalZeroAtTop
    // @User: Standard
    AP_GROUPINFO("HI_RC25_OPTION", 44, RC_CHANNELS_SUBCLASS, obj_channels[24].option, 0),
#endif
#if NUM_RC_CHANNELS > 25
    // @Param: HI_RC26_OPTION
    // @CopyFieldsFrom: RC1_OPTION
    // @DisplayName: RC26 input option
    // @Description: Function assigned to this RC channel
    // @SortValues: AlphabeticalZeroAtTop
    // @User: Standard
    AP_GROUPINFO("HI_RC26_OPTION", 45, RC_CHANNELS_SUBCLASS, obj_channels[25].option, 0),
#endif
#if NUM_RC_CHANNELS > 26
    // @Param: HI_RC27_OPTION
    // @CopyFieldsFrom: RC1_OPTION
    // @DisplayName: RC27 input option
    // @Description: Function assigned to this RC channel
    // @SortValues: AlphabeticalZeroAtTop
    // @User: Standard
    AP_GROUPINFO("HI_RC27_OPTION", 46, RC_CHANNELS_SUBCLASS, obj_channels[26].option, 0),
#endif
#if NUM_RC_CHANNELS > 27
    // @Param: HI_RC28_OPTION
    // @CopyFieldsFrom: RC1_OPTION
    // @DisplayName: RC28 input option
    // @Description: Function assigned to this RC channel
    // @SortValues: AlphabeticalZeroAtTop
    // @User: Standard
    AP_GROUPINFO("HI_RC28_OPTION", 47, RC_CHANNELS_SUBCLASS, obj_channels[27].option, 0),
#endif
#if NUM_RC_CHANNELS > 28
    // @Param: HI_RC29_OPTION
    // @CopyFieldsFrom: RC1_OPTION
    // @DisplayName: RC29 input option
    // @Description: Function assigned to this RC channel
    // @SortValues: AlphabeticalZeroAtTop
    // @User: Standard
    AP_GROUPINFO("HI_RC29_OPTION", 48, RC_CHANNELS_SUBCLASS, obj_channels[28].option, 0),
#endif
#if NUM_RC_CHANNELS > 29
    // @Param: HI_RC30_OPTION
    // @CopyFieldsFrom: RC1_OPTION
    // @DisplayName: RC30 input option
    // @Description: Function assigned to this RC channel
    // @SortValues: AlphabeticalZeroAtTop
    // @User: Standard
    AP_GROUPINFO("HI_RC30_OPTION", 49, RC_CHANNELS_SUBCLASS, obj_channels[29].option, 0),
#endif
#if NUM_RC_CHANNELS > 30
    // @Param: HI_RC31_OPTION
    // @CopyFieldsFrom: RC1_OPTION
    // @DisplayName: RC31 input option
    // @Description: Function assigned to this RC channel
    // @SortValues: AlphabeticalZeroAtTop
    // @User: Standard
    AP_GROUPINFO("HI_RC31_OPTION", 50, RC_CHANNELS_SUBCLASS, obj_channels[30].option, 0),
#endif
#if NUM_RC_CHANNELS > 31
    // @Param: HI_RC32_OPTION
    // @CopyFieldsFrom: RC1_OPTION
    // @DisplayName: RC32 input option
    // @Description: Function assigned to this RC channel
    // @SortValues: AlphabeticalZeroAtTop
    // @User: Standard
    AP_GROUPINFO("HI_RC32_OPTION", 51, RC_CHANNELS_SUBCLASS, obj_channels[31].option, 0),
#endif
//OWEND

    AP_GROUPEND
};

#endif  // AP_RC_CHANNEL_ENABLED
