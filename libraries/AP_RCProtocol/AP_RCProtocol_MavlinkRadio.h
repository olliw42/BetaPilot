//*****************************************************
//OW RADIOLINK
// (c) olliw, www.olliw.eu, GPL3
// for mLRS
//*****************************************************
#pragma once

#include "AP_RCProtocol_config.h"

#if AP_RCPROTOCOL_MAVLINK_RADIO_ENABLED

#include "AP_RCProtocol.h"


class AP_RCProtocol_MAVLinkRadio : public AP_RCProtocol_Backend {
public:

    AP_RCProtocol_MAVLinkRadio(AP_RCProtocol &_frontend);

    // update from mavlink messages
    void update_radio_rc_channels(const mavlink_radio_rc_channels_t* packet) override;
    void update_radio_link_stats(const mavlink_radio_link_stats_t* packet) override;
};

#endif // AP_RCPROTOCOL_MAVLINK_RADIO_ENABLED

