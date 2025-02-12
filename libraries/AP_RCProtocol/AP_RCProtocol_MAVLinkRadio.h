
#pragma once

#include "AP_RCProtocol_config.h"

#if AP_RCPROTOCOL_MAVLINK_RADIO_ENABLED

#include "AP_RCProtocol.h"


class AP_RCProtocol_MAVLinkRadio : public AP_RCProtocol_Backend {
public:

    using AP_RCProtocol_Backend::AP_RCProtocol_Backend;

    // update from mavlink messages
    void update_radio_rc_channels(const mavlink_radio_rc_channels_t* packet) override;
//OW RADIOLINK
    void update_mlrs_radio_link_stats(const mavlink_mlrs_radio_link_stats_t* packet) override;
    void update_mlrs_radio_link_info(const mavlink_mlrs_radio_link_information_t* packet) override;
//OWEND
};

#endif // AP_RCPROTOCOL_MAVLINK_RADIO_ENABLED

