//*****************************************************
//OW RADIOLINK
// (c) olliw, www.olliw.eu, GPL3
// for mLRS
//*****************************************************

#include "AP_RCProtocol_config.h"

#if AP_RCPROTOCOL_MAVLINK_RADIO_ENABLED

#include "AP_RCProtocol_MAVLinkRadio.h"

// constructor
AP_RCProtocol_MAVLinkRadio::AP_RCProtocol_MAVLinkRadio(AP_RCProtocol &_frontend) :
    AP_RCProtocol_Backend(_frontend)
{}

void AP_RCProtocol_MAVLinkRadio::update_radio_rc_channels(const mavlink_radio_rc_channels_t* packet)
{
    uint8_t count = packet->count;
    if (count >= MAX_RCIN_CHANNELS) count = MAX_RCIN_CHANNELS;

    uint16_t rc_chan[MAX_RCIN_CHANNELS];
    for (uint8_t i = 0; i < count; i++) {
        // The channel values are in centered 13 bit format. Range is [-4096,4096], center is 0.
        // According to specification, the conversion to PWM is x * 5/32 + 1500.
        rc_chan[i] = ((int32_t)packet->channels[i] * 5) / 32 + 1500;
    }

    bool failsafe = (packet->flags & RADIO_RC_CHANNELS_FLAGS_FAILSAFE);

    add_input(count, rc_chan, failsafe, rssi, link_quality);
}

void AP_RCProtocol_MAVLinkRadio::update_radio_link_stats(const mavlink_radio_link_stats_t* packet)
{
    link_quality = packet->rx_LQ;
    if (packet->rx_receive_antenna == UINT8_MAX || packet->rx_rssi2 == UINT8_MAX) {
        rssi = packet->rx_rssi1;
    } else
    if (packet->rx_receive_antenna == 1) {
        rssi = packet->rx_rssi2;
    } else {
        rssi = packet->rx_rssi1;
    }
}

#endif // AP_RCPROTOCOL_MAVLINK_RADIO_ENABLED

