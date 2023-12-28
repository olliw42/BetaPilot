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

    add_input(count, rc_chan, failsafe);
}

void AP_RCProtocol_MAVLinkRadio::update_radio_link_stats(const mavlink_radio_link_stats_t* packet)
{
    // update the backend's fields

    rx_link_quality = (packet->rx_LQ != UINT8_MAX) ? packet->rx_LQ : -1;

    int32_t _rssi = -1;

    if (packet->rx_receive_antenna == UINT8_MAX || packet->rx_rssi2 == UINT8_MAX) {
        // no diversity
        if (packet->rx_rssi1 != UINT8_MAX) _rssi = packet->rx_rssi1;
    } else
    if (packet->rx_receive_antenna == 1) {
        // diversity, receiving on antenna 1
        if (packet->rx_rssi2 != UINT8_MAX) _rssi = packet->rx_rssi2; // UINT8_MAX should not happen, but play it safe
    } else {
        // diversity, receiving on antenna 0
        if (packet->rx_rssi1 != UINT8_MAX) _rssi = packet->rx_rssi1; // UINT8_MAX should not happen, but play it safe
    }

    if (_rssi == -1) { // no rssi value set
        rssi = -1;
        return;
    }

    if (packet->flags & RADIO_LINK_STATS_FLAGS_RSSI_DBM) {
        // rssi is in dBm, convert to AP rssi using the same logic as in CRSF driver
        // AP rssi: -1 for unknown, 0 for no link, 255 for maximum link
        if (_rssi < 50) {
            rssi = 255;
        } else if (_rssi > 120) {
            rssi = 0;
        } else {
            rssi = int16_t(roundf((1.0f - (_rssi - 50.0f) / 70.0f) * 255.0f));
        }
    } else {
        // _rssi is 0..254, scale it to 0..255 with rounding
        rssi = (_rssi * 255 + 127) / 254;
    }
}

#endif // AP_RCPROTOCOL_MAVLINK_RADIO_ENABLED

