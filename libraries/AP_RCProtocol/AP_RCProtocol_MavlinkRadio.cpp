/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Code by OlliW42
 */

#include "AP_RCProtocol_MavlinkRadio.h"
#include <GCS_MAVLink/GCS_MAVLink.h>


AP_RCProtocol_MavlinkRadio::AP_RCProtocol_MavlinkRadio(AP_RCProtocol &_frontend) :
    AP_RCProtocol_Backend(_frontend)
{
}


void AP_RCProtocol_MavlinkRadio::update(void)
{
    if (frontend.mavlink_radio.rc_channels_updated) {
        frontend.mavlink_radio.rc_channels_updated = false;

        uint8_t count = frontend.mavlink_radio.rc_channels.count;
        if (count >= MAX_RCIN_CHANNELS) count = MAX_RCIN_CHANNELS;

        uint16_t rc_chan[MAX_RCIN_CHANNELS];
        for (uint8_t i = 0; i < count; i++) {
            rc_chan[i] = ((int32_t)frontend.mavlink_radio.rc_channels.channels[i] * 5) / 32 + 1500;
        }

        bool failsafe = (frontend.mavlink_radio.rc_channels.flags & RADIO_RC_CHANNELS_FLAGS_FAILSAFE);

        add_input(count, rc_chan, failsafe, rssi, link_quality);
    }

    if (frontend.mavlink_radio.link_stats_updated) {
        frontend.mavlink_radio.link_stats_updated = false;

        link_quality = frontend.mavlink_radio.link_stats.rx_LQ;
        if (frontend.mavlink_radio.link_stats.rx_receive_antenna == UINT8_MAX ||
            frontend.mavlink_radio.link_stats.rx_rssi2 == UINT8_MAX) {
            rssi = frontend.mavlink_radio.link_stats.rx_rssi1;
        } else
        if (frontend.mavlink_radio.link_stats.rx_receive_antenna == 1) {
            rssi = frontend.mavlink_radio.link_stats.rx_rssi2;
        } else {
            rssi = frontend.mavlink_radio.link_stats.rx_rssi1;
        }
    }
}
