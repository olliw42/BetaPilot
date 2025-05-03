#include "AP_RCProtocol_config.h"

#if AP_RCPROTOCOL_DRONECAN_ENABLED

#include <AP_CANManager/AP_CANManager.h>
#include <AP_DroneCAN/AP_DroneCAN.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include "AP_RCProtocol_DroneCAN.h"

extern const AP_HAL::HAL& hal;

#define LOG_TAG "RCInput"

AP_RCProtocol_DroneCAN::Registry AP_RCProtocol_DroneCAN::registry;
AP_RCProtocol_DroneCAN *AP_RCProtocol_DroneCAN::_singleton;

bool AP_RCProtocol_DroneCAN::subscribe_msgs(AP_DroneCAN* ap_dronecan)
{
    const auto driver_index = ap_dronecan->get_driver_index();

    return (Canard::allocate_sub_arg_callback(ap_dronecan, &handle_rcinput, driver_index) != nullptr);
}

AP_RCProtocol_DroneCAN* AP_RCProtocol_DroneCAN::get_dronecan_backend(AP_DroneCAN* ap_dronecan, uint8_t node_id)
{
    if (_singleton == nullptr) {
        return nullptr;
    }

    if (ap_dronecan == nullptr) {
        return nullptr;
    }

    for (auto &device : registry.detected_devices) {
        if (device.driver == nullptr) {
            continue;
        }
        if (device.ap_dronecan != ap_dronecan) {
            continue;
        }
        if (device.node_id != node_id ) {
            continue;
        }
        return device.driver;
    }

    // not found in registry; add it if possible.
    for (auto &device : registry.detected_devices) {
        if (device.ap_dronecan == nullptr) {
            device.ap_dronecan = ap_dronecan;
            device.node_id = node_id;
            device.driver = _singleton;
            return device.driver;
        }
    }

    return nullptr;
}

void AP_RCProtocol_DroneCAN::handle_rcinput(AP_DroneCAN *ap_dronecan, const CanardRxTransfer& transfer, const dronecan_sensors_rc_RCInput &msg)
{
    AP_RCProtocol_DroneCAN* driver = get_dronecan_backend(ap_dronecan, transfer.source_node_id);
    if (driver == nullptr) {
        return;
    }

    auto &rcin = driver->rcin;
    WITH_SEMAPHORE(rcin.sem);
    rcin.quality = msg.quality;
    rcin.status = msg.status;
    rcin.num_channels = MIN(msg.rcin.len, ARRAY_SIZE(rcin.channels));
    for (auto i=0; i<rcin.num_channels; i++) {
        rcin.channels[i] = msg.rcin.data[i];
    }

    rcin.last_sample_time_ms = AP_HAL::millis();
}

void AP_RCProtocol_DroneCAN::update()
{
    {
        WITH_SEMAPHORE(rcin.sem);
        if (rcin.last_sample_time_ms == last_receive_ms) {
            // no new data
            return;
        }
        last_receive_ms = rcin.last_sample_time_ms;

        if (rcin.bits.QUALITY_VALID) {
            frontend._rc_link_status.rssi = rcin.quality;
        } else
        if (rcin.bits.QUALITY_LQ) {
            frontend._rc_link_status.link_quality = rcin.quality;
        } else
        if (rcin.bits.QUALITY_RSSI_DBM) {
            frontend._rc_link_status.rssi_dbm = rcin.quality;
            // AP rssi: -1 for unknown, 0 for no link, 255 for maximum link
            if (rcin.quality < 50) {
                frontend._rc_link_status.rssi = 255;
            } else if ( rcin.quality > 120) {
                frontend._rc_link_status.rssi = 0;
            } else {
                frontend._rc_link_status.rssi = int16_t(roundf((120.0f - rcin.quality) * (255.0f / 70.0f) ));
            }
        } else
        if (rcin.bits.QUALITY_SNR) {
            frontend._rc_link_status.snr = (int8_t)rcin.quality - 128;
        } else {
            frontend._rc_link_status.rssi = -1;
            frontend._rc_link_status.link_quality = -1;
            frontend._rc_link_status.rssi_dbm = -1;
            frontend._rc_link_status.snr = INT8_MIN;
        }

        add_input(
            rcin.num_channels,
            rcin.channels,
            rcin.bits.FAILSAFE,
            frontend._rc_link_status.rssi,
            frontend._rc_link_status.link_quality
            );
    }
}

#endif // AP_RCPROTOCOL_DRONECAN_ENABLED
