//*****************************************************
//OW RADIOLINK
// (c) olliw, www.olliw.eu, GPL3
// for mLRS
//*****************************************************
#pragma once

#include "AP_RCProtocol.h"


class AP_RCProtocol_MavlinkRadio : public AP_RCProtocol_Backend {
public:

    AP_RCProtocol_MavlinkRadio(AP_RCProtocol &_frontend);

    void update(void) override;

private:

    int16_t rssi = -1; // TODO: can't we just use the backend's fields???
    int16_t link_quality = -1;
};
