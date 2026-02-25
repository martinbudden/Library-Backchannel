#pragma once

#include "backchannel_transceiver_base.h"
#include <espnow_transceiver.h>


class BackchannelTransceiverEspnow : public BackchannelTransceiverBase {
public:
    BackchannelTransceiverEspnow(
        EspnowTransceiver& espnow_transceiver,
        const uint8_t* backchannel_mac_address
    );
public:
    virtual int send_data(const uint8_t* data, size_t len) const override;
    virtual void WAIT_FOR_DATA_RECEIVED() override;
    virtual size_t get_received_data_length() const override;
    virtual void set_received_data_length_to_zero() override;
    virtual uint32_t get_tick_count_delta_and_reset() override;
#if defined(LIBRARY_RECEIVER_USE_ESPNOW)
protected:
    EspnowTransceiver& _espnow_transceiver;
    EspnowTransceiver::received_data_t _received_data;
    EspnowTransceiver::peer_data_t _peer_data {};
#endif
};
