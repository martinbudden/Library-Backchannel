#include "backchannel_transceiver_espnow.h"


#if defined(LIBRARY_RECEIVER_USE_ESPNOW)

BackchannelTransceiverEspnow::BackchannelTransceiverEspnow(
        EspnowTransceiver& espnow_transceiver,
        const uint8_t* backchannel_mac_address
    ) :
    _espnow_transceiver(espnow_transceiver),
    _received_data(_received_data_buffer, sizeof(_received_data_buffer))
{
    static_assert(sizeof(_transmit_data_buffer) >= ESP_NOW_MAX_DATA_LEN && "transmit buffer too small");
    static_assert(sizeof(_received_data_buffer) >= ESP_NOW_MAX_DATA_LEN && "receive buffer too small");

    _peer_data.received_data_ptr = &_received_data;
    _espnow_transceiver.add_secondary_peer(_received_data, backchannel_mac_address);
}

void BackchannelTransceiverEspnow::WAIT_FOR_DATA_RECEIVED()
{
    _espnow_transceiver.WAIT_FOR_SECONDARY_DATA_RECEIVED();
}

int BackchannelTransceiverEspnow::send_data(const uint8_t* data, size_t len) const
{
    return _espnow_transceiver.send_data_secondary(data, len);
}

size_t BackchannelTransceiverEspnow::get_received_data_length() const
{
    return _received_data.len;
}

void BackchannelTransceiverEspnow::set_received_data_length_to_zero()
{
    _received_data.len = 0;
}

uint32_t BackchannelTransceiverEspnow::get_tick_count_delta_and_reset()
{
    return _espnow_transceiver.get_tick_count_delta_and_reset();
}

#endif // LIBRARY_RECEIVER_USE_ESPNOW
