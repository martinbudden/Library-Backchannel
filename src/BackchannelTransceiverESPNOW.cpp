#include "BackchannelTransceiverESPNOW.h"


#if defined(LIBRARY_RECEIVER_USE_ESPNOW)

BackchannelTransceiverESPNOW::BackchannelTransceiverESPNOW(
        EspnowTransceiver& espnowTransceiver,
        const uint8_t* backchannelMacAddress
    ) :
    _espnow_transceiver(espnowTransceiver),
    _received_data(_receivedDataBuffer, sizeof(_receivedDataBuffer))
{
    static_assert(sizeof(_transmitDataBuffer) >= ESP_NOW_MAX_DATA_LEN && "transmit buffer too small");
    static_assert(sizeof(_receivedDataBuffer) >= ESP_NOW_MAX_DATA_LEN && "receive buffer too small");

    _peer_data.receivedDataPtr = &_received_data;
    _espnow_transceiver.add_secondary_peer(_received_data, backchannelMacAddress);
}

void BackchannelTransceiverESPNOW::WAIT_FOR_DATA_RECEIVED()
{
    _espnow_transceiver.WAIT_FOR_SECONDARY_DATA_RECEIVED();
}

int BackchannelTransceiverESPNOW::sendData(const uint8_t* data, size_t len) const
{
    return _espnow_transceiver.send_data_secondary(data, len);
}

size_t BackchannelTransceiverESPNOW::getReceivedDataLength() const
{
    return _received_data.len;
}

void BackchannelTransceiverESPNOW::setReceivedDataLengthToZero()
{
    _received_data.len = 0;
}

uint32_t BackchannelTransceiverESPNOW::get_tick_count_delta_and_reset()
{
    return _espnow_transceiver.get_tick_count_delta_and_reset();
}

#endif // LIBRARY_RECEIVER_USE_ESPNOW
