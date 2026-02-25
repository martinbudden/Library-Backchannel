#include "backchannel_base.h"
#include "backchannel_transceiver_base.h"


BackchannelBase::BackchannelBase(BackchannelTransceiverBase& backchannel_transceiver) :
        _backchannel_transceiver(backchannel_transceiver),
        _transmit_data_buffer_ptr(backchannel_transceiver.get_transmit_data_buffer_ptr()),
        _transmit_data_buffer_size(backchannel_transceiver.get_transmit_data_buffer_size()),
        _received_data_buffer_ptr(backchannel_transceiver.get_received_data_buffer_ptr()),
        _received_data_buffer_size(backchannel_transceiver.get_received_data_buffer_size())
    {}

void BackchannelBase::WAIT_FOR_DATA_RECEIVED()
{
    _backchannel_transceiver.WAIT_FOR_DATA_RECEIVED();
}

int BackchannelBase::send_data(const uint8_t* data, size_t len) const
{
    return _backchannel_transceiver.send_data(data, len);
}

