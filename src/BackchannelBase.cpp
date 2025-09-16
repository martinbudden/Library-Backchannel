#include "BackchannelBase.h"
#include "BackchannelTransceiverBase.h"


BackchannelBase::BackchannelBase(BackchannelTransceiverBase& backchannelTransceiver) :
        _backchannelTransceiver(backchannelTransceiver),
        _transmitDataBufferPtr(backchannelTransceiver.getTransmitDataBufferPtr()),
        _transmitDataBufferSize(backchannelTransceiver.getTransmitDataBufferSize()),
        _receivedDataBufferPtr(backchannelTransceiver.getReceivedDataBufferPtr()),
        _receivedDataBufferSize(backchannelTransceiver.getReceivedDataBufferSize())
    {}

void BackchannelBase::WAIT_FOR_DATA_RECEIVED()
{
    _backchannelTransceiver.WAIT_FOR_DATA_RECEIVED();
}

int BackchannelBase::sendData(const uint8_t* data, size_t len) const
{
    return _backchannelTransceiver.sendData(data, len);
}

