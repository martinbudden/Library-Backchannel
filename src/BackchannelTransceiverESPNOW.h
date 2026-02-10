#pragma once

#include "BackchannelTransceiverBase.h"
#include <EspnowTransceiver.h>


class BackchannelTransceiverESPNOW : public BackchannelTransceiverBase {
public:
    BackchannelTransceiverESPNOW(
        EspnowTransceiver& espnowTransceiver,
        const uint8_t* backchannelMacAddress
    );
public:
    virtual int sendData(const uint8_t* data, size_t len) const override;
    virtual void WAIT_FOR_DATA_RECEIVED() override;
    virtual size_t getReceivedDataLength() const override;
    virtual void setReceivedDataLengthToZero() override;
    virtual uint32_t getTickCountDeltaAndReset() override;
#if defined(LIBRARY_RECEIVER_USE_ESPNOW)
protected:
    EspnowTransceiver& _espnowTransceiver;
    EspnowTransceiver::received_data_t _received_data;
    EspnowTransceiver::peer_data_t _peer_data {};
#endif
};
