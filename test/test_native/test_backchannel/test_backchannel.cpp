#include "BackchannelBase.h"
#include "BackchannelStabilizedVehicle.h"
#include "BackchannelTransceiverBase.h"
#include "CommandPacket.h"

#include <unity.h>

#if !defined(AHRS_TASK_INTERVAL_MICROSECONDS)
enum { AHRS_TASK_INTERVAL_MICROSECONDS = 5000 };
#endif


void setUp()
{
}

void tearDown()
{
}

class BackchannelTransceiverNull : public BackchannelTransceiverBase {
public:
    virtual ~BackchannelTransceiverNull() = default;
    // BackchannelTransceiverNull is not copyable or moveable
    BackchannelTransceiverNull(const BackchannelTransceiverNull&) = delete;
    BackchannelTransceiverNull& operator=(const BackchannelTransceiverNull&) = delete;
    BackchannelTransceiverNull(BackchannelTransceiverNull&&) = delete;
    BackchannelTransceiverNull& operator=(BackchannelTransceiverNull&&) = delete;

    int sendData(const uint8_t* data, size_t len) const override { (void)data; (void)len; return 0; }
    void WAIT_FOR_DATA_RECEIVED() override {}
    size_t getReceivedDataLength() const override { return 0; }
    void setReceivedDataLengthToZero() override {}
    uint32_t getTickCountDeltaAndReset() override { return 0; }
};

void test_backchannel()
{
    enum { MAX_COMMAND_PACKET_SIZE = 250 };
    static_assert(sizeof(CommandPacketReserved) <= MAX_COMMAND_PACKET_SIZE);
    static_assert(sizeof(CommandPacketControl) <= MAX_COMMAND_PACKET_SIZE);
    static_assert(sizeof(CommandPacketRequestData) <= MAX_COMMAND_PACKET_SIZE);
    static_assert(sizeof(CommandPacketSetPID) <= MAX_COMMAND_PACKET_SIZE);
    static_assert(sizeof(CommandPacketSetOffset) <= MAX_COMMAND_PACKET_SIZE);
    static_assert(sizeof(CommandPacketSetFilter) <= MAX_COMMAND_PACKET_SIZE);

    enum { MAX_COMMAND_PACKET_MSP_SIZE = 260 };
    static_assert(sizeof(CommandPacketMSP) <= MAX_COMMAND_PACKET_MSP_SIZE);
}

int main([[maybe_unused]] int argc, [[maybe_unused]] char **argv)
{
    UNITY_BEGIN();

    RUN_TEST(test_backchannel);

    UNITY_END();
}
