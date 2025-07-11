#include "BackchannelBase.h"
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
