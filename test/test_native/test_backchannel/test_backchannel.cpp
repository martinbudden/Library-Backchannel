#include "backchannel_base.h"
#include "backchannel_stabilized_vehicle.h"
#include "backchannel_transceiver_base.h"
#include "command_packet.h"
#include "sv_telemetry_data.h"

#include <ahrs.h>

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
    int send_data(const uint8_t* data, size_t len) const override { (void)data; (void)len; return 0; }
    void WAIT_FOR_DATA_RECEIVED() override {}
    size_t get_received_data_length() const override { return 0; }
    void set_received_data_length_to_zero() override {}
    uint32_t get_tick_count_delta_and_reset() override { return 0; }
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

void test_sv_telemetry_data()
{
    enum { MAX_TD_PACKET_SIZE = 250 };
    static_assert(sizeof(TD_RESERVED) <= MAX_TD_PACKET_SIZE);
    static_assert(sizeof(TD_MINIMAL) <= MAX_TD_PACKET_SIZE);
    static_assert(sizeof(TD_TASK_INTERVALS) <= MAX_TD_PACKET_SIZE);
    static_assert(sizeof(TD_TASK_INTERVALS_EXTENDED) <= MAX_TD_PACKET_SIZE);
    static_assert(sizeof(TD_AHRS) <= MAX_TD_PACKET_SIZE);
    static_assert(sizeof(TD_PID) <= MAX_TD_PACKET_SIZE);
    static_assert(sizeof(TD_PID_EXTENDED) <= MAX_TD_PACKET_SIZE);
    static_assert(sizeof(TD_FC_QUADCOPTER) <= MAX_TD_PACKET_SIZE);

    enum { MAX_TD_MSP_PACKET_SIZE = 260 };
    static_assert(sizeof(TD_MSP) <= MAX_TD_MSP_PACKET_SIZE);
    static_assert(sizeof(TD_BLACKBOX_E) <= MAX_TD_MSP_PACKET_SIZE);
    static_assert(sizeof(TD_BLACKBOX_I) <= MAX_TD_MSP_PACKET_SIZE);
    static_assert(sizeof(TD_BLACKBOX_P) <= MAX_TD_MSP_PACKET_SIZE);
    static_assert(sizeof(TD_BLACKBOX_S) <= MAX_TD_MSP_PACKET_SIZE);

    static_assert(TD_TASK_INTERVALS_EXTENDED::TIME_CHECKS_COUNT == Ahrs::TIME_CHECKS_COUNT);
}

int main([[maybe_unused]] int argc, [[maybe_unused]] char **argv)
{
    UNITY_BEGIN();

    RUN_TEST(test_backchannel);

    UNITY_END();
}
