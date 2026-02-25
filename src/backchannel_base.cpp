#include "backchannel_base.h"
#include "backchannel_transceiver_base.h"
#include "sv_telemetry.h"
#include "sv_telemetry_data.h"


BackchannelBase::BackchannelBase(BackchannelTransceiverBase& backchannel_transceiver, const uint8_t* backchannel_mac_address, const uint8_t* my_mac_address) :
    _backchannel_transceiver(backchannel_transceiver),
    _transmit_data_buffer_ptr(backchannel_transceiver.get_transmit_data_buffer_ptr()),
    _transmit_data_buffer_size(backchannel_transceiver.get_transmit_data_buffer_size()),
    _received_data_buffer_ptr(backchannel_transceiver.get_received_data_buffer_ptr()),
    _received_data_buffer_size(backchannel_transceiver.get_received_data_buffer_size()),
    _backchannel_id(id_from_mac_address(backchannel_mac_address)),
    _telemetry_id(id_from_mac_address(my_mac_address))
{
#if !defined(ESP_NOW_MAX_DATA_LEN)
#define ESP_NOW_MAX_DATA_LEN (250)
#endif
    // NOTE: esp_now_send runs at a high priority, so shorter packets mean less blocking of the other tasks.
    static_assert(sizeof(TD_TASK_INTERVALS_EXTENDED) <= ESP_NOW_MAX_DATA_LEN); // 12
    static_assert(sizeof(TD_TASK_INTERVALS_EXTENDED) <= ESP_NOW_MAX_DATA_LEN); // 28
    static_assert(sizeof(TD_AHRS) <= ESP_NOW_MAX_DATA_LEN); // 60
    //static_assert(sizeof(TD_RECEIVER) <= ESP_NOW_MAX_DATA_LEN); // 40
}

void BackchannelBase::WAIT_FOR_DATA_RECEIVED()
{
    _backchannel_transceiver.WAIT_FOR_DATA_RECEIVED();
}

int BackchannelBase::send_data(const uint8_t* data, size_t len) const
{
    return _backchannel_transceiver.send_data(data, len);
}

uint32_t BackchannelBase::id_from_mac_address(const uint8_t* mac_address)
{
    // use the last 4 bytes of th _mac_address as ID
    const uint8_t* pM = mac_address;
    const uint32_t ret =  (*(pM + 2U) << 24U) | (*(pM + 3U) << 16U) | (*(pM + 4U) << 8U) | *(pM + 5U); // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic,hicpp-signed-bitwise)

    return ret;
}

/*!
If data was received then interpret it as a packet and return true.
Four types of packets may be received:

1. A command packet, for example a command to switch off the motors.
2. A request to transmit telemetry. In this case format the telemetry data and send it.
3. A request to set a PID value. In this case set the PID value and then send back a TD_PIDS packet for display.
4. A request to set an IMU offset value. In this case set the offset value, but don't send back an TD_AHRS packet for display,
   since the request will have come from within
*/
bool BackchannelBase::processed_received_packet(backchannel_parameter_group_t& pg)
{
    //Serial.printf("processed_received_packet\r\n");
    const size_t received_data_length = _backchannel_transceiver.get_received_data_length();
    if (received_data_length > 0) {
        // We have a packet, so process it

        //Serial.printf("rdLen:%d\r\n", received_data_length);
        _backchannel_transceiver.set_received_data_length_to_zero();

        // NOLINTBEGIN(cppcoreguidelines-pro-type-reinterpret-cast,bugprone-branch-clone)
        const auto* const controlPacket = reinterpret_cast<const CommandPacketControl*>(_received_data_buffer_ptr);
#if defined(USE_DEBUG_PRINTF_BACKCHANNEL)
        Serial.printf("BSV processed_received_packet id:%x, type:%d, len:%d value:%d\r\n", static_cast<unsigned int>(controlPacket->id), controlPacket->type, controlPacket->len, controlPacket->value);
#endif
        if (controlPacket->id == _backchannel_id) {
            // it's our packet, so process it

            //Serial.printf("Backchannel::update id:%x, type:%d, len:%d value:%d\r\n", controlPacket->id, controlPacket->type, controlPacket->len, controlPacket->value);
            switch (controlPacket->type) {
            case CommandPacketControl::TYPE:
                packet_control(pg, *reinterpret_cast<const CommandPacketControl*>(_received_data_buffer_ptr));
                return true;
            case CommandPacketRequestData::TYPE:
                packet_request_data(pg, *reinterpret_cast<const CommandPacketRequestData*>(_received_data_buffer_ptr));
                return true;
            case CommandPacketSetPid::TYPE:
                packet_set_pid(pg, *reinterpret_cast<const CommandPacketSetPid*>(_received_data_buffer_ptr));
                return true;
            case CommandPacketSetOffset::TYPE:
                packet_set_offset(pg, *reinterpret_cast<const CommandPacketSetOffset*>(_received_data_buffer_ptr));
                return true;
            default:
                // do nothing
                break;
            } // end switch
        }
        // NOLINTEND(cppcoreguidelines-pro-type-reinterpret-cast,bugprone-branch-clone)
    }

    return false;
}

bool BackchannelBase::packet_request_data(backchannel_parameter_group_t& pg, const CommandPacketRequestData& packet)
{
#if defined(USE_DEBUG_PRINTF_BACKCHANNEL)
    Serial.printf("BSV packet_request_data packet type:%d, len:%d, request_type:%d, value_type:%d\r\n", packet.type, packet.len, packet.request_type, packet.value_type);
#endif
    _request_type = packet.request_type;
    send_packet(pg, packet.value_type);
    return true;
}

/*!
Called from within main task function.

Once _request_type has been set, this will continue to send packets until _request_type is set to NO_REQUEST
*/
bool BackchannelBase::send_packet(backchannel_parameter_group_t& pg, uint8_t sub_command)
{
#if defined(USE_DEBUG_PRINTF_BACKCHANNEL)
    if (_request_type != CommandPacketRequestData::NO_REQUEST) {
        //Serial.printf("BSV send_packet request_type:%d, sub_command:%d\r\n", _request_type, sub_command);
    }
#endif
    (void)pg;
    (void)sub_command;

    switch (_request_type) {
    case CommandPacketRequestData::NO_REQUEST: {
        return false;
    }
    case CommandPacketRequestData::REQUEST_STOP_SENDING_DATA: {
        // send a minimal packet so the client can reset its screen
        const size_t len = pack_telemetry_data_minimal(_transmit_data_buffer_ptr, _telemetry_id, _sequence_number);
        send_data(_transmit_data_buffer_ptr, len);
        // set _request_type to NO_REQUEST so no further data sent
        _request_type = CommandPacketRequestData::NO_REQUEST;
        break;
    }
#if false
    case CommandPacketRequestData::REQUEST_TASK_INTERVAL_DATA: {
        const size_t len = pack_telemetry_data_task_intervals(_transmit_data_buffer_ptr, _telemetry_id, _sequence_number,
            *_ahrs.get_task(),
            *_vehicle_controller.get_task(),
            _main_task ?  _main_task->get_tick_count_delta() : 0,
            _backchannel_transceiver.get_tick_count_delta_and_reset()
        );
        //Serial.printf("tiLen:%d\r\n", len);
        send_data(_transmit_data_buffer_ptr, len);
        break;
    }
    case CommandPacketRequestData::REQUEST_TASK_INTERVAL_EXTENDED_DATA: {
        const size_t len = pack_telemetry_data_task_intervals_extended(_transmit_data_buffer_ptr, _telemetry_id, _sequence_number,
            _ahrs,
            _vehicle_controller,
            _main_task ? _main_task->get_tick_count_delta() : 0,
            _backchannel_transceiver.get_tick_count_delta_and_reset(),
            static_cast<uint32_t>(_receiver.get_dropped_packet_count_delta())
        );
        //Serial.printf("tiLen:%d\r\n", len);
        send_data(_transmit_data_buffer_ptr, len);
        break;
    }
    case CommandPacketRequestData::REQUEST_AHRS_DATA: {
        const ahrs_data_t& ahrs_data {};
        const size_t len = pack_telemetry_data_ahrs(_transmit_data_buffer_ptr, _telemetry_id, _sequence_number, _ahrs, ahrs_data);
        //Serial.printf("ahrsLen:%d\r\n", len);
        send_data(_transmit_data_buffer_ptr, len);
        break;
    }
    case CommandPacketRequestData::REQUEST_RECEIVER_DATA: {
        const size_t len = pack_telemetry_data_receiver(_transmit_data_buffer_ptr, _telemetry_id, _sequence_number, _receiver);
        //Serial.printf("receiverLen:%d\r\n", len);
        send_data(_transmit_data_buffer_ptr, len);
        break;
    }
#endif
    default:
        return false;
    } // end switch
    return true;
}
