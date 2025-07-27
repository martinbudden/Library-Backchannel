# Backchannel Library ![license](https://img.shields.io/badge/license-MIT-green) ![open source](https://badgen.net/badge/open/source/blue?icon=github)

This library contains a number of classes that can be the basis for a Backchannel.

## Backchannel Class Structure

```mermaid
classDiagram
    class BackchannelTransceiverBase {
        <<abstract>>
        sendData() const int *
        WAIT_FOR_DATA_RECEIVED() *
        getReceivedDataLength() const size_t *
        setReceivedDataLengthToZero() *
        getTickCountDeltaAndReset() uint32_t *
        #uint8_t _transmitDataBuffer[512]
        #uint8_t _receivedDataBuffer[256]
    }

    class BackchannelBase {
        <<abstract>>
        WAIT_FOR_DATA_RECEIVED()
        sendData() const int
        processedReceivedPacket() bool *
        sendPacket() bool *
    }
    BackchannelBase o-- BackchannelTransceiverBase : calls WAIT_FOR_DATA_RECEIVED sendData

    BackchannelTransceiverBase <|-- BackchannelTransceiverUDP

    BackchannelTransceiverBase <|-- BackchannelTransceiverESPNOW
    BackchannelTransceiverESPNOW o-- ESPNOW_Transceiver

    BackchannelBase <|-- BackchannelStabilizedVehicle
    class BackchannelStabilizedVehicle {
        _backchannelID uint32_t
        _telemetryID uint32_t
        +sendPacket(uint8_t subCommand) bool override;
        #processedReceivedPacket() bool override;
        #virtual packetRequestData() bool
        #virtual packetSetOffset() bool
        #virtual packetControl() bool
        #virtual packetSetPID() bool
    }
    BackchannelStabilizedVehicle o-- MainTask
    BackchannelStabilizedVehicle o-- AHRS
    BackchannelStabilizedVehicle o-- ReceiverBase
    BackchannelStabilizedVehicle o-- VehicleControllerBase

    class TaskBase {
        uint32_t _taskIntervalMicroSeconds
    }
    TaskBase <|-- BackchannelTask
    class BackchannelTask {
        +loop()
        -task() [[noreturn]]
    }
    BackchannelTask o-- BackchannelBase : calls processedReceivedPacket sendPacket
```
