/**
 * @file roof_controller.h
 * @brief INDI driver for Roof Controller device
 * @author Your Name
 * @date 2024
 *
 * This driver controls a roof controller device via serial communication.
 * Features include:
 * - Roof open/close control
 * - Roof position monitoring
 * - Safety interlock status
 * - Weather sensor integration
 */

#pragma once

#include <defaultdevice.h>
#include <indipropertynumber.h>
#include <indipropertyswitch.h>
#include <indipropertytext.h>
#include <connectionplugins/connectionserial.h>
#include <indicom.h>
#include <indilogger.h>
#include <indibase.h>

#include <memory>
#include <string>
#include <vector>
#include <cstdint>

class RoofController : public INDI::DefaultDevice
{
public:
    RoofController();
    virtual ~RoofController();

    virtual const char *getDefaultName() override;
    virtual bool initProperties() override;
    virtual bool updateProperties() override;
    virtual bool ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n) override;
    virtual bool ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n) override;
    virtual bool ISSnoopDevice(XMLEle *root) override;

protected:
    virtual bool Connect() override;
    virtual bool Disconnect() override;
    virtual void TimerHit() override;

private:
    // Serial port
    Connection::Serial *serialConnection;

    // Connection property
    ISwitchVectorProperty ConnectionSP;
    ISwitch ConnectionS[2];

    // Arm/Disarm control (safety feature)
    ISwitchVectorProperty ArmControlSP;
    ISwitch ArmControlS[2]; // Arm, Disarm
    bool isArmed = false; // Track arm state

    // Roof control
    ISwitchVectorProperty RoofControlSP;
    ISwitch RoofControlS[3]; // Open, Close, Stop

    // Lock control
    ISwitchVectorProperty LockControlSP;
    ISwitch LockControlS[3]; // Engage, Disengage, Stop

    // Roof status
    INumberVectorProperty RoofStatusNP;
    INumber RoofStatusN[3]; // Position (%), Open status, Close status

    // Lock status
    INumberVectorProperty LockStatusNP;
    INumber LockStatusN[1]; // Lock state

    // Telemetry (voltages, current, limit switches)
    INumberVectorProperty TelemetryNP;
    INumber TelemetryN[7]; // H-bridge current, 5V voltage, 12V voltage, Limit U1, Limit U2, Limit L1, Limit L2

    // Helper functions
    bool sendCommand(uint8_t commandByte);
    bool readStatus();
    void parseStatus(const std::vector<uint8_t> &data);
    bool isConnected() const 
    { 
        if (!serialConnection) return false;
        int fd = serialConnection->getPortFD();
        return fd >= 0;
    }
    
    // Get port file descriptor for direct I/O
    int getPortFD() const { return serialConnection ? serialConnection->getPortFD() : -1; }

    // Status parsing state machine
    enum StatusState {
        STATE_IDLE,
        STATE_READING_SYNC,
        STATE_READING_PAYLOAD
    };
    StatusState statusState = STATE_IDLE;
    std::vector<uint8_t> statusBuffer;
    uint8_t syncBytesRead = 0;
    static constexpr uint8_t SYNC_BYTE = 0x50;
    static constexpr size_t STATUS_PAYLOAD_SIZE = 18; // 3 floats (12) + 4 limit switches (4) + 2 state bytes (2)
    static constexpr size_t STATUS_PACKET_SIZE = 21; // 3 sync + 18 payload

    // Command codes (from firmware)
    static constexpr uint8_t CMD_RAISE_ROOF = 0xAB;
    static constexpr uint8_t CMD_LOWER_ROOF = 0xCD;
    static constexpr uint8_t CMD_STOP_ROOF = 0xEF;
    static constexpr uint8_t CMD_ENGAGE_LOCK = 0x12;
    static constexpr uint8_t CMD_DISENGAGE_LOCK = 0x34;
    static constexpr uint8_t CMD_STOP_LOCK = 0x56;
    static constexpr uint8_t CMD_HEARTBEAT = 0xF1;

    // Timer for periodic status updates and heartbeat
    int updatePeriod = 1000; // milliseconds (1 second)
    int heartbeatPeriod = 250; // milliseconds (4 times per second for heartbeat)
};

