/**
 * @file telescope_controller.h
 * @brief INDI driver for Telescope Controller AUX device
 * @author Your Name
 * @date 2024
 *
 * This driver controls a telescope controller device via serial communication.
 * Features include:
 * - 3 heaters (enable/disable)
 * - Lens cap (open/close)
 * - Flat light (enable/disable)
 * - Telemetry (temperatures and system states)
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

class TelescopeController : public INDI::DefaultDevice
{
public:
    TelescopeController();
    virtual ~TelescopeController();

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

    // Heater controls (3 heaters)
    ISwitchVectorProperty Heater1SP;
    ISwitch Heater1S[2];
    ISwitchVectorProperty Heater2SP;
    ISwitch Heater2S[2];
    ISwitchVectorProperty Heater3SP;
    ISwitch Heater3S[2];

    // Lens cap control
    ISwitchVectorProperty LensCapSP;
    ISwitch LensCapS[2];

    // Flat light control
    ISwitchVectorProperty FlatLightSP;
    ISwitch FlatLightS[2];

    // Telemetry properties
    INumberVectorProperty TelemetryNP;
    INumber TelemetryN[19]; // 4 temps + 9 heater states + 3 flat light states + 3 lens cap states

    // Status text
    ITextVectorProperty StatusTP;
    IText StatusT[1];

    // Helper functions
    bool sendCommand(uint8_t commandByte);
    bool readTelemetry();
    void parseTelemetry(const std::vector<uint8_t> &data);
    bool isConnected() const 
    { 
        if (!serialConnection) return false;
        int fd = serialConnection->getPortFD();
        return fd >= 0;
    }
    
    // Get port file descriptor for direct I/O
    int getPortFD() const { return serialConnection ? serialConnection->getPortFD() : -1; }

    // Telemetry parsing state machine
    enum TelemetryState {
        STATE_IDLE,
        STATE_READING_SYNC,
        STATE_READING_PAYLOAD
    };
    TelemetryState telemetryState = STATE_IDLE;
    std::vector<uint8_t> telemetryBuffer;
    uint8_t syncBytesRead = 0;
    static constexpr uint8_t SYNC_BYTE = 0x50;
    static constexpr size_t TELEMETRY_PAYLOAD_SIZE = 31; // 4 floats (16) + 9 heater bytes + 3 light + 3 lens cap
    static constexpr size_t TELEMETRY_PACKET_SIZE = 34; // 3 sync + 31 payload

    // Timer for periodic telemetry updates
    int updatePeriod = 100; // milliseconds (100ms = 10Hz for faster updates)
};

