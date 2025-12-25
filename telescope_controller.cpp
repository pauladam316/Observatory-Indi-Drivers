/**
 * @file telescope_controller.cpp
 * @brief INDI driver implementation for Telescope Controller AUX device
 */

#include "telescope_controller.h"

#include <cstring>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <vector>

// We declare an auto pointer to TelescopeController.
std::unique_ptr<TelescopeController> telescopeController(new TelescopeController());

TelescopeController::TelescopeController()
{
    setVersion(1, 0);
    serialConnection = nullptr;
}

TelescopeController::~TelescopeController()
{
    delete serialConnection;
}

const char *TelescopeController::getDefaultName()
{
    return "Telescope Controller";
}

bool TelescopeController::initProperties()
{
    // Must init parent properties first
    INDI::DefaultDevice::initProperties();

    // Serial port configuration
    serialConnection = new Connection::Serial(this);
    serialConnection->setDefaultPort("/dev/tty_telescope_controller");
    serialConnection->setDefaultBaudRate(Connection::Serial::B_57600);
    serialConnection->registerHandshake([&]() { return true; });
    registerConnection(serialConnection);

    // Connection property
    IUFillSwitch(&ConnectionS[0], "CONNECT", "Connect", ISS_OFF);
    IUFillSwitch(&ConnectionS[1], "DISCONNECT", "Disconnect", ISS_OFF);
    IUFillSwitchVector(&ConnectionSP, ConnectionS, 2, getDeviceName(), "CONNECTION", "Connection", MAIN_CONTROL_TAB, IP_RW, ISR_1OFMANY, 0, IPS_IDLE);

    // Heater 1 control
    IUFillSwitch(&Heater1S[0], "HEATER1_ON", "On", ISS_OFF);
    IUFillSwitch(&Heater1S[1], "HEATER1_OFF", "Off", ISS_OFF);
    IUFillSwitchVector(&Heater1SP, Heater1S, 2, getDeviceName(), "HEATER1", "Heater 1", MAIN_CONTROL_TAB, IP_RW, ISR_1OFMANY, 0, IPS_IDLE);

    // Heater 2 control
    IUFillSwitch(&Heater2S[0], "HEATER2_ON", "On", ISS_OFF);
    IUFillSwitch(&Heater2S[1], "HEATER2_OFF", "Off", ISS_OFF);
    IUFillSwitchVector(&Heater2SP, Heater2S, 2, getDeviceName(), "HEATER2", "Heater 2", MAIN_CONTROL_TAB, IP_RW, ISR_1OFMANY, 0, IPS_IDLE);

    // Heater 3 control
    IUFillSwitch(&Heater3S[0], "HEATER3_ON", "On", ISS_OFF);
    IUFillSwitch(&Heater3S[1], "HEATER3_OFF", "Off", ISS_OFF);
    IUFillSwitchVector(&Heater3SP, Heater3S, 2, getDeviceName(), "HEATER3", "Heater 3", MAIN_CONTROL_TAB, IP_RW, ISR_1OFMANY, 0, IPS_IDLE);

    // Lens cap control
    IUFillSwitch(&LensCapS[0], "LENS_CAP_OPEN", "Open", ISS_OFF);
    IUFillSwitch(&LensCapS[1], "LENS_CAP_CLOSE", "Close", ISS_OFF);
    IUFillSwitchVector(&LensCapSP, LensCapS, 2, getDeviceName(), "LENS_CAP", "Lens Cap", MAIN_CONTROL_TAB, IP_RW, ISR_1OFMANY, 0, IPS_IDLE);

    // Flat light control
    IUFillSwitch(&FlatLightS[0], "FLAT_LIGHT_ON", "On", ISS_OFF);
    IUFillSwitch(&FlatLightS[1], "FLAT_LIGHT_OFF", "Off", ISS_OFF);
    IUFillSwitchVector(&FlatLightSP, FlatLightS, 2, getDeviceName(), "FLAT_LIGHT", "Flat Light", MAIN_CONTROL_TAB, IP_RW, ISR_1OFMANY, 0, IPS_IDLE);

    // Telemetry properties
    // Temperatures (4)
    IUFillNumber(&TelemetryN[0], "AMBIENT_TEMP", "Ambient Temp (°C)", "%.2f", -50.0, 100.0, 0.0, 0.0);
    IUFillNumber(&TelemetryN[1], "HEATER1_TEMP", "Heater 1 Temp (°C)", "%.2f", -50.0, 100.0, 0.0, 0.0);
    IUFillNumber(&TelemetryN[2], "HEATER2_TEMP", "Heater 2 Temp (°C)", "%.2f", -50.0, 100.0, 0.0, 0.0);
    IUFillNumber(&TelemetryN[3], "HEATER3_TEMP", "Heater 3 Temp (°C)", "%.2f", -50.0, 100.0, 0.0, 0.0);
    
    // Heater 1 states (3)
    IUFillNumber(&TelemetryN[4], "HEATER1_DRIVER_STATE", "Heater 1 Driver State", "%.0f", 0.0, 255.0, 1.0, 0.0);
    IUFillNumber(&TelemetryN[5], "HEATER1_MANUAL_STATE", "Heater 1 Manual State", "%.0f", 0.0, 255.0, 1.0, 0.0);
    IUFillNumber(&TelemetryN[6], "HEATER1_REAL_STATE", "Heater 1 Real State", "%.0f", 0.0, 255.0, 1.0, 0.0);
    
    // Heater 2 states (3)
    IUFillNumber(&TelemetryN[7], "HEATER2_DRIVER_STATE", "Heater 2 Driver State", "%.0f", 0.0, 255.0, 1.0, 0.0);
    IUFillNumber(&TelemetryN[8], "HEATER2_MANUAL_STATE", "Heater 2 Manual State", "%.0f", 0.0, 255.0, 1.0, 0.0);
    IUFillNumber(&TelemetryN[9], "HEATER2_REAL_STATE", "Heater 2 Real State", "%.0f", 0.0, 255.0, 1.0, 0.0);
    
    // Heater 3 states (3)
    IUFillNumber(&TelemetryN[10], "HEATER3_DRIVER_STATE", "Heater 3 Driver State", "%.0f", 0.0, 255.0, 1.0, 0.0);
    IUFillNumber(&TelemetryN[11], "HEATER3_MANUAL_STATE", "Heater 3 Manual State", "%.0f", 0.0, 255.0, 1.0, 0.0);
    IUFillNumber(&TelemetryN[12], "HEATER3_REAL_STATE", "Heater 3 Real State", "%.0f", 0.0, 255.0, 1.0, 0.0);
    
    // Flat light states (3)
    IUFillNumber(&TelemetryN[13], "FLAT_LIGHT_DRIVER_STATE", "Flat Light Driver State", "%.0f", 0.0, 255.0, 1.0, 0.0);
    IUFillNumber(&TelemetryN[14], "FLAT_LIGHT_MANUAL_STATE", "Flat Light Manual State", "%.0f", 0.0, 255.0, 1.0, 0.0);
    IUFillNumber(&TelemetryN[15], "FLAT_LIGHT_REAL_STATE", "Flat Light Real State", "%.0f", 0.0, 255.0, 1.0, 0.0);
    
    // Lens cap states (3)
    IUFillNumber(&TelemetryN[16], "LENS_CAP_DRIVER_STATE", "Lens Cap Driver State", "%.0f", 0.0, 255.0, 1.0, 0.0);
    IUFillNumber(&TelemetryN[17], "LENS_CAP_MANUAL_STATE", "Lens Cap Manual State", "%.0f", 0.0, 255.0, 1.0, 0.0);
    IUFillNumber(&TelemetryN[18], "LENS_CAP_REAL_STATE", "Lens Cap Real State", "%.0f", 0.0, 255.0, 1.0, 0.0);
    
    IUFillNumberVector(&TelemetryNP, TelemetryN, 19, getDeviceName(), "TELEMETRY", "Telemetry", MAIN_CONTROL_TAB, IP_RO, 0, IPS_IDLE);

    // Status text
    IUFillText(&StatusT[0], "STATUS", "Status", "Disconnected");
    IUFillTextVector(&StatusTP, StatusT, 1, getDeviceName(), "STATUS", "Status", MAIN_CONTROL_TAB, IP_RO, 60, IPS_IDLE);

    return true;
}

bool TelescopeController::updateProperties()
{
    INDI::DefaultDevice::updateProperties();

    if (isConnected())
    {
        defineProperty(&Heater1SP);
        defineProperty(&Heater2SP);
        defineProperty(&Heater3SP);
        defineProperty(&LensCapSP);
        defineProperty(&FlatLightSP);
        defineProperty(&TelemetryNP);
        defineProperty(&StatusTP);

        // Start timer for periodic telemetry updates
        // Use shorter period to read more frequently
        SetTimer(100); // 100ms = 10 times per second
    }
    else
    {
        deleteProperty(Heater1SP.name);
        deleteProperty(Heater2SP.name);
        deleteProperty(Heater3SP.name);
        deleteProperty(LensCapSP.name);
        deleteProperty(FlatLightSP.name);
        deleteProperty(TelemetryNP.name);
        deleteProperty(StatusTP.name);
    }

    return true;
}

bool TelescopeController::Connect()
{
    if (DefaultDevice::Connect())
    {
        IUSaveText(&StatusT[0], "Connected");
        IDSetText(&StatusTP, nullptr);
        
        // Initialize telemetry parsing state
        telemetryState = STATE_IDLE;
        telemetryBuffer.clear();
        syncBytesRead = 0;
        
        // Read initial telemetry
        readTelemetry();
        
        return true;
    }
    
    IUSaveText(&StatusT[0], "Connection failed");
    IDSetText(&StatusTP, nullptr);
    return false;
}

bool TelescopeController::Disconnect()
{
    DefaultDevice::Disconnect();
    IUSaveText(&StatusT[0], "Disconnected");
    IDSetText(&StatusTP, nullptr);
    
    // Reset telemetry parsing state
    telemetryState = STATE_IDLE;
    telemetryBuffer.clear();
    syncBytesRead = 0;
    
    return true;
}

bool TelescopeController::ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n)
{
    if (dev != nullptr && strcmp(dev, getDeviceName()) == 0)
    {
        // Heater 1
        if (!strcmp(name, Heater1SP.name))
        {
            IUUpdateSwitch(&Heater1SP, states, names, n);
            int index = IUFindOnSwitchIndex(&Heater1SP);
            
            if (index == 0) // On
            {
                if (sendCommand(0x01)) // HEATER_1_ENABLE
                {
                    Heater1SP.s = IPS_OK;
                }
                else
                {
                    Heater1SP.s = IPS_ALERT;
                }
            }
            else // Off
            {
                if (sendCommand(0x02)) // HEATER_1_DISABLE
                {
                    Heater1SP.s = IPS_OK;
                }
                else
                {
                    Heater1SP.s = IPS_ALERT;
                }
            }
            
            IDSetSwitch(&Heater1SP, nullptr);
            return true;
        }

        // Heater 2
        if (!strcmp(name, Heater2SP.name))
        {
            IUUpdateSwitch(&Heater2SP, states, names, n);
            int index = IUFindOnSwitchIndex(&Heater2SP);
            
            if (index == 0) // On
            {
                if (sendCommand(0x03)) // HEATER_2_ENABLE
                {
                    Heater2SP.s = IPS_OK;
                }
                else
                {
                    Heater2SP.s = IPS_ALERT;
                }
            }
            else // Off
            {
                if (sendCommand(0x04)) // HEATER_2_DISABLE
                {
                    Heater2SP.s = IPS_OK;
                }
                else
                {
                    Heater2SP.s = IPS_ALERT;
                }
            }
            
            IDSetSwitch(&Heater2SP, nullptr);
            return true;
        }

        // Heater 3
        if (!strcmp(name, Heater3SP.name))
        {
            IUUpdateSwitch(&Heater3SP, states, names, n);
            int index = IUFindOnSwitchIndex(&Heater3SP);
            
            if (index == 0) // On
            {
                if (sendCommand(0x05)) // HEATER_3_ENABLE
                {
                    Heater3SP.s = IPS_OK;
                }
                else
                {
                    Heater3SP.s = IPS_ALERT;
                }
            }
            else // Off
            {
                if (sendCommand(0x06)) // HEATER_3_DISABLE
                {
                    Heater3SP.s = IPS_OK;
                }
                else
                {
                    Heater3SP.s = IPS_ALERT;
                }
            }
            
            IDSetSwitch(&Heater3SP, nullptr);
            return true;
        }

        // Lens cap
        if (!strcmp(name, LensCapSP.name))
        {
            IUUpdateSwitch(&LensCapSP, states, names, n);
            int index = IUFindOnSwitchIndex(&LensCapSP);
            
            if (index == 0) // Open
            {
                if (sendCommand(0x07)) // LENS_CAP_OPEN
                {
                    LensCapSP.s = IPS_OK;
                }
                else
                {
                    LensCapSP.s = IPS_ALERT;
                }
            }
            else // Close
            {
                if (sendCommand(0x08)) // LENS_CAP_CLOSE
                {
                    LensCapSP.s = IPS_OK;
                }
                else
                {
                    LensCapSP.s = IPS_ALERT;
                }
            }
            
            IDSetSwitch(&LensCapSP, nullptr);
            return true;
        }

        // Flat light
        if (!strcmp(name, FlatLightSP.name))
        {
            IUUpdateSwitch(&FlatLightSP, states, names, n);
            int index = IUFindOnSwitchIndex(&FlatLightSP);
            
            if (index == 0) // On
            {
                if (sendCommand(0x09)) // LIGHT_ON
                {
                    FlatLightSP.s = IPS_OK;
                }
                else
                {
                    FlatLightSP.s = IPS_ALERT;
                }
            }
            else // Off
            {
                if (sendCommand(0x0A)) // LIGHT_OFF
                {
                    FlatLightSP.s = IPS_OK;
                }
                else
                {
                    FlatLightSP.s = IPS_ALERT;
                }
            }
            
            IDSetSwitch(&FlatLightSP, nullptr);
            return true;
        }
    }

    return INDI::DefaultDevice::ISNewSwitch(dev, name, states, names, n);
}

bool TelescopeController::ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n)
{
    if (dev != nullptr && strcmp(dev, getDeviceName()) == 0)
    {
        // Handle number properties if needed
    }

    return INDI::DefaultDevice::ISNewNumber(dev, name, values, names, n);
}

bool TelescopeController::ISSnoopDevice(XMLEle *root)
{
    return INDI::DefaultDevice::ISSnoopDevice(root);
}

void TelescopeController::TimerHit()
{
    if (!isConnected())
    {
        SetTimer(updatePeriod);
        return;
    }

    // Read telemetry periodically
    readTelemetry();

    SetTimer(updatePeriod);
}

bool TelescopeController::sendCommand(uint8_t commandByte)
{
    if (!isConnected())
    {
        return false;
    }

    // Protocol: 3 sync bytes (0x50) + 1 command byte
    uint8_t cmd[4] = {SYNC_BYTE, SYNC_BYTE, SYNC_BYTE, commandByte};
    int nbytes_written = 0;
    
    DEBUGF(INDI::Logger::DBG_DEBUG, "Sending command: 0x%02X", commandByte);
    
    int fd = getPortFD();
    if (fd < 0)
    {
        DEBUG(INDI::Logger::DBG_ERROR, "Invalid port file descriptor");
        return false;
    }
    
    int rc = tty_write(fd, (const char *)cmd, 4, &nbytes_written);
    if (rc != TTY_OK || nbytes_written != 4)
    {
        DEBUGF(INDI::Logger::DBG_ERROR, "Failed to write command to serial port: rc=%d, bytes=%d", rc, nbytes_written);
        return false;
    }

    return true;
}

bool TelescopeController::readTelemetry()
{
    if (!isConnected())
    {
        return false;
    }

    int fd = getPortFD();
    if (fd < 0)
    {
        return false;
    }
    
    // Read all available data in a loop to get the latest packet
    char buffer[256];
    int nbytes_read = 0;
    std::vector<uint8_t> latestCompletePacket;
    bool foundCompletePacket = false;
    
    // Keep reading until no more data is available
    while (true)
    {
        int rc = tty_read(fd, buffer, sizeof(buffer), 0, &nbytes_read);
        if (rc != TTY_OK || nbytes_read <= 0)
        {
            break; // No more data available
        }
        
        DEBUGF(INDI::Logger::DBG_DEBUG, "Read %d bytes from serial port", nbytes_read);

        // Process each byte through the state machine
        for (int i = 0; i < nbytes_read; i++)
        {
            uint8_t byte = static_cast<uint8_t>(buffer[i]);
            
            switch (telemetryState)
            {
                case STATE_IDLE:
                    if (byte == SYNC_BYTE)
                    {
                        // New sync sequence - discard any previous incomplete packet
                        telemetryBuffer.clear();
                        telemetryBuffer.push_back(byte);
                        syncBytesRead = 1;
                        telemetryState = STATE_READING_SYNC;
                    }
                    break;
                    
                case STATE_READING_SYNC:
                    if (byte == SYNC_BYTE)
                    {
                        telemetryBuffer.push_back(byte);
                        syncBytesRead++;
                        if (syncBytesRead == 3)
                        {
                            telemetryState = STATE_READING_PAYLOAD;
                        }
                    }
                    else
                    {
                        // Sync sequence broken, reset
                        telemetryState = STATE_IDLE;
                        telemetryBuffer.clear();
                        syncBytesRead = 0;
                        // Check if this byte is a new sync byte
                        if (byte == SYNC_BYTE)
                        {
                            telemetryBuffer.push_back(byte);
                            syncBytesRead = 1;
                            telemetryState = STATE_READING_SYNC;
                        }
                    }
                    break;
                    
                case STATE_READING_PAYLOAD:
                    telemetryBuffer.push_back(byte);
                    if (telemetryBuffer.size() == TELEMETRY_PACKET_SIZE)
                    {
                        // Complete packet received - save it (will be overwritten by newer packets)
                        latestCompletePacket = telemetryBuffer;
                        foundCompletePacket = true;
                        DEBUGF(INDI::Logger::DBG_DEBUG, "Received complete telemetry packet (%zu bytes)", telemetryBuffer.size());
                        // Reset to look for next packet
                        telemetryState = STATE_IDLE;
                        telemetryBuffer.clear();
                        syncBytesRead = 0;
                    }
                    else if (telemetryBuffer.size() > TELEMETRY_PACKET_SIZE)
                    {
                        // Packet too large, reset
                        telemetryState = STATE_IDLE;
                        telemetryBuffer.clear();
                        syncBytesRead = 0;
                    }
                    break;
            }
        }
    }
    
    // Process only the latest complete packet
    if (foundCompletePacket)
    {
        parseTelemetry(latestCompletePacket);
        return true;
    }

    return false;
}

void TelescopeController::parseTelemetry(const std::vector<uint8_t> &data)
{
    if (data.size() != TELEMETRY_PACKET_SIZE)
    {
        DEBUGF(INDI::Logger::DBG_WARNING, "Invalid telemetry packet size: %zu (expected %zu)", data.size(), TELEMETRY_PACKET_SIZE);
        return; // Invalid packet size
    }

    // Verify sync bytes
    if (data[0] != SYNC_BYTE || data[1] != SYNC_BYTE || data[2] != SYNC_BYTE)
    {
        DEBUGF(INDI::Logger::DBG_WARNING, "Invalid sync bytes in telemetry packet: %02X %02X %02X", data[0], data[1], data[2]);
        return; // Invalid sync bytes
    }

    size_t offset = 3; // Skip sync bytes

    // Parse 4 temperature readings (floats, 4 bytes each)
    // Order: temp_1 (heater 1), temp_2 (heater 2), temp_3 (heater 3), temp_reference (ambient)
    float temp1, temp2, temp3, ambient;
    
    memcpy(&temp1, &data[offset], sizeof(float));
    offset += sizeof(float);
    memcpy(&temp2, &data[offset], sizeof(float));
    offset += sizeof(float);
    memcpy(&temp3, &data[offset], sizeof(float));
    offset += sizeof(float);
    memcpy(&ambient, &data[offset], sizeof(float));
    offset += sizeof(float);

    // Parse heater states (3 bytes each: driver_state, manual_state, real_state)
    uint8_t heater1_driver = data[offset++];
    uint8_t heater1_manual = data[offset++];
    uint8_t heater1_real = data[offset++];
    
    uint8_t heater2_driver = data[offset++];
    uint8_t heater2_manual = data[offset++];
    uint8_t heater2_real = data[offset++];
    
    uint8_t heater3_driver = data[offset++];
    uint8_t heater3_manual = data[offset++];
    uint8_t heater3_real = data[offset++];

    // Parse flat light states
    uint8_t light_driver = data[offset++];
    uint8_t light_manual = data[offset++];
    uint8_t light_real = data[offset++];

    // Parse lens cap states
    uint8_t lenscap_driver = data[offset++];
    uint8_t lenscap_manual = data[offset++];
    uint8_t lenscap_real = data[offset++];

    // Update telemetry values
    // Temperatures (4)
    TelemetryN[0].value = ambient;  // Ambient temp
    TelemetryN[1].value = temp1;    // Heater 1 temp
    TelemetryN[2].value = temp2;    // Heater 2 temp
    TelemetryN[3].value = temp3;    // Heater 3 temp
    
    // Heater 1 states (3)
    TelemetryN[4].value = heater1_driver;
    TelemetryN[5].value = heater1_manual;
    TelemetryN[6].value = heater1_real;
    
    // Heater 2 states (3)
    TelemetryN[7].value = heater2_driver;
    TelemetryN[8].value = heater2_manual;
    TelemetryN[9].value = heater2_real;
    
    // Heater 3 states (3)
    TelemetryN[10].value = heater3_driver;
    TelemetryN[11].value = heater3_manual;
    TelemetryN[12].value = heater3_real;
    
    // Flat light states (3)
    TelemetryN[13].value = light_driver;
    TelemetryN[14].value = light_manual;
    TelemetryN[15].value = light_real;
    
    // Lens cap states (3)
    TelemetryN[16].value = lenscap_driver;
    TelemetryN[17].value = lenscap_manual;
    TelemetryN[18].value = lenscap_real;

    // Debug output
    DEBUGF(INDI::Logger::DBG_DEBUG, 
           "Telemetry: Ambient=%.2f°C, H1=%.2f°C, H2=%.2f°C, H3=%.2f°C",
           ambient, temp1, temp2, temp3);
    DEBUGF(INDI::Logger::DBG_DEBUG,
           "States: H1(d=%d,m=%d,r=%d) H2(d=%d,m=%d,r=%d) H3(d=%d,m=%d,r=%d) Light(d=%d,m=%d,r=%d) Cap(d=%d,m=%d,r=%d)",
           heater1_driver, heater1_manual, heater1_real,
           heater2_driver, heater2_manual, heater2_real,
           heater3_driver, heater3_manual, heater3_real,
           light_driver, light_manual, light_real,
           lenscap_driver, lenscap_manual, lenscap_real);

    TelemetryNP.s = IPS_OK;
    IDSetNumber(&TelemetryNP, nullptr);

    // Update switch states based on real_state
    // Heater 1
    if (heater1_real != 0)
    {
        Heater1S[0].s = ISS_ON;
        Heater1S[1].s = ISS_OFF;
    }
    else
    {
        Heater1S[0].s = ISS_OFF;
        Heater1S[1].s = ISS_ON;
    }
    IDSetSwitch(&Heater1SP, nullptr);

    // Heater 2
    if (heater2_real != 0)
    {
        Heater2S[0].s = ISS_ON;
        Heater2S[1].s = ISS_OFF;
    }
    else
    {
        Heater2S[0].s = ISS_OFF;
        Heater2S[1].s = ISS_ON;
    }
    IDSetSwitch(&Heater2SP, nullptr);

    // Heater 3
    if (heater3_real != 0)
    {
        Heater3S[0].s = ISS_ON;
        Heater3S[1].s = ISS_OFF;
    }
    else
    {
        Heater3S[0].s = ISS_OFF;
        Heater3S[1].s = ISS_ON;
    }
    IDSetSwitch(&Heater3SP, nullptr);

    // Lens cap
    if (lenscap_real != 0)
    {
        LensCapS[0].s = ISS_ON;
        LensCapS[1].s = ISS_OFF;
    }
    else
    {
        LensCapS[0].s = ISS_OFF;
        LensCapS[1].s = ISS_ON;
    }
    IDSetSwitch(&LensCapSP, nullptr);

    // Flat light
    if (light_real != 0)
    {
        FlatLightS[0].s = ISS_ON;
        FlatLightS[1].s = ISS_OFF;
    }
    else
    {
        FlatLightS[0].s = ISS_OFF;
        FlatLightS[1].s = ISS_ON;
    }
    IDSetSwitch(&FlatLightSP, nullptr);
}

