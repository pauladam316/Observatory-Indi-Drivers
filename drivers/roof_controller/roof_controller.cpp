/**
 * @file roof_controller.cpp
 * @brief INDI driver implementation for Roof Controller device
 */

#include "roof_controller.h"

#include <cstring>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <vector>

// We declare an auto pointer to RoofController.
std::unique_ptr<RoofController> roofController(new RoofController());

RoofController::RoofController()
{
    setVersion(1, 0);
    serialConnection = nullptr;
    isArmed = false;
}

RoofController::~RoofController()
{
    delete serialConnection;
}

const char *RoofController::getDefaultName()
{
    return "Roof Controller";
}

bool RoofController::initProperties()
{
    // Must init parent properties first
    INDI::DefaultDevice::initProperties();

    // Serial port configuration
    serialConnection = new Connection::Serial(this);
    serialConnection->setDefaultPort("/dev/tty_roof_controller");
    serialConnection->setDefaultBaudRate(Connection::Serial::B_57600);
    serialConnection->registerHandshake([&]() { return true; });
    registerConnection(serialConnection);

    // Connection property
    IUFillSwitch(&ConnectionS[0], "CONNECT", "Connect", ISS_OFF);
    IUFillSwitch(&ConnectionS[1], "DISCONNECT", "Disconnect", ISS_OFF);
    IUFillSwitchVector(&ConnectionSP, ConnectionS, 2, getDeviceName(), "CONNECTION", "Connection", MAIN_CONTROL_TAB, IP_RW, ISR_1OFMANY, 0, IPS_IDLE);

    // Arm/Disarm control (safety feature - must arm before raise/lower)
    IUFillSwitch(&ArmControlS[0], "ARM", "Arm", ISS_OFF);
    IUFillSwitch(&ArmControlS[1], "DISARM", "Disarm", ISS_OFF);
    IUFillSwitchVector(&ArmControlSP, ArmControlS, 2, getDeviceName(), "ARM_CONTROL", "Arm Control", MAIN_CONTROL_TAB, IP_RW, ISR_1OFMANY, 0, IPS_IDLE);

    // Roof control
    IUFillSwitch(&RoofControlS[0], "ROOF_OPEN", "Open", ISS_OFF);
    IUFillSwitch(&RoofControlS[1], "ROOF_CLOSE", "Close", ISS_OFF);
    IUFillSwitch(&RoofControlS[2], "ROOF_STOP", "Stop", ISS_OFF);
    IUFillSwitchVector(&RoofControlSP, RoofControlS, 3, getDeviceName(), "ROOF_CONTROL", "Roof Control", MAIN_CONTROL_TAB, IP_RW, ISR_ATMOST1, 0, IPS_IDLE);

    // Lock control
    IUFillSwitch(&LockControlS[0], "LOCK_ENGAGE", "Engage", ISS_OFF);
    IUFillSwitch(&LockControlS[1], "LOCK_DISENGAGE", "Disengage", ISS_OFF);
    IUFillSwitch(&LockControlS[2], "LOCK_STOP", "Stop", ISS_OFF);
    IUFillSwitchVector(&LockControlSP, LockControlS, 3, getDeviceName(), "LOCK_CONTROL", "Lock Control", MAIN_CONTROL_TAB, IP_RW, ISR_ATMOST1, 0, IPS_IDLE);

    // Roof status
    IUFillNumber(&RoofStatusN[0], "POSITION", "Position (%)", "%.1f", 0.0, 100.0, 0.0, 0.0);
    IUFillNumber(&RoofStatusN[1], "IS_OPEN", "Is Open", "%.0f", 0.0, 1.0, 1.0, 0.0);
    IUFillNumber(&RoofStatusN[2], "IS_CLOSED", "Is Closed", "%.0f", 0.0, 1.0, 1.0, 0.0);
    IUFillNumberVector(&RoofStatusNP, RoofStatusN, 3, getDeviceName(), "ROOF_STATUS", "Roof Status", MAIN_CONTROL_TAB, IP_RO, 0, IPS_IDLE);

    // Lock status
    IUFillNumber(&LockStatusN[0], "LOCK_STATE", "Lock State", "%.0f", 0.0, 4.0, 1.0, 0.0);
    IUFillNumberVector(&LockStatusNP, LockStatusN, 1, getDeviceName(), "LOCK_STATUS", "Lock Status", MAIN_CONTROL_TAB, IP_RO, 0, IPS_IDLE);

    // Telemetry
    IUFillNumber(&TelemetryN[0], "H_BRIDGE_CURRENT", "H-Bridge Current (A)", "%.3f", 0.0, 100.0, 0.0, 0.0);
    IUFillNumber(&TelemetryN[1], "VOLTAGE_5V", "5V Voltage (V)", "%.2f", 0.0, 10.0, 0.0, 0.0);
    IUFillNumber(&TelemetryN[2], "VOLTAGE_12V", "12V Voltage (V)", "%.2f", 0.0, 20.0, 0.0, 0.0);
    IUFillNumber(&TelemetryN[3], "LIMIT_U1", "Limit Switch U1", "%.0f", 0.0, 1.0, 1.0, 0.0);
    IUFillNumber(&TelemetryN[4], "LIMIT_U2", "Limit Switch U2", "%.0f", 0.0, 1.0, 1.0, 0.0);
    IUFillNumber(&TelemetryN[5], "LIMIT_L1", "Limit Switch L1", "%.0f", 0.0, 1.0, 1.0, 0.0);
    IUFillNumber(&TelemetryN[6], "LIMIT_L2", "Limit Switch L2", "%.0f", 0.0, 1.0, 1.0, 0.0);
    IUFillNumberVector(&TelemetryNP, TelemetryN, 7, getDeviceName(), "TELEMETRY", "Telemetry", MAIN_CONTROL_TAB, IP_RO, 0, IPS_IDLE);

    return true;
}

bool RoofController::updateProperties()
{
    INDI::DefaultDevice::updateProperties();

    if (isConnected())
    {
        defineProperty(&ArmControlSP);
        defineProperty(&RoofControlSP);
        defineProperty(&LockControlSP);
        defineProperty(&RoofStatusNP);
        defineProperty(&LockStatusNP);
        defineProperty(&TelemetryNP);

        // Reset arm state on connect
        isArmed = false;
        ArmControlS[0].s = ISS_OFF;
        ArmControlS[1].s = ISS_ON;
        ArmControlSP.s = IPS_IDLE;
        IDSetSwitch(&ArmControlSP, nullptr);

        // Start timer for periodic status updates
        SetTimer(updatePeriod);
    }
    else
    {
        deleteProperty(ArmControlSP.name);
        deleteProperty(RoofControlSP.name);
        deleteProperty(LockControlSP.name);
        deleteProperty(RoofStatusNP.name);
        deleteProperty(LockStatusNP.name);
        deleteProperty(TelemetryNP.name);
        
        // Reset arm state on disconnect
        isArmed = false;
    }

    return true;
}

bool RoofController::Connect()
{
    if (DefaultDevice::Connect())
    {
        // Initialize status parsing state
        statusState = STATE_IDLE;
        statusBuffer.clear();
        syncBytesRead = 0;
        
        // Reset arm state
        isArmed = false;
        
        // Read initial status
        readStatus();
        
        // Send initial heartbeat
        sendCommand(CMD_HEARTBEAT);
        
        return true;
    }
    
    return false;
}

bool RoofController::Disconnect()
{
    DefaultDevice::Disconnect();
    
    // Reset status parsing state
    statusState = STATE_IDLE;
    statusBuffer.clear();
    syncBytesRead = 0;
    
    // Reset arm state
    isArmed = false;
    
    return true;
}

bool RoofController::ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n)
{
    if (dev != nullptr && strcmp(dev, getDeviceName()) == 0)
    {
        // Arm/Disarm control
        if (!strcmp(name, ArmControlSP.name))
        {
            IUUpdateSwitch(&ArmControlSP, states, names, n);
            int index = IUFindOnSwitchIndex(&ArmControlSP);
            
            if (index == 0) // Arm
            {
                isArmed = true;
                ArmControlSP.s = IPS_OK;
            }
            else // Disarm
            {
                isArmed = false;
                ArmControlSP.s = IPS_OK;
            }
            
            IDSetSwitch(&ArmControlSP, nullptr);
            return true;
        }

        // Roof control
        if (!strcmp(name, RoofControlSP.name))
        {
            IUUpdateSwitch(&RoofControlSP, states, names, n);
            int index = IUFindOnSwitchIndex(&RoofControlSP);
            
            if (index == 0) // Open
            {
                if (!isArmed)
                {
                    RoofControlSP.s = IPS_ALERT;
                    DEBUG(INDI::Logger::DBG_WARNING, "Roof open command rejected - not armed");
                }
                else if (sendCommand(CMD_RAISE_ROOF))
                {
                    RoofControlSP.s = IPS_OK;
                    // Auto-disarm after command
                    isArmed = false;
                    ArmControlS[0].s = ISS_OFF;
                    ArmControlS[1].s = ISS_ON;
                    ArmControlSP.s = IPS_IDLE;
                    IDSetSwitch(&ArmControlSP, nullptr);
                }
                else
                {
                    RoofControlSP.s = IPS_ALERT;
                }
            }
            else if (index == 1) // Close
            {
                if (!isArmed)
                {
                    RoofControlSP.s = IPS_ALERT;
                    DEBUG(INDI::Logger::DBG_WARNING, "Roof close command rejected - not armed");
                }
                else if (sendCommand(CMD_LOWER_ROOF))
                {
                    RoofControlSP.s = IPS_OK;
                    // Auto-disarm after command
                    isArmed = false;
                    ArmControlS[0].s = ISS_OFF;
                    ArmControlS[1].s = ISS_ON;
                    ArmControlSP.s = IPS_IDLE;
                    IDSetSwitch(&ArmControlSP, nullptr);
                }
                else
                {
                    RoofControlSP.s = IPS_ALERT;
                }
            }
            else if (index == 2) // Stop (works regardless of arm state)
            {
                if (sendCommand(CMD_STOP_ROOF))
                {
                    RoofControlSP.s = IPS_OK;
                }
                else
                {
                    RoofControlSP.s = IPS_ALERT;
                }
            }
            
            IDSetSwitch(&RoofControlSP, nullptr);
            return true;
        }

        // Lock control
        if (!strcmp(name, LockControlSP.name))
        {
            IUUpdateSwitch(&LockControlSP, states, names, n);
            int index = IUFindOnSwitchIndex(&LockControlSP);
            
            if (index == 0) // Engage
            {
                if (!isArmed)
                {
                    LockControlSP.s = IPS_ALERT;
                    DEBUG(INDI::Logger::DBG_WARNING, "Lock engage command rejected - not armed");
                }
                else if (sendCommand(CMD_ENGAGE_LOCK))
                {
                    LockControlSP.s = IPS_OK;
                    // Auto-disarm after command
                    isArmed = false;
                    ArmControlS[0].s = ISS_OFF;
                    ArmControlS[1].s = ISS_ON;
                    ArmControlSP.s = IPS_IDLE;
                    IDSetSwitch(&ArmControlSP, nullptr);
                }
                else
                {
                    LockControlSP.s = IPS_ALERT;
                }
            }
            else if (index == 1) // Disengage
            {
                if (!isArmed)
                {
                    LockControlSP.s = IPS_ALERT;
                    DEBUG(INDI::Logger::DBG_WARNING, "Lock disengage command rejected - not armed");
                }
                else if (sendCommand(CMD_DISENGAGE_LOCK))
                {
                    LockControlSP.s = IPS_OK;
                    // Auto-disarm after command
                    isArmed = false;
                    ArmControlS[0].s = ISS_OFF;
                    ArmControlS[1].s = ISS_ON;
                    ArmControlSP.s = IPS_IDLE;
                    IDSetSwitch(&ArmControlSP, nullptr);
                }
                else
                {
                    LockControlSP.s = IPS_ALERT;
                }
            }
            else if (index == 2) // Stop (works regardless of arm state)
            {
                if (sendCommand(CMD_STOP_LOCK))
                {
                    LockControlSP.s = IPS_OK;
                }
                else
                {
                    LockControlSP.s = IPS_ALERT;
                }
            }
            
            IDSetSwitch(&LockControlSP, nullptr);
            return true;
        }
    }

    return INDI::DefaultDevice::ISNewSwitch(dev, name, states, names, n);
}

bool RoofController::ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n)
{
    if (dev != nullptr && strcmp(dev, getDeviceName()) == 0)
    {
        // Handle number properties if needed
    }

    return INDI::DefaultDevice::ISNewNumber(dev, name, values, names, n);
}

bool RoofController::ISSnoopDevice(XMLEle *root)
{
    return INDI::DefaultDevice::ISSnoopDevice(root);
}

void RoofController::TimerHit()
{
    if (!isConnected())
    {
        SetTimer(updatePeriod);
        return;
    }

    // Send heartbeat periodically (firmware requires this to keep commands active)
    static int heartbeatCounter = 0;
    heartbeatCounter++;
    if (heartbeatCounter >= (updatePeriod / heartbeatPeriod))
    {
        sendCommand(CMD_HEARTBEAT);
        heartbeatCounter = 0;
    }

    // Read status periodically
    readStatus();

    SetTimer(updatePeriod);
}

bool RoofController::sendCommand(uint8_t commandByte)
{
    if (!isConnected())
    {
        return false;
    }

    // Protocol: 3 sync bytes (0x50) + 1 command byte
    // TODO: Adjust protocol based on actual device requirements
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

bool RoofController::readStatus()
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
    
    // Read all available data
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
            
            switch (statusState)
            {
                case STATE_IDLE:
                    if (byte == SYNC_BYTE)
                    {
                        statusBuffer.clear();
                        statusBuffer.push_back(byte);
                        syncBytesRead = 1;
                        statusState = STATE_READING_SYNC;
                    }
                    break;
                    
                case STATE_READING_SYNC:
                    if (byte == SYNC_BYTE)
                    {
                        statusBuffer.push_back(byte);
                        syncBytesRead++;
                        if (syncBytesRead == 3)
                        {
                            statusState = STATE_READING_PAYLOAD;
                        }
                    }
                    else
                    {
                        statusState = STATE_IDLE;
                        statusBuffer.clear();
                        syncBytesRead = 0;
                        if (byte == SYNC_BYTE)
                        {
                            statusBuffer.push_back(byte);
                            syncBytesRead = 1;
                            statusState = STATE_READING_SYNC;
                        }
                    }
                    break;
                    
                case STATE_READING_PAYLOAD:
                    statusBuffer.push_back(byte);
                    if (statusBuffer.size() == STATUS_PACKET_SIZE)
                    {
                        latestCompletePacket = statusBuffer;
                        foundCompletePacket = true;
                        DEBUGF(INDI::Logger::DBG_DEBUG, "Received complete status packet (%zu bytes)", statusBuffer.size());
                        statusState = STATE_IDLE;
                        statusBuffer.clear();
                        syncBytesRead = 0;
                    }
                    else if (statusBuffer.size() > STATUS_PACKET_SIZE)
                    {
                        statusState = STATE_IDLE;
                        statusBuffer.clear();
                        syncBytesRead = 0;
                    }
                    break;
            }
        }
    }
    
    // Process only the latest complete packet
    if (foundCompletePacket)
    {
        parseStatus(latestCompletePacket);
        return true;
    }

    return false;
}

void RoofController::parseStatus(const std::vector<uint8_t> &data)
{
    if (data.size() != STATUS_PACKET_SIZE)
    {
        DEBUGF(INDI::Logger::DBG_WARNING, "Invalid status packet size: %zu (expected %zu)", data.size(), STATUS_PACKET_SIZE);
        return;
    }

    // Verify sync bytes
    if (data[0] != SYNC_BYTE || data[1] != SYNC_BYTE || data[2] != SYNC_BYTE)
    {
        DEBUGF(INDI::Logger::DBG_WARNING, "Invalid sync bytes in status packet: %02X %02X %02X", data[0], data[1], data[2]);
        return;
    }

    size_t offset = 3; // Skip sync bytes

    // Parse 3 analog sensor values (floats, 4 bytes each)
    // Order: h_bridge_current, voltage_5v, voltage_12v
    float h_bridge_current, voltage_5v, voltage_12v;
    
    memcpy(&h_bridge_current, &data[offset], sizeof(float));
    offset += sizeof(float);
    memcpy(&voltage_5v, &data[offset], sizeof(float));
    offset += sizeof(float);
    memcpy(&voltage_12v, &data[offset], sizeof(float));
    offset += sizeof(float);

    // Parse 4 digital limit switches (1 byte each)
    uint8_t limit_u1 = data[offset++];
    uint8_t limit_u2 = data[offset++];
    uint8_t limit_l1 = data[offset++];
    uint8_t limit_l2 = data[offset++];

    // Parse roof state (1 byte)
    uint8_t roof_state = data[offset++];

    // Parse lock state (1 byte)
    uint8_t lock_state = data[offset++];

    // Update telemetry values
    TelemetryN[0].value = h_bridge_current;
    TelemetryN[1].value = voltage_5v;
    TelemetryN[2].value = voltage_12v;
    TelemetryN[3].value = limit_u1;
    TelemetryN[4].value = limit_u2;
    TelemetryN[5].value = limit_l1;
    TelemetryN[6].value = limit_l2;
    TelemetryNP.s = IPS_OK;
    IDSetNumber(&TelemetryNP, nullptr);

    // Update roof status
    // Roof states: 0=UNKNOWN, 1=RAISING, 2=LOWERING, 3=RAISED, 4=LOWERED
    float position = 0.0;
    float is_open = 0.0;
    float is_closed = 0.0;
    
    if (roof_state == 3) // RAISED
    {
        position = 100.0;
        is_open = 1.0;
        is_closed = 0.0;
    }
    else if (roof_state == 4) // LOWERED
    {
        position = 0.0;
        is_open = 0.0;
        is_closed = 1.0;
    }
    else if (roof_state == 1) // RAISING
    {
        position = 50.0; // Intermediate
        is_open = 0.0;
        is_closed = 0.0;
    }
    else if (roof_state == 2) // LOWERING
    {
        position = 50.0; // Intermediate
        is_open = 0.0;
        is_closed = 0.0;
    }
    else // UNKNOWN
    {
        position = 0.0;
        is_open = 0.0;
        is_closed = 0.0;
    }
    
    RoofStatusN[0].value = position;
    RoofStatusN[1].value = is_open;
    RoofStatusN[2].value = is_closed;
    RoofStatusNP.s = IPS_OK;
    IDSetNumber(&RoofStatusNP, nullptr);

    // Update lock status
    LockStatusN[0].value = lock_state;
    LockStatusNP.s = IPS_OK;
    IDSetNumber(&LockStatusNP, nullptr);

    DEBUGF(INDI::Logger::DBG_DEBUG, 
           "Status: Roof=%d, Lock=%d, Current=%.3fA, 5V=%.2fV, 12V=%.2fV",
           roof_state, lock_state, h_bridge_current, voltage_5v, voltage_12v);
}

