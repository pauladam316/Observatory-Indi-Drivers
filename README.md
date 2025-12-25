# Observatory INDI Drivers

A monorepo containing custom INDI drivers for observatory equipment.

## Drivers

### Telescope Controller
An INDI driver for controlling a telescope controller AUX device via serial communication.

**Features:**
- Heater Control: Enable/disable 3 independent heaters
- Lens Cap Control: Open/close the lens cap
- Flat Light Control: Enable/disable the flat light
- Telemetry: Real-time monitoring of temperatures and system states

**Default serial port:** `/dev/tty_telescope_controller`  
**Default baud rate:** `57600`

### Roof Controller
An INDI driver for controlling a roof controller device via serial communication.

**Features:**
- **Safety System**: ARM/DISARM control - must arm before roof/lock movement commands
- **Roof Control**: Open, close, and stop roof movement
- **Lock Control**: Engage, disengage, and stop lock mechanism
- **Telemetry**: Real-time monitoring of:
  - H-bridge current draw
  - 5V and 12V supply voltages
  - Limit switch states (4 switches: U1, U2, L1, L2)
  - Roof position and state
  - Lock state
- **Auto-disarm**: System automatically disarms after each movement command for safety

**Default serial port:** `/dev/tty_roof_controller`  
**Default baud rate:** `57600`

**Safety Features:**
- ARM command must be sent before raise/lower commands (roof or lock)
- System auto-disarms after each movement command
- STOP commands work regardless of arm state (emergency stop)

## Requirements

- INDI library (libindi-dev)
- CMake 3.5 or higher
- C++17 compatible compiler

## Building

### Build All Drivers

To build all drivers:

```bash
mkdir build
cd build
cmake ..
make
sudo make install
```

### Build Individual Drivers

To build only specific drivers, use CMake options:

```bash
mkdir build
cd build

# Build only Telescope Controller
cmake -DBUILD_ALL_DRIVERS=OFF -DBUILD_TELESCOPE_CONTROLLER=ON ..

# Build only Roof Controller
cmake -DBUILD_ALL_DRIVERS=OFF -DBUILD_ROOF_CONTROLLER=ON ..

# Build both (explicit)
cmake -DBUILD_ALL_DRIVERS=OFF -DBUILD_TELESCOPE_CONTROLLER=ON -DBUILD_ROOF_CONTROLLER=ON ..

make
sudo make install
```

### Build Options

- `BUILD_ALL_DRIVERS` (default: ON) - Build all available drivers
- `BUILD_TELESCOPE_CONTROLLER` (default: ON) - Build Telescope Controller driver
- `BUILD_ROOF_CONTROLLER` (default: ON) - Build Roof Controller driver

## Project Structure

```
observatory_indi_drivers/
├── CMakeLists.txt              # Root build configuration
├── drivers/
│   ├── telescope_controller/
│   │   ├── CMakeLists.txt      # Driver-specific build config
│   │   ├── telescope_controller.h
│   │   └── telescope_controller.cpp
│   └── roof_controller/
│       ├── CMakeLists.txt      # Driver-specific build config
│       ├── roof_controller.h
│       └── roof_controller.cpp
└── README.md
```

## Adding New Drivers

To add a new driver:

1. Create a new directory under `drivers/` (e.g., `drivers/my_driver/`)
2. Create `CMakeLists.txt` in the new directory (see existing drivers as templates)
3. Add your driver source files (`.h` and `.cpp`)
4. Update the root `CMakeLists.txt` to include your driver:
   - Add a `BUILD_MY_DRIVER` option
   - Add `add_subdirectory(drivers/my_driver)` conditionally

## Configuration

Each driver will appear in INDI as a separate device. Serial port settings can be configured in the INDI client connection settings.

## Exposed Properties

### Telescope Controller
- **HEATER1, HEATER2, HEATER3**: Switch properties to enable/disable heaters
- **LENS_CAP**: Switch property to open/close lens cap
- **FLAT_LIGHT**: Switch property to enable/disable flat light
- **TELEMETRY**: Number property with 19 values:
  - Ambient temperature, Heater 1-3 temperatures
  - Heater states (driver, manual, real) for each heater
  - Flat light states (driver, manual, real)
  - Lens cap states (driver, manual, real)

### Roof Controller
- **ARM_CONTROL**: Switch property to arm/disarm the system (required before movement)
- **ROOF_CONTROL**: Switch property to open/close/stop the roof
- **LOCK_CONTROL**: Switch property to engage/disengage/stop the lock
- **ROOF_STATUS**: Number property with 3 values:
  - Position (%): 0-100% (0=closed, 100=open, 50=in motion)
  - Is Open: 1.0 if fully open, 0.0 otherwise
  - Is Closed: 1.0 if fully closed, 0.0 otherwise
- **LOCK_STATUS**: Number property with 1 value:
  - Lock State: 0=UNKNOWN, 1=RAISING/ENGAGING, 2=LOWERING/DISENGAGING, 3=RAISED/ENGAGED, 4=LOWERED/DISENGAGED
- **TELEMETRY**: Number property with 7 values:
  - H-Bridge Current (A): Motor current draw
  - 5V Voltage (V): 5V supply voltage
  - 12V Voltage (V): 12V supply voltage
  - Limit Switch U1, U2: Upper limit switches (0 or 1)
  - Limit Switch L1, L2: Lower limit switches (0 or 1)

## Protocol Implementation

### Telescope Controller Protocol

**Command Format (TO device):**
Commands are sent as: `[0x50, 0x50, 0x50, command_byte]`
- `0x01` = Heater 1 Enable
- `0x02` = Heater 1 Disable
- `0x03` = Heater 2 Enable
- `0x04` = Heater 2 Disable
- `0x05` = Heater 3 Enable
- `0x06` = Heater 3 Disable
- `0x07` = Lens Cap Open
- `0x08` = Lens Cap Close
- `0x09` = Flat Light On
- `0x0A` = Flat Light Off

**Telemetry Format (FROM device):**
- Header: `[0x50, 0x50, 0x50]` (3 sync bytes)
- Payload (31 bytes): temperatures, heater states, light states, lens cap states
- Total packet size: 34 bytes

### Roof Controller Protocol

**Command Format (TO device):**
Commands are sent as: `[0x50, 0x50, 0x50, command_byte]`
- `0xAB` = Raise Roof
- `0xCD` = Lower Roof
- `0xEF` = Stop Roof
- `0x12` = Engage Lock
- `0x34` = Disengage Lock
- `0x56` = Stop Lock
- `0xF1` = Heartbeat (sent periodically to keep commands active)

**Note:** ARM/DISARM is a driver-side safety feature only. The firmware does not receive ARM/DISARM commands. The driver enforces that the system must be armed before sending movement commands, and automatically disarms after each movement command.

**Telemetry Format (FROM device):**
Telemetry packets are automatically sent by the device:
- Header: `[0x50, 0x50, 0x50]` (3 sync bytes)
- Payload (18 bytes):
  - 3 floats (12 bytes): H-bridge current (A), 5V voltage (V), 12V voltage (V)
  - 4 bytes: Limit switch states (U1, U2, L1, L2) - 0 or 1
  - 1 byte: Roof state (0=UNKNOWN, 1=RAISING, 2=LOWERING, 3=RAISED, 4=LOWERED)
  - 1 byte: Lock state (0=UNKNOWN, 1=RAISING/ENGAGING, 2=LOWERING/DISENGAGING, 3=RAISED/ENGAGED, 4=LOWERED/DISENGAGED)
- Total packet size: 21 bytes

**Safety Protocol (Driver-Side):**
The ARM/DISARM safety system is implemented entirely in the INDI driver, not in the firmware:
1. Driver requires ARM state before allowing roof or lock movement commands
2. After a movement command is sent, the driver automatically disarms
3. STOP commands work regardless of arm state (emergency stop)
4. Heartbeat command (`0xF1`) must be sent periodically (every 250ms) to keep commands active in firmware

## Usage

### Telescope Controller
1. Connect to the device via INDI client (KStars, etc.)
2. Configure the serial port if different from default
3. Use the switches to control heaters, lens cap, and flat light
4. Monitor telemetry values in real-time

### Roof Controller
1. Connect to the device via INDI client (KStars, etc.)
2. Configure the serial port if different from default
3. **ARM the system** before sending any movement commands
4. Use the roof control switches to open/close/stop the roof
5. Use the lock control switches to engage/disengage/stop the lock
6. Monitor telemetry values (current, voltages, limit switches) and status (roof position, lock state) in real-time

**Important Safety Notes:**
- Always ARM before sending roof or lock movement commands
- The system will automatically disarm after each movement command
- STOP commands work immediately regardless of arm state
- Monitor limit switches and telemetry to ensure safe operation

## Development Notes

- All drivers use serial communication via INDI's Connection::Serial plugin
- Telemetry/status updates are handled via periodic timers
- Error handling is in place for connection failures and command errors
- Each driver implements a state machine for parsing incoming data packets
- Roof Controller requires heartbeat commands every 250ms to keep movement commands active
- Roof Controller implements safety features: ARM requirement, auto-disarm, and emergency stop

## License

[Add your license here]
