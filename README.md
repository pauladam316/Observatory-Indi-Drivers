# Telescope Controller INDI Driver

An INDI driver for controlling a telescope controller AUX device via serial communication.

## Features

- **Heater Control**: Enable/disable 3 independent heaters
- **Lens Cap Control**: Open/close the lens cap
- **Flat Light Control**: Enable/disable the flat light
- **Telemetry**: Real-time monitoring of:
  - Ambient temperature
  - Heater temperatures (3 heaters)
  - Lens cap state
  - Flat light state

## Requirements

- INDI library (libindi-dev)
- CMake 3.5 or higher
- C++11 compatible compiler

## Building

```bash
mkdir build
cd build
cmake ..
make
sudo make install
```

## Configuration

The driver will appear in INDI as an AUX device named "Telescope Controller". 

### Serial Port Settings

Default serial port: `/dev/tty_telescope_controller`  
Default baud rate: `57600`

These can be configured in the INDI client connection settings.

## Protocol Implementation

The driver implements the complete device protocol:

### Command Format (TO device)
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

### Telemetry Format (FROM device)
Telemetry packets are automatically sent by the device:
- Header: `[0x50, 0x50, 0x50]` (3 sync bytes)
- Payload (31 bytes):
  - 4 floats (16 bytes): Heater 1 temp, Heater 2 temp, Heater 3 temp, Ambient temp
  - 9 bytes: Heater states (3 bytes each: driver_state, manual_state, real_state)
  - 3 bytes: Flat light states (driver_state, manual_state, real_state)
  - 3 bytes: Lens cap states (driver_state, manual_state, real_state)
- Total packet size: 34 bytes

The driver uses a state machine to parse incoming telemetry packets and updates all properties automatically.

## Usage

1. Connect to the device via INDI client (KStars, etc.)
2. Configure the serial port if different from default
3. Use the switches to control heaters, lens cap, and flat light
4. Monitor telemetry values in real-time

## Development Notes

- Telemetry is updated every 1 second (configurable via `updatePeriod`)
- All commands are sent with `\r\n` line endings (may need adjustment based on device)
- Error handling is in place for connection failures and command errors

## License

[Add your license here]

