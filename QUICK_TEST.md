# Quick Test Guide

## Fastest Way to Test

### 1. Build and Install
```bash
cd build
cmake ..
make
sudo make install
```

### 2. Verify Serial Port
```bash
ls -l /dev/tty_telescope_controller
```

The driver is configured to use `/dev/tty_telescope_controller` by default.

### 3. Run INDI Server (Verbose)
```bash
indiserver -v -v -v TelescopeControllerDriver
```

You'll see verbose output showing:
- Connection attempts
- Serial communication
- Telemetry packets being received
- Parsed temperature and state values

### 4. Connect (in another terminal)
```bash
indi_setprop "Telescope Controller.CONNECTION.CONNECT=On"
```

### 5. Watch Telemetry Update
```bash
watch -n 1 'indi_getprop "Telescope Controller.TELEMETRY.*"'
```

### 6. Test a Command
```bash
indi_setprop "Telescope Controller.HEATER1.HEATER1_ON=On"
```

## What to Look For

In the `indiserver` output, you should see:
- `[DEBUG] Sending command: 0x01` - Commands being sent
- `[DEBUG] Read X bytes from serial port` - Data being received
- `[DEBUG] Received complete telemetry packet (34 bytes)` - Packets detected
- `[DEBUG] Telemetry: Ambient=25.50°C, H1=24.30°C...` - Parsed values

If you see errors:
- `[WARNING] Invalid telemetry packet size` - Check baud rate (should be 57600)
- `[WARNING] Invalid sync bytes` - Check serial connection
- `[ERROR] Failed to write command` - Check serial port permissions

## Quick Troubleshooting

**Permission denied?**
```bash
sudo usermod -a -G dialout $USER
# Then logout and login again, or use sudo for testing
```

**Port not found?**
```bash
# Check what ports are available
dmesg | grep -i tty
```

**No telemetry?**
- Verify device is powered and sending data
- Check baud rate is 57600
- Try monitoring port directly: `minicom -D /dev/tty_telescope_controller -b 57600`

