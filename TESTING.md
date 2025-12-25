# Testing the Telescope Controller INDI Driver

This guide explains how to test the driver with your real device using the INDI server and CLI tools.

## Prerequisites

1. INDI library installed (`libindi-dev` on Ubuntu/Debian)
2. Driver built and installed
3. Device connected via serial port (check with `ls -l /dev/ttyUSB*` or `ls -l /dev/ttyACM*`)

## Building and Installing

```bash
mkdir build
cd build
cmake ..
make
sudo make install
```

The driver executable will be installed to `/usr/bin/TelescopeControllerDriver` (or wherever INDI drivers are installed).

## Testing with INDI Server (Verbose Mode)

### Step 1: Find Your Serial Port

```bash
# List serial devices
ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null

# Or use dmesg to see recent connections
dmesg | tail -20
```

The device should be at `/dev/tty_telescope_controller`.

### Step 2: Verify Serial Port

The driver is configured to use `/dev/tty_telescope_controller` by default. If you need to change it:
- Edit `telescope_controller.cpp` line 40 and rebuild, OR
- Use INDI client to configure the port (recommended)

### Step 3: Run INDI Server with Verbose Output

Run the INDI server with maximum verbosity to see all communication:

```bash
indiserver -v -v -v TelescopeControllerDriver
```

The `-v -v -v` flags enable maximum verbosity. You should see:
- Driver initialization
- Connection attempts
- Serial port communication
- Property updates
- Telemetry parsing

### Step 4: Connect to the Device

In another terminal, use `indi_setprop` to connect:

```bash
indi_setprop "Telescope Controller.CONNECTION.CONNECT=On"
```

You should see output in the indiserver terminal showing:
- Connection success
- Properties being defined
- Telemetry packets being received and parsed

### Step 5: Monitor Telemetry

Watch telemetry values update in real-time:

```bash
# Watch telemetry (updates every second)
watch -n 1 'indi_getprop "Telescope Controller.TELEMETRY.*"'
```

Or get a single snapshot:

```bash
indi_getprop "Telescope Controller.TELEMETRY.*"
```

Expected output:
```
Telescope Controller.TELEMETRY.AMBIENT_TEMP=25.50
Telescope Controller.TELEMETRY.HEATER1_TEMP=24.30
Telescope Controller.TELEMETRY.HEATER2_TEMP=23.80
Telescope Controller.TELEMETRY.HEATER3_TEMP=24.10
Telescope Controller.TELEMETRY.LENS_CAP_STATE=0
Telescope Controller.TELEMETRY.FLAT_LIGHT_STATE=0
```

### Step 6: Test Commands

Test heater controls:

```bash
# Enable Heater 1
indi_setprop "Telescope Controller.HEATER1.HEATER1_ON=On"

# Disable Heater 1
indi_setprop "Telescope Controller.HEATER1.HEATER1_OFF=On"

# Enable Heater 2
indi_setprop "Telescope Controller.HEATER2.HEATER2_ON=On"

# Enable Heater 3
indi_setprop "Telescope Controller.HEATER3.HEATER3_ON=On"
```

Test lens cap:

```bash
# Open lens cap
indi_setprop "Telescope Controller.LENS_CAP.LENS_CAP_OPEN=On"

# Close lens cap
indi_setprop "Telescope Controller.LENS_CAP.LENS_CAP_CLOSE=On"
```

Test flat light:

```bash
# Turn on flat light
indi_setprop "Telescope Controller.FLAT_LIGHT.FLAT_LIGHT_ON=On"

# Turn off flat light
indi_setprop "Telescope Controller.FLAT_LIGHT.FLAT_LIGHT_OFF=On"
```

### Step 7: Check Switch States

Verify that switch states update from telemetry:

```bash
indi_getprop "Telescope Controller.HEATER1.*"
indi_getprop "Telescope Controller.HEATER2.*"
indi_getprop "Telescope Controller.HEATER3.*"
indi_getprop "Telescope Controller.LENS_CAP.*"
indi_getprop "Telescope Controller.FLAT_LIGHT.*"
```

## Debugging Tips

### Check Serial Communication

If you see connection issues, verify serial port permissions:

```bash
# Add user to dialout group (may need to logout/login)
sudo usermod -a -G dialout $USER

# Or use sudo for testing
sudo indiserver -v -v -v TelescopeControllerDriver
```

### Monitor Raw Serial Data

In a separate terminal, you can monitor the serial port directly (while driver is NOT connected):

```bash
# Install minicom or use screen
sudo apt-get install minicom

# Monitor serial port
minicom -D /dev/tty_telescope_controller -b 57600

# Or with screen
screen /dev/tty_telescope_controller 57600
```

Press `Ctrl+A` then `K` to exit screen.

### Check INDI Server Logs

The verbose output will show:
- `[INFO]` - Normal operations
- `[DEBUG]` - Detailed debugging info
- `[ERROR]` - Errors

Look for:
- `Connected to serial port` - Connection successful
- `Reading telemetry` - Telemetry being read
- `Parsing telemetry packet` - Packet parsing (if you add debug output)
- `Property updated` - Properties being updated

### Common Issues

1. **Permission denied**: Add user to `dialout` group or use `sudo`
2. **Port not found**: Check device path with `ls -l /dev/tty*`
3. **No telemetry**: Verify baud rate is 57600 and device is sending data
4. **Connection fails**: Check if another process is using the serial port

## Quick Test Script

Save this as `test_driver.sh`:

```bash
#!/bin/bash

echo "Starting INDI server..."
indiserver -v -v -v TelescopeControllerDriver &
SERVER_PID=$!

sleep 2

echo "Connecting to device..."
indi_setprop "Telescope Controller.CONNECTION.CONNECT=On"

sleep 3

echo "Current telemetry:"
indi_getprop "Telescope Controller.TELEMETRY.*"

echo ""
echo "Testing Heater 1..."
indi_setprop "Telescope Controller.HEATER1.HEATER1_ON=On"
sleep 2
indi_getprop "Telescope Controller.HEATER1.*"

echo ""
echo "Press Ctrl+C to stop server"
wait $SERVER_PID
```

Make it executable and run:
```bash
chmod +x test_driver.sh
./test_driver.sh
```

## Using with INDI Clients

Once verified via CLI, you can use with:
- **KStars/Ekos**: Add device in Ekos profile manager
- **indi_web_manager**: Web-based INDI control
- **QIndiControl**: Qt-based INDI control panel

The device will appear as "Telescope Controller" in the device list.

