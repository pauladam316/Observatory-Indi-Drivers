# Adding TelescopeControllerDriver to INDI Server Service

## Step 1: Build and Install the Driver

First, make sure the driver is built and installed:

```bash
cd ~/telescope_controller_driver/build
make
sudo make install
```

This will install `TelescopeControllerDriver` to `/usr/bin/` (or wherever INDI drivers are installed).

## Step 2: Verify Driver Location

Check where the driver was installed:

```bash
which TelescopeControllerDriver
```

Or check if it's in the standard location:

```bash
ls -l /usr/bin/TelescopeControllerDriver
```

## Step 3: Update Systemd Service

Edit your systemd service file:

```bash
sudo systemctl edit --full runindiserver.service
```

Or manually edit:

```bash
sudo nano /etc/systemd/system/runindiserver.service
```

## Step 4: Add Driver to ExecStart

Add `TelescopeControllerDriver` to the ExecStart line. Your updated service file should look like:

```ini
[Unit]
Description=Setup indi server

[Service]
Type=simple
User=root
ExecStart=indiserver -p 6123 -u /tmp/indiserver2/ indi_eqmod_telescope indi_canon_ccd indi_asi_focuser TelescopeControllerDriver
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
```

**Note:** The driver name in the ExecStart line is just `TelescopeControllerDriver` (no path needed if it's in `/usr/bin/`).

## Step 5: Reload and Restart Service

After editing the service file:

```bash
# Reload systemd to pick up changes
sudo systemctl daemon-reload

# Restart the service
sudo systemctl restart runindiserver.service

# Check status
sudo systemctl status runindiserver.service

# View logs to verify driver loaded
sudo journalctl -u runindiserver.service -f
```

## Step 6: Verify Driver is Running

Check the logs to see if the driver started successfully:

```bash
sudo journalctl -u runindiserver.service | grep -i "telescope\|controller"
```

You should see messages indicating the driver is loaded and available.

## Alternative: Using Full Path

If the driver isn't in your PATH, you can use the full path:

```ini
ExecStart=indiserver -p 6123 -u /tmp/indiserver2/ indi_eqmod_telescope indi_canon_ccd indi_asi_focuser /usr/bin/TelescopeControllerDriver
```

## Troubleshooting

**Driver not found:**
- Verify installation: `which TelescopeControllerDriver`
- Check permissions: `ls -l /usr/bin/TelescopeControllerDriver`
- Make sure it's executable: `sudo chmod +x /usr/bin/TelescopeControllerDriver`

**Service won't start:**
- Check logs: `sudo journalctl -u runindiserver.service -n 50`
- Verify driver compiles and runs: `TelescopeControllerDriver --help` (if supported)

**Driver appears but can't connect:**
- Check serial port permissions
- Verify device is connected: `ls -l /dev/ttyUSB*`
- Check INDI client can see the device

