# Quick Start Guide - ESP32 Robot Controller

## 🚀 Quick Setup (5 Minutes)

### Step 1: Build the Firmware
```bash
cd c:\Users\Brahmworks\OneDrive\Documents\GitHub\Reactor4-shell\Machani_ESP32lib\src\esp32_main
python -m platformio run -e esp32dev
```

**Expected Output:**
```
========================= [SUCCESS] Took 10.98 seconds =========================
RAM:   [          ]   3.4% (used 11028 bytes from 327680 bytes)
Flash: [==        ]  17.5% (used 183793 bytes from 1048576 bytes)
```

### Step 2: Flash to ESP32
```bash
python -m platformio run -e esp32dev --target upload
```

### Step 3: Connect Ethernet
- Connect Ethernet cable to ESP32 RMII interface
- Ensure DHCP is available on your network
- Wait for IP assignment (automatic)

### Step 4: Run Tests
```bash
# Simple interactive menu
python run_test.py

# Or direct command
python test_client.py

# Or specific test
python test_client.py --test torso
```

---

## 📊 What's Implemented

### Network Communication
- ✅ Ethernet via UDP (ports 5555/5556)
- ✅ Auto IP assignment (DHCP)
- ✅ JSON message format
- ✅ Pub/Sub routing

### Pub/Sub Topics

**Receive (Subscribe):**
- `torso_cmd` - {"angle": -135..+135, "speed": 0..999, "accel": 0..100}
- `neck_cmd` - {"angle": -30..+30, "speed": 0..999, "accel": 0..100}
- `head_cmd` - {"angle": -15..+15, "speed": 0..999, "accel": 0..100}
- `batt` - {"batt": 0..100, "ps": true/false, "hb_out": 1..255}

**Send (Publish):**
- `touch` - {"state": "tap|double|up|down|long"}
- `led` - {"state": "listen|mute|loading|offline|connecting|low_battery"}
- `reb_jet` - {"reb_jet": true/false}

---

## 🧪 Testing Guide

### Quick Validation
```bash
# This will:
# 1. Wait for ESP32 handshake
# 2. Test all motor commands
# 3. Test all sensor/LED publishes
# 4. Display results with timestamps
python test_client.py
```

### Individual Motor Tests
```bash
python test_client.py --test torso    # Test torso motor
python test_client.py --test neck     # Test neck motor
python test_client.py --test head     # Test head motor
```

### Sensor/Event Tests
```bash
python test_client.py --test touch    # Test touch events
python test_client.py --test led      # Test LED states
python test_client.py --test battery  # Test battery info
python test_client.py --test reboot   # Test reboot signal
```

### Specific ESP32 IP
```bash
python test_client.py --esp32-ip 192.168.1.100
```

---

## 📁 Project Structure

```
esp32_main/
├── QUICK_START.md                    ← You are here
├── IMPLEMENTATION_README.md          ← Full documentation
├── COMPLETION_SUMMARY.md             ← Technical details
├── test_client.py                    ← Main test script (445 lines)
├── run_test.py                       ← Interactive test menu
├── components/
│   ├── rtos_blink_esp32/
│   │   ├── iot_protocol.c           ← Network protocol (369 lines)
│   │   ├── iot_protocol.h           ← Protocol API (162 lines)
│   │   ├── rtos_blink_esp32.c       ← Main app (78 lines)
│   │   ├── rtos_blink_esp32.h
│   │   └── CMakeLists.txt
│   ├── uart_standard/                ← UART debugging
│   └── esp_eth_enc28j60/             ← Optional Ethernet PHY
├── src/
│   └── main.c                        ← Entry point
└── lib/
    └── servo_control/                ← Servo motor lib
```

---

## 🔧 Configuration

### Default Settings
- **UART Baud**: 115200
- **Ethernet Port (ESP32)**: 5555
- **Ethernet Port (Windows)**: 5556
- **Max Subscriptions**: 10
- **Max Payload**: 512 bytes
- **IP Assignment**: DHCP (automatic)

### Modify Settings
Edit `components/rtos_blink_esp32/iot_protocol.c`:
```c
#define UDP_PORT 5555              // Change UDP port
#define MAX_SUBSCRIPTIONS 10       // Change max subscriptions
#define MAX_PAYLOAD 512            // Change max payload size
#define HANDSHAKE_TIMEOUT_MS 5000  // Change timeout
```

---

## 📋 Checklist

Before deployment:
- [ ] Code compiles successfully
- [ ] Firmware flashed to ESP32
- [ ] Ethernet cable connected
- [ ] DHCP server available
- [ ] Python 3.6+ installed
- [ ] test_client.py runs successfully
- [ ] Motor commands received (check logs)
- [ ] Sensor data published (check test client output)

---

## 🐛 Troubleshooting

### ESP32 not responding
1. Check Ethernet connection
2. Verify DHCP is available: `ipconfig` on Windows
3. Check power supply (ESP32 needs stable 3.3V)
4. Review serial logs: `python -m platformio device monitor`

### Test client says "No handshake"
1. Wait 10 seconds for boot
2. Check ESP32 has IP address
3. Verify network allows UDP broadcast
4. Try with specific IP: `python test_client.py --esp32-ip 192.168.1.X`

### Motor commands not working
1. Check callback is registered
2. Verify JSON payload format is correct
3. Monitor ESP32 logs for errors
4. Check motor controller is initialized

### Slow communication
1. Reduce network load
2. Check Ethernet cable quality
3. Verify buffer sizes are sufficient
4. Monitor WiFi interference

---

## 📞 Support

### Log Monitoring
```bash
# Monitor ESP32 logs
python -m platformio device monitor --baud 115200
```

### Debug Build
```bash
# Build with debug symbols
python -m platformio run -e esp32dev -v
```

### Clear Build Cache
```bash
# Full clean rebuild
python -m platformio run -e esp32dev --target clean
python -m platformio run -e esp32dev
```

---

## ✅ Verification

After flashing, you should see:
1. **ESP32 Startup** → Ethernet initialization message
2. **IP Assignment** → DHCP IP received
3. **Handshake Sent** → Broadcast message to Windows
4. **Ready for Commands** → Listening on port 5555

Example serial output:
```
I (timestamp) rtos_blink: Initializing IoT Protocol...
I (timestamp) iot_protocol: Ethernet initialized successfully. IP: 192.168.1.X
I (timestamp) iot_protocol: Performing handshake...
I (timestamp) iot_protocol: Handshake sent
```

Then run test:
```bash
python test_client.py
```

Expected output:
```
============================================================
ESP32 Robot Controller Pub/Sub Test Suite
============================================================
[HH:MM:SS.xxx] Client initialized
[HH:MM:SS.xxx] Listening on port 5556
[HH:MM:SS.xxx] Sending to 5555

=== TEST: Waiting for ESP32 Handshake ===
[HH:MM:SS.xxx] RX from 192.168.1.X: {"topic":"system","payload":{"type":"handshake"...}}
[HH:MM:SS.xxx] SUCCESS: Received handshake from esp32_robot

[SUCCESS] All tests passed!
```

---

**Status**: 🟢 Ready to Deploy  
**Last Updated**: December 11, 2025  
**Maintainer**: Brahm Works
