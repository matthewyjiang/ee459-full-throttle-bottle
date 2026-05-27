# Stack n' Go

**EE459 — Team 2:** Matthew Jiang, Xiaolei Yu, Richard Ochoa
University of Southern California

A modular, smart lunch container system that maintains food and beverages at optimal temperatures using active and passive thermal regulation. Stackable compartments connect via a bayonet-mount locking mechanism and communicate over I2C. A Wi-Fi-enabled web interface lets you monitor and adjust each compartment's temperature from any device on the same network.

---

## Repository Structure

```
.
├── att/                  # ATtiny85 slave firmware (avr-gcc)
│   ├── main.c
│   ├── usiTwiSlave.c/h   # I2C slave library
│   ├── i2c_secondary.c/h
│   └── Makefile
├── esp32_master/         # ESP32 master firmware (ESP-IDF / CMake)
│   ├── main/
│   │   └── esp32_master.c
│   └── CMakeLists.txt
└── esp/                  # Legacy ESP8266 code (deprecated, not used)
```

---

## System Overview

| Layer | Component | Role |
|---|---|---|
| Master controller | ESP32 | Wi-Fi, HTTP server, I2C master, FreeRTOS |
| Slave controller | ATtiny85-20 | PID temperature loop, PWM heater control |
| Communication | I2C (2-wire) | Up to 127 slave modules on shared bus |
| Heating | 13 W 12 V polyimide pad + IRL540 MOSFET | PWM-controlled heat |
| Sensing | TMP36 analog sensor | Temperature feedback to ATtiny ADC |
| Power | 3 × 18650 Li-Ion (11.1 V nominal) | ~3.5 h continuous heating per charge |
| Charging | 3S USB-C charging board | Single-port recharge |
| Regulation | LM7805 (12 V → 5 V), LM3940 (5 V → 3.3 V) | Per-rail regulation |

The ESP32 polls each ATtiny every second via I2C and serves a mobile-friendly dashboard over HTTP. Each ATtiny runs an independent PID loop (Kp = 6.0, Ki = 0.1, Kd = 0.0, 500 ms period) and reports temperature as two bytes (integer + fractional).

### Web API

| Endpoint | Description |
|---|---|
| `GET /data` | JSON array of all heaters — address, current temp, setpoint, status |
| `GET /set?address=x&temp=y` | Set heater at I2C address `x` to `y` °C (clamped to 100 °C max) |

---

## Building and Flashing

### Prerequisites

- [ESP-IDF](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/get-started/) (set `IDF_PATH` and source `export.sh`)
- `avr-gcc`, `avrdude`, USBtinyISP programmer

### ESP32 Master

```bash
cd esp32_master

# Configure Wi-Fi credentials (first time)
idf.py menuconfig   # Component config → Wi-Fi → SSID / Password

# Build
idf.py build

# Flash
idf.py -p /dev/ttyUSB0 flash

# Monitor serial output
idf.py -p /dev/ttyUSB0 monitor
```

### ATtiny85 Slave

Each slave must be flashed with a unique I2C address. Edit the address constant in `att/main.c` before flashing each unit.

```bash
cd att

# Build
make

# Set fuses (one-time per chip)
make fuse

# Flash
make flash
```

The Makefile targets `usbtiny` as the programmer. Adjust `PROGRAMMER` in `att/Makefile` if you use a different ISP.

---

## Known Limitations / Future Work

- **Wi-Fi provisioning:** SSID and password are compiled in. A production build should create its own hotspot for first-time setup.
- **Level shifting:** The prototype runs 3.3 V ESP32 logic into 5 V ATtiny I2C lines without an explicit level shifter. This works due to I2C's open-drain topology but is not production-safe.
- **Battery management:** No cell-level balancing, undervoltage lockout, or overcurrent protection in the current charging board. A proper BMS is needed for a production unit.
- **Slave addressing:** I2C addresses are set at flash time. Physical shorting pads on the connector would allow auto-addressing with identical firmware across all slaves.
- **Sensor noise:** The TMP36 exhibits up to 50 mV (~10 °C) of noise. A 5-sample moving average is used as a workaround. Digital sensors (e.g., MCP9808) would be more robust.
- **No watchdog timers:** A firmware lockup requires a manual power cycle on both MCUs.
