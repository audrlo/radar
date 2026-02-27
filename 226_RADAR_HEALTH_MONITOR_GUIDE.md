# Radar-Based Ambient Health Monitor — Complete Build Guide

## TI IWR6843AOPEVM + Seeed MR60BHA2 Dual-Sensor System on Raspberry Pi 5

---

## Project Context

This project builds a standalone, non-invasive health monitoring system for elderly care using millimeter-wave (mmWave) radar. The goal is continuous 24/7 monitoring of a senior's room — tracking vital signs, detecting falls, observing bed entry and exit, classifying activity levels, and flagging anomalies — all without cameras, wearables, or any device the person needs to remember to put on.

The concept is modeled after systems like Caspar Health's radar-based elder monitoring platform, which uses ceiling-mounted 60GHz radar to detect falls with 98% accuracy, track heart rate and breathing during sleep, monitor nightly bathroom visits, and generate daily care reports by feeding sensor data into an LLM. Caspar runs in assisted living facilities and hospitals, where a single radar unit per room replaces manual bed checks and reduces response time for fall events from minutes to seconds.

Our system replicates this capability at the edge using a Raspberry Pi 5 as the compute platform and two complementary radar sensors that each do what they're best at. The TI IWR6843AOPEVM provides full raw point cloud access — 3D position and velocity of every detected reflection in the room — which gives us the data we need to build custom fall detection, activity classification (lying, sitting, standing, walking, fallen), room occupancy tracking, and gait analysis. The Seeed MR60BHA2 runs dedicated on-chip DSP firmware optimized specifically for extracting heart rate and breathing rate from chest wall micro-movements, giving us reliable vitals without needing to build our own vital signs algorithm from scratch. Together, the two sensors cover the full monitoring picture: the TI gives us spatial awareness and event detection, the Seeed gives us physiological signals.

All data flows into a local SQLite database for trend analysis, baseline computation, and anomaly detection. The system computes per-patient baselines over time and alerts when current readings deviate significantly — the same personalized approach Caspar uses, but running entirely on-device. Daily summaries can be pushed to a cloud LLM for natural-language care report generation, or integrated with a family notification app, nurse call system, or voice companion like Sam.

Target deployment environments include private homes (bedroom monitoring for aging-in-place seniors), assisted living facilities, hospital rooms, and rehabilitation centers. The system requires no patient compliance, preserves complete visual privacy (radar only, no images), and runs continuously on ~12W of power.

---

## System Architecture Overview

```
┌─────────────────────────────────────────────────────────────────────┐
│                    RASPBERRY PI 5 (8GB)                              │
│                                                                     │
│  ┌─────────────┐   ┌──────────────┐   ┌─────────────────────────┐  │
│  │ TI IWR6843  │   │ Seeed        │   │ HLK-LD2450 (optional)  │  │
│  │ AOPEVM      │   │ MR60BHA2     │   │ Bathroom zone sensor   │  │
│  │             │   │              │   │                         │  │
│  │ Provides:   │   │ Provides:    │   │ Provides:               │  │
│  │ · Point     │   │ · Heart rate │   │ · Presence in zone      │  │
│  │   cloud     │   │ · Breathing  │   │ · Target tracking       │  │
│  │ · Fall      │   │   rate       │   │ · Occupancy duration    │  │
│  │   detection │   │ · Sleep      │   │                         │  │
│  │ · Activity  │   │   staging    │   │ Connects via:           │  │
│  │   classify  │   │ · Presence   │   │ USB-UART adapter        │  │
│  │ · Gait      │   │              │   │ → /dev/ttyUSB0          │  │
│  │   speed     │   │ Connects via:│   └─────────────────────────┘  │
│  │             │   │ GPIO UART    │                                 │
│  │ Connects:   │   │ → /dev/      │   ┌─────────────────────────┐  │
│  │ USB → dual  │   │   ttyAMA0    │   │ HealthTracker Engine    │  │
│  │ UART ports  │   │              │   │                         │  │
│  │ /dev/ttyACM0│   └──────────────┘   │ · Rolling baselines     │  │
│  │ /dev/ttyACM1│                      │ · Z-score anomalies     │  │
│  └─────────────┘                      │ · Activity state log    │  │
│                                       │ · SQLite logging        │  │
│                                       │ · Alert generation      │  │
│                                       │ · Daily summaries       │  │
│                                       └─────────────────────────┘  │
│                                                                     │
│  Output: Terminal display + SQLite database + Alert hooks           │
└─────────────────────────────────────────────────────────────────────┘
```

**Why two sensors instead of one?** The TI IWR6843 gives raw point cloud data and can extract vitals from its own firmware, but its vital signs accuracy improves significantly when the subject is perfectly stationary within 1.5m. The Seeed MR60BHA2 runs a purpose-built DSP pipeline for vitals that's been tuned specifically for heart/breathing extraction and handles more real-world conditions. Using both means the TI handles everything spatial (where is the person, are they falling, what are they doing) while the Seeed handles the physiological signal extraction — each doing what it's optimized for.

---

## Bill of Materials

| Part | Purpose | Price | Source | Notes |
|------|---------|-------|--------|-------|
| Raspberry Pi 5 (8GB) | Main compute | ~$80 | raspberrypi.com, Adafruit, SparkFun | 8GB recommended for point cloud processing |
| Pi 5 27W USB-C PSU | Pi power | ~$12 | raspberrypi.com | Official PSU recommended for stability |
| Pi 5 active cooler | Thermal management | ~$5 | raspberrypi.com | Prevents throttling during continuous operation |
| MicroSD 64GB A2 | OS + data storage | ~$10 | Amazon | A2 rated for sustained write from SQLite logging |
| **TI IWR6843AOPEVM** | Primary radar — point cloud, fall detection, activity | ~$55 | ti.com, Mouser, DigiKey | 60GHz FMCW, antenna-on-package |
| **TI MMWAVEICBOOST** | Carrier board for IWR6843 | ~$70 | ti.com, Mouser, DigiKey | Provides USB interface, power regulation, flash capability |
| 5V/2.5A USB wall adapter | Power for MMWAVEICBOOST | ~$8 | Amazon | **Do NOT power from Pi** — draw is too high |
| 2× micro-USB to USB-A cables | Connect MMWAVEICBOOST | ~$10 | Amazon | One for data (USB1→Pi), one for power (USB2→wall) |
| **Seeed MR60BHA2** | Vitals radar — heart rate, breathing rate | ~$33 | seeedstudio.com, Mouser | 60GHz, pre-processed vitals over UART |
| HLK-LD2450 | Secondary zone radar (bathroom) | ~$4 | AliExpress, Amazon | 24GHz, multi-target, optional |
| CP2102 USB-UART adapter | Connect LD2450 to Pi USB | ~$3 | Amazon, AliExpress | Only needed if using LD2450 |
| F-F dupont wires (10-pack) | MR60BHA2 GPIO wiring | ~$3 | Amazon, AliExpress | Need at least 4 wires |
| Mounting hardware | Ceiling/wall bracket + screws | ~$5 | Hardware store | Plastic enclosure — radar passes through plastic, not metal |

**Total: ~$298** (with LD2450) or ~$291 (without LD2450)

### Where to Buy — Direct Links

**TI IWR6843AOPEVM:**
- TI store: ti.com → search "IWR6843AOPEVM"
- Mouser: mouser.com part# 595-IWR6843AOPEVM
- DigiKey: digikey.com part# 296-IWR6843AOPEVM-ND

**TI MMWAVEICBOOST:**
- TI store: ti.com → search "MMWAVEICBOOST"
- Mouser: mouser.com part# 595-MMWAVEICBOOST
- DigiKey: digikey.com part# 296-MMWAVEICBOOST-ND

**Seeed MR60BHA2:**
- Seeed Studio: seeedstudio.com → search "MR60BHA2 60GHz"
- Mouser: mouser.com → search "Seeed MR60BHA2"

---

## Hardware Assembly

### Step 1: Mount IWR6843AOPEVM onto MMWAVEICBOOST

The IWR6843AOPEVM is a small eval module that plugs into the larger MMWAVEICBOOST carrier board via a 60-pin high-density connector.

```
    ┌──────────────────────────────────────┐
    │        IWR6843AOPEVM                 │
    │   ┌─────────────────────────────┐    │
    │   │  ┌───┐                      │    │
    │   │  │ANT│  Antenna-on-         │    │
    │   │  │PKG│  Package (AOP)       │    │
    │   │  └───┘                      │    │
    │   │                             │    │
    │   │  60-pin connector           │    │
    │   │  ████████████████           │    │
    │   └─────────┬───────────────────┘    │
    │             │                        │
    │     plug into 60-pin socket          │
    │             │                        │
    │   ┌─────────▼───────────────────┐    │
    │   │                             │    │
    │   │    MMWAVEICBOOST            │    │
    │   │    (carrier board)          │    │
    │   │                             │    │
    │   │  [USB1]       [USB2]        │    │
    │   │  Data Port    Power/Flash   │    │
    │   │                             │    │
    │   │  [SOP Jumpers]              │    │
    │   │  SOP0   SOP1   SOP2        │    │
    │   └─────────────────────────────┘    │
    └──────────────────────────────────────┘
```

Align the 60-pin connector and press firmly until seated. The AOP antenna side of the IWR6843 should face away from the MMWAVEICBOOST board (outward toward the room when mounted).

### Step 2: Set SOP Jumpers

The Sense-on-Power (SOP) jumpers on the MMWAVEICBOOST control boot mode:

```
    For NORMAL OPERATION (functional mode):
        SOP0: OPEN   (no jumper)
        SOP1: OPEN   (no jumper)
        SOP2: CLOSED (jumper installed)    ← only this one

    For FLASHING FIRMWARE (one-time setup):
        SOP0: OPEN
        SOP1: OPEN
        SOP2: OPEN   (all open — removes jumper from SOP2)

    After flashing, put the SOP2 jumper back for functional mode.
```

### Step 3: Wire MMWAVEICBOOST to Raspberry Pi 5

The MMWAVEICBOOST has two micro-USB ports. USB1 is the data port, USB2 is power/flash.

```
    ┌─────────────────────────────────┐
    │         MMWAVEICBOOST           │
    │                                 │
    │  [USB1 - Data] ────────────────────► Pi 5 USB-A port
    │   micro-USB        USB-A cable  │    (creates /dev/ttyACM0
    │                                 │     and /dev/ttyACM1)
    │                                 │
    │  [USB2 - Power/Flash] ─────────────► 5V/2.5A wall adapter
    │   micro-USB        USB-A cable  │    (NOT the Pi — the radar
    │                                 │     draws ~2-3W, too much
    │                                 │     for Pi USB current limits)
    └─────────────────────────────────┘
```

**CRITICAL:** Do not power the MMWAVEICBOOST from the Pi's USB ports. The IWR6843 draws 2-3W and will cause brownouts or boot failures on the Pi. Always use a separate wall adapter.

When USB1 is plugged into the Pi, two virtual serial ports appear:

```
    /dev/ttyACM0 → "User UART" (data port)
                    Point cloud + vitals TLV data streams here
                    Baud rate: 921600

    /dev/ttyACM1 → "CLI UART" (config port)
                    Send .cfg chirp configuration commands here
                    Baud rate: 115200
```

### Step 4: Wire Seeed MR60BHA2 to Pi GPIO

The MR60BHA2 connects directly to the Pi's GPIO UART pins. Four wires total.

```
                    ┌─────────────────────────────┐
                    │     Seeed MR60BHA2           │
                    │     (component side up)      │
                    │                              │
                    │  [VCC] [GND] [TX] [RX]       │
                    └───┬─────┬─────┬─────┬────────┘
                        │     │     │     │
                        │     │     │     │
     Dupont wires:      │     │     │     │
     (female-female)    │     │     │     │
                        │     │     │     │
                    ┌───▼─────▼─────▼─────▼────────┐
                    │   Raspberry Pi 5 GPIO         │
                    │                               │
                    │   Pin 4  (5V)    ◄── VCC      │
                    │   Pin 6  (GND)   ◄── GND      │
                    │   Pin 10 (RXD0)  ◄── TX       │  ← Sensor TX → Pi RX
                    │   Pin 8  (TXD0)  ◄── RX       │  ← Pi TX → Sensor RX
                    └───────────────────────────────┘
```

**IMPORTANT — UART Crossover:**
```
    MR60BHA2 TX  ───────►  Pi RXD (GPIO15, Pin 10)
    MR60BHA2 RX  ◄───────  Pi TXD (GPIO14, Pin 8)

    TX goes to RX. RX goes to TX. Classic serial crossover.
    If you connect TX-to-TX, you'll get no data.
```

### Step 5: Wire HLK-LD2450 (Optional — Bathroom Zone)

Since GPIO UART is used by the MR60BHA2, the LD2450 connects through a USB-to-UART adapter:

```
    ┌──────────────┐         ┌──────────────────┐
    │  HLK-LD2450  │         │  USB-UART Adapter │
    │              │         │  (CP2102/CH340)   │
    │  VCC ────────┼────────►│  5V               │
    │  GND ────────┼────────►│  GND              │
    │  TX  ────────┼────────►│  RX               │  ← TX to RX crossover
    │  RX  ────────┼────────►│  TX               │  ← RX to TX crossover
    └──────────────┘         └────────┬──────────┘
                                      │ USB-A
                                      ▼
                              Pi 5 USB-A port
                              (appears as /dev/ttyUSB0)
```

### Full System Wiring Diagram

```
    ┌──────────────┐    USB-A     ┌─────────────────────────┐
    │   Pi 5       │◄────────────┤  MMWAVEICBOOST           │
    │              │  (data)      │  + IWR6843AOPEVM         │
    │  USB-A port  │              │                          │
    │              │              │  USB2 ──► 5V wall adapter│
    │              │              └──────────────────────────┘
    │              │
    │  USB-A port ◄┼───────── CP2102 USB-UART ◄── LD2450
    │              │
    │  GPIO UART:  │
    │  Pin 4  (5V) ├──► MR60BHA2 VCC
    │  Pin 6 (GND) ├──► MR60BHA2 GND
    │  Pin 8 (TXD) ├──► MR60BHA2 RX
    │  Pin 10(RXD) ├──◄ MR60BHA2 TX
    │              │
    │  USB-C ◄─────┼──► 27W Pi PSU
    └──────────────┘
```

### Pi 5 GPIO Pinout Reference (Relevant Pins)

```
                    3V3 [1 ] [2 ] 5V        ◄── MR60BHA2 VCC
                   SDA [3 ] [4 ] 5V
                   SCL [5 ] [6 ] GND        ◄── MR60BHA2 GND
               GPIO 4  [7 ] [8 ] GPIO14/TXD ◄── MR60BHA2 RX (Pi TX → Sensor RX)
                   GND  [9 ] [10] GPIO15/RXD ◄── MR60BHA2 TX (Sensor TX → Pi RX)
              GPIO 17  [11] [12] GPIO18
              GPIO 27  [13] [14] GND
              GPIO 22  [15] [16] GPIO23
                   3V3 [17] [18] GPIO24
         GPIO10/MOSI   [19] [20] GND
         GPIO9/MISO    [21] [22] GPIO25
         GPIO11/SCLK   [23] [24] GPIO8/CE0
                   GND  [25] [26] GPIO7/CE1
              GPIO 0   [27] [28] GPIO1
              GPIO 5   [29] [30] GND
              GPIO 6   [31] [32] GPIO12
             GPIO 13   [33] [34] GND
       GPIO19/PCM_FS   [35] [36] GPIO16
      GPIO 26          [37] [38] GPIO20
                   GND  [39] [40] GPIO21
```

### Full Pinout Summary Table

```
    Pi 5 Pin         │  Connected To           │  Purpose
    ─────────────────┼─────────────────────────┼──────────────────
    Pin 4  (5V)      │  MR60BHA2 VCC           │  Vitals sensor power
    Pin 6  (GND)     │  MR60BHA2 GND           │  Ground
    Pin 8  (TXD0)    │  MR60BHA2 RX            │  Pi sends to vitals sensor
    Pin 10 (RXD0)    │  MR60BHA2 TX            │  Vitals sensor sends to Pi
    USB-A Port 1     │  MMWAVEICBOOST USB1      │  TI radar data (ttyACM0/1)
    USB-A Port 2     │  CP2102 USB-UART adapter │  LD2450 bathroom sensor
    USB-C            │  27W PSU                 │  Pi power
```

---

## Sensor Placement

```
    RECOMMENDED ROOM LAYOUT — CEILING/HIGH WALL MOUNT

    ┌──────────────────────────────────────────────┐
    │                   CEILING                     │
    │                                               │
    │    [TI IWR6843 + MMWAVEICBOOST]              │  Mount 2.0-2.5m high
    │     antenna facing down into room            │  on ceiling or high wall
    │               \                               │
    │                \  ← 120° FOV cone            │
    │                 \                             │
    │    [MR60BHA2]    \                            │  Mount near the TI sensor
    │     aimed at      \                           │  or on opposite wall
    │     bed area       \                          │  aimed at bed, 0.5-2m
    │                     \                         │  from person's chest
    │    ┌─────────────────\───────┐                │
    │    │     BED          \     │                 │
    │    │                   X    │                 │
    │    │   (person)             │                 │
    │    └────────────────────────┘                 │
    │                                               │
    │    [LD2450]                                   │  Mount at bathroom
    │    aimed at bathroom door                    │  doorway, ~1m height
    └──────────────────────────────────────────────┘
```

**Placement guidelines:**

The TI IWR6843 has a wide 120° FOV and 6m range. Mount it on the ceiling directly above or adjacent to the bed area, antenna side facing down. At 2.5m ceiling height, the cone covers roughly a 5m diameter circle on the floor — enough for a full bedroom. The point cloud gives x/y/z positions of all reflections, so the fall detection and activity classifier work anywhere in the coverage area.

The Seeed MR60BHA2 has a narrower effective range for vitals — about 1.5m for reliable heart/breathing. Mount it closer to the bed, aimed at the person's torso. A nightstand-level mount (0.5-1m from chest) or a wall mount near the headboard works well. It doesn't need line-of-sight to the whole room since it's only doing vitals.

The LD2450 covers a secondary zone (bathroom, hallway). Mount it at the bathroom doorway, ~1m height, aimed across the doorway so it can detect anyone passing through or lingering inside.

**CRITICAL:** Radar signals pass through plastic, wood, drywall, and fabric — but NOT through metal. Use plastic enclosures, or cut a window in metal enclosures for the antenna. Keep sensors away from metal surfaces, HVAC ducts, and large mirrors, which cause multipath reflections and false detections.

---

## Firmware Setup

### TI IWR6843 Firmware Flashing (One-Time, Requires PC)

The IWR6843 ships with demo firmware, but you may want to flash a specific demo (vital signs or people counting). This is a one-time setup done on a Windows or Linux x86 PC — NOT on the Pi (the TI tools don't support ARM).

**What you need:**
- A Windows or Linux x86 PC (not the Pi)
- TI UniFlash tool (free download from ti.com)
- TI mmWave SDK (free, ~2GB download from ti.com)
- A micro-USB cable

**Procedure:**

1. Download and install TI UniFlash from ti.com/tool/UNIFLASH
2. Download the mmWave SDK from ti.com/tool/MMWAVE-SDK — you need the demo binaries
3. On the MMWAVEICBOOST, set ALL SOP jumpers to OPEN (flash mode)
4. Connect MMWAVEICBOOST USB2 to the PC via micro-USB
5. Open UniFlash, select your device (IWR6843), and flash the appropriate .bin:
   - For fall detection + activity tracking: `people_counting` demo binary
   - For vital signs: `vital_signs` demo binary
   - The demo binaries are in the SDK under `packages/ti/demo/xwrXXXX/`
6. After flashing, set SOP2 back to CLOSED (functional mode)
7. Disconnect from PC, connect to Pi as described above

**If you're not sure which firmware to flash:** Start with the `people_counting` demo. It provides point cloud data and basic vitals, which is what our software expects by default. The `vital_signs` demo gives better vitals accuracy but loses the point cloud (and therefore fall detection). Since the MR60BHA2 handles vitals in our dual-sensor setup, `people_counting` is the right choice.

### Chirp Configuration Tuning

After firmware is flashed, the sensor's behavior is controlled by a chirp configuration file (.cfg) that you send to the CLI UART port on every boot. The configuration defines the radar's operating parameters: frequency range, bandwidth, number of chirps per frame, frame rate, CFAR detection thresholds, and field-of-view limits.

Our software includes two built-in configs:

**PEOPLE_COUNTING_CFG** (default, recommended):
- Range: 0.5m - 6.0m
- Frame rate: ~10 fps
- Multi-target point cloud
- Clutter removal enabled
- Optimized for activity classification and fall detection

**VITAL_SIGNS_CFG** (for TI-based vitals extraction):
- Range: 0.25m - 2.0m
- Frame rate: ~20 fps
- Single-target, stationary subject
- Higher range resolution for micro-motion detection
- Optimized for chest wall displacement measurement

The configs are sent automatically by the Python driver on startup. To tune for your specific room, you can:

1. Connect the MMWAVEICBOOST to a PC running TI's mmWave Visualizer
2. Load the default config and observe the point cloud in real-time
3. Adjust CFAR thresholds (detection sensitivity), FOV limits, and range bounds
4. Export the tuned .cfg and paste it into `iwr6843_sensor.py`

**Chirp configuration reference:** TI mmWave SDK → "Chirp Design Guide" (~40 pages). The key parameters to tune are `profileCfg` (bandwidth, chirp slope — controls range resolution) and `cfarFovCfg` (range and doppler bounds — controls what the sensor "sees").

### MR60BHA2 Setup (No Firmware Flashing Needed)

The Seeed MR60BHA2 runs self-contained firmware. No flashing, no configuration. Just wire it up and it starts outputting vitals data over UART at 115200 baud immediately on power-up. The protocol is documented below in the software section.

### LD2450 Setup (No Firmware Flashing Needed)

Same as the MR60BHA2 — the LD2450 runs fixed firmware and outputs target tracking data over UART at 256000 baud immediately on power-up.

---

## Pi 5 Software Setup

### Step 1: Enable UART

```bash
sudo raspi-config
# → Interface Options → Serial Port
# → Login shell over serial: NO
# → Serial port hardware enabled: YES
# Reboot

# Verify UART is enabled
ls -la /dev/serial0    # should symlink to /dev/ttyAMA0
```

### Step 2: Verify All Sensor Connections

```bash
# Check TI radar (USB)
ls /dev/ttyACM*
# Should show: /dev/ttyACM0 /dev/ttyACM1

# Check MR60BHA2 (GPIO UART)
ls -la /dev/ttyAMA0
# Should exist and be accessible

# Check LD2450 (USB-UART adapter, if connected)
ls /dev/ttyUSB*
# Should show: /dev/ttyUSB0

# Quick connectivity test — TI radar
python3 -c "
import serial
data_port = serial.Serial('/dev/ttyACM0', 921600, timeout=1)
cli_port = serial.Serial('/dev/ttyACM1', 115200, timeout=1)
print('TI Data port:', data_port.is_open)
print('TI CLI port:', cli_port.is_open)
data_port.close()
cli_port.close()
"

# Quick connectivity test — MR60BHA2
python3 -c "
import serial
ser = serial.Serial('/dev/ttyAMA0', 115200, timeout=2)
print('MR60BHA2 connected:', ser.is_open)
data = ser.read(100)
print(f'Received {len(data)} bytes: {data.hex()[:60]}...')
ser.close()
"
```

### Step 3: Install Python Dependencies

```bash
pip install pyserial numpy scipy --break-system-packages
```

---

## Software Architecture

### Project Structure

```
radar-project/
├── main.py                          # Main application — runs both sensors + health tracker
├── common/
│   ├── radar_interface.py           # Abstract sensor interface + shared data types
│   └── ld2450_sensor.py             # HLK-LD2450 bathroom zone driver
├── sensors/
│   ├── iwr6843_sensor.py            # TI IWR6843 driver (point cloud + fall detection)
│   └── mr60bha2_sensor.py           # Seeed MR60BHA2 driver (heart rate + breathing)
└── health_data.db                   # SQLite database (created on first run)
```

### Data Flow

```
TI IWR6843                      Seeed MR60BHA2
    │                                │
    │ USB (921600 baud)              │ GPIO UART (115200 baud)
    │ TLV binary frames              │ Proprietary binary frames
    ▼                                ▼
┌────────────────────┐    ┌────────────────────┐
│ iwr6843_sensor.py  │    │ mr60bha2_sensor.py │
│                    │    │                    │
│ Parse TLV frames   │    │ Parse UART frames  │
│ Extract point cloud│    │ Extract heart rate │
│ Run fall detection │    │ Extract breathing  │
│ Classify activity  │    │ Detect presence    │
│ Track centroids    │    │ Monitor sleep      │
└────────┬───────────┘    └────────┬───────────┘
         │                         │
         │  PresenceData           │  VitalSigns
         │  FallEvent              │  PresenceData
         │  ActivityState          │  FallEvent (on-chip)
         ▼                         ▼
    ┌──────────────────────────────────┐     ┌──────────────────┐
    │         HealthTracker            │     │  LD2450 Sensor   │
    │                                  │◄────│  (bathroom zone) │
    │  · Fuse vitals from MR60BHA2    │     └──────────────────┘
    │  · Fuse activity from IWR6843   │
    │  · Compute rolling baselines    │
    │  · Z-score anomaly detection    │
    │  · Bed entry/exit tracking      │
    │  · Inactivity monitoring        │
    │  · Bathroom duration alerts     │
    └──────────┬───────────────────────┘
               │
               ▼
    ┌──────────────────────────────────┐
    │         SQLite Database          │
    │                                  │
    │  Tables:                         │
    │  · vitals    (HR, BR, timestamp) │
    │  · activity  (state, duration)   │
    │  · alerts    (type, severity)    │
    │  · baselines (mean, std)         │
    │  · daily_summary (aggregates)    │
    └──────────────────────────────────┘
               │
               ▼
    Terminal output + Alert hooks
    (→ Sam voice, push notifications,
     nurse call, LLM care reports)
```

### Shared Data Types (radar_interface.py)

All sensors implement the same `RadarSensor` abstract interface and emit these shared types:

**VitalSigns:** heart_rate_bpm, breathing_rate_bpm, confidence scores, timestamp
**PresenceData:** is_present, activity state, distance, movement energy, timestamp
**FallEvent:** timestamp, confidence, x/y/z location
**Alert:** type (FALL_DETECTED, BED_EXIT, VITAL_ANOMALY, PROLONGED_INACTIVITY, ROOM_EMPTY), message, severity (1=info, 2=warning, 3=critical), data dict
**ActivityState:** UNKNOWN, EMPTY, LYING, SITTING, STANDING, WALKING, FALLEN

### TI IWR6843 Driver Details (iwr6843_sensor.py)

**Connection:** Dual UART over USB — /dev/ttyACM0 (data, 921600 baud) + /dev/ttyACM1 (CLI, 115200 baud)

**Boot sequence:** On `sensor.start()`, the driver opens both ports, then sends the chirp configuration line-by-line to the CLI port. Each line gets an echo + "Done" response. After `sensorStart`, the data port begins streaming TLV frames.

**TLV frame format:** Every frame begins with the 8-byte magic word `02 01 04 03 06 05 08 07`, followed by a 40-byte header containing frame number, number of detected objects, and number of TLV payloads. Each TLV has a 4-byte type, 4-byte length, and variable payload.

**TLV types parsed:**
- Type 1 (DETECTED_POINTS): Point cloud — array of (x, y, z, doppler) float32 per point
- Type 7 (SIDE_INFO): SNR and noise per point — int16 pairs
- Type 6 (VITAL_SIGNS): Heart rate + breathing rate (firmware-dependent offset, last 8 bytes)

**Fall detection algorithm:** State machine with 4 states:
1. `monitoring` → Normal operation. Watches for rapid centroid height drop.
2. `falling` → Triggered when centroid drops >0.5m in <1.5 seconds AND current height <0.3m. Waits 3 seconds for confirmation.
3. `fallen` → Confirmed if person stays below 0.3m with doppler <0.2 m/s for 3 seconds. Emits FallEvent with x/y/z location and 85% confidence.
4. `cooldown` → 30-second cooldown to prevent repeat alerts. Returns to monitoring.

False alarm rejection: if height recovers during the `falling` state (person sat down quickly), the state machine returns to `monitoring` without alerting.

**Activity classification:** Height-based thresholds on the point cloud centroid:
- z > 1.0m AND doppler > 0.3 m/s → WALKING
- z > 0.8m → STANDING
- z > 0.4m, spread > 0.3 → SITTING
- z > 0.4m, spread ≤ 0.3 → LYING
- z < 0.2m → FALLEN

These thresholds assume a ceiling mount at ~2.5m. They MUST be calibrated for your specific installation height.

**Chirp configs included:**
- `PEOPLE_COUNTING_CFG`: 0.5-6m range, ~10fps, multi-target, clutter removal, 120° FOV
- `VITAL_SIGNS_CFG`: 0.25-2m range, ~20fps, single-target, high range resolution

### Seeed MR60BHA2 Driver Details (mr60bha2_sensor.py)

**Connection:** GPIO UART at 115200 baud, 8N1 → /dev/ttyAMA0

**Protocol format:**
- Header: 0x53 0x59 (2 bytes)
- Control word: 2 bytes
- Command word: 2 bytes (message type)
- Data length: 2 bytes (little-endian uint16)
- Payload: N bytes
- Checksum: 1 byte (XOR of all preceding bytes)
- End marker: 0x54 0x43 (2 bytes)

**Message types parsed:**
- 0x0301 — Human presence (1 byte: 0=empty, 1=present)
- 0x0302 — Movement status (1 byte: 0=none, 1=stationary, 2=active)
- 0x0305 — Body movement energy (float32, 0-100 range)
- 0x0501 — Breathing rate (float32, breaths/min)
- 0x0502 — Heart rate (float32, beats/min)
- 0x0503 — Breathing waveform (raw float32 array)
- 0x0601 — Sleep status (1 byte: 0=awake, 1=light sleep, 2=deep sleep)
- 0x8001 — Fall detection (1 byte: 0=no fall, 1=fall detected)
- 0x0102 — Module heartbeat (keep-alive, ignored)

**Vitals sanity filtering:** Heart rate values outside 30-200 BPM and breathing rates outside 5-40 BPM are discarded. Vitals are emitted at most once per second.

**Bed exit detection:** When sleep status transitions from sleep (light/deep) to awake, the driver emits a BED_EXIT alert with 30-second cooldown to prevent repeat triggers from tossing/turning.

**Note on fall detection:** The MR60BHA2 has on-chip fall detection (message 0x8001), but it's less accurate than the TI's point-cloud-based approach (~80-85% vs 95%+ accuracy). In our dual-sensor setup, we primarily rely on the TI for fall detection. The MR60BHA2 fall output is still parsed and emitted as a secondary signal — if both sensors agree a fall happened, confidence is very high.

### LD2450 Driver Details (ld2450_sensor.py)

**Connection:** USB-UART adapter → /dev/ttyUSB0 at 256000 baud

**Protocol:** Fixed 30-byte frames. Header 0xAA 0xFF 0x03 0x00, then three 8-byte target slots (x, y in mm as signed int16, speed in mm/s), footer 0x55 0xCC plus 2-byte check.

**Zone monitoring:** The driver checks if any target is detected in the configured zone (default: any target present = occupied). It tracks occupancy start time and reports duration. The HealthTracker alerts if bathroom occupancy exceeds 20 minutes.

### HealthTracker Engine (main.py)

The HealthTracker is the core logic that fuses data from all sensors:

**Baseline computation:** Maintains rolling 1-hour windows (3600 readings at 1Hz) for heart rate and breathing rate. Every 100 readings, it recomputes mean and standard deviation and saves to SQLite. On startup, it loads the most recent baselines from the database, so per-patient personalization persists across reboots.

**Anomaly detection:** For each vitals reading, computes a z-score against the patient's baseline. If |reading - baseline_mean| / baseline_std > 2.5, generates a VITAL_ANOMALY alert. Severity 2 (warning) for z > 2.5, severity 3 (critical) for z > 3.5.

**Activity tracking:** Logs every activity state change with duration. Detects bed entry (transition to LYING) and exit (transition from LYING to any other state). Alerts on prolonged inactivity (>30 minutes of very low movement while not in bed).

**Bathroom monitoring:** Checks LD2450 zone occupancy every second. Alerts if occupied > 20 minutes.

**Daily summary generation:** `tracker.get_daily_summary()` returns a JSON dict with: average HR/BR, total sleep hours (time in LYING state), total active hours (WALKING + STANDING), activity breakdown, alert counts, and current baselines. This is the payload you'd send to a cloud LLM for care report generation.

### SQLite Database Schema

```sql
-- Every vitals reading from MR60BHA2 (1Hz)
CREATE TABLE vitals (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    timestamp REAL,
    heart_rate REAL,
    breathing_rate REAL,
    hr_confidence REAL,
    br_confidence REAL
);

-- Activity state changes from IWR6843
CREATE TABLE activity (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    timestamp REAL,
    state TEXT,          -- 'lying', 'sitting', 'standing', 'walking', 'fallen'
    duration_s REAL
);

-- All alerts
CREATE TABLE alerts (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    timestamp REAL,
    alert_type TEXT,     -- 'fall_detected', 'bed_exit', 'vital_anomaly', etc.
    severity INTEGER,    -- 1=info, 2=warning, 3=critical
    message TEXT,
    data TEXT             -- JSON blob with alert-specific details
);

-- Aggregated daily stats
CREATE TABLE daily_summary (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    date TEXT UNIQUE,
    avg_heart_rate REAL,
    avg_breathing_rate REAL,
    total_sleep_hours REAL,
    total_active_hours REAL,
    fall_count INTEGER,
    alert_count INTEGER
);

-- Per-patient baselines (persist across reboots)
CREATE TABLE baselines (
    key TEXT PRIMARY KEY,    -- 'hr_mean', 'hr_std', 'br_mean', 'br_std'
    value REAL,
    updated_at REAL
);
```

---

## Running the System

### Start with Both Sensors

```bash
cd radar-project

# Full system — TI for spatial + Seeed for vitals + LD2450 for bathroom
python main.py \
    --ti-data-port /dev/ttyACM0 \
    --ti-cli-port /dev/ttyACM1 \
    --ti-config people_counting \
    --vitals-port /dev/ttyAMA0 \
    --ld2450 /dev/ttyUSB0

# Without bathroom sensor
python main.py \
    --ti-data-port /dev/ttyACM0 \
    --ti-cli-port /dev/ttyACM1 \
    --vitals-port /dev/ttyAMA0
```

### Test Individual Sensors

```bash
# Test TI IWR6843 alone — should show point cloud + activity
python sensors/iwr6843_sensor.py

# Test Seeed MR60BHA2 alone — should show heart rate + breathing
python sensors/mr60bha2_sensor.py

# Test LD2450 alone — should show target positions
python common/ld2450_sensor.py
```

### Query the Database

```bash
# Recent vitals
sqlite3 health_data.db "SELECT * FROM vitals ORDER BY timestamp DESC LIMIT 10;"

# All critical alerts
sqlite3 health_data.db "SELECT * FROM alerts WHERE severity >= 3;"

# Today's summary
sqlite3 health_data.db "SELECT * FROM daily_summary ORDER BY date DESC LIMIT 1;"

# Activity breakdown
sqlite3 health_data.db "SELECT state, SUM(duration_s)/3600.0 as hours FROM activity GROUP BY state;"
```

---

## Alert Types and Severity

| Alert | Trigger | Severity | Source |
|-------|---------|----------|--------|
| FALL_DETECTED | TI: rapid height drop >0.5m, confirmed at floor level for 3s | 3 (critical) | IWR6843 point cloud |
| FALL_DETECTED | MR60BHA2 on-chip fall flag (secondary confirmation) | 3 (critical) | MR60BHA2 firmware |
| BED_EXIT | Sleep status transitions from sleep → awake | 2 (warning) | MR60BHA2 sleep status |
| VITAL_ANOMALY | HR or BR deviates >2.5σ from patient baseline | 2-3 | MR60BHA2 vitals + HealthTracker |
| PROLONGED_INACTIVITY | <0.05 movement energy for >30 min while not in bed | 2 (warning) | IWR6843 activity + HealthTracker |
| ROOM_EMPTY | No presence detected for configurable duration | 1 (info) | IWR6843 + MR60BHA2 |
| BATHROOM_LONG | LD2450 zone occupied >20 min | 2 (warning) | LD2450 + HealthTracker |

---

## Integration Points for Sam / External Systems

The `handle_alert()` function in main.py is the hook point for integrating with external systems:

```python
def handle_alert(alert: Alert):
    # Currently: logs to SQLite + prints to terminal
    # Add your integrations here:

    # 1. Sam voice alert to the senior
    # sam_voice.speak(f"Alert: {alert.message}")

    # 2. Push notification to family companion app
    # api.post("/alerts", json=alert.to_dict())

    # 3. Nurse call system integration
    # if alert.severity >= 3:
    #     nurse_call.trigger(room_id, alert)

    # 4. Daily summary → cloud LLM → care report
    # summary = tracker.get_daily_summary()
    # report = llm.generate_care_report(summary)
```

---

## Power Budget

| Component | Draw | Notes |
|-----------|------|-------|
| Raspberry Pi 5 (8GB) | ~5-8W | Under load with continuous processing |
| TI IWR6843 + MMWAVEICBOOST | ~2-3W | Separate 5V/2.5A supply |
| Seeed MR60BHA2 | ~0.5W | Powered from Pi GPIO 5V |
| HLK-LD2450 | ~0.3W | Powered through USB-UART adapter |
| **Total system** | **~10-12W** | Two wall outlets needed (Pi PSU + radar PSU) |

For deployment in a safety-critical environment, consider adding a UPS HAT or battery backup (~$25-40) to keep the system running through power outages.

---

## Troubleshooting

**No /dev/ttyACM* ports when MMWAVEICBOOST is connected:**
- Check that USB1 (data port) is connected, not just USB2
- Check that 5V power is connected to USB2
- Verify SOP2 jumper is installed (functional mode, not flash mode)
- Try `dmesg | tail -20` after plugging in to see USB enumeration

**MR60BHA2 connected but no data:**
- Verify TX/RX crossover — sensor TX → Pi RXD (Pin 10), sensor RX → Pi TXD (Pin 8)
- Confirm UART is enabled: `ls -la /dev/serial0` should point to /dev/ttyAMA0
- Check voltage: the MR60BHA2 needs 5V, not 3.3V
- Try reading raw bytes: `python3 -c "import serial; s=serial.Serial('/dev/ttyAMA0',115200,timeout=5); print(s.read(200).hex())"`

**TI radar starts but point cloud is empty:**
- The chirp config may not match your firmware version — check CLI response for errors
- Try `sensorStop` then `sensorStart` manually via the CLI port
- Distance may be outside the configured range (check `cfarFovCfg` bounds in the config)

**Heart rate reads 0 or jumps wildly:**
- Person must be within 1.5m of MR60BHA2 and relatively still
- Takes 15-30 seconds of stillness for the vitals DSP to lock on
- Movement (walking, gesturing) disrupts vitals extraction — this is normal

**Fall detection false positives:**
- Adjust height thresholds in `_classify_activity()` for your ceiling height
- The `z_drop > 0.5` threshold in `_run_fall_detection()` may need tuning
- Pets, falling objects, and quick sitting can trigger — the 3-second confirmation window helps but isn't perfect
- For the MR60BHA2 on-chip fall detection, sensitivity cannot be tuned (firmware fixed)
