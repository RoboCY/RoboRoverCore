# RoboRover Core (v2.1) — Kypruino/Robo Core+-Powered Educational Robot Base

RoboRover Core is an educational, Arduino-compatible robot platform built around **Kypruino/Robo Core+ UNO+ (v0.8.0+)** and a **RoboRover Core PCB chassis (v2.1)**. It combines motors, sensing, feedback, power, and expansion into a compact, hackable robot base for STEM learning, university labs, and maker projects.

**v2.1** keeps the full v1.1 project set and pinout philosophy, but upgrades the sensing and power platform:
- **Line sensing moves to a dedicated 12-bit I²C ADC** (TLA2528) and grows from **3 → 5 reflectance sensors**, freeing the controller's `A0/A1/A2` analog pins.
- **Adds a 3-axis accelerometer** (LIS2DH12) for tilt/motion sensing.
- **Adds a battery fuel gauge** (BQ27441) for true state-of-charge, voltage, and current.
- Dedicated 3.3 V rail, on-board level-shifting, and a load switch for cleaner power.

This repository contains:
- **Verified pinout & hardware definition (RoboRover Core v2.1)**
- **10+ example projects** (Arduino `.ino` sketches) that progressively teach robotics fundamentals — the same lessons as v1.1, adapted to the v2.1 hardware

---

## Highlights

- **Drive:** 2× N20 micro metal gear motors + dual H-bridge (DRV8836 in PHASE/ENABLE mode)
- **Sensing:** Ultrasonic distance, IR obstacle sensors, **5× line sensors (via I²C ADC)**, 2× LDR light sensors, IR remote receiver, wheel encoders, **3-axis accelerometer (new)**
- **Feedback:** 0.91" I²C OLED, NeoPixels, buzzer, pushbuttons (Kypruino/Robo Core+ onboard)
- **Power:** Single 18650 Li-ion with **USB-C** charging & protection on-board, plus a **battery fuel gauge (new)**
- **Expandability:** Breadboard/prototyping area + I²C/UART/GPIO access + **3 spare ADC analog inputs (new)**

---

## Getting Started

### 1) Requirements
- Arduino IDE **or** PlatformIO
- Board selection (Arduino IDE): **Arduino UNO** (Kypruino/Robo Core+ is UNO-compatible)
- USB-C data cable

### 2) Install libraries (Arduino Library Manager)
Depending on the projects you compile, you may need:
- `Wire` (built-in)
- `Adafruit GFX Library`
- `Adafruit SSD1306`
- `Adafruit NeoPixel`
- `IRremote` (TinyIRReceiver is used in the IR remote project)
- **New in v2.1:**
  - A **TLA2528** ADC driver for the line sensors (use the RoboRover helper library, or a TI TLA2528 / generic I²C ADC library)
  - A **LIS2DH12** accelerometer library (e.g., `SparkFun LIS2DH12` or `Adafruit LIS2DH`)
  - A **BQ27441** fuel-gauge library (e.g., `SparkFun BQ27441 LiPo Fuel Gauge`)

> Tip: the simplest path is to use the **RoboRover Core v2.1 support library**, which wraps the TLA2528 line-sensor reads, the accelerometer, and the fuel gauge behind friendly calls.

### 3) Upload a first sketch
Start with:
- `projects/01_HelloRobot/01_HelloRobot.ino`

### 4) Open the interactive manual (tablet or PC)
Use RoboRover Lab Explorer for onboarding, reference, and project navigation:
- https://roborovercore.apps.robo.com.cy/

---

## Repo Structure (recommended)

```text
.
├── README.md
├── docs
│   ├── RoboRover_v2.1_Pinout_Hardware_Definition.md
│   └── (optional) schematics, board renders, BOM notes
└── projects
    ├── 01_SystemIgnition_V2
    ├── 02_CyberAudioVisuals_V2
    ├── 03_MissionControlDashboard_V2
    ├── 04_ReflexiveAutonomy_V2
    ├── 05_PrecisionDistanceLock_V2
    ├── 06_PhototropicNavigation_V2
    ├── 07_TacticalTeleoperation_V2
    ├── 08_CommandFusionHub_V2
    ├── 09_HybridLineFollower_V2
    └── 10_ChromaTiltSphere_V2
```

---

## RoboRover Core v2.1 — Pinout (Verified)

> **What changed from v1.1:** Line sensors are **no longer** on `A0/A1/A2`. They are now read over I²C from the **TLA2528 12-bit ADC** (and there are 5 of them). Everything else on the controller side is the same. `A0/A1/A2` are now free for your own use.

### Motors (DRV8836, PHASE/ENABLE mode)
| Function | Pin |
|---|---|
| Left motor PWM (ENABLE) | `D11` |
| Right motor PWM (ENABLE) | `D10` |
| Left motor DIR (PHASE) | `PCF8574 P0` (I²C `0x20`) |
| Right motor DIR (PHASE) | `PCF8574 P1` (I²C `0x20`) |

**Direction mapping used by the example code**
- Forward: `P0=0, P1=0`
- Backward: `P0=1, P1=1`
- Spin Right: `P0=1, P1=0`
- Spin Left: `P0=0, P1=1`

> Motors are unchanged from v1.1: speed is PWM on `D11`/`D10`, direction is a bit on the PCF8574, MODE/nSLEEP are fixed on-board (always PH/EN mode, always awake).

### Ultrasonic (HC-SR04 class)
- `TRIG = D4`
- `ECHO = D5`

### Line Sensors (5×, via I²C ADC — **changed in v2.1**)
Read from the **TLA2528 8-channel, 12-bit I²C ADC**. The five reflectance sensors plug in through the **daughter IR board**.

| Sensor (position) | ADC channel |
|---|---|
| Sensor 0 (far left)\* | `TLA2528 AIN0` |
| Sensor 1 (left)\* | `TLA2528 AIN1` |
| Sensor 2 (center)\* | `TLA2528 AIN2` |
| Sensor 3 (right)\* | `TLA2528 AIN3` |
| Sensor 4 (far right)\* | `TLA2528 AIN4` |

- **TLA2528 ADC address:** set on-board (typical for this family is `0x10` — confirm against the RoboRover v2.1 support library).
- \*Left-to-right physical order should be confirmed on your unit / in Lab Explorer; the channel numbers (AIN0–AIN4) are fixed.
- The same ADC also provides **3 spare analog inputs** — `AIN5/AIN6/AIN7` — broken out on header **H7** for your own sensors.

### Light Sensors (LDR, analog)
- Left: `A6`
- Right: `A7`  
Note: `A6/A7` are **analog-only** on UNO-class MCUs.

### IR Remote Receiver
- `A3` (used as digital input)

### Wheel Encoders
- Right encoder: `D2` (INT0)
- Left encoder: `D3` (INT1)

### I²C Bus + Devices
- SDA: `A4`
- SCL: `A5`
- PCF8574 I/O expander: address `0x20`
- SSD1306 OLED: typical address `0x3C`
- **TLA2528 ADC (line sensors):** address set on-board (typical `0x10` — confirm) — **new in v2.1**
- **LIS2DH12 accelerometer:** address `0x18` or `0x19` (SA0 strap — confirm) — **new in v2.1**
- **BQ27441 fuel gauge:** address `0x55` (fixed) — **new in v2.1**

> The accelerometer and fuel gauge sit on an internal 3.3 V I²C segment that is level-shifted to the 5 V bus on-board. From your code it is the **same `Wire` bus** — just use the addresses above. No special handling needed.

### Obstacle IR Sensors (Digital via PCF8574 — unchanged)
- Left obstacle: `PCF8574 P2`
- Right obstacle: `PCF8574 P3`

**PCF8574 note (important):** it is quasi-bidirectional. For input bits (e.g., `P2/P3`), firmware should keep them written HIGH (“released”) before reading.

**PCF8574 bit map (v2.1)**

| Bit | Function |
|---|---|
| P0 | Left motor direction |
| P1 | Right motor direction |
| P2 | Left obstacle IR (digital input) |
| P3 | Right obstacle IR (digital input) |
| P4–P7 | Free expander I/O (broken out on header H8) |

---

## Pin Sharing Notes (Kypruino/Robo Core+ onboard features vs RoboRover hardware)

Kypruino/Robo Core+ UNO+ includes onboard buttons/LEDs/buzzer/NeoPixels on fixed pins. Some pins can overlap with robot functions depending on how you use the system. Practical guidance:

- If you use **Ultrasonic** (`D4/D5`), avoid using Kypruino/Robo Core+ button functions that rely on `D4`.
- If you use **Encoders** (`D2/D3`), avoid using Kypruino/Robo Core+ button functions that rely on `D2`.
- NeoPixels (`D8`) and buzzer (`D9`) are dedicated onboard peripherals (used in multiple demo projects).
- **New in v2.1 — good news:** because line sensing moved to the I²C ADC, the analog pins `A0/A1/A2` are **free** for your own analog sensors/experiments.

---

## Example Projects

All projects are written for Arduino IDE / PlatformIO and target Kypruino/Robo Core+ UNO+ + RoboRover Core v2.1. They mirror the v1.1 lessons; the projects that touch line sensing now read the **I²C ADC** instead of `A0/A1/A2`.

### 01 — System Ignition
**File:** `01_SystemIgnition_V2.ino`  
A first motor test: forward, stop, backward, stop, spin right/left.  
Concepts: PWM speed control, direction via I²C expander, timing-based motion.  
*v2.1: identical to v1.1 — motors are unchanged.*

### 02 — Cyber Audio-Visuals
**File:** `02_CyberAudioVisuals_V2.ino`  
Press Kypruino/Robo Core+ onboard buttons to trigger LED + buzzer animations (police, chase, random, rainbow).  
Concepts: digital inputs with pullups, simple UI triggers, NeoPixel basics, tones.  
*v2.1: identical to v1.1.*

### 03 — Mission Control Dashboard
**File:** `03_MissionControlDashboard_V2.ino`  
Live “graph dashboard” on the OLED + Serial Monitor output: ultrasonic distance, line sensors, LDR bars, obstacle states.  
Concepts: I²C peripherals, visualization, sensor sampling, simple telemetry.  
*v2.1: line bars now read the 5-channel TLA2528 ADC; you can also add **battery %** (BQ27441) and a **tilt indicator** (LIS2DH12) to the dashboard.*

### 04 — Reflexive Autonomy
**File:** `04_ReflexiveAutonomy_V2.ino`  
Drives forward until an obstacle is detected (left has priority), then reverses and spins away.  
Concepts: reactive autonomy, digital sensing via expander, behavior rules.  
*v2.1: identical to v1.1 — obstacle IR is still on PCF8574 `P2/P3`.*

### 05 — Precision Distance Lock
**File:** `05_PrecisionDistanceLock_V2.ino`  
Maintains a target distance using deadband control: forward/back/stop to hold spacing.  
Concepts: time-of-flight measurement, threshold + deadband, simple closed-loop behavior.  
*v2.1: identical to v1.1.*

### 06 — Phototropic Navigation
**File:** `06_PhototropicNavigation_V2.ino`  
Spins to face the brighter light source using two LDRs (left/right).  
Concepts: analog sensing, differential control, deadband, proportional intuition.  
*v2.1: identical to v1.1 — LDRs still on `A6/A7`.*

### 07 — Tactical Teleoperation
**File:** `07_TacticalTeleoperation_V2.ino`  
Drive RoboRover with a standard IR remote. Arrow keys move while held; number keys set LED modes; beeps for feedback.  
Concepts: IR decoding, command mapping, stateful motor commands, UI control without a PC.  
*v2.1: identical to v1.1 — IR receiver still on `A3`.*

### 08 — Command Fusion Hub
**File:** `08_CommandFusionHub_V2.ino`

### 09 — Hybrid Line Follower
**File:** `09_HybridLineFollower_V2.ino`

### 10 — Chroma Tilt Sphere
**File:** `10_ChromaTiltSphere_V2.ino`

---

## RoboRover Lab Explorer (Interactive Web Manual)

RoboRover Lab Explorer is the official interactive web manual for RoboRover Core. It provides guided onboarding, pin mappings, live reference material, and project navigation.

**Important:** Lab Explorer is designed for **tablet or PC** (recommended). Open it in a modern browser:

- https://roborovercore.apps.robo.com.cy/

## Troubleshooting

- **OLED not found:** confirm I²C address (`0x3C` typical), wiring, and that no other sketch is holding the bus.
- **Line sensors read zero / not responding (v2.1):** confirm the **daughter IR board** is seated in `CN7`/`CN8`, and that the **TLA2528 ADC address** in your code matches the board (typical `0x10` — confirm). Unlike v1.1, line sensors are **not** on `A0/A1/A2` anymore.
- **Accelerometer/fuel gauge not detected (v2.1):** run an I²C scanner; expect the OLED (`0x3C`), PCF8574 (`0x20`), TLA2528 (≈`0x10`), LIS2DH12 (`0x18`/`0x19`), and BQ27441 (`0x55`).
- **Obstacle sensors inverted:** some modules output inverted logic. Add a software invert flag (or flip the condition).
- **IR remote issues:** confirm you are using a common NEC remote and the IR receiver is on `A3`.
- **Motors swapped:** if your robot turns the wrong way, swap left/right in software (or swap motor connectors) and keep the pinout consistent.

---

## Migrating a v1.1 sketch to v2.1

In most projects the only change is **how line sensors are read**:

```text
v1.1:   int left   = analogRead(A0);
        int center = analogRead(A1);
        int right  = analogRead(A2);

v2.1:   // read 5 channels from the TLA2528 over I2C (via the support library)
        int s0 = roverReadLine(0);   // AIN0
        int s1 = roverReadLine(1);   // AIN1
        int s2 = roverReadLine(2);   // AIN2
        int s3 = roverReadLine(3);   // AIN3
        int s4 = roverReadLine(4);   // AIN4
```

Everything else — motors (`D11`/`D10` + PCF8574 `P0`/`P1`), ultrasonic (`D4`/`D5`), encoders (`D2`/`D3`), LDRs (`A6`/`A7`), IR remote (`A3`), obstacle IR (PCF8574 `P2`/`P3`), OLED (`0x3C`) — is unchanged.

---

## Contributing

PRs are welcome, especially for:
- Additional lessons (PID speed control, odometry, wall following, sensor fusion, accelerometer-assisted driving, battery-aware behaviors)

Please keep changes:
- UNO-compatible (ATmega328P class constraints)
- Well-commented for educational use
- Hardware-accurate for RoboRover Core v2.1

---


---

## Links / Support

- For workshops, education programs, and support: `support@robo.com.cy`
- For issues/bugs: open a GitHub Issue in this repo
