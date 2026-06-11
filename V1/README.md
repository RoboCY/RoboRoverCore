# RoboRover Core (v1.1) — Kypruino/Robo Core+-Powered Educational Robot Base

RoboRover Core is an educational, Arduino-compatible robot platform built around **Kypruino/Robo Core+ UNO+ (v0.8.0+)** and a **RoboRover Core PCB chassis (v1.1)**. It combines motors, sensing, feedback, power, and expansion into a compact, hackable robot base for STEM learning, university labs, and maker projects.

This repository contains:
- **Verified pinout & hardware definition (RoboRover Core v1.1)**
- **10+ example projects** (Arduino `.ino` sketches) that progressively teach robotics fundamentals

---

## Highlights

- **Drive:** 2× N20 micro metal gear motors + dual H-bridge (DRV8836 in PHASE/ENABLE mode)
- **Sensing:** Ultrasonic distance, IR obstacle sensors, 3× line sensors, 2× LDR light sensors, IR remote receiver, wheel encoders
- **Feedback:** 0.91" I²C OLED, NeoPixels, buzzer, pushbuttons (Kypruino/Robo Core+ onboard)
- **Power:** Single 18650 Li-ion with **USB-C** charging & protection on-board
- **Expandability:** Breadboard/prototyping area + I²C/UART/GPIO access

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
│   ├── RoboRover_Pinout_Hardware_Definition.md
│   └── (optional) schematics, board renders, BOM notes
└── projects
    ├── 01_HelloRobot
    ├── 02_SoundLightShow
    ├── 03_OLEDStatusDashboard
    ├── 04_ObstacleAvoidanceIR
    ├── 05_TargetDistanceUltrasonic
    ├── 06_LightSeeker
    ├── 07_IRRemoteControl
    ├── 08_LineFollowerEdge
    ├── 09_LineFollower
    ├── 10_LineFollowerEnhanced
    └── 11_WiFiCommandCenter
```

---

## RoboRover Core v1.1 — Pinout (Verified)

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

### Ultrasonic (HC-SR04 class)
- `TRIG = D4`
- `ECHO = D5`

### Line Sensors (Analog reflectance)
- Left: `A0`
- Center: `A1`
- Right: `A2`

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
- PCF8574 I/O expander: typical address `0x20`
- SSD1306 OLED: typical address `0x3C`

### Obstacle IR Sensors (Digital via PCF8574)
- Left obstacle: `PCF8574 P2`
- Right obstacle: `PCF8574 P3`

**PCF8574 note (important):** it is quasi-bidirectional. For input bits (e.g., `P2/P3`), firmware should keep them written HIGH (“released”) before reading.

---

## Pin Sharing Notes (Kypruino/Robo Core+ onboard features vs RoboRover hardware)

Kypruino/Robo Core+ UNO+ includes onboard buttons/LEDs/buzzer/NeoPixels on fixed pins. Some pins can overlap with robot functions depending on how you use the system. Practical guidance:

- If you use **Ultrasonic** (`D4/D5`), avoid using Kypruino/Robo Core+ button functions that rely on `D4`.
- If you use **Encoders** (`D2/D3`), avoid using Kypruino/Robo Core+ button functions that rely on `D2`.
- NeoPixels (`D8`) and buzzer (`D9`) are dedicated onboard peripherals (used in multiple demo projects).

---

## Example Projects

All projects are written for Arduino IDE / PlatformIO and target Kypruino/Robo Core+ UNO+ + RoboRover Core v1.1.

### 01 — Hello Robot! (motor patterns)
**File:** `01_HelloRobot.ino`  
A first motor test: forward, stop, backward, stop, spin right/left.  
Concepts: PWM speed control, direction via I²C expander, timing-based motion.

### 02 — Sound & Light Show (button triggered)
**File:** `02_SoundLightShow.ino`  
Press Kypruino/Robo Core+ onboard buttons to trigger LED + buzzer animations (police, chase, random, rainbow).  
Concepts: digital inputs with pullups, simple UI triggers, NeoPixel basics, tones.

### 03 — OLED Status Dashboard (live sensors)
**File:** `03_OLEDStatusDashboard.ino`  
Live “graph dashboard” on the OLED + Serial Monitor output: ultrasonic distance, line sensors, LDR bars, obstacle states.  
Concepts: I²C peripherals, visualization, sensor sampling, simple telemetry.

### 04 — Obstacle Avoidance (IR obstacle sensors)
**File:** `04_ObstacleAvoidanceIR.ino`  
Drives forward until an obstacle is detected (left has priority), then reverses and spins away.  
Concepts: reactive autonomy, digital sensing via expander, behavior rules.

### 05 — Target Distance Control (ultrasonic)
**File:** `05_TargetDistanceUltrasonic.ino`  
Maintains a target distance using deadband control: forward/back/stop to hold spacing.  
Concepts: time-of-flight measurement, threshold + deadband, simple closed-loop behavior.

### 06 — Light Seeker (LDR differential)
**File:** `06_LightSeeker.ino`  
Spins to face the brighter light source using two LDRs (left/right).  
Concepts: analog sensing, differential control, deadband, proportional intuition.

### 07 — IR Remote Control (NEC)
**File:** `07_IRRemoteControl.ino`  
Drive RoboRover with a standard IR remote. Arrow keys move while held; number keys set LED modes; beeps for feedback.  
Concepts: IR decoding, command mapping, stateful motor commands, UI control without a PC.

### 08 — Line Follower Edge
**File:** `08_LineFollowerEdge.ino`

### 09 — Line Follower (3-sensor state table + calibration)
**File:** `09_LineFollower.ino`  
3-IR line follower that self-calibrates thresholds (spin sampling) then follows using a simple decision table.  
Concepts: calibration, thresholding, state logic, recovery behavior when line is lost.

### 10 — Line Follower Enhanced
**File:** `10_LineFollowerEnhanced.ino`

---

## RoboRover Lab Explorer (Interactive Web Manual)

RoboRover Lab Explorer is the official interactive web manual for RoboRover Core. It provides guided onboarding, pin mappings, live reference material, and project navigation.

**Important:** Lab Explorer is designed for **tablet or PC** (recommended). Open it in a modern browser:

- https://roborovercore.apps.robo.com.cy/

## Troubleshooting

- **OLED not found:** confirm I²C address (`0x3C` typical), wiring, and that no other sketch is holding the bus.
- **Obstacle sensors inverted:** some modules output inverted logic. Add a software invert flag (or flip the condition).
- **IR remote issues:** confirm you are using a common NEC remote and the IR receiver is on `A3`.
- **Motors swapped:** if your robot turns the wrong way, swap left/right in software (or swap motor connectors) and keep the pinout consistent.

---

## Contributing

PRs are welcome, especially for:
- Additional lessons (PID speed control, odometry, wall following, sensor fusion)

Please keep changes:
- UNO-compatible (ATmega328P class constraints)
- Well-commented for educational use
- Hardware-accurate for RoboRover Core v1.1

---


---

## Links / Support

- For workshops, education programs, and support: `support@robo.com.cy`
- For issues/bugs: open a GitHub Issue in this repo
