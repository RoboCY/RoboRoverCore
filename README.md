# Kypruino RoboRover Core — Official Code Repository

Open-source firmware, libraries, and example projects for the **Kypruino UNO+ v0.6.2+** controller and the **RoboRover Core** educational robot base. This repo is designed for fast onboarding (plug, upload, learn) and a clean path from beginner projects to more advanced robotics (encoders, PID, sensor fusion, telemetry).

---

## Table of Contents
- [What this repo includes](#what-this-repo-includes)
- [Hardware overview](#hardware-overview)
- [Quick start](#quick-start)
- [Pin map (Kypruino ↔ RoboRover Core)](#pin-map-kypruino--roborover-core)
- [Example projects](#example-projects)
- [Development notes](#development-notes)
- [Contributing](#contributing)

---

## What this repo includes
- **Drivers / libraries** for RoboRover Core subsystems (motors, encoders, line sensors, ultrasonic, IR, OLED, Wi‑Fi port, I/O expander).
- **Ready-to-run example projects** with extension ideas.
- **Teacher-friendly exercises** that map code → hardware → observable behavior (OLED, LEDs, buzzer).
- **PlatformIO + Arduino IDE compatibility** for a smooth classroom and maker workflow.

---

## Hardware overview

### Kypruino UNO+ v0.6.2+
Arduino UNO R3–compatible board with built-in learning helpers (RGB LEDs / NeoPixels, buzzer, buttons, I²C ports, OLED port, Wi‑Fi module port).

### RoboRover Core Robot Base
A PCB-based robot chassis featuring:
- **2× N20 micro metal gear motors** + dual motor driver
- **Wheel encoders** (for speed/distance feedback)
- **Ultrasonic obstacle avoidance**
- **IR distance / line sensors** + **LDR light sensors**
- **IR remote receiver**
- **0.91" OLED** (I²C)
- **Single 18650 Li‑ion** with **USB‑C charging & protection**
- **Breadboard / prototyping area** + expansion headers

---

## Quick start

### 1) Install toolchain
Choose one:
- **Arduino IDE** (select board: *Arduino UNO* for Kypruino UNO+ compatibility)
- **PlatformIO** (recommended for structured projects and libraries)

### 2) USB driver (if needed)
Kypruino uses a USB‑UART bridge (CP2102). Install the driver if your PC does not detect the board.

### 3) Connect & upload
1. Connect Kypruino to your PC using a **USB‑C data cable**.
2. Open an example under `examples/`.
3. Build & upload.
4. Follow the Serial Monitor / OLED output (depending on the example).

---

## Pin map (Kypruino ↔ RoboRover Core)

> Pin assignments below reflect the intended Kypruino + RoboRover Core wiring and the robot schematic. RoboRover Core uses a **PCF8574 I/O expander** for motor direction control and obstacle sensor inputs.

### Actuators / Outputs
| Function | Pins | Notes |
|---|---|---|
| Motor 1 Speed (PWM) | **D11** | Speed via PWM |
| Motor 1 Direction | **PCF8574 P0** | Direction via I/O expander |
| Motor 2 Speed (PWM) | **D10** | Speed via PWM |
| Motor 2 Direction | **PCF8574 P1** | Direction via I/O expander |
| NeoPixels / RGB LEDs (Kypruino onboard) | **D8** | State indication / animations |
| Buzzer (Kypruino onboard) | **D9** | Tones / alerts |
| OLED Display (0.91") | **A4 (SDA), A5 (SCL)** | I²C display |

### Sensors / Inputs
| Function | Pins | Notes |
|---|---|---|
| Line sensors (L/C/R) | **A0 / A1 / A2** | Analog reflectance |
| Light sensors (L/R LDR) | **A6 / A7** | Light follow/avoid |
| Wheel encoder (Right) | **D2 (INT0)** | Interrupt input |
| Wheel encoder (Left) | **D3 (INT1)** | Interrupt input |
| Ultrasonic TRIG / ECHO | **D4 / D5** | Time-of-flight |
| Obstacle sensors (L/R) | **PCF8574 P2 / P3** | Digital via expander |
| IR remote receiver | **A3 (digital input)** | IR protocol decoding |

### Connectivity
| Function | Pins | Notes |
|---|---|---|
| Wi‑Fi module port (ESP8266‑style) | **D12 / D13** | Remote control + telemetry |

### Important note on shared pins
Some Kypruino onboard features can overlap with robot functions depending on mode/firmware strategy. Examples in this repo avoid conflicts by design, and advanced examples document any remaps/constraints.

---

## Example projects
A practical set of projects maintained in this repo includes:
1. Hello Robot (static motor patterns)
2. Sensors Reading (serial/OLED)
3. Line Follower (basic → PID)
4. Ultrasonic Obstacle Avoidance
5. Light‑Seeker / Light‑Avoider
6. IR Remote‑Controlled Rover
7. Wi‑Fi Tele‑Op (phone/PC control)
8. OLED Status Dashboard (battery, speed, sensors)
9. Edge Guard (table‑edge detection)
10. Sound & Light Show (RGB + buzzer)

---

## Development notes

### Educational design goals
- Make every example **observable**: use OLED / LEDs / buzzer for feedback, not only Serial.
- Keep a consistent API style across modules (motors, sensors, UI).
- Include “extension ideas” per project so students can iterate beyond the baseline.

### Electrical / safety
- RoboRover Core uses a **single 18650 Li‑ion** with **USB‑C charge/protection**; follow standard Li‑ion handling practices and do not short the cell or bypass protection circuitry.

---

## Contributing
Contributions are welcome:
- Bug reports (include board revision, wiring notes, and reproduction steps)
- New example projects with clear learning outcomes
- Driver improvements and performance tuning (encoders, PID, filtering)

Please open an issue first for major changes.
