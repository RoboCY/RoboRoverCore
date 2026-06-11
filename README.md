# 🤖 RoboRover Core — Arduino Robot Sketches

> A growing collection of Arduino sketches for the **RoboRover Core** educational robot platform. Plug in, upload, and watch a little robot come to life. 🎉

---

## What is this?

**RoboRover Core** is a compact, hackable robot base built for STEM learning, university labs, and anyone who enjoys tinkering with hardware. It pairs a custom **Kypruino/Robo Core+ UNO+** microcontroller board with a purpose-built PCB chassis packed with sensors, NeoPixels, a buzzer, an OLED screen, and two peppy N20 micro-motors.

This repo holds all the example sketches — organised by hardware version — so you can go from *"blink an LED"* to *"autonomous line-following robot"* one project at a time.

---

## Hardware Versions

| | V1 (PCB v1.1) | V2 (PCB v2.1) |
|---|---|---|
| Motors | 2× N20 via DRV8836 | Same |
| Line sensors | 3× analog (A0–A2) | **5× via I²C TLA2528 ADC** |
| Distance | HC-SR04 ultrasonic | Same |
| IR obstacle | 2× sensors | Same |
| Light sensing | 2× LDR | Same |
| IR remote | Yes | Yes |
| OLED display | 0.91″ I²C | Same |
| NeoPixels | Yes | Same |
| Accelerometer | ✗ | **LIS2DH12 (new)** |
| Battery gauge | ✗ | **BQ27441 fuel gauge (new)** |
| Power | 18650 Li-ion + USB-C | Same + 3.3 V rail |

Both versions run on an **Arduino UNO-compatible** toolchain — no exotic setup required.

---

## Project List

### V1 — Getting Started on PCB v1.1

| # | Sketch | What it does |
|---|--------|--------------|
| 01 | `01_HelloRobot` | First motor test — forward, back, spin. Classic. |
| 02 | `02_SoundLightShow` | Buzzer + NeoPixels doing their thing |
| 03 | `03_OLEDStatusDashboard` | Live sensor readings on the OLED |
| 04 | `04_ObstacleAvoidanceIR` | IR sensors make the robot dodge obstacles |
| 05 | `05_TargetDistanceUltrasonic` | Stop exactly N cm from a wall |
| 06 | `06_LightSeeker` | LDR sensors guide the robot toward the light |
| 07 | `07_IRRemoteControl` | Drive the robot with an IR remote |
| 09 | `09_LineFollowerEdge` | Simple edge-detection line following |
| 10 | `10_LineFollower` | Proper PD line follower |
| 11 | `11_LineFollowerEnhanced` | Enhanced line follower with smarter logic |
| 12 | `12_WiFiCommandCenter` | Web-based control panel over Wi-Fi |

### V2 — Levelled Up on PCB v2.1

| # | Sketch | What it does |
|---|--------|--------------|
| 01–07 | *(V1 classics, v2 hardware)* | All the favourites, adapted for v2.1 pins |
| 13 | `13_DashboardWithRemoteControl_2` | OLED dashboard + IR remote combined |
| 14 | `14_HybridLineFollower` | 5-sensor PD line follower using I²C ADC |
| 15 | `15_RainbowBubbleLevelAccel` | Bubble level on OLED + rainbow NeoPixels react to tilt |

---

## Quick Start

### 1. What you need
- Arduino IDE (**2.x recommended**) or PlatformIO
- USB-C data cable
- A RoboRover Core robot 🤖

### 2. Board setup
Select **Arduino UNO** in the board manager — Kypruino/Robo Core+ UNO+ is fully UNO-compatible.

### 3. Install libraries
Open Library Manager and grab these (only install what the sketch needs):

```
Wire                  (built-in)
Adafruit GFX Library
Adafruit SSD1306
Adafruit NeoPixel
IRremote
```

**V2 only — extra libraries:**
```
SparkFun LIS2DH12     (accelerometer)
SparkFun BQ27441      (battery fuel gauge)
```
> The RoboRover Core v2.1 support library wraps the TLA2528 ADC, accelerometer, and fuel gauge behind friendly one-liners if you'd rather skip the low-level stuff.

### 4. Flash your first sketch
Open `V1/01_HelloRobot/01_HelloRobot.ino`, hit **Upload**, and watch the robot boogie. 🕺

### 5. Interactive manual
The **RoboRover Lab Explorer** is a web app with guided project walk-throughs, pinout references, and onboarding:
👉 [roborovercore.apps.robo.com.cy](https://roborovercore.apps.robo.com.cy/)

---

## Repo Layout

```
RoboRoverCore/
├── README.md          ← you are here
├── V1/
│   ├── README.md      ← full v1.1 pinout & hardware reference
│   ├── 01_HelloRobot/
│   ├── 02_SoundLightShow/
│   └── ...
└── V2/
    ├── README.md      ← full v2.1 pinout & hardware reference
    ├── 01_HelloRobot_2/
    ├── 14_HybridLineFollower/
    └── ...
```

Each sketch folder is self-contained — open the `.ino`, install the listed libraries, and upload. No monorepo magic needed.

---

## Who is this for?

- **Students** learning embedded systems and robotics
- **Educators** running STEM workshops or university lab sessions
- **Makers** who want a pre-wired sensor platform to hack on
- **Anyone** who just wants to make a small robot do cool things

---

## Contributing

Found a bug? Have a sketch idea? Pull requests and issues are welcome. Keep sketches self-contained, comment the hardware assumptions at the top (see existing files for style), and test before you push.

---

## License

Sketches in this repository are released for educational use. See individual sketch headers for any third-party library attributions.

---

*Built with ❤️, ☕, and solder fumes by the [robo.com.cy](https://robo.com.cy) team.*
