# RoboRover Core V3.1 — Arduino Sketches

RoboRover Core V3.1 is the redesigned RoboRover: a four-layer PCB chassis, a
**Kypruino** (Arduino UNO-compatible) main controller, and **two CH32
coprocessors** that take care of the motors and the sensors, so your sketch
only has to decide what the robot does.

This folder has everything you need to program it:

- **12 example projects** (`01_…_V3` to `12_…_V3`), from blinking lights to
  a PID line follower and driving from your phone
- **`libraries/RoboRoverCore3`**, the Arduino library every V3 sketch uses
- **`WiFiModule_V3`**, the firmware of the Wi-Fi module, in case it ever
  needs reflashing

> Sketches in this folder are for **V3.1 hardware only**. V1 and V2 sketches
> do not run on V3.1, and V3 sketches do not run on older rovers.

---

## What's new in V3.1

| | What changed | What it means for your code |
|---|---|---|
| **Two CH32 coprocessors** | One runs the motors and wheel encoders, one runs the ultrasonic, light and IR obstacle sensors | No more missed encoder pulses or blocking `pulseIn()`. You ask for a speed or a distance and the coprocessor holds it |
| **Closed-loop driving** | Wheel speed is controlled 200 times a second from the encoders | `rover.setSpeed()` keeps both wheels in step, so the rover drives straight. `rover.forward(300)` drives 30 cm, `rover.turnRight(90)` turns 90° |
| **Sunlight-proof IR obstacle sensing** | The IR emitters are modulated and read by a phototransistor | Obstacle detection keeps working outdoors. The sensors calibrate to the floor at start-up |
| **Twin DRV8837 motor drivers** | One driver right at each motor, 6 V boost supply, 220 µF per motor | Full N20 motor performance and less electrical noise |
| **Dual-path power** | Separate power for the electronics and the motors | Heavy motor loads don't disturb the logic side |
| **Four-layer PCB** | Cleaner signal and power distribution | — |
| **Top NeoPixels** | Two extra LEDs on top of the rover | Status you can see from above |
| **RoboBlocks connector** | 6-pin JST connector | Plug in RoboBlocks modules |
| **Bottom charging pads** | For the future stackable charging station | — |
| **Acrylic cover** | Protects the electronics, keeps them visible | — |

Still on board from V2: 5-sensor line array, accelerometer, battery fuel
gauge, 0.91″ OLED, buzzer, IR remote receiver, two light sensors, Wi-Fi
module, 18650 battery with USB-C charging.

---

## Getting started

### 1. Arduino IDE

Arduino IDE 2.x (or PlatformIO). Board: **Tools → Board → Arduino AVR Boards →
Arduino Uno** — the Kypruino is UNO-compatible.

### 2. Libraries

1. **RoboRoverCore3** — copy the folder [`libraries/RoboRoverCore3`](libraries/RoboRoverCore3/)
   into your Arduino `libraries` folder (usually `Documents/Arduino/libraries`)
   and restart the IDE.
2. From the Library Manager (*Sketch → Include Library → Manage Libraries*):
   - `Adafruit NeoPixel` — the RGB lights
   - `IRremote` — the IR remote
   - `U8g2` — the OLED screen

### 3. Upload

Plug the USB cable into the **Kypruino's** USB port and upload. Then unplug
it and run the rover from its battery.

> **While a cable is plugged into the rover's own charging port, the rover is
> switched off.** That is on purpose. Charge first, then drive.

### 4. Start with the projects that don't move

Open `01_LEDs_V3/01_LEDs_V3.ino`, upload, and work your way up the list.

---

## Projects

| # | Project | What it does | Moves? |
|---|---------|--------------|--------|
| 01 | [LEDs](01_LEDs_V3/) | Colours on the eight RGB lights | no |
| 02 | [Audio](02_Audio_V3/) | Beeps and a short tune on the buzzer | no |
| 03 | [MissionControlDashboard](03_MissionControlDashboard_V3/) | Every sensor on the screen; lights, beeps and driving on the remote | only with the arrows |
| 04 | [BasicMovement](04_BasicMovement_V3/) | A square by distance and angle, then a weave, spins and pivots by speed | **yes** |
| 05 | [PrecisionDistanceLock](05_PrecisionDistanceLock_V3/) | Drives forwards and backwards to stay 10 cm from your hand | **yes** |
| 06 | [TacticalTeleoperation](06_TacticalTeleoperation_V3/) | Drive with the IR remote, change the lights | **yes** |
| 07 | [ObstacleDetection](07_ObstacleDetection_V3/) | Drives forwards, stops when something is in the way | **yes** |
| 08 | CommandFusionHub | Wi-Fi project — coming soon | — |
| 09 | [HybridLineFollower](09_HybridLineFollower_V3/) | Fast PD line following; tune Kp and Kd with the remote arrows | **yes** |
| 10 | [ChromaTiltSphere](10_ChromaTiltSphere_V3/) | Tilt the rover to roll a ball around the screen | no |
| 11 | [WiFiDashboard](11_WiFiDashboard_V3/) | Drive from your phone over Wi-Fi, and program what its buttons do | only from the phone |
| 12 | [UnboxingDemo](12_UnboxingDemo_V3/) | The program the rover ships with: drive with the IR remote or a phone | only when you drive it |

The numbers match the V1 and V2 projects, so the same lesson has the same
number on every rover.

The library has ten more short examples, one feature each (*File → Examples →
RoboRoverCore3*): BasicMovement, EncoderMonitor, LineSensorReader,
ObstacleStop, UltrasonicReader, LightSensorReader, BatteryAndAccelMonitor,
LEDs, IRRemoteDriving and TurnCalibration.

---

## Out of the box: the Unboxing Demo

Every rover ships running [12_UnboxingDemo_V3](12_UnboxingDemo_V3/). Switch it
on and drive it straight away — with the IR remote, a phone, or both at once.
Both can do the same things.

### From your phone

1. Join the Wi-Fi network **`RoboRover-XXXX`** (`RoboRover-` and four
   letters or digits, different for every rover).
2. Open **`http://192.168.4.1`** in Chrome (Android) or Safari (iPhone), and
   turn the phone sideways.
3. Drive with the half-circle joystick.

| On the phone | What it does |
|---|---|
| Joystick | Up drives, along the flat edge spins on the spot. The **green strip** in the middle drives dead straight — the wheel encoders keep both wheels in step |
| Reverse | Pushing up then drives backwards |
| Avoid | The rover won't drive forwards into an obstacle (IR sensors, or ultrasonic under 15 cm) |
| Speed slider | Top speed of the joystick |
| Speed · distance (top bar) | Live speed in cm/s and distance driven, from the encoders. Tap it to reset the distance |
| Lights, Beep, Dance, Custom A, Custom B | Run on the rover — Custom A says the distance ahead, Custom B the battery level |
| Piano | Turns the buttons into a keyboard for the rover's buzzer |
| STOP | Stops everything |
| Console (`>_`) | Type words to the rover — try `help` |
| Gamepad (switch it on in the Console) | Left stick drives, pulled back it reverses. B = STOP, A = Beep, X = Lights, Y = Dance |
| Tilt drive (Console) | Hold the joystick area and tilt the phone. Browsers only allow tilt sensors on `https` pages, so on the rover's own page Android Chrome needs a one-time setting: `chrome://flags` → *Insecure origins treated as secure* → add `http://192.168.4.1` |

The rover stops by itself when you press STOP, when the phone disconnects,
and when the joystick goes quiet for 0.8 s.

### From the IR remote

| Button | What it does | | Button | What it does |
|---|---|---|---|---|
| ↑ / ↓ (hold) | Drive forwards / backwards, dead straight | | 6 | Custom B (battery level) |
| ← / → (hold) | Spin on the spot | | 7 | Avoid on / off (high beep = on) |
| OK | STOP everything | | 8 | Say the distance driven |
| 1 | Lights (next colour) | | 9 | Piano: buttons 1–8 play notes, 9 ends |
| 2 | Beep | | 0 | Reset the distance driven |
| 3 | Dance | | ✱ / # | Slower / faster |
| 4 | Custom A (distance ahead) | | | |
| 5 | Tune | | | |

While Dance or Tune is running, any button stops it. The rover's screen shows
what it is doing, the distance ahead, the speed and the battery.

**Want to change what the buttons do?** Open
[11_WiFiDashboard_V3](11_WiFiDashboard_V3/) — its `Kypruino_Rover.ino` is the
same program without the IR remote, written for you to edit.

---

## Programming with RoboRoverCore3

```cpp
#include <RoboRoverCore3.h>

RoboRoverCore3 rover;

void setup() {
  rover.begin();              // finds both coprocessors, stops the wheels
  rover.forwardAndWait(300);  // 30 cm forwards, measured by the encoders
  rover.turnRightAndWait(90); // a 90-degree turn on the spot
}

void loop() {
  rover.poll();               // fresh readings from both coprocessors
  if (rover.sensors.hasEcho() && rover.distanceMm() < 150) rover.stop();
}
```

Three ways to drive — pick one:

| Call | You give | The rover |
|---|---|---|
| `rover.forward(mm)`, `rover.turnRight(deg)` | a distance or an angle | drives exactly that far and stops |
| `rover.setSpeed(left, right)` | wheel speeds in encoder ticks per second | holds those speeds and keeps the wheels in step (equal speeds = straight) |
| `rover.drive(left, right)` | raw motor power, −1000 to 1000 | just applies it |

A few things worth knowing:

- **Always call `rover.begin()` first.** The motor coprocessor keeps its last
  command when you upload a new sketch; `begin()` stops the wheels and puts
  everything back to its start-up settings.
- **A drive command holds until you change it.** `rover.setSpeed(1500, 1500);`
  keeps driving until `rover.stop()`.
- **Turns not quite 90°?** Run the library example *TurnCalibration*: it tunes
  the turn for your rover and floor with the remote and saves it, so every
  sketch turns true afterwards.
- **Obstacle sensors seeing things on a light floor?** They calibrate to the
  floor at start-up; call `rover.sensors.calibrateObstacles()` after moving
  to a different floor.

The full API is in [libraries/RoboRoverCore3/README.md](libraries/RoboRoverCore3/README.md).

---

## Pinout

On V3.1 the motors, encoders, ultrasonic, light and obstacle sensors are
behind the two coprocessors — you reach them through the library, not
through pins.

### Kypruino pins

| Pin | Used for | Notes |
|-----|----------|-------|
| `A4` / `A5` | I²C SDA / SCL | Shared by everything below, **100 kHz** |
| `A3` | IR remote receiver | |
| `D8` | NeoPixels | 8 LEDs in one chain |
| `D9` | Buzzer | `tone()` |
| `D12` / `D13` | Wi-Fi module serial | 19200 baud |
| `D3` | Motion coprocessor interrupt | |
| `D2` | Sensors coprocessor interrupt | **Don't press the Kypruino's D2 button** — it shorts this line |
| `A0`–`A2` | Free | Your own sensors |

### I²C addresses

| Address | Device |
|---------|--------|
| `0x14` | Line sensor array (TLA2528, 5 sensors — channel 0 is the rightmost) |
| `0x19` | Accelerometer (LIS2DH12) |
| `0x28` | Motion coprocessor (motors, encoders) |
| `0x29` | Sensors coprocessor (ultrasonic, light, IR obstacle) |
| `0x55` | Battery fuel gauge (BQ27441) |
| `0x3C` | OLED screen |

> Keep the I²C bus at 100 kHz. With U8g2, call `screen.setBusClock(100000)`
> after `begin()` — its default of 400 kHz is too fast for the rover.

---

## Troubleshooting

- **Nothing moves after uploading:** unplug the rover's charging cable —
  the rover is switched off while charging. Run it from the battery.
- **`rover.begin()` returns an error:** switch the rover off and on again,
  then upload once more. Check the battery is charged.
- **The obstacle sensors see "something" everywhere:** they measured a
  different floor at start-up. Switch the rover on where it will drive, or call
  `rover.sensors.calibrateObstacles()` (the Wi-Fi dashboard's Console word
  `calibrate` does it). Dark objects are invisible to IR sensors.
- **The phone finds no `RoboRover-XXXX` network, or `192.168.4.1` won't open:**
  check the phone is still joined to it (some phones switch back to a network
  with internet), then try Chrome.
  If the Wi-Fi module is blank, reflash it — see [WiFiModule_V3](WiFiModule_V3/).
- **The rover turns a little more or less than 90°:** run the TurnCalibration
  library example.
- **Line follower wobbles or loses the line:** tune Kp and Kd with the remote
  arrows in `09_HybridLineFollower_V3`; the values it starts with are a good
  base at full speed.

---

## Folder layout

```
V3/
├── README.md                      ← you are here
├── 01_LEDs_V3/ … 12_UnboxingDemo_V3/   example projects
├── 11_WiFiDashboard_V3/
│   └── Kypruino_Rover/            the Kypruino side — edit this one
├── WiFiModule_V3/                 Wi-Fi module firmware (ESP-01S), comes pre-installed
└── libraries/
    └── RoboRoverCore3/            the library — copy it to Arduino/libraries
```

---

## Links

- Interactive manual (tablet or PC): https://roborovercore.apps.robo.com.cy/
- Workshops, schools and support: `support@robo.com.cy`
- Bugs and ideas: open an issue in this repository
