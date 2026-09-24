# RoboRoverCore3

Host-side Arduino library for RoboRover V3. Runs on the Kypruino (ATmega328P)
and talks to everything on the board. Needs Motion firmware 1.15 and Sensors
1.10 for everything below; the full API is in the RoboRoverCore3 API
reference.

## Why it is layered

```
RoboRoverCore3Bus          raw I2C transport, error handling, bus recovery
  ├── RoboRoverCore3Motion   typed access to the Motion coprocessor  (0x28)
  ├── RoboRoverCore3Sensors  typed access to the Sensors coprocessor (0x29)
  ├── RoboRoverCore3Accel    LIS2DH12    (0x19)
  ├── RoboRoverCore3Battery  BQ27441     (0x55)
  └── RoboRoverCore3LineArray TLA2528    (0x14)
RoboRoverCore3Buzzer       tone() on D9

RoboRoverCore3             student-facing facade: motion, sensors and the
                           line sensors (rover.line) in one object
```

All levels are public. The facade is what a first sketch should use; the
lower layers are what bring-up needs, and removing them would make the
diagnostic sketches impossible to write.

## Minimal use

```cpp
#include <RoboRoverCore3.h>

RoboRoverCore3 rover;

void setup() {
    rover.begin();                  // finds both chips, power-up settings
}

void loop() {
    rover.poll();                   // one consistent snapshot
    if (rover.sensors.hasEcho() && rover.distanceMm() < 200) rover.stop();
    else                                                     rover.setSpeed(800, 800);
    delay(20);
}
```

## Examples

One feature each, as in the V2 library. Nothing moves in the readers; the
driving examples turn the wheels.

| Example | Shows |
|---------|-------|
| `BasicMovement` | drive a distance, turn an angle, drive at a speed and stop |
| `EncoderMonitor` | wheel counts - turn a wheel by hand |
| `LineSensorReader` | the five line sensors (`rover.line`) |
| `ObstacleStop` | drive forward, stop for an obstacle or anything close ahead |
| `UltrasonicReader` | distance ahead |
| `LightSensorReader` | the two light sensors |
| `BatteryAndAccelMonitor` | battery voltage and charge, accelerometer |
| `LEDs` | the eight RGB LEDs (needs Adafruit NeoPixel) |
| `IRRemoteDriving` | drive with the remote's arrows (needs IRremote) |
| `TurnCalibration` | tune 90 degree turns for your rover and floor, and keep the result (needs IRremote) |

## Things that surprise people

**`begin()` resets both chips to their power-up settings.** Neither chip
resets with the Arduino, so without this a sketch would inherit the last
one's speed, settings or faults. Change settings after `begin()`, not before.
The one exception is a track width saved with `saveTrackWidth()`: `begin()`
puts it back.

**Turns are calibrated once per rover.** How far a spin goes depends on the
tyres and the floor. `TurnCalibration` tunes the track width with the remote
and saves it in the Arduino's EEPROM, so every sketch turns true without
changes. The value stays with the Arduino board, not the rover. AVR only.

**`setSpeed()` drives straight.** The chip keeps the wheels in step at the
ratio of the two speeds. Calling it again with the same speeds keeps that
going; new speeds start afresh, so steering from a sketch is never fought.
`drive()` is raw power with no correction; it switches the chip to raw
power itself, ending any move.

**A command holds until you change it.** `setSpeed(800, 800); delay(2000);
stop();` drives for two seconds.

**`stop()` brakes.** The drive is slow decay, so a zero command shorts the
motor. For wheels that turn freely in your fingers use `motion.idle()`, which
drops `nSLEEP` and puts the drivers to sleep.

**`poll()` is a snapshot, not a stream.** One transaction latches the whole
telemetry block, so two readings from the same `poll()` always come from the
same instant. A sketch that polls once a second loses freshness and nothing
else — the coprocessor counts encoder edges on interrupts, so you cannot miss a
pulse by being slow. The line sensors are not in it: call `rover.line.poll()`.

**Start with nothing in front.** `begin()` measures what the obstacle
sensors see of the floor and sets the thresholds from that, so anything in
front at that moment counts as floor.

**The bus is 100 kHz on purpose.** `begin()` sets it. On V2.1 motor switching
noise corrupted 400 kHz transfers badly enough to hang the robot. Anything else
you put on the bus must respect that — U8g2 in particular defaults to 400 kHz
and needs `setBusClock(100000)`.

**Reads must start at a telemetry block base.** The firmware latches the block
on a read from its base address; a mid-block read returns stale data by design.
The typed accessors handle this — raw `read8()` on a telemetry register does
not.

## Optional dependencies

The library itself has none. Individual sketches may need:

| Library | Used by |
|---------|---------|
| `Adafruit_NeoPixel` | anything driving the WS2813C chain on D8 |
| `TinyIRReceiver` (IRremote) | anything using the IR handset |
| `U8g2` | anything using the 0.91" OLED |

Everything comes with `#include <RoboRoverCore3.h>`, including the parts the Arduino
drives itself. Those are deliberately dependency-free — the LIS2DH12 and
BQ27441 drivers are written directly rather than pulling in vendor libraries,
because either would cost more flash than a whole test sketch has spare.

## Error codes

`begin()` returns `RRError`:

| Code | Meaning | Usual cause |
|------|---------|-------------|
| `RR_OK` | fine | |
| `RR_ERR_NACK` | nothing at that address | rover switched off, or SJ3 open — no I2C pull-ups |
| `RR_ERR_TIMEOUT` | bus hung | noise, or a device holding SDA |
| `RR_ERR_SHORT_READ` | truncated reply | |
| `RR_ERR_WHOAMI` | wrong device answered | address clash |
| `RR_ERR_PROTOCOL` | firmware built against a different register map | one chip reflashed, the other not |

`RR_ERR_PROTOCOL` is the one worth reading carefully: both chips must be
reflashed together whenever `RR_PROTOCOL_VERSION` changes.

## Pin definitions

Host pins are defined in `RoboRoverCore3.h`. The rover's pinout and I2C
addresses are in the [V3 README](../../README.md#pinout).
