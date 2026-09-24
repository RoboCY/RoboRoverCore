/* RoboRoverCore3.h - host library for RoboRoverCore3 V3.0
 *
 * Targets ATmega328P (Kypruino) and ESP32-S3 from one source. The AVR sets the
 * limits: 32-byte Wire buffer, no FPU, 2 KB RAM.
 *
 * Three layers, all public. The facade is for students; the lower layers are
 * for bring-up, where you need to see the actual register values.
 *
 *   RoboRoverCore3Bus       raw register transport
 *   RoboRoverCore3Motion    typed access to 0x28
 *   RoboRoverCore3Sensors   typed access to 0x29
 *
 * then the parts the host drives itself: line array, accelerometer, battery
 * gauge and buzzer, and last the facade over the two chips and the line
 * array:
 *
 *   RoboRoverCore3          what a sketch normally uses. One header for
 *                           everything.
 *
 * Requires rr_protocol.h - the SAME file the two CH32 firmwares compile
 * against. Copy it into this folder; do not retype it.
 */

#ifndef ROBOROVERCORE3_H
#define ROBOROVERCORE3_H

#include <Arduino.h>
#include <Wire.h>
#include "rr_protocol.h"

/* Host-direct peripherals - these never touch a coprocessor.
 * All four confirmed against the V3.0 hardware reference (schematic exported
 * 2026-08-19) and against hardware: P7 saw a real remote on A3, and T11/P8
 * see both interrupt lines idle high and assert correctly. */
#define RR_PIN_NEOPIXEL   8   /* FPC7.7,  330R series */
#define RR_NEOPIXEL_COUNT 8   /* WS2813C chain length, LED20 -> LED4 */
#define RR_PIN_BUZZER     9   /* FPC7.8,  buzzer lives on the host board */
#define RR_PIN_IR_RECV   A3   /* FPC7.15, IRM-H638T output */

/* Interrupt lines from the coprocessors, active low, driven by a coprocessor
 * GPIO - never configure these as outputs on the host. */
#define RR_PIN_SENSOR_INT 2   /* FPC7.1 = D2/INT0, from Sensors PD6 */
#define RR_PIN_MOTION_INT 3   /* FPC7.2 = D3/INT1, from Motion PC0 */

enum RRError {
    RR_OK = 0,
    RR_ERR_NACK,        /* device did not acknowledge */
    RR_ERR_TIMEOUT,     /* bus hung; recovery attempted */
    RR_ERR_SHORT_READ,  /* fewer bytes than asked for */
    RR_ERR_WHOAMI,      /* wrong chip at this address */
    RR_ERR_PROTOCOL     /* firmware built against a different register map */
};

/* ---- transport --------------------------------------------------------- */

class RoboRoverCore3Bus {
public:
    explicit RoboRoverCore3Bus(uint8_t addr) : _addr(addr), _err(RR_OK) {}

    /* Call once. clockHz defaults to 100 kHz - the motor rail is noisy and
     * margin is worth more than throughput here. */
    static void begin(uint32_t clockHz = 100000UL);

    /* Drive nine clock pulses on SCL to free a slave that is mid-byte and
     * holding SDA low. Only a master can do this, which is why it lives here
     * and not in the CH32 firmware. Call after a timeout. */
    static void recover(uint8_t sdaPin, uint8_t sclPin);

    bool     write8 (uint8_t reg, uint8_t v);
    bool     write16(uint8_t reg, uint16_t v);   /* low byte then high: the
                                                  * firmware commits on high */
    uint8_t  read8  (uint8_t reg);
    uint16_t read16 (uint8_t reg);
    uint32_t read32 (uint8_t reg);

    /* Checked forms.
     *
     * The plain read8()/read16() above return 0 when the transfer fails,
     * which is indistinguishable from a register that genuinely holds 0.
     * lastError() does say what happened, but nothing forces a caller to
     * look - and a caller who does not gets a plausible wrong answer rather
     * than an error. That is the same ambiguity that made ENC_RAW read as
     * dead encoders on old firmware.
     *
     * These return false instead, so the failure cannot be read as data. */
    bool read8 (uint8_t reg, uint8_t  &out);
    bool read16(uint8_t reg, uint16_t &out);

    /* Reads must start at a telemetry block base or the firmware will not
     * latch and the data may be torn. */
    bool readBlock(uint8_t reg, uint8_t *buf, uint8_t len);

    /* Verify the right chip is present AND speaks our register map version. */
    RRError identify(uint8_t expectWhoAmI);

    RRError lastError() const { return _err; }
    void    clearError()      { _err = RR_OK; }
    uint8_t address() const   { return _addr; }

    bool     present();
    uint8_t  status()  { return read8(RR_REG_STATUS); }
    uint8_t  fault()   { return read8(RR_REG_FAULT);  }
    void     clearFaults() { write8(RR_REG_CONTROL, RR_CTRL_CLEAR_FAULT); }
    uint32_t uptimeMs(){ return read32(RR_REG_UPTIME_MS); }
    void     firmwareVersion(uint8_t &major, uint8_t &minor);

    /* Every setting on this chip back to its power-up value, without a reset.
     * On the Sensors chip that includes recalibrating the obstacle sensors. */
    void     loadDefaults() { write8(RR_REG_CONTROL, RR_CTRL_LOAD_DEFAULTS); }

    /* The chip's status LED: breathing (the default - slow when healthy, fast
     * with a fault) or off. Motion 1.12 / Sensors 1.10 and later. */
    void     setStatusLed(bool on) { write8(RR_REG_STATUS_LED, on ? RR_STATUS_LED_BREATHE : RR_STATUS_LED_OFF); }
    bool     statusLed()           { return read8(RR_REG_STATUS_LED) != RR_STATUS_LED_OFF; }

    /* Which events pull this chip's interrupt line low (RR_PIN_MOTION_INT or
     * RR_PIN_SENSOR_INT): RR_INT_* bits. Flags stay set, and the line low,
     * until cleared. Defaults: Motion FAULT | STALL | MOVE_DONE,
     * Sensors FAULT | OBST_CHANGE. */
    void     setInterruptMask(uint8_t bits)      { write8(RR_REG_INT_CFG, bits); }
    uint8_t  interruptMask()                     { return read8(RR_REG_INT_CFG); }
    uint8_t  interruptFlags()                    { return read8(RR_REG_INT_FLAGS); }
    void     clearInterruptFlags(uint8_t bits = 0xFF) { write8(RR_REG_INT_FLAGS, bits); }

protected:
    uint8_t _addr;
    RRError _err;
};

/* ---- motion, 0x28 ------------------------------------------------------ */

class RoboRoverCore3Motion : public RoboRoverCore3Bus {
public:
    RoboRoverCore3Motion() : RoboRoverCore3Bus(RR_ADDR_MOTION) {}
    RRError begin() { return identify(RR_WHOAMI_MOTION); }

    void setMode(uint8_t mode)   { write8(RR_M_MODE, mode); }
    uint8_t mode()               { return read8(RR_M_MODE); }

    /* In OPEN_LOOP these are permille duty (-1000..1000).
     * In CLOSED_LOOP they are ticks/s. Both motors in one call keeps the two
     * wheels within a few hundred microseconds of each other.
     *
     * ZERO BRAKES, it does not coast. Motion drives the bridge in slow decay,
     * where a zero command lands on both inputs high - the motor shorted. The
     * wheels hold rather than roll. That is usually what you want from a stop
     * and it is what the position loop has always claimed to do, but it is a
     * change from V1.0 firmware. For a genuine freewheel use coast(). */
    void drive(int16_t m1, int16_t m2);
    void stop()  { drive(0, 0); }   /* brakes; see the note above */
    /* The only way to let the wheels freewheel: it releases the bridge
     * entirely rather than commanding zero. */
    void coast() { setMode(RR_MODE_COAST); }
    void brake() { setMode(RR_MODE_BRAKE); }
    void idle()  { setMode(RR_MODE_IDLE); }

    /* Speed controller gains, Q8.8: write RR_Q88(0.10f) so the float never
     * reaches the AVR at runtime. Kp 0-4, Ki 0-1, Kd 0-4. */
    void setPID(int16_t kp, int16_t ki, int16_t kd);

    /* Smooth starts: the most the command may change per second, 200-30000,
     * or 0 (the default) for at once. With drive() that is power, in
     * permille per second - 2000 takes the wheels from stop to full power in
     * half a second. With setSpeed() it is ticks/s per second, an
     * acceleration. Distance moves use setAccel() instead. */
    void setRamp(uint16_t perSecond)   { write16(RR_M_RAMP, perSecond); }
    /* Any non-zero power below this is raised to it, 0-500 permille. */
    void setMinDuty(uint16_t permille) { write16(RR_M_MIN_DUTY, permille); }

    /* Diagnostics for a count that will not move. encoderRaw() is the four
     * encoder pins as they physically sit, before SWAP or INVERT;
     * encoderIrqs() is a rolling tally of edges the coprocessor has
     * actually serviced. Between them they say which of wiring, interrupt
     * or decode is at fault - all three look like a stuck zero from here. */
    uint8_t encoderRaw()            { return read8(RR_M_ENC_RAW); }
    uint8_t encoderIrqs()           { return read8(RR_M_ENC_IRQ); }
    void zeroEncoders()             { write8(RR_M_ENC_ZERO, RR_M_ENC_ZERO_M1 | RR_M_ENC_ZERO_M2); }

    /* ---- distance and turn moves ----
     * Geometry lives on the coprocessor, so millimetres work from any host.
     * The defaults suit the stock rover; change the wheel or track only for
     * different wheels. Real millimetres here - the 0.01 mm register encoding
     * is handled inside.
     *
     * Every setting has an accepted range (RR_M_*_MIN / _MAX in rr_protocol.h).
     * The chip refuses a value outside it, keeps the old one and sets
     * RR_FAULT_CONFIG in fault(). Wheel circumference and track: 30-655 mm,
     * so a diameter of about 9.6-208 mm. */
    void setWheelCircMm(float mm)   { write16(RR_M_WHEEL_CIRC,  hundredths(mm)); }
    void setWheelDiaMm(float mm)    { setWheelCircMm(mm * 3.14159265f); }
    void setTrackWidthMm(float mm)  { write16(RR_M_TRACK_WIDTH, hundredths(mm)); }
    /* Top speed of a distance or turn move, cm/s: it speeds up to this, holds
     * it and slows onto the target. Default about 36, up to about 120 - what
     * the motors reach with no load. On the floor they manage less; from
     * Motion 1.14 a move then goes at the fastest pace both wheels can hold,
     * so it stays straight. False if outside the accepted range. */
    bool  setMoveSpeed(float cmPerSec);
    float moveSpeed();
    void setPositionKp(int16_t q88) { write16(RR_M_POS_KP, (uint16_t)q88); }
    /* Holds the wheels level during a move, and under setSpeed() from Motion
     * 1.15 (to the ratio of the two speeds); 0 lets each run on its own. */
    void setSyncGain(int16_t q88)   { write16(RR_M_SYNC_KP, (uint16_t)q88); }
    void setPositionTol(uint16_t ticks)  { write16(RR_M_POS_TOL, ticks); }
    void setCreepSpeed(uint16_t ticksPerSec) { write16(RR_M_POS_MIN_SPD, ticksPerSec); }
    /* How fast a move may speed up, ticks/s per second, 500-30000; default
     * 4000 (8000 before Motion 1.14). Slowing down follows setPositionKp().
     * Motion 1.12 and later. */
    void setAccel(uint16_t ticksPerSec2)     { write16(RR_M_POS_ACCEL, ticksPerSec2); }
    /* A wheel counts as stalled when pushed with at least dutyPermille
     * (100-1000) and not turning for ms (100-5000; 0 turns detection off), in
     * speed control and during moves. stalled() and the STALL interrupt report
     * it. Motion 1.12 and later. */
    void setStallDetect(uint16_t ms, uint16_t dutyPermille = RR_M_DEF_STALL_DUTY)
    {
        write16(RR_M_STALL_MS, ms);
        write16(RR_M_STALL_DUTY, dutyPermille);
    }
    /* What a stall does: RR_STALL_STOP (the default) ends the move, or zeroes
     * both speeds in setSpeed() control, and brakes - a blocked gearmotor at
     * full power only heats up and drains the battery. waitForMove() then
     * returns false at once. The next command runs normally; the fault stays
     * until clearFaults(). A program that re-sends speeds every loop should
     * check stalled() or it will push again. Direct drive() is never stopped.
     * RR_STALL_REPORT only raises the fault. Motion 1.12 and later. */
    void    setStallAction(uint8_t action) { write8(RR_M_STALL_ACTION, action); }
    uint8_t stallAction()                  { return read8(RR_M_STALL_ACTION); }

    /* ---- current settings, read from the chip ----
     * Gains come back in Q8.8: RR_Q88_TO_FLOAT(q) for a float. ticksPerRev()
     * is fixed for the stock motors (574), and only readable. */
    uint16_t ramp()                 { return read16(RR_M_RAMP); }
    uint16_t minDuty()              { return read16(RR_M_MIN_DUTY); }
    int16_t  pidKp()                { return (int16_t)read16(RR_M_KP); }
    int16_t  pidKi()                { return (int16_t)read16(RR_M_KI); }
    int16_t  pidKd()                { return (int16_t)read16(RR_M_KD); }
    uint16_t ticksPerRev()          { return read16(RR_M_TICKS_PER_REV); }
    float    wheelCircMm()          { return read16(RR_M_WHEEL_CIRC) / 100.0f; }
    float    wheelDiaMm()           { return wheelCircMm() / 3.14159265f; }
    float    trackWidthMm()         { return read16(RR_M_TRACK_WIDTH) / 100.0f; }
    int16_t  positionKp()           { return (int16_t)read16(RR_M_POS_KP); }
    int16_t  syncGain()             { return (int16_t)read16(RR_M_SYNC_KP); }
    uint16_t positionTol()          { return read16(RR_M_POS_TOL); }
    uint16_t creepSpeed()           { return read16(RR_M_POS_MIN_SPD); }
    uint16_t accel()                { return read16(RR_M_POS_ACCEL); }
    uint16_t stallMs()              { return read16(RR_M_STALL_MS); }
    uint16_t stallDuty()            { return read16(RR_M_STALL_DUTY); }

    /* Non-blocking: returns as soon as the move is accepted. turnDeg() is the
     * chip's own conversion, which rounds each wheel's arc down to a whole
     * millimetre - up to a degree short in 90. rover.turnRight() and
     * turnLeft() work the ticks out here instead, to a quarter of a degree. */
    void moveMm(int16_t mm)         { write16(RR_M_MOVE_MM,  (uint16_t)mm); }
    void turnDeg(int16_t deg)       { write16(RR_M_TURN_DEG, (uint16_t)deg); }
    void abortMove()                { write8(RR_M_MOVE_CTRL, RR_MOVE_ABORT); }

    bool isMoving()                 { return (status() & RR_MSTAT_MOVING) != 0; }
    bool moveDone()                 { return (status() & RR_MSTAT_MOVE_DONE) != 0; }

    /* Blocking variants: return true once the move has arrived, false at once
     * if a stall stopped it. timeoutMs is only a safety limit - if the move
     * has not arrived by then it is aborted and false returned. 0, the
     * default, allows the time the move should take at the move speed plus
     * 15 s, so a long or slow move is never cut short. */
    bool waitForMove(uint32_t timeoutMs = 0);
    bool moveMmAndWait(int16_t mm, uint32_t timeoutMs = 0);
    bool turnDegAndWait(int16_t deg, uint32_t timeoutMs = 0);

    /* Drive each wheel to an absolute count, and start. False, without
     * starting, if the targets did not all arrive. */
    bool setTargets(int32_t m1, int32_t m2);
    int32_t target(uint8_t m) { return (int32_t)read32(m ? RR_M_M2_TARGET : RR_M_M1_TARGET); }

    /* One atomic 16-byte read of the whole telemetry block. Call this once per
     * loop, then use the accessors - they read the local copy, not the bus. */
    bool poll();
    int32_t encoder(uint8_t m) const { return _enc[m & 1]; }
    int16_t speed(uint8_t m)   const { return _spd[m & 1]; }
    int16_t duty(uint8_t m)    const { return _duty[m & 1]; }

    bool stalled()         { return (fault() & (RR_MFAULT_STALL_M1 | RR_MFAULT_STALL_M2)) != 0; }

private:
    int32_t _enc[2];
    int16_t _spd[2], _duty[2];
    uint32_t autoTimeoutMs();
    /* Millimetres to the 0.01 mm register. A value that does not fit becomes
     * 0, which the chip refuses with a config fault - rather than wrapping
     * round to a plausible wrong size. */
    static uint16_t hundredths(float mm)
    {
        return (mm > 0.0f && mm <= 655.35f) ? (uint16_t)(mm * 100.0f + 0.5f) : 0u;
    }
};

/* ---- sensors, 0x29 ----------------------------------------------------- */

class RoboRoverCore3Sensors : public RoboRoverCore3Bus {
public:
    RoboRoverCore3Sensors() : RoboRoverCore3Bus(RR_ADDR_SENSORS) {}
    RRError begin() { return identify(RR_WHOAMI_SENSORS); }

    /* Which sensors run: RR_SEN_OBSTACLE | RR_SEN_ULTRASONIC | RR_SEN_LDR (all
     * three by default). One that is off reads 0. */
    void enable(uint8_t mask)          { write8(RR_S_SENSOR_EN, mask); }
    /* How often the obstacle and light readings update, 1-200 Hz; default 50. */
    void setSampleHz(uint8_t hz)       { write8(RR_S_SAMPLE_HZ, hz); }
    void setThresholds(uint16_t a, uint16_t b);
    /* How far below its threshold a reading must fall to clear, 0-90%; default
     * 20, so a flag does not flicker at the edge. */
    void setHysteresis(uint8_t pct)    { write8(RR_S_OBST_HYST, pct); }
    /* Samples summed per obstacle reading, 1-64; default 32. Readings scale
     * with it, so calibrate again (or change the thresholds) after changing it. */
    void setIrSamples(uint8_t n)       { write8(RR_S_IR_SAMPLES, n); }

    /* Obstacle thresholds are raw readings, the same units as irA()/irB().
     * What a sensor reads with nothing in front depends on the floor (white
     * adds about 250) and on the part, so calibrateObstacles() measures it and
     * sets each threshold `margin` above: point the rover at open floor first.
     * 200 trips on a hand at about 11 cm. It runs at power-up and in
     * RoboRoverCore3::begin(), and again whenever you call it - after moving
     * to another floor, say. Never on its own, so a still obstacle stays
     * detected. Takes about 0.2 s. False if margin is outside 50-10000, or the
     * sensor chip did not finish (obstacle sensing switched off, or firmware
     * older than Sensors 1.9).
     *
     * To choose your own: read irA()/irB() with and without the object and
     * pass values in between to setThresholds(). */
    bool calibrateObstacles(uint16_t margin = RR_S_DEF_OBST_MARGIN);
    uint16_t thresholdA()              { return read16(RR_S_OBST_THRESH_A); }
    uint16_t thresholdB()              { return read16(RR_S_OBST_THRESH_B); }
    uint16_t irFloorA()                { return read16(RR_S_OBST_FLOOR_A); }  /* at calibration */
    uint16_t irFloorB()                { return read16(RR_S_OBST_FLOOR_B); }

    /* ---- current settings, read from the chip ---- */
    uint8_t  enabledSensors()          { return read8(RR_S_SENSOR_EN); }
    uint8_t  sampleHz()                { return read8(RR_S_SAMPLE_HZ); }
    uint8_t  hysteresis()              { return read8(RR_S_OBST_HYST); }
    uint8_t  irSamples()               { return read8(RR_S_IR_SAMPLES); }
    uint16_t obstacleMargin()          { return read16(RR_S_OBST_MARGIN); }

    /* One atomic 14-byte read. Same pattern as Motion. */
    bool poll();
    bool     obstacleA()  const { return (_flags & RR_OBST_A_DETECT) != 0; }
    bool     obstacleB()  const { return (_flags & RR_OBST_B_DETECT) != 0; }
    uint16_t irA()        const { return _irA; }
    uint16_t irB()        const { return _irB; }
    uint16_t distanceMm() const { return _mm; }          /* RR_US_NO_ECHO if none */
    bool     hasEcho()    const { return _mm != RR_US_NO_ECHO; }
    uint16_t echoAgeMs()  const { return _age; }
    uint16_t ldr1()       const { return _ldr1; }
    uint16_t ldr2()       const { return _ldr2; }

private:
    uint8_t  _flags;
    uint16_t _irA, _irB, _mm, _age, _ldr1, _ldr2;
};

/* Left/right -> M1/M2. CONFIRMED for V3.0: the hardware reference names U1 as
 * M1 (right) and U2 as M2 (left), and M7 on a real board turned only the
 * right wheel when M1 alone was driven. Every facade method routes through
 * these two indices, so a future board that differs needs only these lines. */
#define RR_MOTOR_RIGHT  0   /* M1 */
#define RR_MOTOR_LEFT   1   /* M2 */

/* ==== the parts the host drives itself ==================================
 *
 * Motion (0x28) and Sensors (0x29) above sit behind the coprocessors.
 * Everything below sits directly on the shared I2C bus, or on a host GPIO,
 * and the host talks to it itself:
 *
 *   RoboRoverCore3Accel      LIS2DH12   U22, 0x19 (0x18 if SJ5 closed)
 *   RoboRoverCore3Battery    BQ27441    U25, 0x55
 *   RoboRoverCore3LineArray  TLA2528    off-board on CN8, 0x14
 *
 * All three reuse RoboRoverCore3Bus for transport, so error handling and the
 * AVR bus-recovery path are identical to the coprocessor classes.
 *
 * Deliberately dependency-free. A LIS2DH12 driver is a dozen registers and a
 * BQ27441 read is a 16-bit standard command; pulling in a vendor library for
 * either would cost more flash than the whole test sketch has spare.
 *
 * All three are confirmed on V3.1 rovers: the line array drives
 * 09_HybridLineFollower, the accelerometer 10_ChromaTiltSphere, and the gauge
 * reads voltage and current. Its percentage is not to be trusted yet - the
 * pack capacity has never been written to it.
 */

/* ---- shared-bus addresses ---------------------------------------------- */

#define RR_ADDR_ACCEL_ALT     0x18u   /* SJ5 closed */
#define RR_ADDR_LINEARRAY     0x14u

/* ---- LIS2DH12 ---------------------------------------------------------- */

#define RR_LIS_WHO_AM_I       0x0Fu
#define RR_LIS_WHOAMI_VALUE   0x33u
#define RR_LIS_CTRL_REG1      0x20u
#define RR_LIS_CTRL_REG4      0x23u
#define RR_LIS_OUT_X_L        0x28u
#define RR_LIS_AUTOINC        0x80u   /* set in the sub-address for a block read */

class RoboRoverCore3Accel : public RoboRoverCore3Bus {
public:
    RoboRoverCore3Accel() : RoboRoverCore3Bus(RR_ADDR_LIS2DH12),
                            _x(0), _y(0), _z(0), _mg(1), _c4(0) {}

    /* Tries 0x19 then 0x18, because SJ5 decides and the board may go either
     * way. On success the object retargets itself to whichever answered, so
     * address() afterwards tells you which strap is fitted. */
    RRError begin();

    /* One 6-byte auto-incremented read. BDU is set at begin(), so the high and
     * low halves of each axis always come from the same sample. */
    bool poll();

    /* Milli-g, whatever range the part is actually in.
     *
     * Output is left-justified, so (raw >> 4) is milli-g at +-2g in every
     * resolution mode - 12-bit high-res, 10-bit normal and 8-bit low-power all
     * put 1 g at the same raw 16-bit value. Only the FULL SCALE changes that,
     * doubling the milli-g each step. So the scale is read back from CTRL_REG4
     * at begin() rather than assumed: a board whose range register does not
     * take the write still reports correct milli-g instead of being silently
     * out by 2x, which reads as a broken sensor rather than a config problem. */
    int16_t x() const { return _x; }
    int16_t y() const { return _y; }
    int16_t z() const { return _z; }

    /* Milli-g per raw unit, derived from the FS bits actually in the part.
     * 1 = +-2g, 2 = +-4g, 4 = +-8g, 12 = +-16g. */
    uint8_t scaleMgPerUnit() const { return _mg; }
    uint8_t ctrlReg4() const       { return _c4; }

    /* Vector magnitude in milli-g. ~1000 at rest whatever the orientation, so
     * it is the cheapest single number that says "the part is alive and the
     * robot is the right way up". Integer sqrt, no FPU. */
    uint16_t magnitude() const;

    /* Which face is up, from the dominant axis. Returns 'X','x','Y','y','Z','z'
     * (capital = positive) or '?' when nothing dominates. */
    char orientation() const;

private:
    int16_t _x, _y, _z;
    uint8_t _mg;   /* milli-g per raw unit, from the FS bits */
    uint8_t _c4;   /* CTRL_REG4 as actually read back */
    void    readScale();
};

/* ---- BQ27441 fuel gauge ------------------------------------------------- */

#define RR_BQ_CONTROL         0x00u
#define RR_BQ_TEMPERATURE     0x02u   /* 0.1 K */
#define RR_BQ_VOLTAGE         0x04u   /* mV */
#define RR_BQ_FLAGS           0x06u
#define RR_BQ_REMAIN_CAP      0x0Cu   /* mAh */
#define RR_BQ_FULL_CAP        0x0Eu   /* mAh */
#define RR_BQ_AVG_CURRENT     0x10u   /* mA, signed; negative = discharging */
#define RR_BQ_SOC             0x1Cu   /* percent */
#define RR_BQ_DEVICE_TYPE     0x0001u /* Control() subcommand */
#define RR_BQ_DEVICE_ID       0x0421u
#define RR_BQ_SET_CFGUPDATE   0x0013u /* Control() subcommands */
#define RR_BQ_SEALED          0x0020u
#define RR_BQ_SOFT_RESET      0x0042u
#define RR_BQ_UNSEAL_KEY      0x8000u /* written twice */
#define RR_BQ_FLAG_CFGUPMODE  0x0010u /* in Flags() */
#define RR_BQ_DESIGN_CAP      0x3Cu   /* mAh, read-only view */
#define RR_BQ_DATA_CLASS      0x3Eu
#define RR_BQ_DATA_BLOCK      0x3Fu
#define RR_BQ_BLOCK_DATA      0x40u   /* 32 bytes */
#define RR_BQ_BLOCK_CHECKSUM  0x60u
#define RR_BQ_BLOCK_CONTROL   0x61u
#define RR_BQ_CLASS_STATE     82u     /* holds Design Capacity at offset 10 */
#define RR_BQ_FULL_AVAIL      0x0Au   /* mAh the cell holds when full, as learned */
#define RR_BQ_CONTROL_STATUS  0x0000u /* Control() subcommand */
#define RR_BQ_CS_QMAX_UP      0x0200u /* the gauge has measured the cell itself */

/* The capacity the gauge starts from. Set once the pack is known; the gauge
 * refines it from use, and the library keeps what it learns (see below). */
#define RR_BATTERY_DEFAULT_MAH 3000u

class RoboRoverCore3Battery : public RoboRoverCore3Bus {
public:
    RoboRoverCore3Battery() : RoboRoverCore3Bus(RR_ADDR_BQ27441),
                              _polls(0), _mv(0), _soc(0), _ma(0), _tempC(0) {}

    /* Issues Control(DEVICE_TYPE) and checks for 0x0421. A plain address ACK
     * is not enough here - several parts live on this bus and an ACK only
     * proves something is at 0x55. Then makes sure the gauge knows the
     * capacity (about a second, only after the battery was disconnected). */
    RRError begin();

    bool poll();

    uint16_t millivolts() const  { return _mv; }
    uint8_t  percent() const     { return _soc; }
    int16_t  milliamps() const   { return _ma; }   /* <0 while discharging */
    int16_t  temperatureC() const{ return _tempC; }
    bool     charging() const    { return _ma > 0; }

    uint16_t deviceType();       /* 0x0421 on a healthy BQ27441-G1 */

    /* The capacity percent() is measured against, mAh. Nothing to set: begin()
     * starts the gauge at 3000 mAh, and the gauge measures the real cell as
     * it charges and discharges. poll() saves what it measures in the last
     * four bytes of the Arduino's EEPROM, and begin() hands it back after the
     * battery has been disconnected - which is when the gauge forgets. */
    uint16_t designCapacity()    { return read16(RR_BQ_DESIGN_CAP); }

private:
    bool     control(uint16_t sub);
    bool     waitCfgUpdate(bool on);
    bool     setDesignCapacity(uint16_t mAh);
    void     learnCapacity();
    uint8_t  _polls;
    uint16_t _mv;
    uint8_t  _soc;
    int16_t  _ma;
    int16_t  _tempC;
};

/* ---- TLA2528 line-follower array ---------------------------------------- */

/* TI's register interface is opcode-framed rather than a plain register
 * pointer, so this one cannot reuse the inherited read8/write8. */
#define RR_TLA_OP_WRITE       0x08u
#define RR_TLA_OP_READ        0x10u
#define RR_TLA_REG_SYSTEM_ST  0x00u
#define RR_TLA_REG_GENERAL_CFG 0x01u
#define RR_TLA_REG_CHANNEL_SEL 0x11u
#define RR_TLA_CHANNELS       8u   /* the part has eight inputs */
#define RR_TLA_DEFAULT_CHANNELS 5u /* the RoboRover array populates five */

class RoboRoverCore3LineArray : public RoboRoverCore3Bus {
public:
    RoboRoverCore3LineArray() : RoboRoverCore3Bus(RR_ADDR_LINEARRAY), _n(0),
                               _used(RR_TLA_DEFAULT_CHANNELS) {}

    RRError begin();

    /* Reads the five sensors, one conversion per transaction. The other three
     * inputs of the chip have no sensor: they float and would mirror whichever
     * real channel was converted before them, so they are never read. */
    bool poll();

    /* 0..4095; white reads high, black low. Channel 0 is the RIGHTMOST sensor,
     * 4 the leftmost. Sensors differ by up to 2x, so a line follower should
     * calibrate each one against its own white and black, in the sketch. */
    uint16_t raw(uint8_t ch) const { return (ch < RR_TLA_CHANNELS) ? _ch[ch] : 0; }
    uint8_t  channels() const { return _n; }

private:
    uint16_t _ch[RR_TLA_CHANNELS];
    uint8_t  _n;
    uint8_t  _used;
    bool     tlaWrite(uint8_t reg, uint8_t val);
};

/* ---- host GPIO peripherals ---------------------------------------------- */

/* The WS2813C chain on D8 and the buzzer on D9 are plain host pins, not I2C.
 * NeoPixel needs cycle-accurate bit timing, so sketches drive it with
 * Adafruit_NeoPixel using RR_PIN_NEOPIXEL and RR_NEOPIXEL_COUNT from the top
 * of this file. The buzzer is just tone(), wrapped here only so a sketch does
 * not have to remember the pin. */

class RoboRoverCore3Buzzer {
public:
    void begin() { pinMode(RR_PIN_BUZZER, OUTPUT); digitalWrite(RR_PIN_BUZZER, LOW); }
    void beep(uint16_t hz = 2000, uint16_t ms = 80) { tone(RR_PIN_BUZZER, hz, ms); }
    void off() { noTone(RR_PIN_BUZZER); digitalWrite(RR_PIN_BUZZER, LOW); }
};

/* ---- facade ------------------------------------------------------------ */

class RoboRoverCore3 {
public:
    /* Brings up the bus, checks both chips and puts them back in their
     * power-up state - wheels stopped, encoders zeroed, every setting at its
     * default, all sensors on, faults cleared - then calibrates the obstacle
     * sensors to the floor. A track width kept with saveTrackWidth() replaces
     * the default. Set your own values after it, not before. Returns
     * RR_OK only if both chips answered with the right WHO_AM_I and a
     * matching protocol version. */
    RRError begin(uint32_t clockHz = 100000UL);

    /* Raw power per wheel, -1000..1000. Switches the motor chip to open loop
     * itself, ending any move; motion.drive() is the low-level form that
     * does not. */
    void drive(int16_t left, int16_t right);
    /* Aborts any move in flight, then commands zero - which BRAKES. Call
     * motion.coast() instead if the robot should roll to a halt. */
    void stop();
    bool poll();                       /* refreshes both chips */

    /* ---- three levels of control, pick one ----
     * drive()     raw duty, you do everything          (bypass)
     * setSpeed()  ticks/s, coprocessor holds speed and keeps the
     *             wheels in step (Motion 1.15)
     * forward()   millimetres, coprocessor holds distance
     */
    void setSpeed(int16_t leftTicksPerSec, int16_t rightTicksPerSec);
    void forward(int16_t mm)  { motion.moveMm(mm); }
    void backward(int16_t mm) { motion.moveMm((int16_t)-mm); }
    void turnRight(int16_t deg) { turn(deg); }
    void turnLeft(int16_t deg)  { turn((int16_t)-deg); }
    bool isMoving()           { return motion.isMoving(); }
    bool waitForMove(uint32_t timeoutMs = 0) { return motion.waitForMove(timeoutMs); }

    /* Blocking one-liners - the shortest path to a working program. */
    bool forwardAndWait(int16_t mm)   { return motion.moveMmAndWait(mm); }
    bool backwardAndWait(int16_t mm)  { return motion.moveMmAndWait((int16_t)-mm); }
    bool turnRightAndWait(int16_t deg){ return turn(deg) && motion.waitForMove(); }
    bool turnLeftAndWait(int16_t deg) { return turn((int16_t)-deg) && motion.waitForMove(); }

    /* Turn calibration. How far a spin turns depends on the tyres and the
     * floor, so the track width - motion.setTrackWidthMm() - can be tuned
     * until 90 really is 90; the TurnCalibration example does it with the
     * remote. saveTrackWidth() keeps the chip's current value in the
     * Arduino's EEPROM, and every begin() after that puts it back, so sketches
     * need nothing of their own. forgetTrackWidth() goes back to the default
     * from the next begin(). The value stays with the Arduino, not the rover:
     * move the Arduino to another rover and calibrate again. AVR only;
     * saveTrackWidth() returns false elsewhere. */
    bool saveTrackWidth();
    void forgetTrackWidth();

    RoboRoverCore3Motion    motion;
    RoboRoverCore3Sensors   sensors;
    /* The line sensors are on their own board, read by the Arduino itself.
     * begin() starts them; poll() leaves them out, because reading all five
     * takes five bus transactions - call line.poll() when following a line.
     * line.channels() is 0 if the array did not answer. */
    RoboRoverCore3LineArray line;

    /* Convenience passthroughs so a student never has to know which chip. */
    bool     obstacleLeft()  { return sensors.obstacleA(); }
    bool     obstacleRight() { return sensors.obstacleB(); }
    uint16_t distanceMm()    { return sensors.distanceMm(); }
    /* Light sensors: brighter reads higher. LDR1 is the left one, LDR2 the
     * right. */
    uint16_t lightLeft()     { return sensors.ldr1(); }
    uint16_t lightRight()    { return sensors.ldr2(); }
    int32_t  leftTicks()     { return motion.encoder(RR_MOTOR_LEFT);  }
    int32_t  rightTicks()    { return motion.encoder(RR_MOTOR_RIGHT); }

private:
    bool turn(int16_t deg);
};

#endif /* ROBOROVERCORE3_H */
