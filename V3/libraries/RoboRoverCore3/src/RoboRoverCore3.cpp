/* RoboRoverCore3.cpp - host library for RoboRoverCore3 V3.0. See RoboRoverCore3.h. */

#include "RoboRoverCore3.h"
#if defined(ARDUINO_ARCH_AVR)
#include <EEPROM.h>
#endif

/* ---- transport --------------------------------------------------------- */

void RoboRoverCore3Bus::begin(uint32_t clockHz)
{
    Wire.begin();
    Wire.setClock(clockHz);

#if defined(ARDUINO_ARCH_AVR)
    /* The AVR Wire library busy-waits forever on a stuck bus. On V2.1 motor
     * switching noise corrupted transfers often enough that this hung the
     * whole robot. A 25 ms ceiling turns a hang into a recoverable error.
     * ESP32's driver has its own timeout and needs none of this. */
    Wire.setWireTimeout(25000UL, true);
#endif
}

void RoboRoverCore3Bus::recover(uint8_t sdaPin, uint8_t sclPin)
{
    /* A slave interrupted mid-byte can hold SDA low forever. Nine clocks walk
     * it past the end of the byte and the ACK, after which it releases. Only a
     * master can drive SCL, so this cannot live in the CH32 firmware. */
    Wire.end();
    pinMode(sclPin, OUTPUT);
    pinMode(sdaPin, INPUT_PULLUP);
    for (uint8_t i = 0; i < 9; i++) {
        digitalWrite(sclPin, LOW);  delayMicroseconds(5);
        digitalWrite(sclPin, HIGH); delayMicroseconds(5);
        if (digitalRead(sdaPin)) break;   /* released early - done */
    }
    /* Manual STOP: SDA low->high while SCL is high. */
    pinMode(sdaPin, OUTPUT);
    digitalWrite(sdaPin, LOW);  delayMicroseconds(5);
    digitalWrite(sclPin, HIGH); delayMicroseconds(5);
    digitalWrite(sdaPin, HIGH); delayMicroseconds(5);
    pinMode(sdaPin, INPUT);
    pinMode(sclPin, INPUT);
    Wire.begin();
}

bool RoboRoverCore3Bus::write8(uint8_t reg, uint8_t v)
{
    Wire.beginTransmission(_addr);
    Wire.write(reg);
    Wire.write(v);
    uint8_t r = Wire.endTransmission();
    _err = (r == 0) ? RR_OK : (r == 5 ? RR_ERR_TIMEOUT : RR_ERR_NACK);
    return _err == RR_OK;
}

bool RoboRoverCore3Bus::write16(uint8_t reg, uint16_t v)
{
    /* Low byte then high byte, in one transaction. The firmware stages the low
     * byte and commits on the high byte, so the value never reaches the
     * control loop half-updated. Splitting this into two transactions would
     * still work, but keeping it atomic is cheaper. */
    Wire.beginTransmission(_addr);
    Wire.write(reg);
    Wire.write((uint8_t)(v & 0xFF));
    Wire.write((uint8_t)(v >> 8));
    uint8_t r = Wire.endTransmission();
    _err = (r == 0) ? RR_OK : (r == 5 ? RR_ERR_TIMEOUT : RR_ERR_NACK);
    return _err == RR_OK;
}

bool RoboRoverCore3Bus::readBlock(uint8_t reg, uint8_t *buf, uint8_t len)
{
    if (len > RR_MAX_PAYLOAD) { _err = RR_ERR_SHORT_READ; return false; }

    Wire.beginTransmission(_addr);
    Wire.write(reg);
    uint8_t r = Wire.endTransmission(false);      /* repeated START */
    if (r != 0) { _err = (r == 5) ? RR_ERR_TIMEOUT : RR_ERR_NACK; return false; }

    uint8_t got = Wire.requestFrom(_addr, len);
    if (got != len) { _err = RR_ERR_SHORT_READ; return false; }
    for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
    _err = RR_OK;
    return true;
}

uint8_t RoboRoverCore3Bus::read8(uint8_t reg)
{
    uint8_t b = 0;
    readBlock(reg, &b, 1);
    return b;
}

bool RoboRoverCore3Bus::read8(uint8_t reg, uint8_t &out)
{
    uint8_t b = 0;
    if (!readBlock(reg, &b, 1)) return false;
    out = b;
    return true;
}

bool RoboRoverCore3Bus::read16(uint8_t reg, uint16_t &out)
{
    uint8_t b[2];
    if (!readBlock(reg, b, 2)) return false;
    out = rr_get_u16(b);
    return true;
}

uint16_t RoboRoverCore3Bus::read16(uint8_t reg)
{
    uint8_t b[2] = {0, 0};
    readBlock(reg, b, 2);
    return rr_get_u16(b);
}

uint32_t RoboRoverCore3Bus::read32(uint8_t reg)
{
    uint8_t b[4] = {0, 0, 0, 0};
    readBlock(reg, b, 4);
    return rr_get_u32(b);
}

bool RoboRoverCore3Bus::present()
{
    Wire.beginTransmission(_addr);
    return Wire.endTransmission() == 0;
}

void RoboRoverCore3Bus::firmwareVersion(uint8_t &major, uint8_t &minor)
{
    uint8_t b[2] = {0, 0};
    readBlock(RR_REG_FW_VERSION, b, 2);
    major = b[0];
    minor = b[1];
}

RRError RoboRoverCore3Bus::identify(uint8_t expectWhoAmI)
{
    uint8_t b[2] = {0, 0};
    if (!readBlock(RR_REG_WHO_AM_I, b, 2)) return _err;
    if (b[0] != expectWhoAmI)        return (_err = RR_ERR_WHOAMI);
    /* A chip flashed from a different revision of rr_protocol.h will answer
     * correctly here and then misinterpret every other register. Catching it
     * at begin() is far cheaper than debugging it at speed. */
    if (b[1] != RR_PROTOCOL_VERSION) return (_err = RR_ERR_PROTOCOL);
    return (_err = RR_OK);
}

/* ---- motion ------------------------------------------------------------ */

void RoboRoverCore3Motion::drive(int16_t m1, int16_t m2)
{
    /* Both commands in one transaction: register auto-increment carries the
     * pointer from M1_CMD through M2_CMD, so the wheels are updated together
     * and only one bus round trip is spent. */
    Wire.beginTransmission(_addr);
    Wire.write(RR_M_M1_CMD);
    Wire.write((uint8_t)((uint16_t)m1 & 0xFF));
    Wire.write((uint8_t)((uint16_t)m1 >> 8));
    Wire.write((uint8_t)((uint16_t)m2 & 0xFF));
    Wire.write((uint8_t)((uint16_t)m2 >> 8));
    uint8_t r = Wire.endTransmission();
    _err = (r == 0) ? RR_OK : (r == 5 ? RR_ERR_TIMEOUT : RR_ERR_NACK);
}

void RoboRoverCore3Motion::setPID(int16_t kp, int16_t ki, int16_t kd)
{
    Wire.beginTransmission(_addr);
    Wire.write(RR_M_KP);
    Wire.write((uint8_t)((uint16_t)kp & 0xFF)); Wire.write((uint8_t)((uint16_t)kp >> 8));
    Wire.write((uint8_t)((uint16_t)ki & 0xFF)); Wire.write((uint8_t)((uint16_t)ki >> 8));
    Wire.write((uint8_t)((uint16_t)kd & 0xFF)); Wire.write((uint8_t)((uint16_t)kd >> 8));
    uint8_t r = Wire.endTransmission();
    _err = (r == 0) ? RR_OK : (r == 5 ? RR_ERR_TIMEOUT : RR_ERR_NACK);
}

bool RoboRoverCore3Motion::poll()
{
    uint8_t b[RR_M_TELEM_LEN];
    if (!readBlock(RR_M_TELEM_BASE, b, RR_M_TELEM_LEN)) return false;
    _enc[0]  = (int32_t)rr_get_u32(&b[0]);
    _enc[1]  = (int32_t)rr_get_u32(&b[4]);
    _spd[0]  = (int16_t)rr_get_u16(&b[8]);
    _spd[1]  = (int16_t)rr_get_u16(&b[10]);
    _duty[0] = (int16_t)rr_get_u16(&b[12]);
    _duty[1] = (int16_t)rr_get_u16(&b[14]);
    return true;
}

/* cm/s <-> the chip's ticks/s, through its own wheel size and ticks per turn,
 * so a changed wheel changes the conversion with it. */
bool RoboRoverCore3Motion::setMoveSpeed(float cmPerSec)
{
    float circ = read16(RR_M_WHEEL_CIRC) / 100.0f;       /* mm */
    float tpr  = (float)read16(RR_M_TICKS_PER_REV);
    if (circ <= 0.0f || tpr <= 0.0f || cmPerSec <= 0.0f) return false;
    float t = cmPerSec * 10.0f / circ * tpr + 0.5f;
    if (t > 65535.0f) return false;
    write16(RR_M_CRUISE, (uint16_t)t);
    return read16(RR_M_CRUISE) == (uint16_t)t;           /* refused if out of range */
}

float RoboRoverCore3Motion::moveSpeed()
{
    float tpr = (float)read16(RR_M_TICKS_PER_REV);
    if (tpr <= 0.0f) return 0.0f;
    return read16(RR_M_CRUISE) / tpr * (read16(RR_M_WHEEL_CIRC) / 100.0f) / 10.0f;
}

/* ---- sensors ----------------------------------------------------------- */

void RoboRoverCore3Sensors::setThresholds(uint16_t a, uint16_t b)
{
    Wire.beginTransmission(_addr);
    Wire.write(RR_S_OBST_THRESH_A);
    Wire.write((uint8_t)(a & 0xFF)); Wire.write((uint8_t)(a >> 8));
    Wire.write((uint8_t)(b & 0xFF)); Wire.write((uint8_t)(b >> 8));
    uint8_t r = Wire.endTransmission();
    _err = (r == 0) ? RR_OK : (r == 5 ? RR_ERR_TIMEOUT : RR_ERR_NACK);
}

bool RoboRoverCore3Sensors::calibrateObstacles(uint16_t margin)
{
    /* An older chip ignores OBST_CAL and reads it back as 0, which would look
     * like a calibration that finished at once. */
    uint8_t maj = 0, min = 0;
    firmwareVersion(maj, min);
    if (maj < 1 || (maj == 1 && min < 9)) return false;

    /* A margin outside its range is refused and the old one kept: calibrating
     * with that would quietly give thresholds the caller did not ask for. */
    write16(RR_S_OBST_MARGIN, margin);
    if (read16(RR_S_OBST_MARGIN) != margin) return false;
    if (!write8(RR_S_OBST_CAL, 1)) return false;

    /* Eight samples: 160 ms at the default 50 Hz. The limit leaves room for a
     * slower sample rate before giving up. */
    uint32_t t0 = millis();
    while (millis() - t0 < 2000UL) {
        delay(20);
        uint8_t busy;
        if (read8(RR_S_OBST_CAL, busy) && busy == 0) return true;
    }
    return false;
}

bool RoboRoverCore3Sensors::poll()
{
    uint8_t b[RR_S_TELEM_LEN];
    if (!readBlock(RR_S_TELEM_BASE, b, RR_S_TELEM_LEN)) return false;
    _flags = b[0];
    _irA   = rr_get_u16(&b[2]);
    _irB   = rr_get_u16(&b[4]);
    _mm    = rr_get_u16(&b[6]);
    _age   = rr_get_u16(&b[8]);
    _ldr1  = rr_get_u16(&b[10]);
    _ldr2  = rr_get_u16(&b[12]);
    return true;
}

/* The time the move in progress should still take at the cruise speed, plus
 * 15 s. A fixed limit cut long or slow moves short - 15 s is only 5.4 m at the
 * default speed - while a move that cannot finish is still let go of: a
 * blocked wheel is stopped by the chip long before this. */
uint32_t RoboRoverCore3Motion::autoTimeoutMs()
{
    uint32_t far = 0;
    if (poll()) {
        for (uint8_t m = 0; m < 2; m++) {
            int32_t d = (int32_t)read32(m ? RR_M_M2_TARGET : RR_M_M1_TARGET) - encoder(m);
            if (d < 0) d = -d;
            if ((uint32_t)d > far) far = (uint32_t)d;
        }
    }
    if (far > 4000000UL) far = 4000000UL;          /* keeps far * 1000 in 32 bits */
    uint16_t cruise = read16(RR_M_CRUISE);
    if (cruise == 0) cruise = 1;
    return far * 1000UL / cruise + 15000UL;
}

bool RoboRoverCore3Motion::waitForMove(uint32_t timeoutMs)
{
    uint8_t  doneReads = 0, stallReads = 0;
    /* Give the coprocessor a moment to raise MOVING before we test it,
     * otherwise a fast host reads the old status and returns immediately. */
    delay(20);
    if (timeoutMs == 0) timeoutMs = autoTimeoutMs();
    uint32_t t0 = millis();
    while (millis() - t0 < timeoutMs) {
        /* Finished only when the chip reports MOVE_DONE, twice running.
         *
         * "MOVING is clear" used to be enough, and it ended a floor test with
         * the wheels still turning: a failed or corrupted read comes back as
         * 0x00, which has MOVING clear. MOVE_DONE is cleared when a move starts
         * and set only when one completes, so a bad byte almost never fakes
         * it - and never twice in a row.
         *
         * A move the chip ended for a stall stops with MOVE_DONE clear and a
         * STALL fault: that returns false at once rather than after the whole
         * timeout, on the same twice-running rule. */
        uint8_t st, f;
        bool ok = read8(RR_REG_STATUS, st);
        if (ok && (st & RR_MSTAT_MOVE_DONE) && !(st & RR_MSTAT_MOVING)) {
            stallReads = 0;
            if (++doneReads >= 2) return true;
        } else if (ok && !(st & RR_MSTAT_MOVING) &&
                   read8(RR_REG_FAULT, f) && (f & (RR_MFAULT_STALL_M1 | RR_MFAULT_STALL_M2))) {
            doneReads = 0;
            if (++stallReads >= 2) return false;
        } else {
            doneReads = stallReads = 0;
        }
        delay(10);
    }
    abortMove();
    return false;
}

bool RoboRoverCore3Motion::moveMmAndWait(int16_t mm, uint32_t timeoutMs)
{
    moveMm(mm);
    return waitForMove(timeoutMs);
}

bool RoboRoverCore3Motion::turnDegAndWait(int16_t deg, uint32_t timeoutMs)
{
    turnDeg(deg);
    return waitForMove(timeoutMs);
}

bool RoboRoverCore3Motion::setTargets(int32_t m1, int32_t m2)
{
    uint8_t b[8];
    rr_put_u32(&b[0], (uint32_t)m1);
    rr_put_u32(&b[4], (uint32_t)m2);
    Wire.beginTransmission(_addr);
    Wire.write(RR_M_M1_TARGET);
    for (uint8_t i = 0; i < 8; i++) Wire.write(b[i]);
    uint8_t r = Wire.endTransmission();
    _err = (r == 0) ? RR_OK : (r == 5 ? RR_ERR_TIMEOUT : RR_ERR_NACK);
    /* A write cut short can leave one target new and the other old; starting
     * would drive to that mixture. */
    if (_err != RR_OK) return false;
    return write8(RR_M_MOVE_CTRL, RR_MOVE_START);
}

/* ---- saved turn calibration -------------------------------------------- */

/* The track width saveTrackWidth() keeps, in the four EEPROM bytes before the
 * battery's: 'R' 'T', then 0.01 mm little-endian. As with the battery, the
 * marker tells a saved value from a blank EEPROM or a sketch's own bytes. */
#if defined(ARDUINO_ARCH_AVR)
static int trackAddr() { return (int)EEPROM.length() - 8; }

static uint16_t savedTrackWidth()
{
    int a = trackAddr();
    if (EEPROM.read(a) != 'R' || EEPROM.read(a + 1) != 'T') return 0;
    uint16_t v = (uint16_t)(EEPROM.read(a + 2) | ((uint16_t)EEPROM.read(a + 3) << 8));
    return v >= RR_M_LENGTH_MIN ? v : 0;
}

bool RoboRoverCore3::saveTrackWidth()
{
    uint16_t v;
    if (!motion.read16(RR_M_TRACK_WIDTH, v) || v < RR_M_LENGTH_MIN) return false;
    int a = trackAddr();
    EEPROM.update(a, 'R');
    EEPROM.update(a + 1, 'T');
    EEPROM.update(a + 2, (uint8_t)v);
    EEPROM.update(a + 3, (uint8_t)(v >> 8));
    return true;
}

void RoboRoverCore3::forgetTrackWidth()
{
    int a = trackAddr();
    EEPROM.update(a, 0xFF);
    EEPROM.update(a + 1, 0xFF);
}
#else
static uint16_t savedTrackWidth() { return 0; }
bool RoboRoverCore3::saveTrackWidth() { return false; }
void RoboRoverCore3::forgetTrackWidth() { }
#endif

/* ---- facade ------------------------------------------------------------ */

RRError RoboRoverCore3::begin(uint32_t clockHz)
{
    RoboRoverCore3Bus::begin(clockHz);
    delay(50);                       /* let both CH32s finish booting */

    /* Resetting or re-uploading the Arduino resets neither chip, so a sketch
     * would inherit whatever the last one left: wheels still turning, a
     * slower move speed, a sensor switched off, a stale fault. Every sketch
     * starts from the chips' power-up state instead. Idle comes first: it
     * stops the wheels and ends a move properly, which loading the defaults
     * alone does not. */
    RRError m = motion.begin();
    if (m == RR_OK) {
        motion.idle();
        motion.loadDefaults();
        uint16_t track = savedTrackWidth();
        if (track) motion.write16(RR_M_TRACK_WIDTH, track);
        motion.zeroEncoders();
        motion.setInterruptMask(RR_M_DEF_INT_CFG);
        motion.clearInterruptFlags();
        motion.clearFaults();
    }

    /* Optional hardware: a missing array leaves line.channels() at 0
     * rather than failing begin(). */
    line.begin();

    RRError e = sensors.begin();
    if (e != RR_OK) return e;
    sensors.loadDefaults();
    sensors.setInterruptMask(RR_S_DEF_INT_CFG);
    sensors.clearInterruptFlags();
    sensors.clearFaults();
    /* Thresholds to suit the floor the rover is on now, not the one it was
     * powered up on. */
    sensors.calibrateObstacles();
    return m;
}

/* Each wheel runs along an arc of the circle whose diameter is the track:
 * pi * track * deg / 360. The chip's TURN_DEG does the same sum but rounds the
 * arc down to whole millimetres, a degree in 90 and coarser than turn
 * calibration needs, so the arc stays in 0.01 mm here and the chip is sent
 * ticks. Every product fits 32 bits across the chip's accepted ranges. */
bool RoboRoverCore3::turn(int16_t deg)
{
    uint16_t track, circ, tpr;
    if (!motion.read16(RR_M_TRACK_WIDTH, track) || !motion.read16(RR_M_WHEEL_CIRC, circ) ||
        !motion.read16(RR_M_TICKS_PER_REV, tpr) || circ == 0 || !motion.poll()) return false;
    uint32_t d = (deg < 0) ? (uint32_t)(-(int32_t)deg) : (uint32_t)deg;
    if (d > RR_TURN_MAX_DEG) d = RR_TURN_MAX_DEG;
    uint32_t arc = ((uint32_t)track * 355u / 113u * d + 180u) / 360u;
    int32_t  t   = (int32_t)(arc / circ * tpr + ((arc % circ) * tpr + circ / 2) / circ);
    if (deg < 0) t = -t;
    /* A right turn runs the right wheel backwards. */
    return motion.setTargets(motion.encoder(RR_MOTOR_RIGHT) - t,
                             motion.encoder(RR_MOTOR_LEFT)  + t);
}

void RoboRoverCore3::drive(int16_t left, int16_t right)
{
    /* Ordered through RR_MOTOR_LEFT / RR_MOTOR_RIGHT rather than written out,
     * so the mapping is stated once in the header and every caller follows. */
    int16_t cmd[2];
    cmd[RR_MOTOR_LEFT]  = left;
    cmd[RR_MOTOR_RIGHT] = right;
    /* The chip starts in IDLE and stays in position mode after a move, and in
     * either a power command does nothing - silently, which is how sketches
     * ended up sitting still. Writing the mode it is already in resets
     * nothing on the chip, so this costs one short write on the bus. */
    motion.setMode(RR_MODE_OPEN_LOOP);
    motion.drive(cmd[0], cmd[1]);
}

void RoboRoverCore3::stop()
{
    motion.abortMove();
    motion.drive(0, 0);
}

void RoboRoverCore3::setSpeed(int16_t left, int16_t right)
{
    int16_t cmd[2];
    cmd[RR_MOTOR_LEFT]  = left;
    cmd[RR_MOTOR_RIGHT] = right;
    motion.setMode(RR_MODE_CLOSED_LOOP);
    motion.drive(cmd[0], cmd[1]);
}

bool RoboRoverCore3::poll()
{
    bool a = motion.poll();
    bool b = sensors.poll();
    return a && b;
}

/* ==== the parts the host drives itself ================================== */

/* ---- LIS2DH12 ---------------------------------------------------------- */

RRError RoboRoverCore3Accel::begin()
{
    /* SJ5 picks the address and the board may be built either way, so try the
     * documented default first and fall back rather than making the caller
     * care. */
    const uint8_t cand[2] = { RR_ADDR_LIS2DH12, RR_ADDR_ACCEL_ALT };

    for (uint8_t i = 0; i < 2; i++) {
        _addr = cand[i];
        clearError();
        if (read8(RR_LIS_WHO_AM_I) == RR_LIS_WHOAMI_VALUE) {
            /* 100 Hz, all three axes on. Nothing here needs faster, and a
             * lower rate keeps the part quiet on a bus shared with the
             * coprocessors. */
            write8(RR_LIS_CTRL_REG1, 0x57);
            /* BDU so a block read cannot mix two samples, +-2g, high
             * resolution (12-bit). */
            write8(RR_LIS_CTRL_REG4, 0x88);

            readScale();
            return (_err = RR_OK);
        }
    }

    _addr = RR_ADDR_LIS2DH12;      /* leave it pointing somewhere sane */
    return (_err = RR_ERR_WHOAMI);
}

/* Read the range back and believe the part, not the write. If the range bits
 * did not take, scaling from the assumed value puts every reading out by a
 * clean factor of two - which looks exactly like a faulty sensor and is not. */
void RoboRoverCore3Accel::readScale()
{
    _c4 = read8(RR_LIS_CTRL_REG4);
    switch ((_c4 >> 4) & 3) {
    case 0:  _mg = 1;  break;   /* +-2g  */
    case 1:  _mg = 2;  break;   /* +-4g  */
    case 2:  _mg = 4;  break;   /* +-8g  */
    default: _mg = 12; break;   /* +-16g */
    }
}

bool RoboRoverCore3Accel::poll()
{
    uint8_t b[6];

    /* The LIS2DH12 only auto-increments its sub-address when the MSB is set.
     * Without this you read OUT_X_L six times and every axis comes out the
     * same - which looks like a wiring fault rather than a driver bug. */
    if (!readBlock((uint8_t)(RR_LIS_OUT_X_L | RR_LIS_AUTOINC), b, 6)) return false;

    /* 12-bit, left justified. Shift down by 4, then 1 mg per count at +-2g in
     * high-resolution mode. */
    _x = (int16_t)(((int16_t)((uint16_t)b[0] | ((uint16_t)b[1] << 8)) >> 4) * _mg);
    _y = (int16_t)(((int16_t)((uint16_t)b[2] | ((uint16_t)b[3] << 8)) >> 4) * _mg);
    _z = (int16_t)(((int16_t)((uint16_t)b[4] | ((uint16_t)b[5] << 8)) >> 4) * _mg);
    return true;
}

uint16_t RoboRoverCore3Accel::magnitude() const
{
    /* Integer sqrt by Newton. Squares of three 12-bit values need 32 bits, and
     * the AVR has no FPU worth using here. */
    uint32_t s = (uint32_t)((int32_t)_x * _x) +
                 (uint32_t)((int32_t)_y * _y) +
                 (uint32_t)((int32_t)_z * _z);
    if (s == 0) return 0;

    uint32_t r = s, prev = 0;
    for (uint8_t i = 0; i < 20 && r != prev; i++) {
        prev = r;
        r = (r + s / r) >> 1;
    }
    return (uint16_t)((r > 65535UL) ? 65535UL : r);
}

char RoboRoverCore3Accel::orientation() const
{
    int32_t ax = _x < 0 ? -(int32_t)_x : _x;
    int32_t ay = _y < 0 ? -(int32_t)_y : _y;
    int32_t az = _z < 0 ? -(int32_t)_z : _z;

    /* 700 mg is a comfortable margin: an axis pointing at gravity reads ~1000
     * and the other two sit near zero, so anything below this means the board
     * is tilted between faces and no single answer is honest. */
    if (az >= ax && az >= ay && az > 700) return _z > 0 ? 'Z' : 'z';
    if (ay >= ax && ay >= az && ay > 700) return _y > 0 ? 'Y' : 'y';
    if (ax >= ay && ax >= az && ax > 700) return _x > 0 ? 'X' : 'x';
    return '?';
}

/* ---- BQ27441 ------------------------------------------------------------ */

/* The capacity the gauge measured, kept in the last four bytes of the
 * Arduino's EEPROM: 'R' 'B', then mAh little-endian. The marker tells a saved
 * value from a blank EEPROM or a sketch's own bytes. The gauge itself forgets
 * everything when the battery is disconnected - it has no memory of its own -
 * and it is on the Arduino's bus, not a coprocessor's, so this is where it
 * can live. Only the AVR keeps it; elsewhere the gauge starts from 3000 mAh. */
#if defined(ARDUINO_ARCH_AVR)
static uint16_t savedCapacity()
{
    int a = (int)EEPROM.length() - 4;
    if (EEPROM.read(a) != 'R' || EEPROM.read(a + 1) != 'B') return 0;
    return (uint16_t)(EEPROM.read(a + 2) | ((uint16_t)EEPROM.read(a + 3) << 8));
}
static void saveCapacity(uint16_t mAh)
{
    int a = (int)EEPROM.length() - 4;
    EEPROM.update(a, 'R');
    EEPROM.update(a + 1, 'B');
    EEPROM.update(a + 2, (uint8_t)mAh);
    EEPROM.update(a + 3, (uint8_t)(mAh >> 8));
}
#else
static uint16_t savedCapacity() { return 0; }
static void saveCapacity(uint16_t) { }
#endif

/* Control() subcommands go into 0x00/0x01; any answer is read back from the
 * same place. */
bool RoboRoverCore3Battery::control(uint16_t sub)
{
    Wire.beginTransmission(_addr);
    Wire.write((uint8_t)RR_BQ_CONTROL);
    Wire.write((uint8_t)(sub & 0xFF));
    Wire.write((uint8_t)(sub >> 8));
    if (Wire.endTransmission() != 0) { _err = RR_ERR_NACK; return false; }
    return true;
}

uint16_t RoboRoverCore3Battery::deviceType()
{
    if (!control(RR_BQ_DEVICE_TYPE)) return 0;
    delay(2);                       /* the gauge needs a moment to answer */
    return read16(RR_BQ_CONTROL);
}

/* The datasheet allows up to a second to enter or leave config update. */
bool RoboRoverCore3Battery::waitCfgUpdate(bool on)
{
    for (uint8_t i = 0; i < 100; i++) {
        uint16_t f;
        if (read16(RR_BQ_FLAGS, f) && (((f & RR_BQ_FLAG_CFGUPMODE) != 0) == on)) return true;
        delay(20);
    }
    return false;
}

/* The TI sequence for a data-memory change: unseal, enter config update,
 * rewrite one 32-byte block of the State subclass with a fresh checksum, then
 * soft-reset so the gauge restarts its model with the new capacity, and seal
 * again. The checksum write is what commits the block, so a transfer that
 * dies before it leaves the old values in place. */
bool RoboRoverCore3Battery::setDesignCapacity(uint16_t mAh)
{
    if (mAh < 100 || mAh > 17000) return false;         /* energy must fit 16 bits */
    if (designCapacity() == mAh) return true;           /* already set */

    uint8_t blk[32];
    uint32_t mWh = (uint32_t)mAh * 37u / 10u;           /* 3.7 V nominal cell */
    bool ok = control(RR_BQ_UNSEAL_KEY) && control(RR_BQ_UNSEAL_KEY) &&
              control(RR_BQ_SET_CFGUPDATE) && waitCfgUpdate(true);

    if (ok) {
        ok = write8(RR_BQ_BLOCK_CONTROL, 0x00) &&
             write8(RR_BQ_DATA_CLASS, RR_BQ_CLASS_STATE) &&
             write8(RR_BQ_DATA_BLOCK, 0x00);
        delay(2);
        /* 32 bytes in two halves: the AVR Wire buffer is 32 including the
         * register byte. */
        ok = ok && readBlock(RR_BQ_BLOCK_DATA, blk, 16) &&
                   readBlock(RR_BQ_BLOCK_DATA + 16, blk + 16, 16);
    }
    if (ok) {
        uint8_t sum = 0;
        blk[10] = (uint8_t)(mAh >> 8);  blk[11] = (uint8_t)mAh;      /* big-endian */
        blk[12] = (uint8_t)(mWh >> 8);  blk[13] = (uint8_t)mWh;
        for (uint8_t i = 10; i < 14 && ok; i++) ok = write8((uint8_t)(RR_BQ_BLOCK_DATA + i), blk[i]);
        for (uint8_t i = 0; i < 32; i++) sum = (uint8_t)(sum + blk[i]);
        ok = ok && write8(RR_BQ_BLOCK_CHECKSUM, (uint8_t)(255u - sum));
        delay(2);
    }

    /* Leave config update whatever happened above, so the gauge keeps
     * running; SOFT_RESET is the documented way out. */
    control(RR_BQ_SOFT_RESET);
    waitCfgUpdate(false);
    control(RR_BQ_SEALED);

    return ok && designCapacity() == mAh;
}

RRError RoboRoverCore3Battery::begin()
{
    if (!present()) return (_err = RR_ERR_NACK);

    /* An address ACK only proves something is at 0x55. Several parts share
     * this bus, so confirm it is actually the gauge. */
    if (deviceType() != RR_BQ_DEVICE_ID) return (_err = RR_ERR_WHOAMI);

    /* After a battery disconnect the gauge is back on its factory default.
     * Hand it the last capacity it measured, or 3000 mAh before it has
     * measured any; if it already has that, this costs one read. */
    uint16_t mAh = savedCapacity();
    if (mAh < 100 || mAh > 17000) mAh = RR_BATTERY_DEFAULT_MAH;
    setDesignCapacity(mAh);
    return (_err = RR_OK);
}

/* Once the gauge has measured the cell itself (QMAX_UP), its full capacity
 * is worth keeping. Saved only on a real change - more than 5% - so the
 * EEPROM sees a handful of writes over the life of a battery. */
void RoboRoverCore3Battery::learnCapacity()
{
    uint16_t cs, full;
    if (!control(RR_BQ_CONTROL_STATUS)) return;
    delay(2);
    if (!read16(RR_BQ_CONTROL, cs) || !(cs & RR_BQ_CS_QMAX_UP)) return;
    if (!read16(RR_BQ_FULL_AVAIL, full) || full < 100 || full > 17000) return;

    uint16_t saved = savedCapacity();
    uint16_t diff  = (full > saved) ? (uint16_t)(full - saved) : (uint16_t)(saved - full);
    if (saved == 0 || diff > saved / 20) saveCapacity(full);
}

bool RoboRoverCore3Battery::poll()
{
    clearError();
    _mv  = read16(RR_BQ_VOLTAGE);
    _soc = (uint8_t)read16(RR_BQ_SOC);
    _ma  = (int16_t)read16(RR_BQ_AVG_CURRENT);

    /* Temperature is reported in 0.1 K. Kelvin to Celsius, rounded, without
     * floating point: (raw - 2731.5) / 10 becomes (raw - 2732 + 5) / 10. */
    {
        int32_t k10 = (int32_t)read16(RR_BQ_TEMPERATURE);
        _tempC = (int16_t)((k10 - 2732 + 5) / 10);
    }
    bool ok = lastError() == RR_OK;

    /* About every half minute at the once-a-second pace poll() needs. */
    if (++_polls >= 30) { _polls = 0; learnCapacity(); }
    return ok;
}

/* ---- TLA2528 ------------------------------------------------------------ */

bool RoboRoverCore3LineArray::tlaWrite(uint8_t reg, uint8_t val)
{
    Wire.beginTransmission(_addr);
    Wire.write((uint8_t)RR_TLA_OP_WRITE);
    Wire.write(reg);
    Wire.write(val);
    if (Wire.endTransmission() != 0) { _err = RR_ERR_NACK; return false; }
    return true;
}

RRError RoboRoverCore3LineArray::begin()
{
    _n = 0;
    if (!present()) return (_err = RR_ERR_NACK);

    /* There is no ID register on this part, so "it ACKed and its status
     * register is readable" is as far as identification goes. The register
     * map lists this board as assumed rather than confirmed - see the header. */
    Wire.beginTransmission(_addr);
    Wire.write((uint8_t)RR_TLA_OP_READ);
    Wire.write((uint8_t)RR_TLA_REG_SYSTEM_ST);
    if (Wire.endTransmission(false) != 0) return (_err = RR_ERR_NACK);
    if (Wire.requestFrom(_addr, (uint8_t)1) != 1) return (_err = RR_ERR_SHORT_READ);
    (void)Wire.read();

    _n = _used;
    return (_err = RR_OK);
}

bool RoboRoverCore3LineArray::poll()
{
    if (!_n) return false;
    clearError();

    /* Convert only the populated inputs. Floating inputs are not merely
     * useless: converting them leaves the sample-and-hold charged from the
     * last real channel, which is what makes unconnected pins appear to
     * track a neighbouring sensor. */
    for (uint8_t c = 0; c < _used; c++) {
        if (!tlaWrite(RR_TLA_REG_CHANNEL_SEL, c)) return false;

        if (Wire.requestFrom(_addr, (uint8_t)2) != 2) {
            _err = RR_ERR_SHORT_READ;
            return false;
        }
        uint16_t hi = (uint16_t)Wire.read();
        uint16_t lo = (uint16_t)Wire.read();
        /* 12-bit result sits in the top of a 16-bit big-endian frame. */
        _ch[c] = (uint16_t)(((hi << 8) | lo) >> 4);
    }
    for (uint8_t c = _used; c < RR_TLA_CHANNELS; c++) _ch[c] = 0;
    return true;
}
