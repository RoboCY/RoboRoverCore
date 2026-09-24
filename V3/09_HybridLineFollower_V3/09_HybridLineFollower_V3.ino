/* 09_HybridLineFollower_V3 - fast PD line following, 5 IR sensors
 *
 * Black line on white. 500 RPM motors driven on raw power: the steering
 * correction goes straight to the wheels every 10 ms.
 * Battery operation, no serial. The lights and the buzzer show the state; the
 * screen shows Kp and Kd, and is only redrawn in full while the wheels are
 * stopped, so it never holds up the control loop.
 *
 * FLOW
 *   power on   -> IDLE      (blue)
 *   OK         -> CALIBRATE (yellow)  auto-sweep, ~1.6 s
 *              -> READY     (green)   or FAIL (red) if a channel is dead
 *   OK         -> RUN       (white)
 *   OK         -> back to READY
 *   arrows     -> change Kp and Kd, any time, no recalibration
 *   *          -> recalibrate
 *   #          -> stop dead
 *
 * CALIBRATION
 *   Put the rover ON the line, centred, ON THE FLOOR - it moves. It pivots
 *   right, then left through centre, then right back, recording min and max
 *   per channel as they cross the line. Threshold per channel is the midpoint
 *   of its own min and max.
 *
 *   A channel whose span comes out under MIN_SPAN turns the LEDs red and the
 *   sketch REFUSES to arm, with one low buzz per failed channel number so it
 *   can be identified without a display. A line follower steering on a dead
 *   channel does not fail visibly, it just drives badly.
 *
 * WHY THE ERROR IS NOT THE THRESHOLD
 *   The midpoint threshold decides LINE PRESENT / LOST, where a hard yes-no
 *   is right. It is deliberately NOT what steers. Five binary sensors give
 *   about nine distinct error values, and at this speed that quantisation IS
 *   the oscillation. Each channel is normalised 0..1000 against its own
 *   calibrated span and the error is their weighted centroid.
 *
 * CORNER BRAKING - why running flat out into a corner looked jittery
 *   Reducing only the inner wheel steers, but it does not slow the rover
 *   down. Carrying full base speed into a curve means the PD has to fight
 *   momentum it should never have had, and the correction it needs is large
 *   and late. That reads as jitter and it is really understeer plus overshoot.
 *
 *   So base is now the STRAIGHT-LINE speed, scaled down by an estimate of how
 *   hard the current curve is:
 *
 *     curve   = curveE*|err| + curveD*|derr|
 *     effBase = base - curve, floored at minBase
 *
 *   |err| says how far off the line we already are. |derr| is the leading
 *   half - the error starts MOVING before it gets large, so the rate term is
 *   what gives any warning at all with sensors that only see the ground
 *   directly beneath them. There is no lookahead on this robot; the rate of
 *   change is the closest thing to it.
 *
 *   The braking is real, not just a smaller number. The drive is slow decay,
 *   so a reduced command actively shorts the motor rather than coasting.
 *
 * ASYMMETRIC SLEW - what makes the braking actually work
 *   The slew limit exists to stop wheelspin on acceleration. Applying the
 *   same limit to DEceleration would throttle exactly the corner braking
 *   above, so the two directions have separate limits. Slip on the way down
 *   matters far less than having the brake arrive in time.
 *
 * THE DERIVATIVE IS MEASURED OVER FOUR STEPS, NOT ONE
 *   The centroid is a ratio of five noisy ADC channels. Over a single 10 ms
 *   step it barely moves, so err - lastErr is mostly noise, and Kd multiplies
 *   that straight into the motors.
 *
 *   That is why raising Kd made fast running WORSE rather than better
 *   damped - an underdamped loop calms down when you add Kd, one that is
 *   amplifying noise gets rougher, and this one got rougher. Differencing
 *   over four steps raises the signal fourfold and leaves the noise alone.
 *
 * TUNING - the arrows change Kp and Kd directly
 *   right/left   Kp up/down
 *   up/down      Kd up/down
 *
 *   Each press moves the value by a sixteenth (6.25%). Kp and Kd show on the
 *   screen, and pressing reset on the Kypruino returns to the starting values.
 *
 *   It starts from Kp 0.75 and Kd 2.00 at full speed, found on the track on a
 *   V3.1 rover with 500 RPM motors. Kp is how hard it steers towards the
 *   line; Kd calms that steering so it does not overshoot. Weaving from side
 *   to side across the line: raise Kd or lower Kp. Running wide in corners:
 *   raise Kp.
 *
 *   curveE and curveD are not on the remote. They set how hard it brakes into
 *   corners; change them in the starting values below.
 */

#include <RoboRoverCore3.h>

/* Must precede the include. A3, as V2. */
#define IR_RECEIVE_PIN  RR_PIN_IR_RECV
#include <TinyIRReceiver.hpp>

#include <Adafruit_NeoPixel.h>
#include <U8g2lib.h>

RoboRoverCore3          rover;
Adafruit_NeoPixel       pixels(RR_NEOPIXEL_COUNT, RR_PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

/* Page buffer, 128 bytes - one 8-pixel tile row at a time.
 *
 * The values line is deliberately tile row 0, so a mid-run update is exactly
 * one buffer: point the buffer at row 0, render, and push 128 bytes with
 * updateDisplayArea. About 11 ms, one skipped control step, invisible.
 *
 * A whole frame is 512 bytes and ~55 ms - five and a half control periods,
 * enough to leave the line - so full redraws happen only with the wheels
 * stopped. A full buffer would make partial updates marginally simpler and
 * cost 512 bytes of RAM, which on a 2 KB part is the difference between
 * comfortable and hunting a stack overflow that presents as a control bug. */
U8G2_SSD1306_128X32_UNIVISION_1_HW_I2C oled(U8G2_R0);

/* ---- remote -------------------------------------------------------------- */

#define RC_OK     0x1Cu
#define RC_STAR   0x16u
#define RC_HASH   0x0Du
#define RC_UP     0x18u
#define RC_DOWN   0x52u
#define RC_LEFT   0x08u
#define RC_RIGHT  0x5Au

/* How far the arrows can take each value. */
#define KP_MIN    RR_Q88(0.10f)
#define KP_MAX    RR_Q88(2.00f)
#define KD_MIN    RR_Q88(0.02f)
#define KD_MAX    RR_Q88(4.00f)


/* ---- tuning -------------------------------------------------------------- */

#define LINE_CH        5
#define LOOP_MS        10u      /* 100 Hz. Five TLA2528 channels are ~3 ms of
                                 * bus on their own, so this is near the
                                 * practical ceiling - and a PD term needs a
                                 * CONSTANT dt far more than a short one. */
/* Acceleration limit for the COMMON-MODE speed, permille per 10 ms step.
 *
 * This is a property of the tyres and the floor, not of how fast you want to
 * go, so it stays fixed while you change Kp and Kd.
 *
 * It replaces the old separate launch ramp. A standing start and a corner
 * exit are the same event - commanded speed rising faster than the tyres can
 * take - and having only the launch limited was why it still span the wheels
 * coming out of bends, where the brake releases and the base recovers with
 * nothing but the per-motor limit in the way.
 *
 * 20/step = 2000 permille/s, so 0 -> full in about half a second. */
#define ACCEL_STEP      20

#define SLEW_UP        100      /* per-motor backstop, mostly catches the
                                 * inner wheel recovering after a corner */
#define SLEW_DOWN      400      /* braking must be allowed to arrive */

/* Derivative baseline, in control steps.
 *
 * err - lastErr over ONE 10 ms step is mostly noise: the centroid barely
 * moves in 10 ms, so the real change is small while the ADC noise is not.
 * Kd then multiplies a signal that is mostly noise, which is why raising Kd
 * made fast running JITTERIER rather than better damped.
 *
 * Differencing over four steps instead multiplies the real signal by four
 * and leaves the noise roughly where it was - about four times the signal to
 * noise, for 20 ms of effective lag. An IIR filter heavy enough to clean up
 * the one-step difference costs more lag than that and damps the genuine
 * signal too.
 *
  * Kd and curveD in the starting values below are scaled for THIS baseline.
 * Change the window and both need rescaling by the same factor. */
#define DERIV_WIN      4

#define US_STOP_MM     200u
#define US_GO_MM       250u     /* hysteresis, or it stutters on the boundary */

#define CAL_SPIN       550      /* pivots on the floor; 350 was too weak to swing the array across the line */
#define CAL_SWEEP_MS   400u
#define MIN_SPAN       200      /* of 4095 */

/* Lost-line search. It keeps DRIVING and arcs toward wherever the line last
 * was, tightening the arc the longer it stays lost, instead of stopping and
 * pivoting on the spot.
 *
 * On a track of linked corners a lost line almost always means the line
 * turned harder than the array could follow, and it is a few centimetres to
 * one side rather than gone. An arc that keeps moving sweeps the sensors
 * across it and rejoins at speed; a pivot throws away all the momentum to
 * solve a problem that a gentle curve solves without stopping.
 *
 * The arc starts wide and tightens, so an easy recovery stays fast and a real
 * loss still ends up pivoting before it gives up. */
#define LOST_HOLD_MS   1200u    /* total search time before stopping */
#define SEARCH_TURN0   300      /* opening differential, permille */
#define SEARCH_TURN_MAX 1400    /* tightest, reached near the end */
#define SEARCH_INNER_MIN (-600) /* how far the inner wheel may reverse */

#define LINE_MIN_TOTAL 300

struct Profile {
    int16_t kp, kd;          /* Q8.8 */
    int16_t base;            /* permille, STRAIGHT-LINE speed */
    int16_t innerFloor;      /* permille, negative lets the inner wheel pivot */
    int16_t curveE, curveD;  /* Q8.8, corner braking from |err| and |derr| */
    int16_t minBase;         /* permille, never brake below this */
};

/* Starting values. Kp and Kd were found by hand on the track, 2026-09-17, on a
 * V3.1 rover with 500 RPM motors. Speed, inner-wheel floor and corner braking
 * are the fastest profile of the earlier V3.0 tuning, which ran well with them.
 *
 * Kd and curveD are scaled for the four-step derivative above, so they are
 * roughly a quarter of what the same feel would need on a one-step difference. */
static Profile prof = {
  /*  kp             kd             base  inner  curveE          curveD          minB */
    RR_Q88(0.75f), RR_Q88(2.00f), 1000,  -400, RR_Q88(1.30f), RR_Q88(1.05f),  540
};

/* ---- state --------------------------------------------------------------- */

enum { ST_IDLE, ST_CAL, ST_READY, ST_FOLLOW, ST_BLOCKED, ST_LOST, ST_FAIL, ST_NOARRAY };
static uint8_t state = ST_IDLE, prevPix = 0xFF;

static uint16_t calMin[LINE_CH], calMax[LINE_CH], calThresh[LINE_CH];
static uint8_t  failCh   = 0xFF;
static bool     calValid = false;

static uint16_t norm[LINE_CH];
static int16_t  err = 0, dTerm = 0;
static int16_t  errHist[DERIV_WIN];
static uint8_t  errIdx = 0;
static int8_t   lostDir = 1;

/* Clearing the history matters on every restart: a stale entry from before a
 * stop makes the first derivative after it enormous, and the first thing the
 * rover does is a full-authority correction it never needed. */
static void derivReset(void)
{
    uint8_t i;
    for (i = 0; i < DERIV_WIN; i++) errHist[i] = 0;
    errIdx = 0;
    dTerm  = 0;
}

static int16_t  cmdL = 0, cmdR = 0;
static int16_t  baseNow  = 0;   /* common-mode speed, traction limited */

static uint32_t lastLoop = 0, calStart = 0, lostStart = 0, lastValDraw = 0;
static bool     valuesDirty = false;
static uint8_t  calPhase = 0;

/* Channel 0 is the RIGHTMOST sensor, so it weighs +1000: a positive error
 * means the line moved RIGHT, and the fix is to slow the RIGHT wheel. */
static int16_t weightOf(uint8_t c)
{
    return (int16_t)(1000 - ((int32_t)c * 2000) / (LINE_CH - 1));
}

static int16_t clampI(int32_t v, int32_t lo, int32_t hi)
{
    if (v < lo) return (int16_t)lo;
    if (v > hi) return (int16_t)hi;
    return (int16_t)v;
}

/* Redraws on arrival at a state where the wheels are stopped. A full frame is
 * ~55 ms, so it must never fire on a transition into a moving state - and the
 * ones that matter to read (READY with the calibration result, FAIL, IDLE)
 * are all stationary anyway. */
static void setState(uint8_t s)
{
    if (s == state) return;
    state = s;
    if (wheelsStopped()) drawAll();
}

/* ---- output -------------------------------------------------------------- */

static void setPixels(uint8_t r, uint8_t g, uint8_t b)
{
    for (uint8_t i = 0; i < RR_NEOPIXEL_COUNT; i++) pixels.setPixelColor(i, r, g, b);
    pixels.show();
}

/* Only on a change. show() disables interrupts for a few hundred microseconds,
 * and doing that every 10 ms would sit on top of both the IR receiver and the
 * I2C timing. */
static void paintPixels(void)
{
    if (state == prevPix) return;
    prevPix = state;
    switch (state) {
    case ST_IDLE:    setPixels(0, 0, 40);   break;   /* blue    */
    case ST_CAL:     setPixels(40, 30, 0);  break;   /* yellow  */
    case ST_READY:   setPixels(0, 40, 0);   break;   /* green   */
    case ST_FOLLOW:  setPixels(20, 20, 20); break;   /* white   */
    case ST_BLOCKED: setPixels(60, 0, 0);   break;   /* red     */
    case ST_LOST:    setPixels(40, 0, 40);  break;   /* magenta */
    case ST_FAIL:    setPixels(60, 0, 0);   break;   /* red     */
    default:         setPixels(60, 20, 0);  break;   /* orange  */
    }
}

static void beep(uint16_t hz, uint16_t ms) { tone(RR_PIN_BUZZER, hz, ms); }

/* ---- display ------------------------------------------------------------- */

/* Q8.8 -> "0.74" without dragging in float printing. */
static void fmtQ88(char *out, uint8_t n, int16_t q)
{
    int16_t h = (int16_t)(((int32_t)q * 100) >> 8);      /* hundredths */
    snprintf(out, n, "%d.%02d", h / 100, abs(h) % 100);
}

static const char *stateName(void)
{
    switch (state) {
    case ST_IDLE:    return "IDLE  OK=calib";
    case ST_CAL:     return "CALIBRATING";
    case ST_READY:   return "READY OK=run";
    case ST_FOLLOW:  return "RUNNING";
    case ST_BLOCKED: return "BLOCKED";
    case ST_LOST:    return "SEARCHING";
    case ST_FAIL:    return "CAL FAIL  *=retry";
    default:         return "NO LINE ARRAY";
    }
}

/* Tile row 0 only: Kp and Kd. This is the row that changes while the rover is
 * moving, and the only one ever pushed mid-run. */
static void renderValues(void)
{
    char kp[8], kd[8], buf[26];
    fmtQ88(kp, sizeof(kp), prof.kp);
    fmtQ88(kd, sizeof(kd), prof.kd);
    snprintf(buf, sizeof(buf), "Kp %s  Kd %s", kp, kd);
    oled.drawStr(0, 7, buf);
}

static void renderAll(void)
{
    char buf[26];
    renderValues();
    oled.drawStr(0, 15, stateName());
    snprintf(buf, sizeof(buf), "speed %d", prof.base);
    oled.drawStr(0, 23, buf);
    oled.drawStr(0, 31, "<> Kp  ^v Kd");
}

/* Full frame: four pages, 512 bytes, ~55 ms. Wheels stopped only. */
static void drawAll(void)
{
    oled.firstPage();
    do { renderAll(); } while (oled.nextPage());
}

/* One tile row: 128 bytes, ~11 ms. Safe while running, on a keypress - never
 * on a timer, or it would cost a control step every frame.
 *
 * setBufferCurrTileRow(0) aims the single-page buffer at the top row, so the
 * y=7 baseline in renderValues() lands inside it. Without that the buffer is
 * still wherever the last full redraw left it and this draws nothing. */
static void drawValuesOnly(void)
{
    oled.setBufferCurrTileRow(0);
    oled.clearBuffer();
    renderValues();
    oled.updateDisplayArea(0, 0, 16, 1);
}

static bool wheelsStopped(void)
{
    return state == ST_IDLE || state == ST_READY ||
           state == ST_FAIL || state == ST_NOARRAY;
}

static void showTuning(void)
{
    if (wheelsStopped()) drawAll();
    else if (state != ST_CAL) valuesDirty = true;
    /* Deferred while moving, so a held arrow does not cost a control step per
     * repeat. Nothing at all during CAL: the sweep is timed, and stalling it
     * would corrupt the min/max it is collecting. */
}

/* Acceleration and deceleration have separate limits. The anti-slip case is
 * spinning the wheels up; braking into a corner is the opposite problem and
 * throttling it would defeat the whole point of the curve term. Compared on
 * magnitude, so it stays correct when a wheel is running in reverse. */
static int16_t slew(int16_t cur, int16_t target)
{
    int16_t d   = target - cur;
    int16_t lim = (abs(target) > abs(cur)) ? SLEW_UP : SLEW_DOWN;
    if (d >  lim) return cur + lim;
    if (d < -lim) return cur - lim;
    return target;
}

/* Every motor command goes through here, so the slew limit cannot be bypassed
 * by a code path that forgot about it. */
static void driveSlewed(int16_t l, int16_t r)
{
    cmdL = slew(cmdL, clampI(l, -1000, 1000));
    cmdR = slew(cmdR, clampI(r, -1000, 1000));
    rover.drive(cmdL, cmdR);
}

/* The one path that skips the slew limit, because a stop should be immediate. */
static void haltNow(void)
{
    cmdL = cmdR = 0;
    baseNow = 0;
    rover.drive(0, 0);
}


/* One sixteenth up or down, never zero, so a press always moves something
 * even at the bottom of the range. */
static int16_t nudge(int16_t v, bool up, int16_t lo, int16_t hi)
{
    int16_t s = (int16_t)(v / 16);
    if (s < 1) s = 1;
    return clampI(up ? (int32_t)v + s : (int32_t)v - s, lo, hi);
}


/* ---- line ---------------------------------------------------------------- */

/* Fills norm[] and err. Returns false when no line is under the array - and
 * also on an I2C failure, which the caller treats the same way: it searches
 * for the line, then stops if it has not found it within LOST_HOLD_MS. */
static bool readLine(void)
{
    int32_t num = 0, den = 0;
    uint8_t c, onLine = 0;

    if (!rover.line.poll()) return false;

    for (c = 0; c < LINE_CH; c++) {
        uint16_t raw  = rover.line.raw(c);
        int32_t  span = (int32_t)calMax[c] - (int32_t)calMin[c];
        int32_t  b;

        if (span < 1) span = 1;         /* cannot happen once armed */

        /* Black reads LOWER, so blackness rises as the reading falls. */
        b = ((int32_t)calMax[c] - (int32_t)raw) * 1000 / span;
        norm[c] = (uint16_t)clampI(b, 0, 1000);

        if (raw < calThresh[c]) onLine++;

        num += (int32_t)weightOf(c) * (int32_t)norm[c];
        den += (int32_t)norm[c];
    }

    if (!onLine || den < LINE_MIN_TOTAL) return false;

    err = (int16_t)(num / den);
    lostDir = (err >= 0) ? 1 : -1;
    return true;
}

/* ---- calibration --------------------------------------------------------- */

static void calBegin(void)
{
    uint8_t c;
    for (c = 0; c < LINE_CH; c++) { calMin[c] = 0xFFFF; calMax[c] = 0; }
    calValid = false;
    failCh   = 0xFF;
    calPhase = 0;
    calStart = millis();

    /* The sweep IS the calibration, so the mode is set here rather than by
     * whichever path started it. Reached from OK in IDLE and from * at any
     * time, and one of those would otherwise leave Motion in IDLE - nSLEEP
     * low, wheels dead, every channel recording the same surface and the span
     * check failing for a reason unrelated to the sensors. */
    rover.motion.setMode(RR_MODE_OPEN_LOOP);

    setState(ST_CAL);
    beep(1500, 60);
}

static void calFeed(void)
{
    uint8_t c;
    if (!rover.line.poll()) return;
    for (c = 0; c < LINE_CH; c++) {
        uint16_t raw = rover.line.raw(c);
        if (raw < calMin[c]) calMin[c] = raw;
        if (raw > calMax[c]) calMax[c] = raw;
    }
}

static void calFinish(void)
{
    uint8_t c;
    bool ok = true;

    haltNow();
    rover.motion.idle();

    for (c = 0; c < LINE_CH; c++) {
        int16_t span = (int16_t)((int32_t)calMax[c] - (int32_t)calMin[c]);
        calThresh[c] = (uint16_t)(((uint32_t)calMin[c] + calMax[c]) / 2u);
        if (span < MIN_SPAN && ok) { ok = false; failCh = c; }
    }

    calValid = ok;
    if (ok) { setState(ST_READY); beep(2200, 120); }
    else    { setState(ST_FAIL);  beep(300, 400);  }
}

/* Right, then left through centre, then right back to where it started. */
static void calStep(uint32_t now)
{
    uint32_t t = now - calStart;

    calFeed();

    if (calPhase == 0) {
        driveSlewed(CAL_SPIN, -CAL_SPIN);
        if (t >= CAL_SWEEP_MS) { calPhase = 1; calStart = now; }
    } else if (calPhase == 1) {
        driveSlewed(-CAL_SPIN, CAL_SPIN);
        if (t >= CAL_SWEEP_MS * 2u) { calPhase = 2; calStart = now; }
    } else {
        driveSlewed(CAL_SPIN, -CAL_SPIN);
        if (t >= CAL_SWEEP_MS) calFinish();
    }
}

/* On FAIL, buzz the failing channel number so it can be read without a
 * display: one low pulse per channel index, plus one, roughly every 3 s. */
static void failChirp(uint32_t now)
{
    static uint32_t next = 0;
    static uint8_t  left = 0;

    if ((int32_t)(now - next) < 0) return;

    if (left == 0) {                 /* start a group after the long rest */
        left = (uint8_t)(failCh + 1u);
        next = now + 400u;
        return;
    }

    beep(350, 150);
    left--;
    next = now + (left ? 300u : 2500u);
}

/* ---- control ------------------------------------------------------------- */

static void follow(uint32_t now)
{
    int32_t p, d, corr, curve;
    int16_t effBase, l, r;

    /* Ultrasonic gate first: nothing below matters if we are about to hit
     * something. */
    if (rover.sensors.hasEcho() && rover.sensors.distanceMm() < US_STOP_MM) {
        haltNow();
        setState(ST_BLOCKED);
        beep(900, 80);
        return;
    }

    if (!readLine()) {
        uint32_t t;
        int32_t  turn;
        int16_t  outer, inner;

        if (state != ST_LOST) { setState(ST_LOST); lostStart = now; beep(700, 60); }
        t = now - lostStart;

        if (t > LOST_HOLD_MS) {
            haltNow();
            rover.motion.idle();
            setState(ST_READY);
            beep(400, 250);
            return;
        }

        /* Keep driving and arc toward the last known side, tightening as the
         * search goes on. Momentum is expensive to rebuild, so it is only
         * thrown away once the wide arcs have already failed. */
        outer = (int16_t)(baseNow ? baseNow / 2 : prof.base / 2);
        turn  = SEARCH_TURN0 + (int32_t)t;
        if (turn > SEARCH_TURN_MAX) turn = SEARCH_TURN_MAX;
        inner = clampI((int32_t)outer - turn, SEARCH_INNER_MIN, 1000);

        if (lostDir > 0) driveSlewed(outer, inner);   /* line went right */
        else             driveSlewed(inner, outer);
        return;
    }

    setState(ST_FOLLOW);

    /* Derivative over DERIV_WIN steps, not one. dt is fixed by the loop, so
     * this is still a plain difference - just over a long enough baseline
     * that the real movement outweighs the ADC noise. */
    {
        int16_t old = errHist[errIdx];
        errHist[errIdx] = err;
        errIdx = (uint8_t)((errIdx + 1u) % DERIV_WIN);
        dTerm = (int16_t)(err - old);
    }

    p = ((int32_t)prof.kp * err)   >> 8;
    d = ((int32_t)prof.kd * dTerm) >> 8;
    corr = clampI(p + d, -2000, 2000);

    /* Corner braking. |err| is how far off we already are; |dTerm| is the only
     * leading indicator available on a robot whose sensors see nothing ahead
     * of the axle. Slow decay means a reduced command shorts the motor, so
     * this brakes rather than coasts. */
    curve = (((int32_t)prof.curveE * abs(err))   >> 8)
          + (((int32_t)prof.curveD * abs(dTerm)) >> 8);

    /* Common-mode speed: rise is traction limited, fall is not.
     *
     * Braking must land immediately or the corner term is pointless, but
     * accelerating faster than ACCEL_STEP just spins the tyres - and a
     * spinning tyre steers nothing, so slip during corner exit costs more
     * than the speed it was trying to gain.
     *
     * The differential is applied AFTER this and is not rate limited here,
     * so the rover keeps full steering authority while the base is still
     * creeping up. Limiting the two together would mean it cannot steer
     * during a launch, which is exactly when it is least stable. */
    effBase = clampI((int32_t)prof.base - curve, prof.minBase, prof.base);

    if (effBase > baseNow) {
        baseNow += ACCEL_STEP;
        if (baseNow > effBase) baseNow = effBase;
    } else {
        baseNow = effBase;
    }
    effBase = baseNow;

    /* Subtractive: the outer wheel keeps effBase, the inner one gives way. */
    if (corr >= 0) {                      /* line is right -> slow the right */
        l = effBase;
        r = clampI((int32_t)effBase - corr, prof.innerFloor, 1000);
    } else {
        l = clampI((int32_t)effBase + corr, prof.innerFloor, 1000);
        r = effBase;
    }

    driveSlewed(l, r);
}

/* ---- keys ---------------------------------------------------------------- */

static void onKey(uint8_t cmd)
{
    /* Live tuning, any time, including mid-run. Pressing reset on the Kypruino
     * restores the starting values. */
    if (cmd == RC_RIGHT || cmd == RC_LEFT || cmd == RC_UP || cmd == RC_DOWN) {
        bool up = (cmd == RC_RIGHT || cmd == RC_UP);

        if (cmd == RC_RIGHT || cmd == RC_LEFT)
            prof.kp = nudge(prof.kp, up, KP_MIN, KP_MAX);
        else
            prof.kd = nudge(prof.kd, up, KD_MIN, KD_MAX);

        /* Pitch tracks the value being moved, so you can hear which way you
         * are going without looking away from the rover. */
        beep((uint16_t)(900 + ((cmd == RC_UP || cmd == RC_DOWN)
                               ? prof.kd : prof.kp * 2)), 40);
        showTuning();
        return;
    }

    if (cmd == RC_OK) {
        if (state == ST_IDLE || state == ST_FAIL) {
            calBegin();
        } else if (state == ST_READY && calValid) {
            rover.motion.setMode(RR_MODE_OPEN_LOOP);
            baseNow = 0;
            derivReset();
            setState(ST_FOLLOW);
            beep(1800, 80);
        } else if (state == ST_FOLLOW || state == ST_LOST || state == ST_BLOCKED) {
            haltNow();
            rover.motion.idle();
            setState(ST_READY);
            beep(600, 120);
        }
    } else if (cmd == RC_STAR) {
        haltNow();
        calBegin();
    } else if (cmd == RC_HASH) {
        haltNow();
        rover.motion.idle();
        setState(calValid ? ST_READY : ST_IDLE);
        beep(500, 150);
    }
}

/* ---- setup / loop -------------------------------------------------------- */

void setup()
{
    pixels.begin();
    pixels.setBrightness(60);
    pixels.clear();
    pixels.show();
    pinMode(RR_PIN_BUZZER, OUTPUT);

    rover.begin();

    /* U8g2 defaults to 400 kHz and shares Wire with the coprocessors, so it
     * would raise THEIR bus speed too. 100 kHz is deliberate on this board. */
    oled.begin();
    oled.setBusClock(100000);
    oled.setFont(u8g2_font_5x8_tf);

    if (rover.line.channels() == 0) setState(ST_NOARRAY);

    initPCIInterruptForTinyReceiver();

    /* Only the ultrasonic sensor is needed here, so the others are switched
     * off. */
    rover.sensors.enable(RR_SEN_ULTRASONIC);
    rover.motion.idle();

    drawAll();
    beep(2000, 120);          /* alive */
}

void loop()
{
    uint32_t now = millis();

    if (TinyReceiverDecode()) {
        uint8_t cmd = TinyIRReceiverData.Command;
        bool    rep = (TinyIRReceiverData.Flags & IRDATA_FLAGS_IS_REPEAT) != 0;
        bool    arrow = (cmd == RC_UP || cmd == RC_DOWN ||
                         cmd == RC_LEFT || cmd == RC_RIGHT);

        /* Repeats are honoured for the arrows ONLY. Sweeping Kp across its
         * range is 36 presses otherwise, which makes tuning miserable. They
         * stay blocked for everything else: a held OK would start and stop
         * the rover nine times a second. */
        if (!rep || arrow) onKey(cmd);
    }

    /* Flush a pending values redraw at no more than 5 Hz. Held arrows arrive
     * at about 9 Hz and each mid-run redraw costs a control step, so they are
     * coalesced - and because this is deferred rather than dropped, the value
     * you stop on is always the value shown. */
    if (valuesDirty && (uint32_t)(now - lastValDraw) >= 200u) {
        valuesDirty = false;
        lastValDraw = now;
        drawValuesOnly();
    }

    if ((uint32_t)(now - lastLoop) < LOOP_MS) { paintPixels(); return; }
    lastLoop = now;

    switch (state) {
    case ST_CAL:
        calStep(now);
        break;

    case ST_FOLLOW:
    case ST_LOST:
        rover.sensors.poll();
        follow(now);
        break;

    case ST_BLOCKED:
        rover.sensors.poll();
        /* Resume only past the far threshold, never at the stop distance. */
        if (!rover.sensors.hasEcho() || rover.sensors.distanceMm() > US_GO_MM) {
            baseNow  = 0;          /* re-ramp, do not snap back to speed */
            derivReset();
            setState(ST_FOLLOW);
        }
        break;

    case ST_FAIL:
        failChirp(now);
        break;

    default:
        break;
    }

    paintPixels();
}
