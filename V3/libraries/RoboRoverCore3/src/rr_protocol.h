/* rr_protocol.h - RoboRover V3.0 coprocessor I2C contract
 *
 * SINGLE SOURCE OF TRUTH. Compiled into all three consumers:
 *   - Motion firmware   (CH32V003, U15)
 *   - Sensors firmware  (CH32V003, U3)
 *   - Host library      (ATmega328P / ESP32, Arduino)
 *
 * Implements RoboRover_V3_RegMap_V1.1.md. If the two disagree, the .md governs
 * and this file is the bug.
 *
 * Plain C99, no dependencies, no MCU headers. Safe on every target.
 */

#ifndef RR_PROTOCOL_H
#define RR_PROTOCOL_H

#include <stdint.h>

/* ---- protocol identity ------------------------------------------------- */

#define RR_PROTOCOL_VERSION   0x11u   /* 0xMN -> V1.1 */

#define RR_ADDR_MOTION        0x28u
#define RR_ADDR_SENSORS       0x29u

#define RR_WHOAMI_MOTION      0x4Du   /* 'M' */
#define RR_WHOAMI_SENSORS     0x53u   /* 'S' */

/* Neighbours on the shared bus. Listed so address allocation stays honest. */
#define RR_ADDR_LIS2DH12      0x19u   /* 0x18 if SJ5 closed */
#define RR_ADDR_BQ27441       0x55u

/* AVR Wire buffer is 32 B total (addr + reg + payload). Never exceed. */
#define RR_MAX_PAYLOAD        30u

/* ---- common register block (both chips) -------------------------------- */

#define RR_REG_WHO_AM_I       0x00u  /* R  1 */
#define RR_REG_PROTOCOL_VER   0x01u  /* R  1 */
#define RR_REG_FW_VERSION     0x02u  /* R  2  [major, minor] */
#define RR_REG_STATUS         0x04u  /* R  1 */
#define RR_REG_FAULT          0x05u  /* R/W1C 1 */
#define RR_REG_CONTROL        0x06u  /* W  1, self-clearing */
#define RR_REG_INT_CFG        0x07u  /* RW 1 */
#define RR_REG_INT_FLAGS      0x08u  /* R/W1C 1 */
#define RR_REG_UPTIME_MS      0x0Cu  /* R  4, uint32 */

/* Status LED, added after V1.1 in unused space: the breathing LED each chip
 * drives (Motion LED1, Sensors LED2). Off is for photos and for readings of
 * the light sensors that it could otherwise brighten. Motion 1.12 and
 * Sensors 1.10; an older chip reads 0 and ignores writes. */
#define RR_REG_STATUS_LED     0x09u  /* RW 1 */
#define RR_STATUS_LED_OFF     0u
#define RR_STATUS_LED_BREATHE 1u      /* slow when healthy, fast on a fault */

/* CONTROL bits */
#define RR_CTRL_SOFT_RESET    (1u << 0)
#define RR_CTRL_CLEAR_FAULT   (1u << 1)
#define RR_CTRL_LATCH         (1u << 2)
#define RR_CTRL_LOAD_DEFAULTS (1u << 3)

/* INT_CFG / INT_FLAGS share bit positions, meaning is per-chip */
#define RR_INT_FAULT          (1u << 0)  /* both */
#define RR_INT_M_FAILSAFE     (1u << 1)  /* motion: never raised since 1.12 */
#define RR_INT_M_STALL        (1u << 2)  /* motion */
#define RR_INT_M_MOVE_DONE    (1u << 3)  /* motion */
#define RR_INT_S_NEW_SAMPLE   (1u << 1)  /* sensors */
#define RR_INT_S_OBST_CHANGE  (1u << 2)  /* sensors */

/* INT_CFG at power-up. LOAD_DEFAULTS leaves INT_CFG alone, so a host that
 * wants the power-up state back writes these itself. */
#define RR_M_DEF_INT_CFG      (RR_INT_FAULT | RR_INT_M_STALL | RR_INT_M_MOVE_DONE)
#define RR_S_DEF_INT_CFG      (RR_INT_FAULT | RR_INT_S_OBST_CHANGE)

/* FAULT bits, low nibble common to both chips */
#define RR_FAULT_WATCHDOG     (1u << 0)
#define RR_FAULT_I2C_RECOVER  (1u << 1)
#define RR_FAULT_CONFIG       (1u << 4)

/* ---- motion, 0x28 ------------------------------------------------------ */

#define RR_M_MODE             0x10u  /* RW 1 */
#define RR_M_M1_CMD           0x12u  /* RW 2  int16 */
#define RR_M_M2_CMD           0x14u  /* RW 2  int16 */
#define RR_M_RAMP             0x16u  /* RW 2  uint16, 0 = off */
#define RR_M_FAILSAFE_MS      0x18u  /* reserved: the command failsafe was
                                      * removed in Motion 1.12; reads 0 */
#define RR_M_MIN_DUTY         0x1Au  /* RW 2  uint16 permille */
#define RR_M_MOTOR_CFG        0x1Cu  /* RW 1  output polarity, see below */

/* Which motors drive backwards for a positive command. The gearmotors are
 * mirror-mounted, so at least one of these is always set; which one depends
 * on how the connectors were crimped, and that is not knowable from the
 * schematic. Runtime rather than a rebuild, because getting it wrong is
 * obvious in one second and fixing it should take about as long.
 *
 * Inverts the BRIDGE only: DUTY_M1/DUTY_M2 keep reporting the commanded
 * value, so a positive command reads back positive either way. Encoder sign
 * is a separate concern - that is ENC_CFG.
 *
 * Added after V1.1 in previously unused space. Every existing field keeps its
 * meaning and the power-on default matches the old built-in behaviour, so
 * PROTOCOL_VERSION is unchanged and an older host is unaffected. */
#define RR_M_INVERT_M1        (1u << 0)
#define RR_M_INVERT_M2        (1u << 1)
#define RR_M_DEF_MOTOR_CFG    RR_M_INVERT_M2   /* M2 = left, mirror-mounted */
#define RR_M_KP               0x20u  /* RW 2  Q8.8 */
#define RR_M_KI               0x22u  /* RW 2  Q8.8 */
#define RR_M_KD               0x24u  /* RW 2  Q8.8 */
#define RR_M_I_LIMIT          0x26u  /* RW 2  uint16 permille */
#define RR_M_PID_RATE_HZ      0x28u  /* R  2  uint16 */
#define RR_M_ENC_CFG          0x2Au  /* RW 1 */
#define RR_M_TICKS_PER_REV    0x2Bu  /* RW 2  uint16 */

/* Encoder diagnostics, read-only. Counting has three places to fail and
 * they look identical from the host - the count just stays at zero:
 *   the pins never move      -> wiring, power, or the input has no level
 *   pins move, no interrupts -> EXTI configuration
 *   interrupts, no counting  -> the decode, or a stuck previous state
 * ENC_RAW reads the four encoder pins as they physically are, before any
 * SWAP or INVERT is applied. ENC_IRQ counts edges the handler has actually
 * serviced and rolls over at 255; two reads a second apart tell you whether
 * anything is arriving at all. */
#define RR_M_ENC_RAW          0x2Du  /* R  1  live pin levels */
#define RR_M_ENC_IRQ          0x2Eu  /* R  1  EXTI edges serviced, wraps */

#define RR_ENC_RAW_M1_A       (1u << 0)   /* PC7, ENC_RIGHT_A */
#define RR_ENC_RAW_M1_B       (1u << 1)   /* PD4, ENC_RIGHT_B */
#define RR_ENC_RAW_M2_A       (1u << 2)   /* PC6, ENC_LEFT_A  */
#define RR_ENC_RAW_M2_B       (1u << 3)   /* PD3, ENC_LEFT_B  */

/* telemetry block: 0x30..0x3F, 16 B, auto-latched on read from base */
#define RR_M_TELEM_BASE       0x30u
#define RR_M_TELEM_LEN        16u
#define RR_M_ENC_M1           0x30u  /* R  4  int32 */
#define RR_M_ENC_M2           0x34u  /* R  4  int32 */
#define RR_M_SPEED_M1         0x38u  /* R  2  int16 ticks/s */
#define RR_M_SPEED_M2         0x3Au  /* R  2  int16 ticks/s */
#define RR_M_DUTY_M1          0x3Cu  /* R  2  int16 permille */
#define RR_M_DUTY_M2          0x3Eu  /* R  2  int16 permille */

#define RR_M_ENC_ZERO         0x40u  /* W  1 */
#define RR_M_ENC_ZERO_M1      (1u << 0)
#define RR_M_ENC_ZERO_M2      (1u << 1)

/* ---- position mode, 0x44..0x5D ----------------------------------------
 * An outer position loop wrapped around the existing speed PID:
 *   target ticks -> position error -> speed setpoint -> speed PID -> duty
 * The host issues a move and is free to do other things; MOVE_DONE fires
 * MOTION_INT when it completes. */

#define RR_M_M1_TARGET        0x44u  /* RW 4  int32 ticks */
#define RR_M_M2_TARGET        0x48u  /* RW 4  int32 ticks */
#define RR_M_CRUISE           0x4Cu  /* RW 2  uint16 ticks/s ceiling in a move */
#define RR_M_POS_KP           0x4Eu  /* RW 2  Q8.8, ticks error -> ticks/s */
#define RR_M_POS_TOL          0x50u  /* RW 2  uint16 ticks, "close enough" */
#define RR_M_POS_MIN_SPD      0x52u  /* RW 2  uint16 ticks/s creep floor */
#define RR_M_WHEEL_CIRC       0x54u  /* RW 2  uint16, 0.01 mm per wheel rev */
#define RR_M_TRACK_WIDTH      0x56u  /* RW 2  uint16, 0.01 mm between wheels */
#define RR_M_MOVE_MM          0x58u  /* W  2  int16 mm, straight; starts the move */
#define RR_M_TURN_DEG         0x5Au  /* W  2  int16 deg, spin in place; starts */
#define RR_M_MOVE_CTRL        0x5Cu  /* W  1 */

/* Speed feed-forward, added after V1.1 in unused space. The speed PID starts
 * each step from KF * |setpoint| + FF_OFFSET (with the setpoint's sign) and
 * corrects from there. Every existing field keeps its meaning, and an older
 * chip reads both as 0 - feed-forward off - so PROTOCOL_VERSION is unchanged.
 * Motion 1.8 and later. */
#define RR_M_KF               0x5Eu  /* RW 2  Q8.8, permille per tick/s */
#define RR_M_FF_OFFSET        0x60u  /* RW 2  uint16 permille */

/* How hard a move holds the two wheels level, in ticks/s of correction per
 * tick one wheel is ahead (as a share of each wheel's own distance). 0 lets
 * each wheel run to its count on its own, which S-curves. Motion 1.10; an
 * older chip reads 0 and ignores writes. From Motion 1.15 it also holds
 * closed-loop speeds to their ratio, restarting whenever a command changes. */
#define RR_M_SYNC_KP          0x62u  /* RW 2  Q8.8 */

/* Motion 1.12, same terms. POS_ACCEL is how fast a move may speed up; slowing
 * down follows POS_KP. STALL_MS / STALL_DUTY say when a wheel counts as
 * stalled: pushed with at least STALL_DUTY and not turning for STALL_MS, in
 * closed loop and during moves. STALL_MS = 0 turns stall detection off. */
#define RR_M_POS_ACCEL        0x64u  /* RW 2  uint16 ticks/s per s */
#define RR_M_STALL_MS         0x66u  /* RW 2  uint16 ms, 0 = off */
#define RR_M_STALL_DUTY       0x68u  /* RW 2  uint16 permille */
#define RR_M_DEF_POS_ACCEL    4000u   /* 8000 before Motion 1.14 */
#define RR_M_DEF_STALL_MS     400u
#define RR_M_DEF_STALL_DUTY   700u

/* What a stall does. STOP ends a move, or zeroes both speed commands in
 * closed loop, and brakes: a gearmotor held at full power heats the motor and
 * the driver and drains the battery for nothing. Both wheels, since stopping
 * one leaves the rover pivoting round the blocked one. The STALL fault stays
 * set until cleared; the next command runs normally. Open loop is never
 * stopped - there the host chose the power. REPORT only raises the fault. */
#define RR_M_STALL_ACTION     0x6Au  /* RW 1 */
#define RR_STALL_REPORT       0u
#define RR_STALL_STOP         1u      /* default */

/* Accepted ranges, inclusive, from Motion 1.12. A write outside its range is
 * refused: the old value stays and FAULT.CONFIG is set, so a typo cannot
 * leave the rover with no acceleration, no cruise speed or a wheel of zero
 * size. Where 0 is listed separately it switches the feature off. Speeds go
 * well past the 500 RPM motors' ~5000 ticks/s so a faster encoder still fits;
 * gains stop where the loops were already unusable on blocks. */
#define RR_M_RAMP_MIN         200u     /* or 0 = no ramp; command units per s */
#define RR_M_RAMP_MAX         30000u
#define RR_M_MIN_DUTY_MAX     500u     /* 0.. permille */
#define RR_M_KP_MAX           1024     /* 0.. Q8.8 = 4.0 */
#define RR_M_KI_MAX           256      /* 0.. Q8.8 = 1.0 */
#define RR_M_KD_MAX           1024     /* 0.. Q8.8 = 4.0 */
#define RR_M_I_LIMIT_MAX      1000u    /* 0.. permille */
#define RR_M_KF_MAX           256      /* 0.. Q8.8 = 1.0, 0 = no feed-forward */
#define RR_M_FF_OFFSET_MAX    500u     /* 0.. permille */
#define RR_M_TICKS_MIN        12u      /* ticks per wheel revolution */
#define RR_M_TICKS_MAX        30000u
#define RR_M_CRUISE_MIN       50u      /* ticks/s */
#define RR_M_CRUISE_MAX       5000u    /* no-load top speed of the stock
                                        * 500 RPM motors on the 6 V rail is
                                        * ~4950; 30000 before Motion 1.14 */
#define RR_M_POS_KP_MIN       64       /* Q8.8 = 0.25 */
#define RR_M_POS_KP_MAX       2560     /* Q8.8 = 10.0 */
#define RR_M_SYNC_KP_MAX      2560     /* 0.. Q8.8 = 10.0, 0 = wheels independent */
#define RR_M_POS_TOL_MIN      1u       /* ticks */
#define RR_M_POS_TOL_MAX      1000u
#define RR_M_POS_MIN_SPD_MIN  20u      /* ticks/s */
#define RR_M_POS_MIN_SPD_MAX  3000u
#define RR_M_LENGTH_MIN       3000u    /* 0.01 mm: wheel circumference and */
#define RR_M_LENGTH_MAX       65535u   /* track, 30.00 .. 655.35 mm */
#define RR_M_POS_ACCEL_MIN    500u     /* ticks/s per s */
#define RR_M_POS_ACCEL_MAX    30000u
#define RR_M_STALL_MS_MIN     100u     /* or 0 = off; ms */
#define RR_M_STALL_MS_MAX     5000u
#define RR_M_STALL_DUTY_MIN   100u     /* permille */
#define RR_M_STALL_DUTY_MAX   1000u

#define RR_MOVE_START         (1u << 0)  /* commit M1_TARGET / M2_TARGET */
#define RR_MOVE_ABORT         (1u << 1)  /* stop now, clear MOVING */

/* mm and degrees are converted on the coprocessor, so every host gets real
 * units for free and the geometry lives next to the encoders that own it.
 * Distance is clamped to +/-10 m so the tick conversion stays inside int32. */
#define RR_MOVE_MAX_MM        10000
#define RR_TURN_MAX_DEG       3600

/* Defaults measured on V3.1 rovers: 44 mm wheels -> 138.23 mm circumference;
 * on the floor 500 mm and 1 m moves came out exact with it. The tyres are
 * 115 mm apart, but they scrub sideways in a spin and the rover turns as if
 * they were further apart. 115.52 mm is the average of five V3.1 rovers
 * calibrated with the library's TurnCalibration - four 90 degree turns, each
 * with its own stop - which ranged 115.07 to 116.03: every one within 0.5 mm,
 * under half a degree per 90. The 119.3 of Motion 1.17-1.20 came from single
 * long spins, which stop once, and overshot 90s by about 3 degrees each.
 * The finish settles within 16 ticks (about 4 mm) and never creeps
 * slower than 250 ticks/s, which floor friction can stall. */
#define RR_M_DEF_WHEEL_CIRC   13823u
#define RR_M_DEF_TRACK_WIDTH  11552u
#define RR_M_DEF_POS_TOL      16u
#define RR_M_DEF_POS_MIN_SPD  250u

/* MODE values */
#define RR_MODE_IDLE          0u  /* nSLEEP low, drivers asleep */
#define RR_MODE_COAST         1u
#define RR_MODE_BRAKE         2u
#define RR_MODE_OPEN_LOOP     3u  /* *_CMD = permille duty, +/-1000 */
#define RR_MODE_CLOSED_LOOP   4u  /* *_CMD = ticks/s */
#define RR_MODE_POSITION      5u  /* drive to *_TARGET, then hold */

/* ENC_CFG bits */
#define RR_ENC_INVERT_M1      (1u << 0)
#define RR_ENC_INVERT_M2      (1u << 1)
#define RR_ENC_SWAP_M1        (1u << 2)
#define RR_ENC_SWAP_M2        (1u << 3)
#define RR_ENC_X4             (1u << 4)

/* motion STATUS bits */
#define RR_MSTAT_READY        (1u << 0)
#define RR_MSTAT_DRV_ENABLED  (1u << 1)
#define RR_MSTAT_CLOSED_LOOP  (1u << 2)
#define RR_MSTAT_FAILSAFE     (1u << 3)  /* never set since Motion 1.12 */
#define RR_MSTAT_SATURATED    (1u << 4)
#define RR_MSTAT_MOVING       (1u << 5)
#define RR_MSTAT_MOVE_DONE    (1u << 6)

/* motion FAULT bits (plus common low nibble) */
#define RR_MFAULT_STALL_M1    (1u << 2)
#define RR_MFAULT_STALL_M2    (1u << 3)

/* motion defaults */
#define RR_M_DEF_PID_RATE_HZ  200u

/* ---- sensors, 0x29 ----------------------------------------------------- */

#define RR_S_SENSOR_EN        0x10u  /* RW 1 */
#define RR_S_SAMPLE_HZ        0x11u  /* RW 1 */
#define RR_S_OBST_THRESH_A    0x12u  /* RW 2  uint16, PA1 channel */
#define RR_S_OBST_THRESH_B    0x14u  /* RW 2  uint16, PA2 channel */
#define RR_S_OBST_HYST        0x16u  /* RW 1  percent of threshold */
#define RR_S_IR_SAMPLES       0x17u  /* RW 1  accumulation count */
#define RR_S_US_MAX_MM        0x18u  /* RW 2  uint16 */
#define RR_S_US_PERIOD_MS     0x1Au  /* RW 1  uint8 */
#define RR_S_LED_MODE         0x1Bu  /* RW 1 */
#define RR_S_LED_STATE        0x1Cu  /* RW 1, 1 = lit; firmware inverts */

/* Obstacle calibration, added after V1.1 in unused space. The empty reading
 * depends on the floor under the sensors (a white floor adds ~250) and on
 * each part's tolerance, so no fixed threshold suits every rover. Writing 1
 * to OBST_CAL measures what each sensor reads now - the floor, with nothing
 * in front - and sets OBST_THRESH_A/B to that plus OBST_MARGIN. It reads 1
 * until done (8 samples, 160 ms at 50 Hz). The chip also calibrates at
 * power-up. Thresholds stay raw readings throughout, so the host can still
 * set its own. Only calibration changes them: a still obstacle stays
 * detected. An older chip reads 0 here and never calibrates. Sensors 1.9. */
#define RR_S_OBST_CAL         0x1Du  /* RW 1  write 1 to calibrate; 1 = busy */
#define RR_S_OBST_MARGIN      0x1Eu  /* RW 2  uint16, added to the floor reading */
#define RR_S_OBST_FLOOR_A     0x20u  /* R  2  uint16, floor at the last calibration */
#define RR_S_OBST_FLOOR_B     0x22u  /* R  2  uint16 */
#define RR_S_DEF_OBST_MARGIN  200u

/* telemetry block: 0x30..0x3D, 14 B, auto-latched on read from base */
#define RR_S_TELEM_BASE       0x30u
#define RR_S_TELEM_LEN        14u
#define RR_S_OBST_FLAGS       0x30u  /* R  1 */
#define RR_S_OBST_A           0x32u  /* R  2  uint16, ambient-subtracted */
#define RR_S_OBST_B           0x34u  /* R  2  uint16, ambient-subtracted */
#define RR_S_US_MM            0x36u  /* R  2  uint16, 0xFFFF = no echo */
#define RR_S_US_AGE_MS        0x38u  /* R  2  uint16 */
#define RR_S_LDR_PD4          0x3Au  /* R  2  uint16 raw ADC */
#define RR_S_LDR_PD5          0x3Cu  /* R  2  uint16 raw ADC */

#define RR_US_NO_ECHO         0xFFFFu

/* SENSOR_EN bits */
#define RR_SEN_OBSTACLE       (1u << 0)
#define RR_SEN_ULTRASONIC     (1u << 1)
#define RR_SEN_LDR            (1u << 2)

/* Diagnostic. With this set, OBST_A reports the raw DARK accumulator and
 * OBST_B the raw LIT accumulator from the most recent channel-A reading,
 * instead of the ambient-subtracted result, and detection is frozen.
 *
 * It exists because |dark - lit| alone cannot distinguish a real reflection
 * from ambient drift between the two sampling windows, or from the emitter
 * current pulling on VDD - which is also the ADC reference. Those have very
 * different fixes and identical symptoms in the subtracted value.
 *
 * Uses a spare bit of an existing register and defaults off, so the layout
 * and every existing field keep their meaning: PROTOCOL_VERSION is not
 * bumped and an older host is unaffected. */
#define RR_SEN_IR_DIAG        (1u << 7)

/* OBST_FLAGS bits */
#define RR_OBST_A_DETECT      (1u << 0)
#define RR_OBST_B_DETECT      (1u << 1)

/* LED_MODE values */
#define RR_LED_OFF            0u
#define RR_LED_AUTO           1u  /* follows obstacle detection */
#define RR_LED_HOST           2u  /* driven by LED_STATE */

/* LED_STATE bits (logical: 1 = lit; PC6/PC7 are active-low in hardware) */
#define RR_LED_28             (1u << 0)
#define RR_LED_29             (1u << 1)

/* sensors STATUS bits */
#define RR_SSTAT_READY        (1u << 0)
#define RR_SSTAT_US_INFLIGHT  (1u << 1)
#define RR_SSTAT_NEW_SAMPLE   (1u << 2)

/* sensors FAULT bits (plus common low nibble) */
#define RR_SFAULT_US_TIMEOUT  (1u << 2)
#define RR_SFAULT_IR_SAT      (1u << 3)

/* sensors defaults */
#define RR_S_DEF_SAMPLE_HZ    50u
#define RR_S_DEF_IR_SAMPLES   32u
#define RR_S_DEF_US_MAX_MM    4000u
#define RR_S_DEF_US_PERIOD_MS 60u

/* Accepted ranges, inclusive, from Sensors 1.10 - same rule as Motion: a
 * write outside is refused, the old value stays and FAULT.CONFIG is set. */
#define RR_S_SAMPLE_HZ_MIN    1u
#define RR_S_SAMPLE_HZ_MAX    200u     /* a sample takes ~3 ms at 32 IR samples */
#define RR_S_THRESH_MIN       1u       /* 0 would trip on nothing */
#define RR_S_HYST_MAX         90u      /* percent */
#define RR_S_IR_SAMPLES_MIN   1u
#define RR_S_IR_SAMPLES_MAX   64u      /* sums must fit 16 bits */
#define RR_S_US_MAX_MM_MIN    20u      /* the sensor's own range, 2 cm .. 4 m */
#define RR_S_US_MAX_MM_MAX    4000u
#define RR_S_US_PERIOD_MIN    30u      /* ms; shorter and echoes overlap */
#define RR_S_OBST_MARGIN_MIN  50u      /* the noise alone is about +-25 */
#define RR_S_OBST_MARGIN_MAX  10000u

/* ---- endian helpers ---------------------------------------------------- */
/* Wire format is little-endian regardless of host. Both current targets are
 * LE, but going through these keeps the contract explicit and portable. */

static inline void rr_put_u16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)(v >> 8);
}

static inline uint16_t rr_get_u16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static inline void rr_put_u32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)((v >> 8) & 0xFFu);
    p[2] = (uint8_t)((v >> 16) & 0xFFu);
    p[3] = (uint8_t)((v >> 24) & 0xFFu);
}

static inline uint32_t rr_get_u32(const uint8_t *p)
{
    return (uint32_t)p[0]
         | ((uint32_t)p[1] << 8)
         | ((uint32_t)p[2] << 16)
         | ((uint32_t)p[3] << 24);
}

/* Q8.8 fixed point. Gains cross the bus as integers so the host needs no FPU. */
#define RR_Q88(x)             ((int16_t)((x) * 256.0f + ((x) < 0 ? -0.5f : 0.5f)))
#define RR_Q88_TO_FLOAT(q)    ((float)(q) / 256.0f)

#endif /* RR_PROTOCOL_H */
