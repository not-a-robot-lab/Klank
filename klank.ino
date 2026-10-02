// Klank - percussion voice for AVR32DD14 (DxCore, 24 MHz internal)
//
// Signal path: DAC0 (PD6) -> C3/R4 -> analog bandpass (U2, tuned by POT1 "F") -> out
//
// Voices (selected with KEY A, stored in EEPROM):
//   1 LOW_SNARE   909-style snare: three membrane modes with pitch drop + high-passed noise
//   2 CLAP        three noise bursts 10 ms apart + noise tail
//   3 COWBELL     two square waves, 540 Hz + 800 Hz
//   4 HIHAT808    XOR of six inharmonic square waves (808 metal)
//   5 HIHAT909    TR-909 open hi-hat sample (hat909.h)
//   6 HIGH_SNARE  KR-55-style snare: 225 Hz + octave, band-passed noise
//
// Pins:
//   PA0          KEY1 A   select voice
//   PD4          KEY2 B   manual trigger
//   PA1          trigger in (inverted by Q1, falling edge = hit)
//   PD7          decay pot
//   PD5          decay CV ((Vcv + 5 V) / 2, 0 V CV = mid scale)
//   PC1/PC2/PC3  charlieplexed LEDs, one LED on at a time:
//                  after power-up and KEY A: LED of the selected voice for SHOW_VOICE_MS
//                    (LED1 = bottom = LOW_SNARE .. LED6 = top = HIGH_SNARE); takes priority
//                  envelope running: LED6 (top, loud) down to LED1 (bottom, -36 dB), 6 dB per LED
//                  otherwise all off
//
// KEY A: the first press shows the current voice, further presses while it is shown select the next.

#include <Arduino.h>
#include <EEPROM.h>
#include "hat909.h"

#if defined(MILLIS_USE_TIMERB0)
#error "TCB0 is the audio sample clock; choose another millis()/micros() timer"
#endif

// ---- test switches ----
#define AUTOTEST       0                          // AUTOTEST_VOICE triggered every 1.5 s, decay cycling 10/40/70/100 %
#define AUTOTEST_VOICE HIHAT909
#define LEDS_OFF       0                          // never light an LED
#define ISR_DIAG       0                          // 1: measure ISR duration into .noinit RAM (read over UPDI), 2: count only
// #define DECAY_FIXED 512                        // ignore pot and CV, use this decay value (0..1023)

// ---- timing ----
#define SAMPLE_RATE   40000UL
#define CTRL_DIV      20                          // control period: 20 samples, work on the even ones
#define CTRL_RATE     (SAMPLE_RATE / CTRL_DIV)    // 2000 Hz

// ---- DAC ----
// The DAC output can source 1 mA but sink only 1 uA (datasheet table 38-23): driving C3/R4 (5k6) it
// flattened every negative half-wave. A 6k8 pull-down from PD6 to GND sinks for it; with the DAC
// centred at 3.0 V and limited to +-1.3 V the pull-down always carries more than the R4 current
// (1.7 V / 6k8 = 0.25 mA > 1.3 V / 5k6 = 0.23 mA) and the DAC sources at most 0.86 mA (4.3 V).
#define DAC_REF       VREF_REFSEL_VDD_gc          // 5 V full scale
#define DAC_MID       614                         // 3.0 V
#define DAC_SWING     266                         // +-1.3 V
#define LV(x)         ((uint16_t)((x) * (uint32_t)DAC_SWING / 512))   // LEVEL values were made for +-512

// ---- envelopes, UI ----
#define DECAY_SCALE   0.7f                        // scales all decay-controlled time constants (pot + CV range)
#define ENV_MAX       0x00FFFFFFUL
#define ENV_TAIL      (ENV_MAX >> 8)              // -48 dB: below this envelopes release quickly
#define CLAP_SPACING  (CTRL_RATE / 100)           // 10 ms between clap bursts
#define SHOW_VOICE_MS 4000                        // how long KEY A shows the selected voice
#define SAVE_DELAY_MS 3000                        // voice is stored this long after the last KEY A press
#define EE_VOICE      0                           // EEPROM address of the selected voice
#define KEY_LOCK_TICKS (CTRL_RATE * 20 / 1000)    // 20 ms debounce for KEY A and KEY B, in control ticks

enum { LOW_SNARE, CLAP, COWBELL, HIHAT808, HIHAT909, HIGH_SNARE, NUM_VOICES };

// Output level per voice (Q8, 256 = 1.0): scales each voice so that its maximum over the
// F range is 5 Vp at the output (10 Vpp). Simulated for C1 = C2 = 22n, R3 = R5 = 1k,
// POT1 A50k (F 242 Hz..12.5 kHz), R4 = 5k6 (BW 1.29 kHz), R1/R2 = 3k3/10k, DAC_REF = VDD;
// recalculate when those change.
static const uint16_t LEVEL[NUM_VOICES] = {
  /* LOW_SNARE  */ LV(258),   // 4.96 Vp -> 5.0 (pitch drop, HP noise 1.3 dB below tone; raw peak 365/512)
  /* CLAP       */ LV(213),   // 6.02 Vp -> 5.0
  /* COWBELL    */ LV(127),   // 10.06 Vp -> 5.0
  /* HIHAT808   */ LV(163),   // 7.83 Vp -> 5.0
  /* HIHAT909   */ LV(225),   // sample x5 (peak 580) x 225/256 = 510: gain stays below 1.0
  /* HIGH_SNARE */ LV(582),   // 3.12 Vp -> 7.1 (+3 dB over the others, by ear; tone:noise 1.1)
};

// phase increment for a 16-bit accumulator at SAMPLE_RATE
#define INC(f)  ((uint16_t)((f) * 65536.0 / SAMPLE_RATE + 0.5))

// COWBELL: two squares
static const uint16_t cowbellInc[2] = { INC(540.0), INC(800.0) };

// HIHAT808: six inharmonic squares (the 808 metal set 205.3 .. 800 Hz, one octave up)
static const uint16_t hatInc[6] = {
  INC(410.6), INC(608.8), INC(739.2), INC(1045.4), INC(1080.0), INC(1600.0)
};

// LOW_SNARE: circular membrane modes (Bessel zeros): fundamental, x1.593, x2.295.
// All three start x1.33 and fall back with tau 5.14 ms (pitch drop). The two upper modes, at 1/8 and
// 1/16 level, decay faster (tau 15 ms) than the fundamental (40 ms) so they only colour the attack.
// (At 1/2 and 1/4 they stood almost 2:3 and were heard as a virtual 117 Hz pitch that jumped up to
// 148 Hz when they faded.) Noise high-passed at 1.57 kHz, tone 1.3 dB above the noise.
static const uint16_t snareBase[3]  = { INC(147.7), INC(235.3), INC(339.0) };
static const uint16_t snareBase3[3] = { INC(147.7) / 3, INC(235.3) / 3, INC(339.0) / 3 };   // pitch-drop depth x1/3
#define SNARE_KP      ((uint16_t)(65536.0 * (1.0 - __builtin_exp(-1000.0 / (5.14 * CTRL_RATE)))))   // pitch envelope, tau 5.14 ms
#define SNARE_KUP     ((uint16_t)(65536.0 * (1.0 - __builtin_exp(-1000.0 / (15.0 * CTRL_RATE)))))   // upper-mode envelope, tau 15 ms

// HIGH_SNARE: from a KR-55 recording: 245 Hz + octave, fixed pitch, tone tau 27 ms;
// noise band 1.6..6 kHz, held 40 ms then decaying; tone 12 dB above the noise
static const uint16_t krInc[2] = { INC(225.0), INC(450.0) };   // x0.92 of the recording, by ear
#define KR_HOLD       (CTRL_RATE * 40 / 1000)     // noise hold, control ticks

// Each voice has two envelopes: env0 (fixed time constant, or following the decay control when
// fixedTauMs = 0) and env1 (always follows the decay control, tauMinMs..tauMaxMs).
struct VoiceDef {
  float fixedTauMs;   // env0 time constant (0 = env0 follows the decay control)
  float tauMinMs;     // decay-controlled time constant range
  float tauMaxMs;
};

static const VoiceDef voices[NUM_VOICES] = {
  /* LOW_SNARE   tone fixed, noise decay      */ { 39.8f,  51.0f,  330.0f },
  /* CLAP        bursts fixed, tail decay     */ {  4.0f,  30.0f,  331.0f },
  /* COWBELL     attack fixed, tail decay     */ {  8.0f,  40.0f,  800.0f },
  /* HIHAT808    decay                        */ {  0.0f,   8.0f,  400.0f },
  /* HIHAT909    decay over the sample        */ {  0.0f,  15.0f, 5000.0f },   // fully clockwise: no decay, whole sample
  /* HIGH_SNARE  tone fixed, noise hold+decay */ { 27.0f,  10.0f,  150.0f },
};

// ---- shared between ISR and loop ----
static volatile uint8_t voice = LOW_SNARE;
static volatile uint8_t saveRequest;      // set by the ISR after a voice change, EEPROM write in loop()
static uint16_t showVoiceTicks;           // written by loop() with interrupts off

// ---- control state (ISR only) ----
static uint16_t k0, k1;                   // envelope decay coefficients (Q16), set in controlA2()
static uint8_t  relShift = 3;             // fast release below -48 dB: 3 = tau 3.2 ms, 5 = tau 12.8 ms
// decay control -> k1 tables (33 points over d = 0..1023) for every voice, computed once in setup(),
// so a voice change takes effect immediately
static uint16_t kTab[NUM_VOICES][33];
static uint16_t kFix[NUM_VOICES];         // fixed-tau coefficient for env0, 0 = env0 follows the decay
static int16_t  dFilt = 512;              // smoothed decay value 0..1023 (pot + CV)
static uint8_t  adcCh;                    // 0: last conversion was the pot (AIN7), 1: CV (AIN5)
static uint16_t adcPot = 512, adcCv = 512;
static uint8_t  ctrlCount;
static uint8_t  trigPrev = PIN1_bm;
static uint8_t  keyAPrev = PIN0_bm, keyBPrev = PIN4_bm;
static uint16_t keyALock, keyBLock;
static uint8_t  clapBursts, clapTimer;
static uint8_t  krHold;                   // HIGH_SNARE noise hold, control ticks
static uint16_t gRawA, gRawB;             // gains before LEVEL (controlC -> controlC2)
static uint16_t ledG;                     // envelope level for the LED display (set in controlC)
static uint8_t  ledShown = 0xFF;

// ---- audio state (ISR only) ----
static uint32_t env0, env1;
static uint16_t gainA, gainB;             // gains of source a and source b, LEVEL applied
static uint16_t metalPh[6];               // HIHAT808 and COWBELL (0, 1) square oscillators
static uint16_t tonePh[3];                // LOW_SNARE and HIGH_SNARE (0, 1) triangle oscillators
static uint16_t snareInc[3], snarePitch;  // LOW_SNARE current increments, pitch envelope (Q16)
static uint16_t snareUp;                  // LOW_SNARE upper-mode envelope (Q16)
static int16_t  noiseLp;                  // snare noise high-pass state
static int16_t  noiseLp2;                 // HIGH_SNARE noise low-pass state
static uint32_t hatPos;                   // HIHAT909 play position, 16.16
static uint16_t lfsr = 0xACE1;
static int32_t  qErr;                     // noise-shaping residue (Q16)
static uint16_t dacOut = DAC_MID << 6;

#if ISR_DIAG
// survive the UPDI reset: read with pymcuprog after entering programming mode
__attribute__((section(".noinit"))) volatile uint16_t diagMagic, diagMax[11], diagAvg;
__attribute__((section(".noinit"))) volatile uint32_t diagOver;
__attribute__((section(".noinit"))) volatile uint32_t diagSum;
__attribute__((section(".noinit"))) volatile uint16_t diagN;
__attribute__((section(".noinit"))) volatile uint32_t diagLost, diagTotal, diagRate;
#endif

// charlieplex: {anode pin, cathode pin} on PORTC, index 0..5 = schematic LED1..LED6
// PC1 -> R9, PC2 -> R10, PC3 -> R11
// LED1/LED4 need the bodge wire from their common node to the R10 net
static const uint8_t ledPins[6][2] = {
  { PIN2_bm, PIN1_bm },   // LED1
  { PIN1_bm, PIN2_bm },   // LED2
  { PIN2_bm, PIN3_bm },   // LED3
  { PIN3_bm, PIN2_bm },   // LED4
  { PIN1_bm, PIN3_bm },   // LED5
  { PIN3_bm, PIN1_bm },   // LED6
};
#define LED_MASK (PIN1_bm | PIN2_bm | PIN3_bm)
#define LED_OFF  0xFE

static inline __attribute__((always_inline)) void ledOn(uint8_t i) {
  if (LEDS_OFF || i == LED_OFF) {
    VPORTC.DIR &= ~LED_MASK;
    return;
  }
  VPORTC.OUT = (VPORTC.OUT & ~LED_MASK) | ledPins[i][0];
  VPORTC.DIR = (VPORTC.DIR & ~LED_MASK) | ledPins[i][0] | ledPins[i][1];
}

static inline __attribute__((always_inline)) void startHit() {
  env0 = ENV_MAX;
  env1 = (voice == CLAP) ? 0 : ENV_MAX;   // clap: tail starts after the bursts
  clapBursts = (voice == CLAP) ? 3 : 0;
  clapTimer = CLAP_SPACING;
  tonePh[0] = tonePh[1] = tonePh[2] = 0;
  snareUp = 0xFFFF;
  snarePitch = 0xFFFF;
  krHold = KR_HOLD;
  hatPos = 0;
}

// signed 16 x unsigned 16 -> signed 32 with the hardware multiplier, inline: avoids the libgcc
// call (__usmulhisi3) and the register saves it forces on the audio interrupt
static inline __attribute__((always_inline)) int32_t mulSU16(int16_t a, uint16_t g) {
  int32_t p;
  uint8_t z;
  asm (
    "clr   %[z]          \n\t"
    "mul   %A[a], %A[g]  \n\t"   // al * gl
    "movw  %A[p], r0     \n\t"
    "mulsu %B[a], %B[g]  \n\t"   // ah * gh (signed x unsigned)
    "movw  %C[p], r0     \n\t"
    "mulsu %B[a], %A[g]  \n\t"   // ah * gl (signed x unsigned), C = sign
    "sbc   %D[p], %[z]   \n\t"
    "add   %B[p], r0     \n\t"
    "adc   %C[p], r1     \n\t"
    "adc   %D[p], %[z]   \n\t"
    "mul   %A[a], %B[g]  \n\t"   // al * gh
    "add   %B[p], r0     \n\t"
    "adc   %C[p], r1     \n\t"
    "adc   %D[p], %[z]   \n\t"
    "clr   r1            \n\t"
    : [p] "=&r" (p), [z] "=&r" (z)
    : [a] "a" (a), [g] "a" (g));
  return p;
}

static inline __attribute__((always_inline)) void decay(uint32_t &env, uint16_t k) {
  env -= ((env >> 8) * k) >> 8;
  if (env < ENV_TAIL) env -= env >> relShift;   // below -48 dB: fast release through the DAC's last bits
}

// Control-rate work (2000 Hz) is split over the ten even samples of each twenty, so no single
// interrupt exceeds two sample periods (a longer one dropped one tick in 16: everything ran at 37.5 kHz).

// A: decay pot + CV
static inline __attribute__((always_inline)) void controlA() {
  // decay pot (PD7) and CV (PD5), converted alternately; one conversion (~20 us) per control tick
  if (!(ADC0.COMMAND & ADC_STCONV_bm)) {
    uint16_t res = ADC0.RES;
    if (adcCh) adcCv = res; else adcPot = res;
    adcCh ^= 1;
    ADC0.MUXPOS = adcCh ? ADC_MUXPOS_AIN5_gc : ADC_MUXPOS_AIN7_gc;
    ADC0.COMMAND = ADC_STCONV_bm;
  }
#ifdef DECAY_FIXED
  int16_t d = DECAY_FIXED;
#else
  int16_t d = (int16_t)(adcPot + adcCv) - 512;     // 0 V CV reads 512
  if (d < 0) d = 0;
  if (d > 1023) d = 1023;
#endif
  dFilt += (d - dFilt) >> 3;              // ~4 ms smoothing against ADC noise
#if AUTOTEST
  {
    static uint16_t tt;
    static uint8_t  step;
    static const int16_t dTest[4] = { 102, 409, 716, 1023 };
    voice = AUTOTEST_VOICE;
    dFilt = dTest[step];
    if (++tt >= CTRL_RATE * 3 / 2) { tt = 0; startHit(); step = (step + 1) & 3; }
  }
#endif
}

// A2: decay table interpolation -> k0, k1
static inline __attribute__((always_inline)) void controlA2() {
  const uint16_t *kt = kTab[voice];
  uint8_t idx = dFilt >> 5, fr = dFilt & 31;
  k1 = kt[idx] + (int16_t)(mulSU16((int16_t)(kt[idx + 1] - kt[idx]), fr) >> 5);
  k0 = kFix[voice] ? kFix[voice] : k1;
  relShift = (voice == CLAP) ? 5 : 3;     // clap: even noise tail, a 3.2 ms release is heard as a cut
}

// B: first envelope
static inline __attribute__((always_inline)) void controlB() {
  decay(env0, k0);
}

// B2: second envelope, HIGH_SNARE noise hold, clap bursts
static inline __attribute__((always_inline)) void controlB2() {
  decay(env1, k1);
  if (voice == HIGH_SNARE && krHold) {    // noise held flat, then decays
    krHold--;
    env1 = ENV_MAX;
  }

  if (clapBursts && --clapTimer == 0) {
    if (--clapBursts) {
      env0 = ENV_MAX;                     // next burst
      clapTimer = CLAP_SPACING;
    } else {
      env1 = ENV_MAX;                     // tail
    }
  }
}

// C: envelopes -> gains of source a and b, LED level
static inline __attribute__((always_inline)) void controlC() {
  uint16_t e0 = env0 >> 8, e1 = env1 >> 8;
  switch (voice) {
    case LOW_SNARE:  gRawA = (e0 >> 1) + (e0 >> 2);   gRawB = (e1 >> 2) + (e1 >> 4) + (e1 >> 5); break;
    case CLAP:       gRawA = e0 > e1 ? e0 : e1;       gRawB = 0;        break;
    case COWBELL:    gRawA = (e0 >> 1) + (e1 >> 1);   gRawB = 0;        break;
    case HIHAT808:   gRawA = e0 - (e0 >> 2);          gRawB = e0 >> 2;  break;
    case HIHAT909:   gRawA = e0;                      gRawB = 0;        break;
    default:         // HIGH_SNARE: tone 0.3125 x1.5 = 0.47, noise 0.375: tone 1 dB above the noise at the attack
                     gRawA = (e0 >> 2) + (e0 >> 4);   gRawB = (e1 >> 2) + (e1 >> 3); break;
  }
  ledG = gRawA + gRawB;
}

// C2: output level (a separate step: the sample path keeps using the previous, scaled gains)
static inline __attribute__((always_inline)) void controlC2() {
  uint32_t gA = ((uint32_t)gRawA * LEVEL[voice]) >> 8;
  uint32_t gB = ((uint32_t)gRawB * LEVEL[voice]) >> 8;
  gainA = gA > 65535 ? 65535 : gA;        // saturate: a wrapped gain is a sudden drop in level
  gainB = gB > 65535 ? 65535 : gB;
}

// D: buttons, sampled every control tick (0.5 ms), 20 ms lockout after each edge
static inline __attribute__((always_inline)) void controlD() {
  // KEY B: trigger
  if (keyBLock) {
    keyBLock--;
  } else {
    uint8_t kb = VPORTD.IN & PIN4_bm;
    if (kb != keyBPrev) {
      keyBPrev = kb;
      keyBLock = KEY_LOCK_TICKS;
      if (!kb) startHit();
    }
  }

  // KEY A: first press shows the current voice, presses while it is shown select the next
  if (keyALock) {
    keyALock--;
  } else {
    uint8_t ka = VPORTA.IN & PIN0_bm;
    if (ka != keyAPrev) {
      keyAPrev = ka;
      keyALock = KEY_LOCK_TICKS;
      if (!ka) {
        if (showVoiceTicks) {
          voice = voice + 1 < NUM_VOICES ? voice + 1 : 0;
          env0 = env1 = 0;
          clapBursts = 0;
          saveRequest = 1;
        }
        showVoiceTicks = (uint32_t)SHOW_VOICE_MS * CTRL_RATE / 1000;
      }
    }
  }
}

// E: LOW_SNARE pitch drop x1.33 -> x1.0, fundamental
static inline __attribute__((always_inline)) void controlE() {
  if (voice == LOW_SNARE) {
    snarePitch -= ((uint32_t)snarePitch * SNARE_KP) >> 16;
    snareInc[0] = snareBase[0] + (uint16_t)(((uint32_t)snareBase3[0] * snarePitch) >> 16);
  }
}

// E2: LOW_SNARE upper modes
static inline __attribute__((always_inline)) void controlE2() {
  if (voice == LOW_SNARE) {
    snareUp -= ((uint32_t)snareUp * SNARE_KUP) >> 16;
    snareInc[1] = snareBase[1] + (uint16_t)(((uint32_t)snareBase3[1] * snarePitch) >> 16);
    snareInc[2] = snareBase[2] + (uint16_t)(((uint32_t)snareBase3[2] * snarePitch) >> 16);
  }
}

// F: LEDs
static inline __attribute__((always_inline)) void controlF() {
  uint8_t led = 0;                        // 0 = above -6 dB .. 5 = above -36 dB, 6 = below
  while (led < 6 && ledG < (0x8000u >> led)) led++;
  if (showVoiceTicks) {
    showVoiceTicks--;
    led = voice;                          // voice display has priority
  } else {
    led = (led == 6) ? LED_OFF : 5 - led; // envelope from the top LED down
  }
  if (led != ledShown) {
    ledShown = led;
    ledOn(led);
  }
}

// audio sample interrupt, SAMPLE_RATE
ISR(TCB0_INT_vect) {
#if ISR_DIAG
  uint16_t diagStart = TCA0.SINGLE.CNT;
#endif
  TCB0.INTFLAGS = TCB_CAPT_bm;
  DAC0.DATA = dacOut;                     // written first: fixed latency, independent of the code path below

  // trigger jack: falling edge on PA1 (Q1 inverts)
  uint8_t in = VPORTA.IN & PIN1_bm;
  bool hit = trigPrev && !in;
  trigPrev = in;
  if (hit) startHit();

  const uint8_t v = voice;                // volatile: read once
  lfsr = (lfsr >> 1) ^ (-(lfsr & 1u) & 0xB400u);
  int16_t noise = (int16_t)(int8_t)(lfsr >> 8) << 2;   // +-512, high byte of the LFSR: no 16-bit shift

  // Each voice produces two sources: a (gainA) and b (gainB, default: white noise)
  int16_t a, b = noise;
  switch (v) {
    case LOW_SNARE: {
      // triangles from the phase high byte (free on AVR): 8-bit resolution
      uint8_t h0 = (uint8_t)((tonePh[0] += snareInc[0]) >> 8);
      if (h0 & 0x80) h0 = ~h0;
      a = ((int16_t)h0 << 2) - 256;       // fundamental, 0..127 -> -256..+252
      if (snareUp > 64) {                 // upper modes only while audible (about the first 100 ms)
        uint8_t h1 = (uint8_t)((tonePh[1] += snareInc[1]) >> 8);
        uint8_t h2 = (uint8_t)((tonePh[2] += snareInc[2]) >> 8);
        if (h1 & 0x80) h1 = ~h1;
        if (h2 & 0x80) h2 = ~h2;
        int16_t up = ((int16_t)h1 >> 1) - 32 + ((int16_t)h2 >> 2) - 16;   // 1/8 and 1/16 of +-256: the fundamental stays the heard pitch
        a += (int16_t)(mulSU16(up, snareUp) >> 16);
      }
      int16_t dn = noise - noiseLp;       // one-pole low-pass, a = 7/32 (two shifts): corner 1.57 kHz
      noiseLp += (dn >> 2) - (dn >> 5);
      b = noise - noiseLp;                // high-passed noise
      break;
    }
    case CLAP:
      a = noise;
      break;
    case COWBELL:
      metalPh[0] += cowbellInc[0];
      metalPh[1] += cowbellInc[1];
      a = ((int16_t)(metalPh[0] >> 15) + (int16_t)(metalPh[1] >> 15) - 1) * 480;
      break;
    case HIHAT808: {
      // XOR of the six squares (their product as +-1): spreads energy up to the top of the F range
      uint8_t x = 0;
      for (uint8_t i = 0; i < 6; i++) {
        metalPh[i] += hatInc[i];
        x ^= metalPh[i] >> 15;
      }
      a = x ? 480 : -480;
      break;
    }
    case HIHAT909:
      if (hatPos < ((uint32_t)(HAT_LEN - 1) << 16)) {
        uint16_t i = hatPos >> 16;
        int16_t s0 = hatOpen[i], s1 = hatOpen[i + 1];
        uint8_t fr = (uint8_t)(hatPos >> 9) & 0x7F;   // 7-bit fraction (bits 9..15; bit 16 is the sample index)
        a = (s0 << 2) + (((s1 - s0) * fr) >> 5);      // x4: 8-bit sample, linear interpolation
        a += a >> 2;                                  // x5 in total, so LEVEL stays below 256
        hatPos += (uint32_t)(65536.0 * HAT_RATE / SAMPLE_RATE + 0.5);
      } else {
        a = 0;
      }
      b = 0;
      break;
    default: {                            // HIGH_SNARE
      int16_t t = 0;
      for (uint8_t i = 0; i < 2; i++) {   // triangles: 225 Hz + half level 450 Hz
        tonePh[i] += krInc[i];
        uint16_t p = tonePh[i];
        if (p & 0x8000) p = ~p;
        int16_t tri = (int16_t)(p >> 6) - 256;
        t += i ? tri >> 1 : tri;
      }
      a = t + (t >> 1);                   // x1.5: keeps the tone gain below 1.0 after LEVEL
      int16_t dn = noise - noiseLp;       // high-pass 1.57 kHz (a = 7/32)
      noiseLp += (dn >> 2) - (dn >> 5);
      int16_t h = noise - noiseLp;
      int16_t dl = h - noiseLp2;          // low-pass ~6 kHz (a = 5/8)
      noiseLp2 += (dl >> 1) + (dl >> 3);
      b = noiseLp2;
      break;
    }
  }

  // first-order noise shaping: the fraction dropped by the 10-bit DAC is added to the next sample,
  // so fades continue smoothly below 1 LSB instead of ending in a 1-LSB rattle that cuts to silence
  int32_t acc = mulSU16(a, gainA) + qErr;
  if (gainB) acc += mulSU16(b, gainB);    // most voices have no second source
  int16_t s = acc >> 16;
  qErr = acc - ((int32_t)s << 16);
  if (s > DAC_SWING)  { s = DAC_SWING;  qErr = 0; }
  if (s < -DAC_SWING) { s = -DAC_SWING; qErr = 0; }
  dacOut = (uint16_t)(DAC_MID + s) << 6;  // DAC0.DATA is left adjusted (bits 15:6)

  if (++ctrlCount >= CTRL_DIV) ctrlCount = 0;
  switch (ctrlCount) {                    // one small piece of control work on every other sample:
    case 0:  controlA();  break;          // the plain sample after it makes up the extra time
    case 2:  controlA2(); break;
    case 4:  controlB();  break;
    case 6:  controlB2(); break;
    case 8:  controlC();  break;
    case 10: controlC2(); break;
    case 12: controlD();  break;
    case 14: controlE();  break;
    case 16: controlE2(); break;
    case 18: controlF();  break;
  }
#if ISR_DIAG == 2
  diagTotal++;                            // minimal: count only, rate checked against millis() in loop()
#elif ISR_DIAG
  {
    uint16_t c = TCB0.CNT;                                   // cycles since the sample tick: latency + prologue + body
    if (TCB0.INTFLAGS & TCB_CAPT_bm) c += F_CPU / SAMPLE_RATE;   // ran into the next period
    (void)diagStart;
    uint8_t cls = (ctrlCount & 1) ? 10 : ctrlCount >> 1;     // 0..9: control slots, 10: plain sample
    if (diagTotal == 80000) { for (uint8_t i = 0; i < 11; i++) diagMax[i] = 0; diagOver = 0; }   // skip start-up
    if (c > diagMax[cls]) diagMax[cls] = c;
    if (c > F_CPU / SAMPLE_RATE) diagOver++;
    diagSum += c;
    diagTotal++;
    if (++diagN == 4096) { diagAvg = diagSum >> 12; diagSum = 0; diagN = 0; }
  }
#endif
}

// ---- setup / loop ----

static uint16_t tauToK(float tauMs) {
  float k = 65536.0f * (1.0f - expf(-1000.0f / (tauMs * CTRL_RATE)));
  if (k < 1.0f) k = 1.0f;
  if (k > 65535.0f) k = 65535.0f;
  return (uint16_t)k;
}

// k1 over the decay range (exponential from tauMin to tauMax) and the fixed env0 coefficient, for every voice
static void buildDecayTables() {
  for (uint8_t v = 0; v < NUM_VOICES; v++) {
    const VoiceDef &vd = voices[v];
    for (uint8_t i = 0; i <= 32; i++) {
      float x = (i < 32 ? i * 32 : 1023) / 1023.0f;
      kTab[v][i] = tauToK(DECAY_SCALE * vd.tauMinMs * powf(vd.tauMaxMs / vd.tauMinMs, x));
    }
    kFix[v] = vd.fixedTauMs > 0.0f ? tauToK(vd.fixedTauMs) : 0;
  }
  kTab[HIHAT909][32] = 0;                 // fully clockwise: no decay, the whole sample
}

void setup() {
#if ISR_DIAG
  {                                       // mulSU16 self test against the C multiply
    const int16_t  ta[] = { 0, 1, -1, 511, -512, 32767, -32768, 1234, -4321, 255, -256, 300 };
    const uint16_t tg[] = { 0, 1, 65535, 32768, 255, 256, 12345, 65280, 40000, 1 };
    uint16_t bad = 0;
    for (uint8_t i = 0; i < sizeof(ta) / 2; i++)
      for (uint8_t j = 0; j < sizeof(tg) / 2; j++)
        if (mulSU16(ta[i], tg[j]) != (int32_t)ta[i] * (int32_t)tg[j]) bad++;
    diagMagic = bad ? 0xBAD0 + bad : 0x600D;
  }
  TCA0.SPLIT.CTRLA = 0;                   // DxCore runs TCA0 in split mode for PWM: stop, reset, back to 16-bit
  TCA0.SINGLE.CTRLESET = TCA_SINGLE_CMD_RESET_gc | 0x03;
  TCA0.SINGLE.CTRLD = 0;
  TCA0.SINGLE.PER = 0xFFFF;
  TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV1_gc | TCA_SINGLE_ENABLE_bm;   // free running, CLK/1
  for (uint8_t i = 0; i < 11; i++) diagMax[i] = 0;
  diagOver = 0; diagSum = 0; diagN = 0; diagAvg = 0; diagLost = 0; diagTotal = 0;
#endif
  pinMode(PIN_PA0, INPUT_PULLUP);         // KEY1 A
  pinMode(PIN_PD4, INPUT_PULLUP);         // KEY2 B
  pinMode(PIN_PA1, INPUT_PULLUP);         // trigger (Q1 collector)

  PORTD.PIN5CTRL = PORT_ISC_INPUT_DISABLE_gc;
  PORTD.PIN6CTRL = PORT_ISC_INPUT_DISABLE_gc;
  PORTD.PIN7CTRL = PORT_ISC_INPUT_DISABLE_gc;

  // ADC, read from controlA() in the ISR; prescaler as set by the core
  VREF.ADC0REF = VREF_REFSEL_VDD_gc;      // set directly: DxCore 1.6.2 analogReference(VDD) skips the write on DD (VDD = 0x55, checks mode < 7)
  ADC0.CTRLA = ADC_ENABLE_bm | ADC_RESSEL_10BIT_gc;
  ADC0.SAMPCTRL = 16;                     // PD5 source impedance is 50 k (R6 || R7)
  ADC0.MUXPOS = ADC_MUXPOS_AIN7_gc;
  ADC0.COMMAND = ADC_STCONV_bm;

  VREF.DAC0REF = DAC_REF | VREF_ALWAYSON_bm;
  DAC0.DATA = dacOut;
  DAC0.CTRLA = DAC_ENABLE_bm | DAC_OUTEN_bm;

  uint8_t v = EEPROM.read(EE_VOICE);
  voice = v < NUM_VOICES ? v : (uint8_t)LOW_SNARE;   // erased EEPROM reads 0xFF
  showVoiceTicks = (uint32_t)SHOW_VOICE_MS * CTRL_RATE / 1000;

  buildDecayTables();

  // TCB0: sample clock
  TCB0.CCMP = F_CPU / SAMPLE_RATE - 1;
  TCB0.CTRLB = TCB_CNTMODE_INT_gc;
  TCB0.INTCTRL = TCB_CAPT_bm;
  CPUINT.LVL1VEC = TCB0_INT_vect_num;     // audio preempts the millis interrupt
  TCB0.CTRLA = TCB_CLKSEL_DIV1_gc | TCB_ENABLE_bm;
}

void loop() {
  static uint32_t tSave;
  static bool saveVoice;

  uint32_t now = millis();
#if ISR_DIAG
  {                                       // ISR executions per second, timed by millis() (TCB1)
    static uint32_t tRate, nRate;
    if (now - tRate >= 1000) {
      noInterrupts(); uint32_t n = diagTotal; interrupts();
      diagRate = (uint32_t)((uint64_t)(n - nRate) * 1000 / (now - tRate)); nRate = n; tRate = now;
    }
  }
#endif

  if (saveRequest) {                      // voice changed by KEY A in the ISR
    saveRequest = 0;
    saveVoice = true;
    tSave = now;
  }

  // one EEPROM write per selection, not per press; update() skips an unchanged value
  if (saveVoice && now - tSave > SAVE_DELAY_MS) {
    saveVoice = false;
    EEPROM.update(EE_VOICE, voice);
  }
}
