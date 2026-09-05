#pragma once

#include "configuration.h"

#ifdef MOONHUT_TANK

#include "concurrency/OSThread.h"
#include <Arduino.h>

// --------------------------------------------------------------------------
// MoonHut tank level: an HC-SR04P ultrasonic ranger pointed down at a water
// surface. Level = tank depth - measured distance.
//
// This is deliberately a PROOF OF CONCEPT and knows nothing about tanks yet: it
// measures distance, filters it, and shows it. The open question it exists to
// answer is whether an HC-SR04 can reach the bottom of a 3 m tank at all -
// nominal range is 4 m but usable range on a weak return (a distant, rippled
// water surface) is closer to 2-3 m, and it degrades exactly when the tank is
// nearly empty and the reading matters most.
//
// The JSN-SR04T waterproof unit uses an identical trigger/echo interface, so
// everything here carries over to it unchanged.
// --------------------------------------------------------------------------

#ifndef MOONHUT_TANK_TRIG_PIN
#define MOONHUT_TANK_TRIG_PIN 17
#endif
#ifndef MOONHUT_TANK_ECHO_PIN
#define MOONHUT_TANK_ECHO_PIN 16
#endif

// --- Second ranger, for the A/B bench ------------------------------------
//
// Optional: define BOTH pins to fit a second trigger/echo sensor alongside the first
// and carry its reading in the same report, so two sensors can be compared on the same
// board, the same water and the same minute.
//
// The second sensor is deliberately PASSIVE. It never feeds the level-rate fit, the
// panel, the fast-drain alert or the report-triggering logic - it is a measurement
// channel for deciding which sensor to keep, not a second source of truth. When the
// winner is known, swap the primary pin defines to it and drop these.
//
// The JSN-SR04T goes on 47/48 because that is the pair the final V4 node has free, so
// the firmware carries over to that board unchanged.
#if defined(MOONHUT_TANK_TRIG_PIN2) && defined(MOONHUT_TANK_ECHO_PIN2)
#define MOONHUT_TANK_DUAL 1
#endif

// Silence between the two bursts. MUST stay above ECHO_TIMEOUT_US so the first sensor's
// ping is fully dead before the second speaks.
//
// Overlap does not present as noise, which is what makes it dangerous: each sensor hears
// the OTHER's burst and reports the flight time to it - a short, stable, entirely
// plausible distance that looks exactly like a real reading of a full tank.
#ifndef MOONHUT_TANK_INTERLEAVE_MS
#define MOONHUT_TANK_INTERLEAVE_MS 60
#endif

// Width of the trigger pulse, microseconds.
//
// 50, not the 10 every HC-SR04 example uses. MEASURED on a JSN-SR04T with the sweep
// build on 2026-09-05: 10 us NEVER produced an echo (0/5 on every attempt), 20 us was
// marginal (one burst rejected at 0.706 m spread, the next clean), and 30 us and above
// were solid 5/5 at 2.116 m with a spread of 0-9 mm.
//
// The JSN has its own MCU sampling that line and simply ignores a pulse it does not
// latch - no error, no echo, indistinguishable from a dead sensor or a wiring fault.
// That cost most of an evening: the sensor was alive and correctly wired the whole time.
//
// 50 satisfies the HC-SR04 datasheet too (it asks for 10 us MINIMUM), so one value
// drives both parts, and it is nothing against the settling gap below.
#ifndef MOONHUT_TANK_TRIG_US
#define MOONHUT_TANK_TRIG_US 50
#endif

// Quiet time after each ping so the previous burst has died away.
#ifndef MOONHUT_TANK_PING_GAP_MS
#define MOONHUT_TANK_PING_GAP_MS 60
#endif

// DIAGNOSTIC: cycle the trigger width across bursts and log which one the sensor
// answers. One flash instead of four, and it names the working width outright.
#ifdef MOONHUT_TANK_TRIG_SWEEP
#define MOONHUT_TANK_TRIG_WIDTHS {10, 20, 30, 50, 100, 200}
#endif

// Seconds between measurements.
#ifndef MOONHUT_TANK_POLL_S
#define MOONHUT_TANK_POLL_S 2
#endif

// How many raw pings go into one reported figure. Ultrasonics throw spurious
// echoes off walls, pipes and ripples constantly; a median discards them without
// the lag of an average. Odd numbers only.
#ifndef MOONHUT_TANK_SAMPLES
#define MOONHUT_TANK_SAMPLES 5
#endif

// A burst must return at least this many echoes to count. Below it there is no
// median to speak of - a "median" of one sample is a single unverified ping wearing
// the same confidence as a five-of-five result, which is exactly how the fridge bus
// glitches got through before they were filtered.
#ifndef MOONHUT_TANK_MIN_ECHOES
#define MOONHUT_TANK_MIN_ECHOES 3
#endif

// ...and the samples must agree to within this, in metres. A wide spread means the
// sensor picked a different target on each ping, so the median is not a measurement,
// it is the least-bad guess among several different objects.
#ifndef MOONHUT_TANK_MAX_SPREAD_M
#define MOONHUT_TANK_MAX_SPREAD_M 0.10f
#endif

// --- Consensus, not range --------------------------------------------------
//
// The original gate demanded every sample fall within MAX_SPREAD_M min-to-max. Ping
// dumps from a JSN showed why that is too brittle: a real target at 1.25 m arrives
// alongside a fixed ~0.247 m RINGDOWN artifact (the transducer hearing its own burst -
// it recurs to the millimetre across unrelated bursts) and a tail of later multipath
// reflections at 1.4-2.1 m. One ringdown sample destroys a min-to-max range on its own,
// so a perfectly good median was being thrown away every time.
//
// Instead: take the median, then count how many samples agree with it within a
// tolerance. A burst is trusted when enough of them do. That ignores one low artifact
// and one high reflection without ever averaging them in.
#ifndef MOONHUT_TANK_AGREE_M
#define MOONHUT_TANK_AGREE_M 0.10f
#endif
#ifndef MOONHUT_TANK_AGREE_MIN
#define MOONHUT_TANK_AGREE_MIN 3
#endif

// Near-field floor. Anything closer than this is discarded BEFORE filtering, because on
// this sensor it is ringdown rather than a target. Raise it above the artifact you
// actually observe: 0.247 m on the cable-mounted transducer, 0.227 m on the original.
// The cost is real blindness below this distance - which the JSN has anyway.
#ifndef MOONHUT_TANK_MIN_VALID_M
#define MOONHUT_TANK_MIN_VALID_M 0.30f
#endif

// Screen policy. On external power the panel never blanks - the whole point of the box
// is a reading you can see. On battery it is allowed to sleep, but e-ink holds its last
// image without power, so a sleeping panel keeps SHOWING a number that is quietly going
// stale. Waking it briefly once an hour keeps that number honest for the cost of one
// refresh.
#ifndef MOONHUT_TANK_BATT_SCREEN_ON_S
#define MOONHUT_TANK_BATT_SCREEN_ON_S 60
#endif
#ifndef MOONHUT_TANK_BATT_REFRESH_S
#define MOONHUT_TANK_BATT_REFRESH_S 3600
#endif

// Report on a change this big, or on the heartbeat below, whichever comes first.
#ifndef MOONHUT_TANK_REPORT_DELTA_M
#define MOONHUT_TANK_REPORT_DELTA_M 0.02f
#endif
// --- Rate of level change -------------------------------------------------
//
// Reported as LEVEL metres per hour: positive = filling, negative = draining.
// Note the sign flip against the raw measurement - the sensor reads distance DOWN to the
// surface, so a rising distance means a FALLING level. Publishing the raw derivative would
// invert every reading a human looks at.
//
// Least-squares slope over a window, not a first-to-last difference: this sensor's noise is
// tens of millimetres, and a two-point difference would report that noise divided by the
// interval as if it were a flow rate.
#ifndef MOONHUT_TANK_RATE_SAMPLES
#define MOONHUT_TANK_RATE_SAMPLES 16 // ring depth
#endif

#ifndef MOONHUT_TANK_RATE_DECIMATE_S
#define MOONHUT_TANK_RATE_DECIMATE_S 60 // keep at most one sample a minute
#endif

// Below this span the slope is not published at all. A rate fitted over a couple of minutes
// of a noisy signal is a random number with units attached.
#ifndef MOONHUT_TANK_RATE_MIN_SPAN_S
#define MOONHUT_TANK_RATE_MIN_SPAN_S 300 // 5 min
#endif

// Fast-drain alert. DISABLED by default (0) and it should stay that way until this tank's
// normal draw has been observed: a threshold guessed before the first day of real data is
// just a source of false alarms, and an alert nobody trusts is worse than no alert.
#ifndef MOONHUT_TANK_FAST_DRAIN_MPH
#define MOONHUT_TANK_FAST_DRAIN_MPH 0.0f
#endif

// How much the distance must move before the PANEL is repainted. Unrelated to the
// reporting delta: this one only protects the e-ink from being burned through.
#ifndef MOONHUT_TANK_REDRAW_DELTA_M
#define MOONHUT_TANK_REDRAW_DELTA_M 0.01f
#endif

// The floor under EVERY report. Without one, a change-triggered report bypassed the
// heartbeat interval completely and could fire at the poll rate.
#ifndef MOONHUT_TANK_MIN_REPORT_S
#define MOONHUT_TANK_MIN_REPORT_S 120
#endif

#ifndef MOONHUT_TANK_REPORT_S
#define MOONHUT_TANK_REPORT_S 60
#endif

// --- Stall detection -------------------------------------------------------
//
// A node in an enclosure at a tank has no serial cable, so "it stopped showing a
// distance" has to be diagnosable from the mesh alone. Two things make that possible:
// every report carries uptime and the consecutive-failure count, and a run of failures
// long enough to matter announces itself instead of going quiet.
//
// The distinction that matters in the log: a node that REBOOTED comes back with a small
// uptime, a node that STALLED keeps counting up. Without uptime in the line the two are
// indistinguishable after the fact, which is exactly the hole this closes.
#ifndef MOONHUT_TANK_STALL_S
#define MOONHUT_TANK_STALL_S 600
#endif

// --- Calibration -----------------------------------------------------------
//
// The node needs a tank height locally to show a percentage on its own panel, which is
// the whole point of having a panel at the tank. It is deliberately RUNTIME-SETTABLE and
// persisted, never a build flag: re-plumbing a tank must be a command, not a reflash.
//
// This does NOT make the node the system of record. The Pi still owns litres, history
// and alerting from the raw distance; the percentage here is a convenience readout for
// whoever is standing in front of the box. Two consumers of one geometry, deliberately -
// so if they ever disagree, the Pi wins.
//
//   tank:height=3.20    metres from the sensor face to the tank floor
//   tank:offset=0.35    optional dead space at the top the sensor cannot see
//   tank:show           report the current calibration
//   tank:clear          forget it and go back to showing distance only
#ifndef MOONHUT_TANK_CFG_PATH
#define MOONHUT_TANK_CFG_PATH "/tankcfg"
#endif

class MoonTankModule : public concurrency::OSThread
{
  public:
    MoonTankModule();

    /// Last filtered distance in metres, or NAN if the last burst found nothing.
    float distanceM() const { return lastM; }

    /// Spread of the samples behind the last reading, in metres. This is the number
    /// that says whether to trust it: a tight cluster is a real surface, a wide one
    /// means the sensor is guessing.
    float spreadM() const { return lastSpreadM; }

    /// How many of the last burst returned an echo at all.
    uint8_t validSamples() const { return lastValid; }

    /// Why the last burst was rejected, or nullptr if it was accepted.
    const char *rejectReason() const { return reject; }

    /// Session min/max, for the bench range test.
    float minM() const { return sessionMinM; }
    float maxM() const { return sessionMaxM; }

    /// True once a tank height has been set. Until then the panel shows distance only
    /// and says so, rather than inventing a percentage from a guessed height.
    bool isCalibrated() const { return tankHeightM > 0.0f; }
    float tankHeight() const { return tankHeightM; }
    float deadTopM() const { return tankOffsetM; }

    /// Water depth in metres, NAN if uncalibrated or the last burst failed.
    float levelM() const;
    /// Fill percentage 0-100, NAN if uncalibrated or the last burst failed.
    float levelPct() const;
    /// Level change rate, metres/hour: + filling, - draining. NAN until the fit has span.
    float levelRateMphPublic() const { return levelRateMph(); }

    /// "tank:<command>" surface. Returns a short human-readable result to DM back.
    const char *handleCommand(const char *body);
    /// Commands are honoured on the fleet channel or a PKI DM, same rule as the fridge.
    bool acceptsCommand(uint8_t channelIndex, bool pkiEncrypted) const;

    void loadCalibration();
    void saveCalibration();

#ifdef MOONHUT_TANK_DUAL
    /// The comparison sensor's last filtered distance, NAN if its burst was rejected.
    float distance2M() const { return lastM2; }
    float spread2M() const { return lastSpread2; }
    uint8_t validSamples2() const { return lastValid2; }
    const char *rejectReason2() const { return reject2; }
#endif

  protected:
    int32_t runOnce() override;

  private:
    float pingOnce(uint8_t trigPin, uint8_t echoPin, uint16_t trigUs); // one cycle, metres, NAN on timeout

    /// One filtered burst on the given pins. Returns false and sets `why` when the
    /// result should not be trusted; `median` is still set for the log unless nothing
    /// answered at all.
    bool burst(uint8_t trigPin, uint8_t echoPin, float &median, float &spread, uint8_t &n, const char *&why,
               uint16_t trigUs = MOONHUT_TANK_TRIG_US);

    void measure();
    void diagnose();   // runs after repeated silence: says WHY there is no echo
    void report(bool force);
    void serviceScreen(uint32_t now);
    void sendLine(const char *text);
    void recordLevel(uint32_t now, float metres);
    float levelRateMph() const; // metres/hour, + = filling, NAN if not enough span

    // Ring of decimated (time, distance) samples, for the least-squares rate fit.
    struct RateSample {
        uint32_t atMs;
        float m;
    };
    RateSample rateBuf[MOONHUT_TANK_RATE_SAMPLES] = {};
    uint8_t rateCount = 0;
    uint8_t rateHead = 0;
    uint32_t lastRateAt = 0;
    bool drainAlarm = false;

    bool calLoaded = false;     // littlefs is not mounted when modules are constructed
    float tankHeightM = 0.0f;   // 0 = uncalibrated
    float tankOffsetM = 0.0f;   // dead space at the top, subtracted from usable depth

    float lastM = NAN;
    float lastSpreadM = NAN;
    // What the last burst actually saw, kept even when the gates rejected it. Throwing
    // the number away is what made a rejected reading undiagnosable from the mesh.
    float lastRawM = NAN;
    uint32_t consecFails = 0;
    uint32_t lastGoodAtMs = 0;
    bool stallAnnounced = false;
    uint8_t lastValid = 0;
    float sessionMinM = NAN;
    float sessionMaxM = NAN;
    uint32_t bursts = 0;
    uint32_t timeouts = 0;
    bool diagnosed = false;
    const char *reject = nullptr;
#ifdef MOONHUT_TANK_DUAL
    float lastM2 = NAN;
    float lastSpread2 = NAN;
    uint8_t lastValid2 = 0;
    const char *reject2 = nullptr;
#endif
    float reportedM = NAN;
    float pendingM = NAN;          // a candidate change, not yet confirmed by a second reading
    float shownM = NAN;            // what the panel is currently displaying
    uint32_t nextReportAt = 0;
    uint32_t nextMinReportAt = 0;  // floor under every report, change-triggered ones included
    uint32_t screenOnSince = 0;
    uint32_t screenOffSince = 0;   // 0 = the panel is lit
    bool wasOnUsb = true;
};

extern MoonTankModule *moonTankModule;

#endif // MOONHUT_TANK
