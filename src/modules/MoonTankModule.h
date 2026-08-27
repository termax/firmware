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
// The floor under EVERY report. Without one, a change-triggered report bypassed the
// heartbeat interval completely and could fire at the poll rate.
#ifndef MOONHUT_TANK_MIN_REPORT_S
#define MOONHUT_TANK_MIN_REPORT_S 120
#endif

#ifndef MOONHUT_TANK_REPORT_S
#define MOONHUT_TANK_REPORT_S 60
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

  protected:
    int32_t runOnce() override;

  private:
    float pingOnce();  // one trigger/echo cycle, metres, NAN on timeout
    void measure();
    void diagnose();   // runs after repeated silence: says WHY there is no echo
    void report(bool force);
    void serviceScreen(uint32_t now);
    void sendLine(const char *text);

    float lastM = NAN;
    float lastSpreadM = NAN;
    uint8_t lastValid = 0;
    float sessionMinM = NAN;
    float sessionMaxM = NAN;
    uint32_t bursts = 0;
    uint32_t timeouts = 0;
    bool diagnosed = false;
    const char *reject = nullptr;
    float reportedM = NAN;
    float pendingM = NAN;          // a candidate change, not yet confirmed by a second reading
    uint32_t nextReportAt = 0;
    uint32_t nextMinReportAt = 0;  // floor under every report, change-triggered ones included
    uint32_t screenOnSince = 0;
    uint32_t screenOffSince = 0;   // 0 = the panel is lit
    bool wasOnUsb = true;
};

extern MoonTankModule *moonTankModule;

#endif // MOONHUT_TANK
