#include "MoonTankModule.h"

#ifdef MOONHUT_TANK

#include "MeshService.h"
#include "PowerStatus.h"
#include "configuration.h"
#include "mesh/Channels.h"
#include "mesh/Router.h"

#if HAS_SCREEN
#include "graphics/Screen.h"
#endif

MoonTankModule *moonTankModule = nullptr;

// Speed of sound at 20 C. It moves ~0.17 %/C, which is nothing over a fridge's
// range and a great deal over a tank's: across a 30 C swing that is ~5 %, and at
// 3.3 m that is 16 cm of phantom level change - easily enough to fake a "tank
// empty". Compensating needs an air temperature, so it is left as a constant here
// and wired to a DS18B20 once the range test says this sensor is worth pursuing.
static constexpr float SPEED_OF_SOUND_MS = 343.0f;

// Bound the echo wait. pulseIn() BLOCKS, and this runs on the same core as the
// LoRa timing - the same trap as the DS18B20's 750 ms conversion. 25 ms is about
// 4.2 m there-and-back, past anything this sensor can honestly resolve, so a
// missing echo costs 25 ms and not a stalled radio.
static constexpr uint32_t ECHO_TIMEOUT_US = 25000;

// Below this the sensor is inside its own dead zone and the figure is meaningless.
static constexpr float MIN_VALID_M = 0.03f;
static constexpr float MAX_VALID_M = 4.5f;

MoonTankModule::MoonTankModule() : concurrency::OSThread("MoonTank")
{
    pinMode(MOONHUT_TANK_TRIG_PIN, OUTPUT);
    digitalWrite(MOONHUT_TANK_TRIG_PIN, LOW);
    pinMode(MOONHUT_TANK_ECHO_PIN, INPUT);
    LOG_INFO("MoonTank: ranger A on TRIG %d / ECHO %d", MOONHUT_TANK_TRIG_PIN, MOONHUT_TANK_ECHO_PIN);
#ifdef MOONHUT_TANK_DUAL
    pinMode(MOONHUT_TANK_TRIG_PIN2, OUTPUT);
    digitalWrite(MOONHUT_TANK_TRIG_PIN2, LOW);
    pinMode(MOONHUT_TANK_ECHO_PIN2, INPUT);
    LOG_INFO("MoonTank: ranger B on TRIG %d / ECHO %d - comparison only, never drives level, rate or alerts",
             MOONHUT_TANK_TRIG_PIN2, MOONHUT_TANK_ECHO_PIN2);
#endif
}

float MoonTankModule::pingOnce(uint8_t trigPin, uint8_t echoPin, uint16_t trigUs)
{
    // 10 us trigger, per the datasheet. The 2 us LOW first guarantees a clean edge
    // even if something left the line high.
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(trigUs);
    digitalWrite(trigPin, LOW);

    const uint32_t us = pulseIn(echoPin, HIGH, ECHO_TIMEOUT_US);
    if (us == 0)
        return NAN; // no echo inside the window

    // Out and back, so half the flight time.
    const float m = (us * 1e-6f * SPEED_OF_SOUND_MS) / 2.0f;
    if (m < MIN_VALID_M || m > MAX_VALID_M)
        return NAN;
    return m;
}

bool MoonTankModule::burst(uint8_t trigPin, uint8_t echoPin, float &median, float &spread, uint8_t &n,
                           const char *&why, uint16_t trigUs)
{
    float s[MOONHUT_TANK_SAMPLES];
    n = 0;
    why = nullptr;
    median = NAN;
    spread = NAN;

    for (uint8_t i = 0; i < MOONHUT_TANK_SAMPLES; i++) {
        const float m = pingOnce(trigPin, echoPin, trigUs);
        if (!isnan(m))
            s[n++] = m;
        // The datasheet asks for >60 ms between pings so the previous burst has
        // died away; less and you measure your own echo coming back off the tank.
        delay(MOONHUT_TANK_PING_GAP_MS);
    }

    if (n == 0) {
        why = "no echo";
        return false;
    }

    // Insertion sort - n is at most a handful.
    for (uint8_t i = 1; i < n; i++) {
        const float k = s[i];
        int8_t j = i - 1;
        while (j >= 0 && s[j] > k) {
            s[j + 1] = s[j];
            j--;
        }
        s[j + 1] = k;
    }

    median = s[n / 2];        // median, not mean: one wild echo must not move it
    spread = s[n - 1] - s[0];

    // Two gates, both learned the hard way on the fridge bus: a reading nobody
    // cross-checked, and a reading whose samples disagree, are both worse than no
    // reading at all - because they look exactly as confident as a good one.
    if (n < MOONHUT_TANK_MIN_ECHOES)
        why = "too few echoes";
    else if (spread > MOONHUT_TANK_MAX_SPREAD_M)
        why = "samples disagree";
    return why == nullptr;
}

void MoonTankModule::measure()
{
    float median = NAN, spread = NAN;
    uint8_t n = 0;
    const char *why = nullptr;
#ifdef MOONHUT_TANK_TRIG_SWEEP
    static const uint16_t widths[] = MOONHUT_TANK_TRIG_WIDTHS;
    const uint16_t trigUs = widths[bursts % (sizeof(widths) / sizeof(widths[0]))];
#else
    const uint16_t trigUs = MOONHUT_TANK_TRIG_US;
#endif
    const bool ok = burst(MOONHUT_TANK_TRIG_PIN, MOONHUT_TANK_ECHO_PIN, median, spread, n, why, trigUs);

    bursts++;
    lastRawM = median;   // kept even when rejected - see the header
#ifdef MOONHUT_TANK_TRIG_SWEEP
    LOG_WARN("MoonTank SWEEP: trigger %u us -> %s (%u/%u echoes)", (unsigned)trigUs,
             ok ? "ECHO" : (why ? why : "?"), n, MOONHUT_TANK_SAMPLES);
#endif
    lastSpreadM = spread;
    lastValid = n;
    reject = ok ? nullptr : why;

#ifdef MOONHUT_TANK_DUAL
    // Never overlap the two. This wait is longer than the echo timeout, so A's burst is
    // fully dead before B speaks - see the header for why an overlap is worse than noise.
    delay(MOONHUT_TANK_INTERLEAVE_MS);
    const bool ok2 = burst(MOONHUT_TANK_TRIG_PIN2, MOONHUT_TANK_ECHO_PIN2, lastM2, lastSpread2, lastValid2, reject2);
    if (!ok2)
        lastM2 = NAN;

    // Logged whatever either one did: a disagreement is as interesting as an agreement,
    // and one sensor going silent where the other still reads is exactly the blind-zone
    // answer the A/B exists to get.
    const float aM = ok ? median : NAN;
    if (!isnan(aM) && !isnan(lastM2))
        LOG_INFO("MoonTank A/B: A %.3f m (sp %.3f, %u/%u)  B %.3f m (sp %.3f, %u/%u)  B-A %+.3f m", aM, spread, n,
                 MOONHUT_TANK_SAMPLES, lastM2, lastSpread2, lastValid2, MOONHUT_TANK_SAMPLES, lastM2 - aM);
    else
        LOG_WARN("MoonTank A/B: A %s (%u/%u)  B %s (%u/%u)", ok ? "ok" : (why ? why : "?"), n, MOONHUT_TANK_SAMPLES,
                 ok2 ? "ok" : (reject2 ? reject2 : "?"), lastValid2, MOONHUT_TANK_SAMPLES);
#endif

    if (n == 0) {
        timeouts++;
        lastM = NAN;
        consecFails++;
        LOG_WARN("MoonTank: no echo (%u of %u bursts have failed)", (unsigned)timeouts, (unsigned)bursts);
        if (!diagnosed && timeouts >= 3) {
            diagnosed = true; // once per boot - it is noisy and the answer does not change
            diagnose();
        }
        return;
    }

    if (reject) {
        lastM = NAN;
        consecFails++;
        LOG_WARN("MoonTank: REJECTED %.3f m - %s (spread %.3f m, %u/%u echoes)", median, reject, spread, n,
                 MOONHUT_TANK_SAMPLES);
        return;
    }

    lastM = median;
    consecFails = 0;
    lastGoodAtMs = millis();
    if (stallAnnounced) {
        stallAnnounced = false;
        char ok[96];
        snprintf(ok, sizeof(ok), "TANK OK|reading again|d=%.3f|up=%lus", lastM, (unsigned long)(millis() / 1000));
        sendLine(ok);
    }

    recordLevel(millis(), lastM);

    if (isnan(sessionMinM) || lastM < sessionMinM)
        sessionMinM = lastM;
    if (isnan(sessionMaxM) || lastM > sessionMaxM)
        sessionMaxM = lastM;

    // Spread is the honesty check on the median. A few millimetres means a real,
    // flat surface; tens of centimetres means the sensor is picking a different
    // target every ping and the median is just the least-bad guess.
    LOG_INFO("MoonTank: %.3f m  (spread %.3f m, %u/%u echoes, session %.3f-%.3f)", lastM, lastSpreadM, lastValid,
             MOONHUT_TANK_SAMPLES, sessionMinM, sessionMaxM);

#if HAS_SCREEN
    // Repaint only when the number on the panel would actually change.
    //
    // This used to forceDisplay() on every sample, i.e. every MOONHUT_TANK_POLL_S: 69
    // repaints in 260 s, measured. With EINK_LIMIT_FASTREFRESH=10 every eleventh is a
    // full-refresh flash, so the panel's ~1e6-refresh budget would be spent in weeks -
    // all of it redisplaying millimetres of ultrasonic noise that round to the same
    // number on screen. Polling stays fast because the READING should be current; the
    // panel only needs repainting when it would look different.
    const bool appeared = isnan(shownM) != isnan(lastM);
    const bool moved = !isnan(lastM) && !isnan(shownM) && fabsf(lastM - shownM) >= MOONHUT_TANK_REDRAW_DELTA_M;
    if (screen && (appeared || moved)) {
        shownM = lastM;
        screen->forceDisplay();
    }
#endif
}

// Why is there no echo? Guessing cost days on the DS18B20 bus; a diagnostic that
// separates the causes costs one flash. Three questions, in the order that narrows
// fastest:
//
//   1. Can TRIG actually drive? (a pin that cannot output explains everything)
//   2. What is ECHO resting at? (stuck LOW = nothing answering or no power;
//      stuck HIGH = miswired, or ECHO tied to something it should not be)
//   3. Does ANY free pin see a pulse when we trigger? (finds a swapped TRIG/ECHO,
//      or a pad whose silkscreen lies - which is exactly what happened on GPIO 47)
#ifdef MOONHUT_TANK_DUAL
#define IS_RANGER_B_PIN(p) ((p) == MOONHUT_TANK_TRIG_PIN2 || (p) == MOONHUT_TANK_ECHO_PIN2)
#else
#define IS_RANGER_B_PIN(p) (false)
#endif

void MoonTankModule::diagnose()
{
    static const uint8_t candidates[] = {15, 16, 17, 40, 41, 42, 47, 48};

    // Ranger B's pins are excluded below wherever this sweep re-modes a pin. Re-moding a
    // pin that another sensor is using is exactly the mistake that made the fridge
    // button read phantom presses - a diagnostic must not disturb working hardware.

    LOG_WARN("MoonTank: --- no echo after %u bursts, diagnosing ---", (unsigned)bursts);

    // 1. Can the trigger pin drive?
    pinMode(MOONHUT_TANK_TRIG_PIN, OUTPUT);
    digitalWrite(MOONHUT_TANK_TRIG_PIN, HIGH);
    delayMicroseconds(50);
    pinMode(MOONHUT_TANK_TRIG_PIN, INPUT);
    const bool trigHigh = digitalRead(MOONHUT_TANK_TRIG_PIN);
    pinMode(MOONHUT_TANK_TRIG_PIN, OUTPUT);
    digitalWrite(MOONHUT_TANK_TRIG_PIN, LOW);
    delayMicroseconds(50);
    pinMode(MOONHUT_TANK_TRIG_PIN, INPUT);
    const bool trigLow = digitalRead(MOONHUT_TANK_TRIG_PIN);
    pinMode(MOONHUT_TANK_TRIG_PIN, OUTPUT);
    digitalWrite(MOONHUT_TANK_TRIG_PIN, LOW);

    if (trigHigh && !trigLow)
        LOG_INFO("MoonTank: GPIO %d drives both ways - TRIG is fine", MOONHUT_TANK_TRIG_PIN);
    else
        LOG_WARN("MoonTank: GPIO %d did NOT follow its own output (high=%d low=%d) - either the pin "
                 "cannot drive, or something external is holding it",
                 MOONHUT_TANK_TRIG_PIN, trigHigh, trigLow);

    // 2. Resting state of every free pin, with the internal pulldown on. A pin held
    // HIGH against a ~45k pulldown has something external driving it.
    LOG_INFO("MoonTank: --- idle levels (HIGH = driven externally) ---");
    for (uint8_t pin : candidates) {
        if (pin == MOONHUT_TANK_TRIG_PIN || IS_RANGER_B_PIN(pin))
            continue;
        pinMode(pin, INPUT_PULLDOWN);
        delayMicroseconds(300);
        const bool high = digitalRead(pin);
        pinMode(pin, INPUT);
        LOG_INFO("MoonTank: GPIO %u %s%s", pin, high ? "HIGH" : "floating",
                 pin == MOONHUT_TANK_ECHO_PIN ? "   <- configured ECHO" : "");
    }

    // 3. Trigger, then watch every candidate at once for a pulse. If the sensor is
    // alive and wired anywhere we can see, this finds it and names the pin.
    LOG_INFO("MoonTank: --- pulse sweep: triggering, watching all free pins ---");
    for (uint8_t pin : candidates)
        if (pin != MOONHUT_TANK_TRIG_PIN && !IS_RANGER_B_PIN(pin))
            pinMode(pin, INPUT);

    uint32_t rose[sizeof(candidates)] = {};
    uint32_t fell[sizeof(candidates)] = {};

    digitalWrite(MOONHUT_TANK_TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(MOONHUT_TANK_TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(MOONHUT_TANK_TRIG_PIN, LOW);

    const uint32_t t0 = micros();
    bool prev[sizeof(candidates)] = {};
    while (micros() - t0 < 40000) { // 40 ms: past anything this sensor can return
        for (uint8_t i = 0; i < sizeof(candidates); i++) {
            if (candidates[i] == MOONHUT_TANK_TRIG_PIN)
                continue;
            const bool now = digitalRead(candidates[i]);
            if (now && !prev[i] && rose[i] == 0)
                rose[i] = micros() - t0;
            if (!now && prev[i] && rose[i] != 0 && fell[i] == 0)
                fell[i] = micros() - t0;
            prev[i] = now;
        }
    }

    bool any = false;
    for (uint8_t i = 0; i < sizeof(candidates); i++) {
        if (rose[i] == 0)
            continue;
        any = true;
        const uint32_t width = fell[i] > rose[i] ? fell[i] - rose[i] : 0;
        LOG_WARN("MoonTank: >>> GPIO %u pulsed: rose at %u us, width %u us (~%.3f m)", candidates[i],
                 (unsigned)rose[i], (unsigned)width, (width * 1e-6f * 343.0f) / 2.0f);
        if (candidates[i] != MOONHUT_TANK_ECHO_PIN)
            LOG_WARN("MoonTank: >>> that is NOT the configured ECHO pin (%d) - rebuild with "
                     "-D MOONHUT_TANK_ECHO_PIN=%u, or move the wire",
                     MOONHUT_TANK_ECHO_PIN, candidates[i]);
    }

    if (!any)
        LOG_WARN("MoonTank: NO pin pulsed. The sensor is not responding at all: check VCC (3V3, and "
                 "the HC-SR04P is the 3.3 V variant - a plain HC-SR04 needs 5 V), check GND, and "
                 "check TRIG really reaches the module. A dead-quiet ECHO is not a pin problem.");

    pinMode(MOONHUT_TANK_ECHO_PIN, INPUT);
}

// Where to report. getByName() falls back to the primary channel when the named one is
// absent, which is what an unprovisioned bench node wants. Resolved by POSITION, never
// from meshtastic_Channel::index - that field is 0 for any channel added over the CLI,
// and trusting it sent the fridge node's broadcasts out on the public channel.
#ifndef MOONHUT_TANK_CHANNEL
#define MOONHUT_TANK_CHANNEL "MoonFleet"
#endif

static ChannelIndex tankChannelIndex()
{
    for (ChannelIndex i = 0; i < channels.getNumChannels(); i++)
        if (strcasecmp(channels.getGlobalId(i), MOONHUT_TANK_CHANNEL) == 0)
            return i;
    return channels.getPrimaryIndex();
}

void MoonTankModule::sendLine(const char *text)
{
    if (!router || !service)
        return;
    meshtastic_MeshPacket *p = router->allocForSending();
    if (!p)
        return;
    p->decoded.portnum = meshtastic_PortNum_TEXT_MESSAGE_APP;
    // Broadcast, not a DM: this node has never exchanged keys with the gateway, so a
    // PKI DM would fail to encrypt. Broadcast on the private channel is the honest
    // option until it is properly provisioned.
    p->to = NODENUM_BROADCAST;
    p->channel = tankChannelIndex();
    p->want_ack = false;
    p->decoded.payload.size = snprintf((char *)p->decoded.payload.bytes, sizeof(p->decoded.payload.bytes), "%s", text);
    service->sendToMesh(p, RX_SRC_LOCAL, false);
    LOG_INFO("MoonTank: sent %s", text);
}

// Keep one sample a minute at most. Polling is every 2 s for a live panel, but a rate fit
// wants spread-out points: 16 samples two seconds apart span 30 s and would measure noise.
void MoonTankModule::recordLevel(uint32_t now, float metres)
{
    if (isnan(metres))
        return; // never let a rejected reading into the fit
    if (rateCount && (now - lastRateAt) < (MOONHUT_TANK_RATE_DECIMATE_S * 1000UL))
        return;
    lastRateAt = now;
    rateBuf[rateHead] = {now, metres};
    rateHead = (uint8_t)((rateHead + 1) % MOONHUT_TANK_RATE_SAMPLES);
    if (rateCount < MOONHUT_TANK_RATE_SAMPLES)
        rateCount++;
}

// Least-squares slope of distance against time, negated so the answer is LEVEL change.
float MoonTankModule::levelRateMph() const
{
    if (rateCount < 3)
        return NAN;

    // Oldest sample first, so the time base is monotonic across the ring wrap.
    const uint8_t start = (uint8_t)((rateHead + MOONHUT_TANK_RATE_SAMPLES - rateCount) % MOONHUT_TANK_RATE_SAMPLES);
    const uint32_t t0 = rateBuf[start].atMs;
    const uint32_t span = rateBuf[(uint8_t)((rateHead + MOONHUT_TANK_RATE_SAMPLES - 1) % MOONHUT_TANK_RATE_SAMPLES)].atMs - t0;
    if (span < (MOONHUT_TANK_RATE_MIN_SPAN_S * 1000UL))
        return NAN; // too short a window to mean anything

    double sx = 0, sy = 0, sxx = 0, sxy = 0;
    for (uint8_t i = 0; i < rateCount; i++) {
        const RateSample &r = rateBuf[(uint8_t)((start + i) % MOONHUT_TANK_RATE_SAMPLES)];
        const double x = (double)(r.atMs - t0) / 3600000.0; // hours
        const double y = (double)r.m;
        sx += x;
        sy += y;
        sxx += x * x;
        sxy += x * y;
    }
    const double n = (double)rateCount;
    const double denom = n * sxx - sx * sx;
    if (denom <= 0)
        return NAN;
    const double slope = (n * sxy - sx * sy) / denom; // metres of DISTANCE per hour
    return (float)(-slope);                           // negate: distance down = level up
}

void MoonTankModule::report(bool force)
{
    const uint32_t now = millis();
    const bool due = (int32_t)(now - nextReportAt) >= 0;

    // A change worth an out-of-band report, CONFIRMED by a second reading before it is
    // believed. The raw sensor wanders further than the delta on a target that is not
    // moving, so an unconfirmed change re-fired on almost every poll: 140 messages/hour,
    // median gap 15s, minimum 1s. That is a mesh problem by itself, and it is the prime
    // suspect for the RF that kept killing the gateway's USB port on 2026-08-27.
    bool moved = false;
    if (!isnan(lastM)) {
        if (isnan(reportedM)) {
            moved = true;
        } else if (fabsf(lastM - reportedM) >= MOONHUT_TANK_REPORT_DELTA_M) {
            // Believe it only when the previous sample said the same thing.
            moved = !isnan(pendingM) && fabsf(lastM - pendingM) < MOONHUT_TANK_REPORT_DELTA_M;
            pendingM = lastM;
        } else {
            pendingM = NAN;
        }
    }

    // Even a confirmed change waits for the floor. A tank that is genuinely filling would
    // otherwise broadcast at the poll rate for as long as it kept moving.
    if (moved && (int32_t)(now - nextMinReportAt) < 0) {
        moved = false;
    }

    if (!force && !due && !moved)
        return;

    char line[224]; // room for ranger B's fields when the A/B rig is fitted
    if (isnan(lastM)) {
        // Carry the rejected median and its spread. "d=?" alone says something is wrong;
        // raw= and sp= say WHAT, which is the difference between diagnosing a boxed node
        // from the mesh and driving out to it with a laptop.
        char raw[24];
        if (isnan(lastRawM))
            snprintf(raw, sizeof(raw), "?");
        else
            snprintf(raw, sizeof(raw), "%.3f", (double)lastRawM);
        snprintf(line, sizeof(line), "TANK|d=?|raw=%s|sp=%.3f|e=%u/%u|why=%s|fails=%lu|up=%lus", raw,
                 (double)lastSpreadM, lastValid, MOONHUT_TANK_SAMPLES, reject ? reject : "no echo",
                 (unsigned long)consecFails, (unsigned long)(millis() / 1000));
    } else {
        // r is LEVEL change in metres/hour: + filling, - draining. "?" until the fit has a
        // long enough window - an unknown rate is said out loud rather than sent as 0.000,
        // which a consumer would otherwise plot as "perfectly steady".
        const float rate = levelRateMph();
        char r[16];
        if (isnan(rate))
            snprintf(r, sizeof(r), "?");
        else
            snprintf(r, sizeof(r), "%+.3f", (double)rate);
        snprintf(line, sizeof(line), "TANK|d=%.3f|sp=%.3f|e=%u/%u|min=%.3f|max=%.3f|r=%s|up=%lus", lastM,
                 lastSpreadM, lastValid, MOONHUT_TANK_SAMPLES, sessionMinM, sessionMaxM, r,
                 (unsigned long)(millis() / 1000));
    }
#ifdef MOONHUT_TANK_DUAL
    // Ranger B rides along on A's report rather than triggering its own: the comparison
    // is read afterwards from the log, and a second sensor must not double the mesh
    // traffic that already had to be throttled once.
    {
        char b[64];
        if (isnan(lastM2))
            snprintf(b, sizeof(b), "|d2=?|e2=%u/%u", lastValid2, MOONHUT_TANK_SAMPLES);
        else
            snprintf(b, sizeof(b), "|d2=%.3f|sp2=%.3f|e2=%u/%u", lastM2, lastSpread2, lastValid2,
                     MOONHUT_TANK_SAMPLES);
        strncat(line, b, sizeof(line) - strlen(line) - 1);
    }
#endif
    sendLine(line);

    // Fast-drain alert, edge-triggered both ways so it cannot spam. Disabled unless a
    // threshold has been set for this tank - see the note in the header.
    if (MOONHUT_TANK_FAST_DRAIN_MPH > 0.0f) {
        const float rate = levelRateMph();
        const bool draining = !isnan(rate) && rate <= -MOONHUT_TANK_FAST_DRAIN_MPH;
        if (draining && !drainAlarm) {
            drainAlarm = true;
            char a[96];
            snprintf(a, sizeof(a), "TANK ALARM|fast drain|r=%+.3f|limit=%.3f", (double)rate,
                     (double)MOONHUT_TANK_FAST_DRAIN_MPH);
            sendLine(a);
        } else if (!draining && drainAlarm && !isnan(rate)) {
            drainAlarm = false;
            sendLine("TANK CLEAR|drain back within limit");
        }
    }

    reportedM = lastM;
    pendingM = NAN;
    nextReportAt = now + (MOONHUT_TANK_REPORT_S * 1000UL);
    nextMinReportAt = now + (MOONHUT_TANK_MIN_REPORT_S * 1000UL);
}

// Keep the panel lit on external power, and honest on battery.
//
// e-ink keeps its last image with the power off, which is a trap: a blanked panel still
// displays a number, so "asleep" and "showing a stale reading" look identical from the
// front. On battery the panel is therefore woken briefly once an hour so what it shows
// is never more than an hour old.
void MoonTankModule::serviceScreen(uint32_t now)
{
#if HAS_SCREEN
    if (!screen)
        return;

    // Sleep ONLY on positive evidence of battery operation. Never on the absence of
    // evidence of mains.
    //
    // PowerStatus is TRI-state: getHasUSB() is `hasUSB == OptTrue`, so an UNKNOWN reading
    // returns false, indistinguishable from "definitely no USB". The previous test was
    //
    //     hasBattery && !getHasUSB()
    //
    // which quietly turned "don't know" into "on battery". Fit a battery to a mains node -
    // as MoonTank now has - and the panel slept 60 s after every boot while sitting on a
    // charger, which is the exact failure this policy exists to prevent. The comment above
    // it even said not to trust getHasUSB(), while the code depended on it.
    //
    // Charging is the most useful signal of the three: a cell cannot charge without an
    // external supply, and unlike getHasUSB() it is derived from a measurement rather than
    // a pin some boards do not wire up.
#ifdef MOONHUT_TANK_NEVER_SLEEP
    // Mains installation: the question does not arise. Declared per-variant rather than
    // inferred, because a permanently-powered node should not depend on power detection
    // working correctly on a board where it demonstrably does not.
    const bool onBattery = false;
#else
    const bool external = !powerStatus                     // no reading at all: assume mains
                          || !powerStatus->getHasBattery() // nothing to run from anyway
                          || powerStatus->getHasUSB()      // positively told USB is there
                          || powerStatus->getIsCharging(); // can only happen on external power
    const bool onBattery = !external;
#endif
    const bool onUsb = !onBattery;

    if (onUsb != wasOnUsb) {
        wasOnUsb = onUsb;
        LOG_INFO("MoonTank: power source is now %s - screen %s", onUsb ? "external" : "battery",
                 onUsb ? "stays on" : "will sleep and refresh hourly");
    }

    if (onUsb) {
        // Mains: never blank, and RE-ASSERT it rather than only undoing our own sleep.
        //
        // PowerFSM darkens the panel on its own timer whenever config.display.screen_on_secs
        // is non-zero (PowerFSM.cpp:380), without ever passing through this module. The
        // first version only tracked the sleep IT had caused, so a PowerFSM blank left
        // screenOffSince at 0 and the panel stayed dark on mains for good - the exact
        // failure this policy exists to prevent. Ask the screen what it is actually doing.
        if (!screen->isScreenOn()) {
            LOG_INFO("MoonTank: panel was dark on external power - waking it");
            screen->setOn(true);
            screen->forceDisplay();
        }
        screenOffSince = 0;
        screenOnSince = now;
        return;
    }

    if (screenOffSince == 0) {
        if (screenOnSince == 0)
            screenOnSince = now;
        if ((now - screenOnSince) >= (MOONHUT_TANK_BATT_SCREEN_ON_S * 1000UL)) {
            screen->setOn(false);
            screenOffSince = now;
        }
    } else if ((now - screenOffSince) >= (MOONHUT_TANK_BATT_REFRESH_S * 1000UL)) {
        // Hourly: wake, let the frame redraw with a current reading, then it blanks
        // again after the on-period above.
        screen->setOn(true);
        screen->forceDisplay();
        screenOffSince = 0;
        screenOnSince = now;
        LOG_INFO("MoonTank: hourly panel refresh on battery");
    }
#endif
}

int32_t MoonTankModule::runOnce()
{
    measure();

    // Announce a stall ONCE, edge-triggered, then say so again when it clears. A node
    // that simply stops reporting a distance is indistinguishable from a node that has
    // gone off the air; this makes "the sensor is not giving me a number" an event the
    // mesh can see rather than an absence somebody has to notice.
    //
    // lastGoodAtMs is 0 until the first good reading, so a node that has NEVER read is
    // measured from boot and alerts on the same timer.
    const uint32_t now = millis();
    if (!stallAnnounced && (now - lastGoodAtMs) > (MOONHUT_TANK_STALL_S * 1000UL)) {
        stallAnnounced = true;
        char a[176];
        snprintf(a, sizeof(a), "TANK STALL|no valid reading for %lus|fails=%lu|raw=%.3f|sp=%.3f|why=%s|up=%lus",
                 (unsigned long)((now - lastGoodAtMs) / 1000), (unsigned long)consecFails, (double)lastRawM,
                 (double)lastSpreadM, reject ? reject : "no echo", (unsigned long)(now / 1000));
        sendLine(a);
        LOG_ERROR("MoonTank: STALLED - %lu consecutive failures", (unsigned long)consecFails);
    }

    report(false);
    serviceScreen(now);
    return MOONHUT_TANK_POLL_S * 1000;
}

#endif // MOONHUT_TANK
