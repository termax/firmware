#include "MoonTankModule.h"

#ifdef MOONHUT_TANK

#include "FSCommon.h" // littlefs: the persisted tank calibration
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
// Every number below is MEASURED or datasheet, and each one has cost time when wrong.
// A sensor whose timeout or dead zone is set for a different part does not read badly -
// it does not read at all, which is indistinguishable from a wiring fault.
const MoonTankSensor MOONHUT_TANK_SENSORS[] = {
    // JSN-SR04T V3.3, transducer soldered on the board. The one in service.
    // 50 us trigger is MEASURED: 10 us produced 0/5 echoes every time, 20 was marginal.
    // The 0.30 m floor is the RINGDOWN artifact - a rock-steady 0.23-0.25 m from this
    // part is the transducer still ringing, not a real short-range measurement.
    {"jsn", 50, 25000, 343.0f, 0.30f, 4.5f, 0, 100, false, 0, "JSN-SR04T V3.3 waterproof, transducer on board"},

    // HC-SR04P, the 3.3 V twin-transducer bench yardstick. Same interface and timing as
    // the JSN; NOT waterproof, so bench reference only, never a tank.
    {"hcsr04", 50, 25000, 343.0f, 0.30f, 4.5f, 0, 100, false, 0, "HC-SR04P 3.3 V bench reference, NOT waterproof"},

    // DYP A02 in PWM mode - waterproof bistatic probe, IP67, 3.3-5 V.
    // THREE things differ from the JSN and all three matter:
    //  * it stays silent for T1 = 10-17 ms BEFORE the echo pulse starts, and pulseIn()
    //    counts that wait against its timeout - 25 ms expires first and every burst
    //    reports "no echo" from a working sensor. 60 ms.
    //  * it is internally temperature compensated and fixes sound at 348 m/s, not 343
    //    (datasheet: S_cm = T_us / 57.5). ~1.5 %, about 3 cm at 2 m.
    //  * a 3 cm blind zone against the JSN's ringdown, so it can see much closer. 0.05
    //    is still conservative here - we have not yet confirmed this part is free of the
    //    ringdown artifact on OUR mounting, only that the datasheet claims 3 cm.
    // No target returns a FIXED ~35 ms pulse, which must be read as "nothing there"
    // rather than believed as ~6 m.
    // 250 ms between pings, against the JSN's 100. MEASURED 2026-09-06: at 100 ms this
    // part answered exactly ONE ping in five and the burst was thrown out as "too few
    // echoes" - the remaining four land inside its recovery window and return nothing.
    // The datasheet's ">70 ms period" is a floor for the trigger, not what the module
    // needs to be ready again. Five pings now cost 1.25 s inside a 3 s poll.
    {"a02", 50, 60000, 348.0f, 0.05f, 4.5f, 35000, 250, true, 5000, "DYP A02 PWM, waterproof bistatic, 3 cm blind zone"},
};
const uint8_t MOONHUT_TANK_SENSOR_COUNT = sizeof(MOONHUT_TANK_SENSORS) / sizeof(MOONHUT_TANK_SENSORS[0]);

const MoonTankSensor *MoonTankModule::lookupSensor(const char *name) const
{
    if (!name || !*name)
        return nullptr;
    for (uint8_t i = 0; i < MOONHUT_TANK_SENSOR_COUNT; i++)
        if (strcasecmp(name, MOONHUT_TANK_SENSORS[i].name) == 0)
            return &MOONHUT_TANK_SENSORS[i];
    return nullptr;
}

// Never returns null: an unknown stored name falls back to the default rather than
// leaving the module with no numbers at all.
const MoonTankSensor *MoonTankModule::activeSensor()
{
    if (!sensor) {
        sensor = lookupSensor(MOONHUT_TANK_SENSOR_DEFAULT);
        if (!sensor)
            sensor = &MOONHUT_TANK_SENSORS[0];
    }
    return sensor;
}

// NOTE: the echo timeout, speed of sound and valid range used to live here as fixed
// constants. They are per-sensor and now come from the active MoonTankSensor profile -
// see the table above and `tank:sensor=`. Nothing should reintroduce a global here.

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
    const MoonTankSensor *sn0 = activeSensor();

    // POLARITY IS PER-SENSOR. An HC-SR04/JSN idles its trigger LOW and is started by a
    // HIGH pulse. A DYP A02 is the other way round: the datasheet says it starts on a
    // FALLING edge and its timing diagram idles RX HIGH, dipping LOW to trigger.
    //
    // Driving an A02 the JSN way still produces a falling edge at the END of the pulse,
    // so it half-works - which is worse than not working. It leaves the trigger line
    // held LOW between pings, the module free-runs, and only the ping that happens to
    // line up returns anything: a steady ONE echo in five, every burst discarded as
    // "too few echoes", with a plausible distance in the one that got through.
    const bool idle = sn0->trigActiveLow;   // resting level
    digitalWrite(trigPin, idle);
    delayMicroseconds(2);
    digitalWrite(trigPin, !idle);
    delayMicroseconds(trigUs);
    digitalWrite(trigPin, idle);            // and LEAVE it resting, not asserted

    const MoonTankSensor *sn = activeSensor();

    // BLANKING. pulseIn() starts listening the instant the trigger fires, and the trigger
    // pin is adjacent to the echo pin - so it latches the few microseconds of crosstalk
    // from its own edge, times THAT, and returns. The ping is consumed and the real echo
    // arriving 11 ms later is never seen.
    //
    // MEASURED on the A02 2026-09-06: 12 runts of 1-12 us and 12 timeouts against a
    // single good 8766 us read, i.e. the "1 of 5 echoes" that survived a ping-gap change
    // and a trigger-polarity change because neither was the cause.
    //
    // Per-sensor, because a JSN legitimately answers in ~500 us and must not be deafened.
    // The A02 cannot reply before T1 = 10 ms, so 5 ms of deafness costs it nothing.
    if (sn->echoBlankUs)
        delayMicroseconds(sn->echoBlankUs);

    const uint32_t us = pulseIn(echoPin, HIGH, sn->echoTimeoutUs);

    // Every path below returns NAN, and they mean COMPLETELY different things: a silent
    // sensor, a sensor explicitly saying "nothing there", and a sensor reading outside
    // the tank. Collapsing them into one NAN is why "1 of 5 echoes" was unreadable -
    // it could equally have been a wiring fault, bad aim, or a filter set too tight.
    // One line per ping settles it, and a burst is only five of them every few seconds.
    const float m = (us * 1e-6f * sn->speedMs) / 2.0f;
    const bool sentinel = sn->deadPulseUs && us > sn->deadPulseUs - 2000 && us < sn->deadPulseUs + 2000;
    const char *verdict = us == 0            ? "TIMEOUT - no pulse at all"
                          : sentinel         ? "NO TARGET - sensor's fixed no-echo pulse"
                          : m < activeFloorM() ? "too near (under the floor)"
                          : m > sn->maxValidM ? "too far (over the profile ceiling)"
                                              : "ok";
    LOG_DEBUG("MoonTank ping: raw %u us -> %.3f m : %s", (unsigned)us, (double)m, verdict);

    if (us == 0 || sentinel)
        return NAN;
    if (m < activeFloorM() || m > sn->maxValidM)
        return NAN;
    return m;
}

bool MoonTankModule::evaluate(float *s, uint8_t n, float &median, float &spread, const char *&why)
{
    why = nullptr;
    median = NAN;
    spread = NAN;

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

    // Cluster the sorted pings: a gap wider than AGREE_M between neighbours starts a new
    // cluster. See the header - the median of a two-population burst is a coin toss.
    uint8_t cstart[MOONHUT_TANK_SAMPLES], csize[MOONHUT_TANK_SAMPLES], nc = 1;
    cstart[0] = 0;
    csize[0] = 1;
    for (uint8_t i = 1; i < n; i++) {
        if (s[i] - s[i - 1] > MOONHUT_TANK_AGREE_M) {
            cstart[nc] = i;
            csize[nc] = 1;
            nc++;
        } else {
            csize[nc - 1]++;
        }
    }
    lastClusters = nc;

    // Choose a cluster. Belief first (the cross-burst median, which survives a bad burst),
    // then size, then distance - farthest wins a tie because everything spurious in a tank
    // is nearer than the water.
    const float ref = stableM();
    int8_t best = -1;
    float bestOff = 0;
    if (!isnan(ref)) {
        for (uint8_t c = 0; c < nc; c++) {
            const float cm = s[cstart[c] + csize[c] / 2];
            const float off = fabsf(cm - ref);
            if (off <= MOONHUT_TANK_TRACK_GATE_M && (best < 0 || off < bestOff)) {
                best = (int8_t)c;
                bestOff = off;
            }
        }
    }
    if (best < 0) {
        for (uint8_t c = 0; c < nc; c++) {
            if (best < 0 || csize[c] > csize[best] ||
                (csize[c] == csize[best] && s[cstart[c]] > s[cstart[best]]))
                best = (int8_t)c;
        }
    }
    const uint8_t b0 = cstart[best], bn = csize[best];
    median = s[b0 + bn / 2];              // median OF THE CHOSEN CLUSTER, not of everything
    spread = s[b0 + bn - 1] - s[b0];      // its range - the trust indicator, now honest
    const uint8_t agree = bn;             // a cluster IS the set that agrees

    if (n < MOONHUT_TANK_MIN_ECHOES)
        why = "too few echoes";
    else if (agree < MOONHUT_TANK_AGREE_MIN)
        why = "no consensus";

    // Dump the individual pings when a burst is rejected. "samples disagree" without the
    // samples is the same hole as "d=?" without the raw value: it says something is wrong
    // and withholds the one thing that says WHAT.
    //
    // The SHAPE is the diagnosis. Sorted ascending:
    //   one low outlier, rest tight   -> transducer ringdown caught on the first ping
    //   one roughly double the others -> a second reflection (wall, pipe, tank floor)
    //   scattered with no structure   -> electrical noise, or a transducer not coupling
    if (why) {
        char dbg[132];
        int p = 0;
        for (uint8_t i = 0; i < n && p < (int)sizeof(dbg) - 12; i++)
            p += snprintf(dbg + p, sizeof(dbg) - p, "%s%.3f", i ? " " : "", (double)s[i]);
        LOG_WARN("MoonTank: pings [%s] -> %s (%u clusters; chose %u of them at %.3f%s)", dbg, why, nc, agree,
                 (double)median, isnan(ref) ? "" : " by belief");
    }
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
    const uint16_t trigUs = activeSensor()->trigUs;
#endif
    const bool ok = evaluate(sampA, nA, median, spread, why);
    n = nA;

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
    // The two never overlap: the state machine waits MOONHUT_TANK_INTERLEAVE_MS between
    // A's last ping and B's first - see the header for why an overlap is worse than noise.
    const bool ok2 = evaluate(sampB, nB, lastM2, lastSpread2, reject2);
    lastValid2 = nB;
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
    // How long the sensor was quiet BEFORE this reading, captured before lastGoodAtMs is
    // overwritten. The cross-burst ring uses it to decide whether its contents are still
    // about the same water.
    const uint32_t gapMs = lastGoodAtMs ? (millis() - lastGoodAtMs) : 0;
    consecFails = 0;
    lastGoodAtMs = millis();
    if (stallAnnounced) {
        stallAnnounced = false;
        char ok[96];
        snprintf(ok, sizeof(ok), "TANK OK|reading again|d=%.3f|up=%lus", lastM, (unsigned long)(millis() / 1000));
        sendLine(ok);
    }

    // A gap long enough to count as a stall means the ring is holding readings from
    // BEFORE it. Their median would be a distance from another era, and recordLevel would
    // stamp it with a fresh timestamp - an old level at a new time is a fabricated rate,
    // which is worse than no rate. Start the window again instead.
    if (gapMs > (MOONHUT_TANK_STALL_S * 1000UL) && stableCount) {
        LOG_INFO("MoonTank: %lus gap - discarding %u stale burst(s) from the rate window",
                 (unsigned long)(gapMs / 1000), (unsigned)stableCount);
        stableCount = 0;
        stableHead = 0;
    }
    // The belief (stableM) may only learn from SINGLE-target bursts. Seen live 2026-09-08:
    // a run of wall bursts, each accepted because 3/5 pings agreed among themselves, dragged
    // stableM onto the wall - and "nearest belief" then chose the wall cluster ("chose 2 of
    // them at 0.765 by belief"). A burst that contained two targets is evidence that the
    // scene is ambiguous, not evidence about where the water is. It is still measured,
    // reported and chosen from; it just cannot teach the belief.
    if (lastClusters == 1)
        pushBurst(lastM);
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

    // GPIO 17 is I2C_SDA on this board (18 is SCL). The bus idles HIGH on its pull-up and
    // carries display traffic constantly, so an untreated sweep reports it as "pulsed" and
    // then advises rebuilding ECHO onto it - which would break the OLED and fix nothing.
    // That false positive was printed and nearly acted on while chasing a swapped JSN.
#if defined(I2C_SDA) && defined(I2C_SCL)
#define IS_I2C_PIN(p) ((p) == I2C_SDA || (p) == I2C_SCL)
#else
#define IS_I2C_PIN(p) (false)
#endif

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
        if (pin == MOONHUT_TANK_TRIG_PIN || IS_RANGER_B_PIN(pin) || IS_I2C_PIN(pin))
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
        if (pin != MOONHUT_TANK_TRIG_PIN && !IS_RANGER_B_PIN(pin) && !IS_I2C_PIN(pin))
            pinMode(pin, INPUT);

    uint32_t rose[sizeof(candidates)] = {};
    uint32_t fell[sizeof(candidates)] = {};

    // MOONHUT_TANK_TRIG_US, not a hardcoded 10. A 10 us trigger does NOTHING on a JSN -
    // it needs 50 - so the sweep was firing a pulse too short to wake the very sensor it
    // was hunting for, and "NO pin pulsed" could mean "never asked" rather than "dead".
    const bool dIdle = activeSensor()->trigActiveLow;
    digitalWrite(MOONHUT_TANK_TRIG_PIN, dIdle);
    delayMicroseconds(2);
    digitalWrite(MOONHUT_TANK_TRIG_PIN, !dIdle);
    delayMicroseconds(activeSensor()->trigUs);
    digitalWrite(MOONHUT_TANK_TRIG_PIN, dIdle);

    const uint32_t t0 = micros();
    bool prev[sizeof(candidates)] = {};
    // 60 ms, not 40: an A02 can spend 17 ms before the pulse starts and 35 ms sending it,
    // which a 40 ms window clips - and a clipped pulse reads as a short one, i.e. a wrong
    // distance rather than an obvious failure.
    const uint32_t sweepUs = activeSensor()->echoTimeoutUs + 5000;
    while (micros() - t0 < sweepUs) {
        for (uint8_t i = 0; i < sizeof(candidates); i++) {
            if (candidates[i] == MOONHUT_TANK_TRIG_PIN || IS_I2C_PIN(candidates[i]))
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
        const uint32_t width = fell[i] > rose[i] ? fell[i] - rose[i] : 0;
        // A rise with no matching fall has width 0 - that is a pin going high and STAYING
        // high, which is a held line or bus noise, not an echo. Reporting it as a pulse is
        // how the sweep came to recommend moving ECHO onto a pin carrying zero information.
        if (width == 0) {
            LOG_INFO("MoonTank: GPIO %u rose at %u us but never fell - held high, not an echo",
                     candidates[i], (unsigned)rose[i]);
            continue;
        }
        any = true;
        LOG_WARN("MoonTank: >>> GPIO %u pulsed: rose at %u us, width %u us (~%.3f m)", candidates[i],
                 (unsigned)rose[i], (unsigned)width, (width * 1e-6f * activeSensor()->speedMs) / 2.0f);
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

// Every accepted burst median goes in the ring, whether or not the fit wants a sample
// this minute - the ring has to be full of RECENT bursts at the moment the decimation
// boundary arrives, or the median is computed over stale readings.
// One line per burst while aiming. Terse on purpose: it is read on a phone, outdoors,
// by someone holding a probe in the other hand.
//
// The reject reason is the payload here, not an afterthought. A steady 0.23 m "ringdown"
// means the module fired and heard nothing back - on a tank that is almost always a probe
// that is not perpendicular to the water, because a flat surface is a mirror and reflects
// a tilted beam away instead of back. That is a different action from "no echo", and the
// person on the ladder needs to be told which.
void MoonTankModule::sendLive()
{
    char line[96];
    const unsigned long left = (unsigned long)((liveUntilMs - millis()) / 1000);
    if (isnan(lastM)) {
        char raw[24];
        if (isnan(lastRawM))
            snprintf(raw, sizeof(raw), "?");
        else
            snprintf(raw, sizeof(raw), "%.3f", (double)lastRawM);
        snprintf(line, sizeof(line), "LIVE|d=?|raw=%s|e=%u/%u|why=%s|%lus left", raw, lastValid,
                 MOONHUT_TANK_SAMPLES, reject ? reject : "no echo", left);
    } else {
        snprintf(line, sizeof(line), "LIVE|d=%.3f|sp=%.0fmm|e=%u/%u|%lus left", (double)lastM,
                 (double)(lastSpreadM * 1000.0f), lastValid, MOONHUT_TANK_SAMPLES, left);
    }
    sendLine(line);
}

void MoonTankModule::pushBurst(float metres)
{
    if (isnan(metres))
        return;
    stableBuf[stableHead] = metres;
    stableHead = (uint8_t)((stableHead + 1) % MOONHUT_TANK_STABLE_WINDOW);
    if (stableCount < MOONHUT_TANK_STABLE_WINDOW)
        stableCount++;
}

// Median of the ring. NAN until it is FULL: a median over two samples is just their mean
// and gives none of the protection this exists for, so the fit waits rather than being
// fed something weaker than it thinks it is getting.
float MoonTankModule::stableM() const
{
    if (stableCount < MOONHUT_TANK_STABLE_WINDOW)
        return NAN;
    float v[MOONHUT_TANK_STABLE_WINDOW];
    memcpy(v, stableBuf, sizeof(v));
    for (uint8_t i = 1; i < MOONHUT_TANK_STABLE_WINDOW; i++) {
        const float k = v[i];
        int8_t j = (int8_t)i - 1;
        while (j >= 0 && v[j] > k) {
            v[j + 1] = v[j];
            j--;
        }
        v[j + 1] = k;
    }
    return v[MOONHUT_TANK_STABLE_WINDOW / 2];
}

// Keep one sample a minute at most. Polling is every 2 s for a live panel, but a rate fit
// wants spread-out points: 16 samples two seconds apart span 30 s and would measure noise.
void MoonTankModule::recordLevel(uint32_t now, float metres)
{
    if (isnan(metres))
        return; // never let a rejected reading into the fit
    if (rateCount && (now - lastRateAt) < (MOONHUT_TANK_RATE_DECIMATE_S * 1000UL))
        return;

    // The CROSS-BURST median, not the burst that happened to land on this boundary. A
    // single object crossing the beam yields five agreeing pings that the in-burst filter
    // accepts, and decimation would otherwise copy that straight into the fit. NAN while
    // the ring fills, which just means the fit starts a few bursts later.
    const float stable = stableM();
    if (isnan(stable))
        return;
    if (fabsf(stable - metres) > MOONHUT_TANK_AGREE_M)
        LOG_INFO("MoonTank: rate fed %.3f m (cross-burst median), not this burst's %.3f m", (double)stable,
                 (double)metres);
    metres = stable;

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

// Called at the end of every burst. Kept out of report() deliberately: report() is
// governed by the change/heartbeat/floor logic, and live aiming must not be entangled
// with any of that - nor allowed to reset those timers.
void MoonTankModule::serviceLive()
{
    if (!liveUntilMs)
        return;
    const uint32_t now = millis();
    if ((int32_t)(now - liveUntilMs) >= 0) {
        liveUntilMs = 0;
        sendLine("LIVE|off (window expired)");
        LOG_INFO("MoonTank: live mode expired");
        return;
    }
    if ((int32_t)(now - nextLiveAtMs) < 0)
        return;
    nextLiveAtMs = now + MOONHUT_TANK_LIVE_PERIOD_S * 1000UL;
    sendLive();
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
        } else if (fabsf(lastM - reportedM) >= activeDeltaM()) {
            // Believe it only when the previous sample said the same thing.
            moved = !isnan(pendingM) && fabsf(lastM - pendingM) < activeDeltaM();
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
        snprintf(line, sizeof(line), "TANK|d=?|raw=%s|sp=%.3f|e=%u/%u|nc=%u|why=%s|fails=%lu|up=%lus", raw,
                 (double)lastSpreadM, lastValid, MOONHUT_TANK_SAMPLES, lastClusters, reject ? reject : "no echo",
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
        snprintf(line, sizeof(line), "TANK|d=%.3f|sp=%.3f|e=%u/%u|nc=%u|min=%.3f|max=%.3f|r=%s|up=%lus", lastM,
                 lastSpreadM, lastValid, MOONHUT_TANK_SAMPLES, lastClusters, sessionMinM, sessionMaxM, r,
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
    nextReportAt = now + (activeReportS() * 1000UL);
    nextMinReportAt = now + (activeMinReportS() * 1000UL);
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
    // Deliberately not in the constructor: modules are built before littlefs is mounted,
    // which is the same trap the fridge roster hit.
    if (!calLoaded) {
        calLoaded = true;
        loadCalibration();
        // OUTSIDE loadCalibration(), which returns early when there is no file yet - and a
        // node that has never been calibrated is exactly the one where you most need to be
        // told which sensor profile it is using.
        const MoonTankSensor *sn = activeSensor();
        // Unconditional: the EFFECTIVE calibration, whatever its source. loadCalibration()
        // only logs when it applied a stored value, so a node running on build defaults
        // looked identical to one that had loaded a zero. That cost a bad diagnosis.
        LOG_INFO("MoonTank: CAL height=%.3f m dead=%.3f m (usable %.3f m)", (double)tankHeightM,
                 (double)tankOffsetM, (double)(tankHeightM - tankOffsetM));
        LOG_INFO("MoonTank: sensor profile '%s' - %s", sn->name, sn->desc);
#if MOONHUT_TANK_LIVE_ON_BOOT_MIN > 0
        // Failsafe window - see the header. Deliberately here rather than in the
        // constructor: littlefs and the channel list are up by now.
        liveUntilMs = millis() + (uint32_t)MOONHUT_TANK_LIVE_ON_BOOT_MIN * 60UL * 1000UL;
        nextLiveAtMs = millis() + (uint32_t)MOONHUT_TANK_LIVE_ON_BOOT_DELAY_S * 1000UL;
        LOG_INFO("MoonTank: live-on-boot for %u min (tank:live=0 to stop)",
                 (unsigned)MOONHUT_TANK_LIVE_ON_BOOT_MIN);
#endif
        LOG_INFO("MoonTank: trig %u us, echo wait %u ms, %.0f m/s, valid %.2f-%.2f m, dead pulse %u us",
                 (unsigned)sn->trigUs, (unsigned)(sn->echoTimeoutUs / 1000), (double)sn->speedMs,
                 (double)sn->minValidM, (double)sn->maxValidM, (unsigned)sn->deadPulseUs);
    }

#ifdef MOONHUT_TANK_TRIG_SWEEP
    static const uint16_t widths[] = MOONHUT_TANK_TRIG_WIDTHS;
    const uint16_t trigUs = widths[bursts % (sizeof(widths) / sizeof(widths[0]))];
#else
    const uint16_t trigUs = activeSensor()->trigUs;
#endif

    // ONE ping, then hand the CPU back. See the header: five back-to-back delay()s
    // starved the button thread and PRG presses inside a burst were never seen.
    if (phase == PHASE_A) {
        const float m = pingOnce(MOONHUT_TANK_TRIG_PIN, MOONHUT_TANK_ECHO_PIN, trigUs);
        if (!isnan(m) && nA < MOONHUT_TANK_SAMPLES)
            sampA[nA++] = m;
        if (++pingIdx < MOONHUT_TANK_SAMPLES)
            return activeSensor()->pingGapMs;
        pingIdx = 0;
#ifdef MOONHUT_TANK_DUAL
        phase = PHASE_B;
        return MOONHUT_TANK_INTERLEAVE_MS; // A must be fully dead before B speaks
#else
        phase = PHASE_EVAL;
        return activeSensor()->pingGapMs;
#endif
    }

#ifdef MOONHUT_TANK_DUAL
    if (phase == PHASE_B) {
        const float m = pingOnce(MOONHUT_TANK_TRIG_PIN2, MOONHUT_TANK_ECHO_PIN2, trigUs);
        if (!isnan(m) && nB < MOONHUT_TANK_SAMPLES)
            sampB[nB++] = m;
        if (++pingIdx < MOONHUT_TANK_SAMPLES)
            return activeSensor()->pingGapMs;
        pingIdx = 0;
        phase = PHASE_EVAL;
        return activeSensor()->pingGapMs;
    }
#endif

    // PHASE_EVAL - judge what was collected, then start the next cycle.
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
    serviceLive();
    serviceScreen(now);

    nA = 0;
#ifdef MOONHUT_TANK_DUAL
    nB = 0;
#endif
    phase = PHASE_A;
    return (int32_t)pollS * 1000;
}


// --- Calibration -----------------------------------------------------------

float MoonTankModule::levelM() const
{
    if (!isCalibrated() || isnan(lastM))
        return NAN;
    // The sensor measures DOWN to the surface, so depth is height minus distance.
    const float d = tankHeightM - lastM;
    return d < 0.0f ? 0.0f : d;
}

float MoonTankModule::levelPct() const
{
    const float d = levelM();
    if (isnan(d))
        return NAN;
    // Usable depth excludes the dead space at the top the sensor cannot see. Without
    // that subtraction a "full" tank reads short of 100 % forever and everyone learns
    // to distrust the number.
    const float usable = tankHeightM - tankOffsetM;
    if (usable <= 0.0f)
        return NAN;
    float pct = (d / usable) * 100.0f;
    if (pct < 0.0f)
        pct = 0.0f;
    if (pct > 100.0f)
        pct = 100.0f;
    return pct;
}

void MoonTankModule::loadCalibration()
{
    auto f = FSCom.open(MOONHUT_TANK_CFG_PATH, FILE_O_READ);
    if (!f)
        return;
    char buf[128] = {0};
    size_t n = f.readBytes(buf, sizeof(buf) - 1);
    f.close();
    // Print exactly what was on disk. Two sessions of reasoning about why this file loads as
    // height 0.000 produced two wrong theories; the bytes themselves cannot be argued with.
    LOG_INFO("MoonTank: cfg file %s = [%s] (%u bytes)", MOONHUT_TANK_CFG_PATH, buf, (unsigned)n);
    if (!n)
        return;
    // Third field added 2026-09-06, fourth (poll seconds) 2026-09-07. sscanf returns the
    // count it actually filled, so a file written by an older build still loads and simply
    // keeps the defaults for the fields it lacks - no migration, no version byte.
    // NOT sscanf. With the file bytes verified correct on the wire -
    // [2.0000 0.3000 jsn 3 0.0000 15 10 0.0000] - sscanf("%f ...") still yielded height 0.000
    // across three flashes on two nights. This toolchain's newlib is built without float
    // scanf support: the format is accepted, the floats never land. atof()/strtof() work -
    // `tank:height=` goes through atof and has always landed. So parse by hand: space-
    // separated fields, strtof for reals, strtoul for integers, and STOP at the first field
    // that fails so an older, shorter file still loads whatever it has.
    float h = 0, o = 0, flr = 0, dlt = 0;
    unsigned poll = 0, rep = 0, minrep = 0;
    char sname[16] = {0};
    int got = 0;
    char *p = buf, *end = nullptr;
    auto nextf = [&](float &out) -> bool {
        while (*p == ' ') p++;
        if (!*p) return false;
        out = strtof(p, &end);
        if (end == p) return false;
        p = end;
        return true;
    };
    auto nextu = [&](unsigned &out) -> bool {
        while (*p == ' ') p++;
        if (!*p) return false;
        out = (unsigned)strtoul(p, &end, 10);
        if (end == p) return false;
        p = end;
        return true;
    };
    auto nexts = [&](char *out, size_t cap) -> bool {
        while (*p == ' ') p++;
        size_t i = 0;
        while (*p && *p != ' ' && i < cap - 1) out[i++] = *p++;
        out[i] = 0;
        return i > 0;
    };
    if (nextf(h)) got = 1;
    if (got == 1 && nextf(o)) got = 2;
    if (got == 2 && nexts(sname, sizeof sname)) got = 3;
    if (got == 3 && nextu(poll)) got = 4;
    if (got == 4 && nextf(flr)) got = 5;
    if (got == 5 && nextu(rep)) got = 6;
    if (got == 6 && nextu(minrep)) got = 7;
    if (got == 7 && nextf(dlt)) got = 8;
    LOG_INFO("MoonTank: cfg parsed got=%d h=%.4f o=%.4f sensor=%s poll=%u floor=%.4f rep=%u minrep=%u delta=%.4f",
             got, (double)h, (double)o, sname, poll, (double)flr, rep, minrep, (double)dlt);
    // Only let a STORED height win if it is real. A zero means the file predates
    // calibration (or was written uncalibrated), and clobbering a build default with it is
    // how a node ends up displaying 0 % at a tank that is half full.
    if (got >= 1 && h > 0.0f) {
        tankHeightM = h;
        tankOffsetM = o;
        LOG_INFO("MoonTank: calibration loaded - height %.3f m, dead top %.3f m", tankHeightM, tankOffsetM);
    }
    if (got >= 3) {
        const MoonTankSensor *sn = lookupSensor(sname);
        if (sn) {
            sensor = sn;
        } else {
            LOG_WARN("MoonTank: stored sensor '%s' is not a known profile - falling back to %s", sname,
                     activeSensor()->name);
        }
    }
    // Range-checked on the way IN as well as from a command: a corrupt or hand-edited file
    // must not be able to park the module on a 0 s busy loop.
    // Reporting cadence. Bounds mirror the command so a hand-edited file cannot set a
    // heartbeat of 0 (which would report every burst and flood the channel).
    if (got >= 8) {
        if (rep == 0 || (rep >= 10 && rep <= 86400))
            reportS = rep;
        if (minrep == 0 || (minrep >= 5 && minrep <= 86400))
            minReportS = minrep;
        if (dlt >= 0.0f && dlt <= 5.0f)
            reportDeltaM = dlt;
        if (reportS || minReportS || reportDeltaM > 0.0f)
            LOG_INFO("MoonTank: report cadence loaded - heartbeat %u s, floor %u s, delta %.3f m",
                     (unsigned)activeReportS(), (unsigned)activeMinReportS(), (double)activeDeltaM());
    }
    if (got >= 5) {
        if (flr >= 0.0f && flr <= MOONHUT_TANK_FLOOR_MAX_M) {
            floorM = flr;
            if (floorM > 0.0f)
                LOG_INFO("MoonTank: near-field floor loaded - %.3f m (overrides the profile)", (double)floorM);
        } else {
            LOG_WARN("MoonTank: stored floor %.3f m is out of range - keeping the profile's", (double)flr);
        }
    }
    if (got >= 4) {
        if (poll >= MOONHUT_TANK_POLL_MIN_S && poll <= MOONHUT_TANK_POLL_MAX_S) {
            pollS = (uint16_t)poll;
            LOG_INFO("MoonTank: poll interval loaded - %u s", (unsigned)pollS);
        } else {
            LOG_WARN("MoonTank: stored poll %u s is out of range %u-%u - keeping %u s", poll,
                     (unsigned)MOONHUT_TANK_POLL_MIN_S, (unsigned)MOONHUT_TANK_POLL_MAX_S, (unsigned)pollS);
        }
    }
}

void MoonTankModule::saveCalibration()
{
    auto f = FSCom.open(MOONHUT_TANK_CFG_PATH, FILE_O_WRITE);
    if (!f) {
        LOG_ERROR("MoonTank: could not write %s", MOONHUT_TANK_CFG_PATH);
        return;
    }
    char buf[128];
    int n = snprintf(buf, sizeof(buf), "%.4f %.4f %s %u %.4f %u %u %.4f", tankHeightM, tankOffsetM,
                     activeSensor()->name, (unsigned)pollS, (double)floorM, (unsigned)reportS,
                     (unsigned)minReportS, (double)reportDeltaM);
    f.write((const uint8_t *)buf, n);
    f.close();
    LOG_INFO("MoonTank: calibration saved - height %.3f m, dead top %.3f m, sensor %s", tankHeightM, tankOffsetM,
             activeSensor()->name);
}

bool MoonTankModule::acceptsCommand(uint8_t channelIndex, bool pkiEncrypted) const
{
    // Same rule as the fridge: the fleet channel, or a PKI DM. Not the default channel,
    // where anything within earshot could recalibrate a tank.
    if (pkiEncrypted)
        return true;
    const char *name = channels.getGlobalId(channelIndex);
    return name && strcasecmp(name, MOONHUT_TANK_CHANNEL) == 0;
}

const char *MoonTankModule::handleCommand(const char *body)
{
    static char reply[160];
    while (*body == ' ')
        body++;

    if (strncasecmp(body, "height=", 7) == 0) {
        const float v = atof(body + 7);
        if (v <= 0.0f || v > 10.0f) {
            snprintf(reply, sizeof(reply), "tank: height must be 0-10 m, got %.3f", (double)v);
            return reply;
        }
        tankHeightM = v;
        saveCalibration();
        snprintf(reply, sizeof(reply), "tank: height=%.3f m, dead top=%.3f m -> usable %.3f m", (double)tankHeightM,
                 (double)tankOffsetM, (double)(tankHeightM - tankOffsetM));
        return reply;
    }
    if (strncasecmp(body, "offset=", 7) == 0) {
        const float v = atof(body + 7);
        if (v < 0.0f || v >= tankHeightM) {
            snprintf(reply, sizeof(reply), "tank: offset must be 0..height (%.3f)", (double)tankHeightM);
            return reply;
        }
        tankOffsetM = v;
        saveCalibration();
        snprintf(reply, sizeof(reply), "tank: dead top=%.3f m -> usable %.3f m", (double)tankOffsetM,
                 (double)(tankHeightM - tankOffsetM));
        return reply;
    }
    if (strncasecmp(body, "report=", 7) == 0 || strncasecmp(body, "minreport=", 10) == 0 ||
        strncasecmp(body, "delta=", 6) == 0) {
        // Analytics resolution: how often a report leaves the node, and how big a change
        // jumps the queue. Separate from `poll=`, which is the MEASUREMENT rate.
        const bool isRep = strncasecmp(body, "report=", 7) == 0;
        const bool isMin = strncasecmp(body, "minreport=", 10) == 0;
        const char *arg = body + (isRep ? 7 : isMin ? 10 : 6);
        if (isRep || isMin) {
            const long v = atol(arg);
            // 0 restores the build default. The floor of 10/5 s exists because a
            // heartbeat near the poll rate turns every burst into a LoRa packet.
            if (v != 0 && (v < (isMin ? 5 : 10) || v > 86400)) {
                snprintf(reply, sizeof(reply), "tank: %s must be 0 or %d-86400 s, got %ld",
                         isMin ? "minreport" : "report", isMin ? 5 : 10, v);
                return reply;
            }
            if (isRep)
                reportS = (uint32_t)v;
            else
                minReportS = (uint32_t)v;
        } else {
            const float v = atof(arg);
            if (v < 0.0f || v > 5.0f) {
                snprintf(reply, sizeof(reply), "tank: delta must be 0-5 m, got %.3f", (double)v);
                return reply;
            }
            reportDeltaM = v;
        }
        saveCalibration();
        snprintf(reply, sizeof(reply), "tank: heartbeat=%us floor=%us delta=%.3fm",
                 (unsigned)activeReportS(), (unsigned)activeMinReportS(), (double)activeDeltaM());
        return reply;
    }
    if (strncasecmp(body, "floor=", 6) == 0) {
        // Reject everything nearer than this. Exists for a float ball hanging in the beam:
        // a float can only ever be CLOSER than the water, so one cutoff separates them.
        // 0 restores the sensor profile's own floor.
        const float v = atof(body + 6);
        if (v < 0.0f || v > MOONHUT_TANK_FLOOR_MAX_M) {
            snprintf(reply, sizeof(reply), "tank: floor must be 0-%.1f m (0 = profile default), got %.3f",
                     (double)MOONHUT_TANK_FLOOR_MAX_M, (double)v);
            return reply;
        }
        if (v > 0.0f && v >= activeSensor()->maxValidM) {
            snprintf(reply, sizeof(reply), "tank: floor %.3f m is at or above this sensor's max %.3f m",
                     (double)v, (double)activeSensor()->maxValidM);
            return reply;
        }
        floorM = v;
        saveCalibration();
        // Say the COST out loud. A floor silently blinds the top of the tank, and someone
        // setting it from a phone cannot see that in a number.
        if (floorM > 0.0f)
            snprintf(reply, sizeof(reply), "tank: floor=%.3f m - readings above this are BLIND (d=? when fuller)",
                     (double)floorM);
        else
            snprintf(reply, sizeof(reply), "tank: floor=profile default (%.3f m)", (double)activeSensor()->minValidM);
        return reply;
    }
    if (strncasecmp(body, "live=", 5) == 0) {
        // Aiming aid: broadcast every burst, including the failures, for a bounded window.
        // NOT persisted and NOT resumed after a reboot - see the header.
        const long v = atol(body + 5);
        if (v < 0 || v > MOONHUT_TANK_LIVE_MAX_MIN) {
            snprintf(reply, sizeof(reply), "tank: live must be 0-%u minutes, got %ld",
                     (unsigned)MOONHUT_TANK_LIVE_MAX_MIN, v);
            return reply;
        }
        if (v == 0) {
            liveUntilMs = 0;
            snprintf(reply, sizeof(reply), "tank: live off");
            return reply;
        }
        liveUntilMs = millis() + (uint32_t)v * 60UL * 1000UL;
        if (!liveUntilMs)     // millis() wrap landing exactly on 0 would read as "off"
            liveUntilMs = 1;
        nextLiveAtMs = millis();  // first line immediately, so you know it took
        snprintf(reply, sizeof(reply), "tank: live %ld min, every %u s. tank:live=0 to stop", v,
                 (unsigned)MOONHUT_TANK_LIVE_PERIOD_S);
        return reply;
    }
    if (strncasecmp(body, "live", 4) == 0 && body[4] == 0) {
        // Bare `tank:live` - the form someone actually types when in a hurry up a ladder.
        liveUntilMs = millis() + (uint32_t)MOONHUT_TANK_LIVE_DEFAULT_MIN * 60UL * 1000UL;
        nextLiveAtMs = millis();
        snprintf(reply, sizeof(reply), "tank: live %u min, every %u s. tank:live=0 to stop",
                 (unsigned)MOONHUT_TANK_LIVE_DEFAULT_MIN, (unsigned)MOONHUT_TANK_LIVE_PERIOD_S);
        return reply;
    }
    if (strncasecmp(body, "poll=", 5) == 0) {
        // Deliberately settable at runtime: the whole point is that a deployed node's
        // cadence can be retuned without a reflash, and this build ships fast ON PURPOSE
        // as the fleet's fast-response ranger testbed. See MOONHUT_TANK_POLL_S.
        const long v = atol(body + 5);
        if (v < MOONHUT_TANK_POLL_MIN_S || v > MOONHUT_TANK_POLL_MAX_S) {
            snprintf(reply, sizeof(reply), "tank: poll must be %u-%u s, got %ld", (unsigned)MOONHUT_TANK_POLL_MIN_S,
                     (unsigned)MOONHUT_TANK_POLL_MAX_S, v);
            return reply;
        }
        pollS = (uint16_t)v;
        saveCalibration();
        // Name the trade-off in the reply. This is set from a phone at a tank with no
        // source to hand, and "why did my node stop answering quickly" is the next
        // question if the number is chosen blind.
        snprintf(reply, sizeof(reply), "tank: poll=%u s (%u pings/burst). reports still floored at %u s",
                 (unsigned)pollS, (unsigned)MOONHUT_TANK_SAMPLES, (unsigned)activeMinReportS());
        return reply;
    }
    if (strncasecmp(body, "sensor=", 7) == 0) {
        const char *want = body + 7;
        const MoonTankSensor *sn = lookupSensor(want);
        if (!sn) {
            // Name every option rather than just refusing - the whole point of moving this
            // to runtime is that it gets set with the sensor in hand and no source to read.
            int off = snprintf(reply, sizeof(reply), "tank: unknown sensor '%s'. try:", want);
            for (uint8_t i = 0; i < MOONHUT_TANK_SENSOR_COUNT && off < (int)sizeof(reply) - 1; i++)
                off += snprintf(reply + off, sizeof(reply) - off, " %s", MOONHUT_TANK_SENSORS[i].name);
            return reply;
        }
        sensor = sn;
        saveCalibration();
        snprintf(reply, sizeof(reply), "tank: sensor=%s (%s) trig %u us, echo wait %u ms, %.0f m/s, valid %.2f-%.2f m",
                 sn->name, sn->desc, (unsigned)sn->trigUs, (unsigned)(sn->echoTimeoutUs / 1000), (double)sn->speedMs,
                 (double)sn->minValidM, (double)sn->maxValidM);
        return reply;
    }
    if (strncasecmp(body, "sensors", 7) == 0) {
        int off = snprintf(reply, sizeof(reply), "tank: sensors:");
        for (uint8_t i = 0; i < MOONHUT_TANK_SENSOR_COUNT && off < (int)sizeof(reply) - 1; i++)
            off += snprintf(reply + off, sizeof(reply) - off, " %s%s", MOONHUT_TANK_SENSORS[i].name,
                            &MOONHUT_TANK_SENSORS[i] == activeSensor() ? "*" : "");
        return reply;
    }
    if (strncasecmp(body, "clear", 5) == 0) {
        tankHeightM = 0.0f;
        tankOffsetM = 0.0f;
        saveCalibration();
        snprintf(reply, sizeof(reply), "tank: calibration cleared - showing distance only");
        return reply;
    }
    if (strncasecmp(body, "show", 4) == 0) {
        if (!isCalibrated()) {
            snprintf(reply, sizeof(reply),
                     "tank: UNCALIBRATED - set tank:height=<m>. d=%.3f m sensor=%s poll=%us floor=%.2f",
                     (double)lastM, activeSensor()->name, (unsigned)pollS, (double)activeFloorM());
            return reply;
        }
        snprintf(reply, sizeof(reply),
                 "tank: sensor=%s poll=%us floor=%.2f height=%.3f dead=%.3f usable=%.3f d=%.3f "
                 "level=%.3f %.0f%%",
                 activeSensor()->name, (unsigned)pollS, (double)activeFloorM(), (double)tankHeightM,
                 (double)tankOffsetM,
                 (double)(tankHeightM - tankOffsetM), (double)lastM, (double)levelM(), (double)levelPct());
        return reply;
    }
    snprintf(reply, sizeof(reply),
             "tank: unknown command. try height=<m>, offset=<m>, floor=<m>, live[=<min>], poll=<s>, "
             "report=<s>, minreport=<s>, delta=<m>, sensor=<name>, sensors, show, clear");
    return reply;
}

#endif // MOONHUT_TANK
