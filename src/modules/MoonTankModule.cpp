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
    LOG_INFO("MoonTank: HC-SR04 on TRIG %d / ECHO %d", MOONHUT_TANK_TRIG_PIN, MOONHUT_TANK_ECHO_PIN);
}

float MoonTankModule::pingOnce()
{
    // 10 us trigger, per the datasheet. The 2 us LOW first guarantees a clean edge
    // even if something left the line high.
    digitalWrite(MOONHUT_TANK_TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(MOONHUT_TANK_TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(MOONHUT_TANK_TRIG_PIN, LOW);

    const uint32_t us = pulseIn(MOONHUT_TANK_ECHO_PIN, HIGH, ECHO_TIMEOUT_US);
    if (us == 0)
        return NAN; // no echo inside the window

    // Out and back, so half the flight time.
    const float m = (us * 1e-6f * SPEED_OF_SOUND_MS) / 2.0f;
    if (m < MIN_VALID_M || m > MAX_VALID_M)
        return NAN;
    return m;
}

void MoonTankModule::measure()
{
    float s[MOONHUT_TANK_SAMPLES];
    uint8_t n = 0;

    for (uint8_t i = 0; i < MOONHUT_TANK_SAMPLES; i++) {
        const float m = pingOnce();
        if (!isnan(m))
            s[n++] = m;
        // The datasheet asks for >60 ms between pings so the previous burst has
        // died away; less and you measure your own echo coming back off the tank.
        delay(60);
    }

    bursts++;
    if (n == 0) {
        timeouts++;
        lastM = NAN;
        lastSpreadM = NAN;
        lastValid = 0;
        LOG_WARN("MoonTank: no echo (%u of %u bursts have failed)", (unsigned)timeouts, (unsigned)bursts);
        if (!diagnosed && timeouts >= 3) {
            diagnosed = true; // once per boot - it is noisy and the answer does not change
            diagnose();
        }
        return;
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

    const float median = s[n / 2];   // median, not mean: one wild echo must not move it
    const float spread = s[n - 1] - s[0];
    lastSpreadM = spread;
    lastValid = n;

    // Two gates, both learned the hard way on the fridge bus: a reading nobody
    // cross-checked, and a reading whose samples disagree, are both worse than no
    // reading at all - because they look exactly as confident as a good one.
    reject = nullptr;
    if (n < MOONHUT_TANK_MIN_ECHOES)
        reject = "too few echoes";
    else if (spread > MOONHUT_TANK_MAX_SPREAD_M)
        reject = "samples disagree";

    if (reject) {
        lastM = NAN;
        LOG_WARN("MoonTank: REJECTED %.3f m - %s (spread %.3f m, %u/%u echoes)", median, reject, spread, n,
                 MOONHUT_TANK_SAMPLES);
        return;
    }

    lastM = median;

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
    if (screen)
        screen->forceDisplay();
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
void MoonTankModule::diagnose()
{
    static const uint8_t candidates[] = {15, 16, 17, 40, 41, 42, 47, 48};

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
        if (pin == MOONHUT_TANK_TRIG_PIN)
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
        if (pin != MOONHUT_TANK_TRIG_PIN)
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

void MoonTankModule::report(bool force)
{
    const uint32_t now = millis();
    const bool due = (int32_t)(now - nextReportAt) >= 0;
    const bool moved = !isnan(lastM) && (isnan(reportedM) || fabsf(lastM - reportedM) >= MOONHUT_TANK_REPORT_DELTA_M);
    if (!force && !due && !moved)
        return;

    char line[128];
    if (isnan(lastM))
        snprintf(line, sizeof(line), "TANK|d=?|e=%u/%u|why=%s", lastValid, MOONHUT_TANK_SAMPLES,
                 reject ? reject : "no echo");
    else
        snprintf(line, sizeof(line), "TANK|d=%.3f|sp=%.3f|e=%u/%u|min=%.3f|max=%.3f", lastM, lastSpreadM, lastValid,
                 MOONHUT_TANK_SAMPLES, sessionMinM, sessionMaxM);
    sendLine(line);

    reportedM = lastM;
    nextReportAt = now + (MOONHUT_TANK_REPORT_S * 1000UL);
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

    // Decide by BATTERY PRESENCE, not getHasUSB(): that returns false on this board even
    // on mains, which blanked a panel that was supposed to stay lit. getHasBattery() is
    // the signal that actually works here - Meshtastic reports batteryLevel 101 when
    // there is no battery, which is precisely the "externally powered" case.
    //
    // Deliberately biased: anything uncertain counts as external power. Failing to sleep
    // on battery costs some charge; failing to stay lit on mains defeats the whole device.
    const bool onBattery = powerStatus && powerStatus->getHasBattery() && !powerStatus->getHasUSB();
    const bool onUsb = !onBattery;

    if (onUsb != wasOnUsb) {
        wasOnUsb = onUsb;
        LOG_INFO("MoonTank: power source is now %s - screen %s", onUsb ? "external" : "battery",
                 onUsb ? "stays on" : "will sleep and refresh hourly");
    }

    if (onUsb) {
        // Mains: never blank. If it went dark on battery earlier, bring it back.
        if (screenOffSince) {
            screen->setOn(true);
            screenOffSince = 0;
            screenOnSince = now;
        }
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
    report(false);
    serviceScreen(millis());
    return MOONHUT_TANK_POLL_S * 1000;
}

#endif // MOONHUT_TANK
