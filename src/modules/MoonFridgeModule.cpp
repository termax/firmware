#include "MoonFridgeModule.h"

#ifdef MOONHUT_FRIDGE

#include "NodeDB.h"
#include "configuration.h"
#include <string.h>

#if HAS_SCREEN
#include "graphics/Screen.h"
#endif

// Only repaint the e-ink when the reading actually moves. A panel refresh is slow,
// visibly flashes, and the variant caps consecutive fast-refreshes
// (EINK_LIMIT_FASTREFRESH=10) - so redrawing every 30 s sample would be both ugly
// and pointless when a fridge drifts by hundredths of a degree.
static constexpr float REDRAW_DELTA_C = 0.2f;

MoonFridgeModule *moonFridgeModule = nullptr;

// DS18B20 at 12-bit resolution needs 750 ms to convert. We never block for it -
// the conversion is kicked off, then collected on a later tick. Blocking here
// would stall the LoRa timing on the same core.
static constexpr uint32_t CONVERSION_MS = 800;

// Re-scan the bus this often while no probe is present, so a probe plugged in
// after boot is picked up without a reboot.
static constexpr uint32_t RESCAN_MS = 30 * 1000UL;

// A probe that has not produced a good reading for this long counts as faulted.
static constexpr uint32_t FAULT_AFTER_MS = 5 * 60 * 1000UL;

static constexpr uint32_t BEEP_PERIOD_MS = 3000;
static constexpr uint16_t BEEP_FREQ_HZ = 2400;
static constexpr uint32_t BEEP_LEN_MS = 250;

MoonFridgeModule::MoonFridgeModule()
    : concurrency::OSThread("MoonFridge"), wire(MOONHUT_ONEWIRE_PIN), sensors(&wire)
{
    sensors.begin();
    // Collect conversions on a later tick instead of blocking for 750 ms.
    sensors.setWaitForConversion(false);
    enumerate();
}

bool MoonFridgeModule::plausible(float c)
{
    // -127 is the library's disconnected sentinel. Exactly 85.0 is the DS18B20's
    // power-on reset value, returned when a probe is read before it has ever
    // completed a conversion - a classic false reading that would otherwise
    // trip the alarm the instant a connector goes marginal.
    if (isnan(c))
        return false;
    if (c <= -55.0f || c >= 125.0f)
        return false;
    if (c == 85.0f)
        return false;
    return true;
}

void MoonFridgeModule::enumerate()
{
    numProbes = 0;
    sensors.begin(); // re-runs the bus search

    uint8_t found = sensors.getDeviceCount();
    for (uint8_t i = 0; i < found && numProbes < MOONHUT_FRIDGE_MAX_PROBES; i++) {
        DeviceAddress addr;
        if (!sensors.getAddress(addr, i))
            continue;
        memcpy(probes[numProbes].addr, addr, sizeof(DeviceAddress));
        probes[numProbes].valid = false;
        probes[numProbes].tempC = NAN;
        probes[numProbes].lastGoodMs = 0;
        probes[numProbes].aboveSinceMs = 0;
        numProbes++;
    }

    // Sort by ROM address so probe numbering is STABLE across reboots. Bus search
    // order is address-ordered in practice, but relying on that would mean the
    // fridge and the freezer could silently swap identities after a power cut.
    for (uint8_t i = 1; i < numProbes; i++) {
        Probe key = probes[i];
        int8_t j = i - 1;
        while (j >= 0 && memcmp(probes[j].addr, key.addr, sizeof(DeviceAddress)) > 0) {
            probes[j + 1] = probes[j];
            j--;
        }
        probes[j + 1] = key;
    }

    for (uint8_t i = 0; i < numProbes; i++) {
        // 12-bit: 0.0625 C steps. Worth the 750 ms on a sensor read every 30 s.
        sensors.setResolution(probes[i].addr, 12);
        const uint8_t *a = probes[i].addr;
        LOG_INFO("MoonFridge: probe %s rom %02x%02x%02x%02x%02x%02x%02x%02x", probeLabel(i), a[0], a[1], a[2], a[3], a[4], a[5],
                 a[6], a[7]);
    }

    if (numProbes == 0) {
        LOG_WARN("MoonFridge: no DS18B20 found on GPIO %d - check the 4.7k pullup and the data wire", MOONHUT_ONEWIRE_PIN);

        // Raw presence-pulse test on the configured pin. This separates "nothing is
        // electrically alive on the bus" from "a device answers but its ROM will not
        // read" - two faults with completely different causes that otherwise look
        // identical from the enumerate() result.
        if (wire.reset())
            LOG_WARN("MoonFridge: GPIO %d DOES see a presence pulse - a device is alive but its ROM will not read "
                     "(marginal pullup, or a counterfeit part)",
                     MOONHUT_ONEWIRE_PIN);
        else
            LOG_WARN("MoonFridge: GPIO %d sees NO presence pulse - nothing is answering. Either DATA does not reach the "
                     "probe, the probe has no power, or the probe is dead. (VDD and a pulled-up DATA both measure 3.3 V, "
                     "so a meter cannot tell them apart - check with ohms, power off.)",
                     MOONHUT_ONEWIRE_PIN);

        scanCandidatePins();
    } else {
        LOG_INFO("MoonFridge: %u probe(s) on GPIO %d", numProbes, MOONHUT_ONEWIRE_PIN);
    }
}

// Diagnostic: when the configured pin finds nothing, try every other GPIO that is
// free on this board and report which one (if any) has a probe on it. Turns "is it
// the wiring or the wrong pin?" into one definitive answer instead of a re-wire and
// reflash per guess.
//
// The candidate list is deliberately restricted to pins verified unused by this
// variant. Never add 19/20 (USB CDC - the port we are talking over) or 26-37
// (SPI flash + octal PSRAM); driving those breaks the link or the boot.
void MoonFridgeModule::scanCandidatePins()
{
    static const uint8_t candidates[] = {15, 16, 17, 40, 41, 42, 47, 48};

    // Step 1: find where the external pullup ACTUALLY lands.
    //
    // With an internal pulldown enabled (~45k) a pin reads LOW unless something far
    // stronger pulls it up. The bus resistor is ~5k, so it wins easily. A pin that
    // reads HIGH here therefore has the resistor physically attached - which maps a
    // silkscreen label to a real GPIO without trusting the label, and distinguishes
    // "wired to the wrong pad" from "wired correctly but the probe is dead".
    LOG_INFO("MoonFridge: --- pullup scan (HIGH = external pullup on that pin) ---");
    bool anyPullup = false;
    for (uint8_t pin : candidates) {
        pinMode(pin, INPUT_PULLDOWN);
        delayMicroseconds(300);
        bool high = digitalRead(pin);
        pinMode(pin, INPUT);
        if (high) {
            anyPullup = true;
            LOG_WARN("MoonFridge: >>> GPIO %u HAS an external pullup - the bus resistor is on THIS pin", pin);
        } else {
            LOG_INFO("MoonFridge: GPIO %u floating - no external pullup", pin);
        }
    }
    if (!anyPullup)
        LOG_WARN("MoonFridge: NO external pullup on ANY free GPIO. The resistor is not electrically connected to a pin we "
                 "can use - so the pad you wired is NOT one of 15/16/17/40/41/42/47/48.");

    LOG_INFO("MoonFridge: scanning other free GPIOs for a 1-Wire device...");
    bool foundAny = false;
    for (uint8_t pin : candidates) {
        if (pin == MOONHUT_ONEWIRE_PIN)
            continue;

        OneWire probeBus(pin);
        // A OneWire reset returns 1 only if something on the bus pulls the line
        // low in the presence-detect slot - i.e. a device really is there.
        if (!probeBus.reset())
            continue;

        uint8_t addr[8];
        probeBus.reset_search();
        if (probeBus.search(addr)) {
            foundAny = true;
            LOG_WARN("MoonFridge: >>> 1-Wire device answered on GPIO %u (rom %02x%02x%02x%02x%02x%02x%02x%02x)", pin, addr[0],
                     addr[1], addr[2], addr[3], addr[4], addr[5], addr[6], addr[7]);
            LOG_WARN("MoonFridge: >>> rebuild with -D MOONHUT_ONEWIRE_PIN=%u", pin);
        } else {
            LOG_INFO("MoonFridge: GPIO %u shows presence but no ROM (marginal pullup?)", pin);
        }
    }
    if (!foundAny)
        LOG_WARN("MoonFridge: no 1-Wire device on ANY free GPIO - this is a wiring fault, not a pin choice");
}

void MoonFridgeModule::readAll()
{
    uint32_t now = millis();
    for (uint8_t i = 0; i < numProbes; i++) {
        float c = sensors.getTempC(probes[i].addr);
        if (plausible(c)) {
            probes[i].tempC = c;
            probes[i].valid = true;
            probes[i].lastGoodMs = now;
        } else {
            probes[i].valid = false;
            LOG_WARN("MoonFridge: %s bad read (%.1f C)", probeLabel(i), c);
        }
    }
}

void MoonFridgeModule::evaluate(uint32_t now)
{
    const float clearAt = MOONHUT_FRIDGE_HIGH_C - MOONHUT_FRIDGE_HYSTERESIS_C;
    bool anyLatched = false;

    for (uint8_t i = 0; i < numProbes; i++) {
        Probe &p = probes[i];
        if (!p.valid) {
            // A failed read is not evidence of warmth - hold the timer rather
            // than either arming or clearing on missing data.
            if (p.aboveSinceMs != 0 && (now - p.aboveSinceMs) >= (MOONHUT_FRIDGE_DWELL_S * 1000UL))
                anyLatched = true;
            continue;
        }

        if (p.tempC > MOONHUT_FRIDGE_HIGH_C) {
            if (p.aboveSinceMs == 0) {
                p.aboveSinceMs = now;
                LOG_INFO("MoonFridge: %s above %.1f C (%.1f) - dwell started", probeLabel(i), (double)MOONHUT_FRIDGE_HIGH_C,
                         p.tempC);
            }
            if ((now - p.aboveSinceMs) >= (MOONHUT_FRIDGE_DWELL_S * 1000UL))
                anyLatched = true;
        } else if (p.tempC < clearAt) {
            if (p.aboveSinceMs != 0)
                LOG_INFO("MoonFridge: %s back below %.1f C (%.1f) - cleared", probeLabel(i), (double)clearAt, p.tempC);
            p.aboveSinceMs = 0;
        }
        // Between clearAt and HIGH_C: hold current state (this is the hysteresis band).
    }

    if (anyLatched != alarm) {
        alarm = anyLatched;
        LOG_WARN("MoonFridge: ALARM %s", alarm ? "ON" : "OFF");
        if (!alarm) {
            uint8_t pin = buzzerPin();
            if (pin)
                noTone(pin);
        }
        nextBeepAt = 0;
    }
}

// Resolve the buzzer pin, or 0 if there isn't one. Never fall back to some other
// GPIO: an earlier version defaulted to MOONHUT_ONEWIRE_PIN here, which would have
// driven the sensor bus and corrupted probe reads.
uint8_t MoonFridgeModule::buzzerPin()
{
    uint8_t pin = (uint8_t)config.device.buzzer_gpio;
#ifdef PIN_BUZZER
    if (!pin)
        pin = PIN_BUZZER;
#endif
    return pin;
}

void MoonFridgeModule::serviceBuzzer(uint32_t now)
{
    if (!alarm)
        return;

    uint8_t pin = buzzerPin();
    if (!pin)
        return;

    if (nextBeepAt == 0 || (int32_t)(now - nextBeepAt) >= 0) {
        // tone() with a duration is fire-and-forget on ESP32 - it does not block.
        tone(pin, BEEP_FREQ_HZ, BEEP_LEN_MS);
        nextBeepAt = now + BEEP_PERIOD_MS;
    }
}

void MoonFridgeModule::maybeRefreshDisplay()
{
#if HAS_SCREEN
    if (!screen)
        return;

    float c = 0.0f;
    const bool haveReading = getTempC(0, c);

    bool changed = (alarm != shownAlarm);
    if (haveReading) {
        if (isnan(shownC) || fabsf(c - shownC) >= REDRAW_DELTA_C)
            changed = true;
    } else if (!isnan(shownC)) {
        changed = true; // reading was lost - stop showing a stale number
    }

    if (!changed)
        return;

    shownC = haveReading ? c : NAN;
    shownAlarm = alarm;
    screen->forceDisplay();
#endif
}

bool MoonFridgeModule::getTempC(uint8_t idx, float &outC) const
{
    if (idx >= numProbes || !probes[idx].valid)
        return false;
    outC = probes[idx].tempC;
    return true;
}

bool MoonFridgeModule::probeFault() const
{
    uint32_t now = millis();
    for (uint8_t i = 0; i < numProbes; i++) {
        if (probes[i].lastGoodMs != 0 && (now - probes[i].lastGoodMs) > FAULT_AFTER_MS)
            return true;
    }
    return false;
}

const char *MoonFridgeModule::probeLabel(uint8_t idx)
{
    static const char *labels[] = {"P1", "P2", "P3", "P4"};
    return idx < (sizeof(labels) / sizeof(labels[0])) ? labels[idx] : "P?";
}

int32_t MoonFridgeModule::runOnce()
{
    uint32_t now = millis();

    if (numProbes == 0) {
        if ((int32_t)(now - nextEnumerateAt) >= 0) {
            enumerate();
            nextEnumerateAt = now + RESCAN_MS;
        }
        return RESCAN_MS;
    }

    switch (phase) {
    case Phase::Convert:
        if ((int32_t)(now - nextSampleAt) >= 0) {
            sensors.requestTemperatures(); // non-blocking: setWaitForConversion(false)
            readReadyAt = now + CONVERSION_MS;
            phase = Phase::Read;
        }
        break;
    case Phase::Read:
        if ((int32_t)(now - readReadyAt) >= 0) {
            readAll();
            evaluate(millis());
            maybeRefreshDisplay();
            phase = Phase::Convert;
            nextSampleAt = millis() + (MOONHUT_FRIDGE_POLL_S * 1000UL);
        }
        break;
    }

    serviceBuzzer(now);

    // Sleep only as long as the nearest pending event allows.
    uint32_t target = (phase == Phase::Read) ? readReadyAt : nextSampleAt;
    int32_t delay = (int32_t)(target - now);
    if (delay < 0)
        delay = 0;
    if (alarm) {
        int32_t beepDelay = (int32_t)(nextBeepAt - now);
        if (beepDelay < 0)
            beepDelay = 0;
        if (beepDelay < delay)
            delay = beepDelay;
    }
    return delay > 0 ? delay : 10;
}

#endif // MOONHUT_FRIDGE
