#include "MoonFridgeModule.h"

#ifdef MOONHUT_FRIDGE

#include "FSCommon.h" // littlefs: persist the ROM -> probe-name mapping
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

// ...and this often once probes ARE present. The bus used to be scanned only while
// numProbes == 0, which meant a probe added to a running node stayed invisible until
// the next reboot, and one that dropped out could never come back. Worse, a probe
// that happened not to answer during the single boot-time scan vanished with no log
// line and no "--" on the panel - it simply was not there. Slower than RESCAN_MS
// because a search costs bus time that the sampling cycle would rather have.
static constexpr uint32_t RESCAN_PRESENT_MS = 60 * 1000UL;

// Where the ROM -> name mapping lives on littlefs.
static const char *NAMES_PATH = "/fridgenames";

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
    loadNames();
    sensors.begin(); // re-runs the bus search

    // Build the new set first, then diff it against the old one. Rebuilding in place
    // would throw away every dwell timer on each rescan, and with a 15 minute dwell
    // the alarm would then never be able to arm.
    Probe found[MOONHUT_FRIDGE_MAX_PROBES];
    uint8_t n = 0;

    uint8_t count = sensors.getDeviceCount();
    for (uint8_t i = 0; i < count && n < MOONHUT_FRIDGE_MAX_PROBES; i++) {
        DeviceAddress addr;
        if (!sensors.getAddress(addr, i))
            continue;

        Probe p;
        memcpy(p.addr, addr, sizeof(DeviceAddress));

        // Carry state forward for a ROM we already knew.
        for (uint8_t j = 0; j < numProbes; j++) {
            if (memcmp(probes[j].addr, addr, sizeof(DeviceAddress)) == 0) {
                p = probes[j];
                break;
            }
        }
        found[n++] = p;
    }

    // Sort by ROM address so slot numbering is stable across reboots. Names are keyed
    // by ROM, so a probe keeps its name even when this reorders the slots.
    for (uint8_t i = 1; i < n; i++) {
        Probe key = found[i];
        int8_t j = i - 1;
        while (j >= 0 && memcmp(found[j].addr, key.addr, sizeof(DeviceAddress)) > 0) {
            found[j + 1] = found[j];
            j--;
        }
        found[j + 1] = key;
    }

    // Announce departures before we lose the old set...
    for (uint8_t i = 0; i < numProbes; i++) {
        bool stillHere = false;
        for (uint8_t j = 0; j < n; j++)
            if (memcmp(probes[i].addr, found[j].addr, sizeof(DeviceAddress)) == 0)
                stillHere = true;
        if (!stillHere)
            LOG_WARN("MoonFridge: probe %s LEFT the bus", probeName(i));
    }

    const uint8_t prevCount = numProbes;
    DeviceAddress prevAddrs[MOONHUT_FRIDGE_MAX_PROBES];
    for (uint8_t i = 0; i < prevCount; i++)
        memcpy(prevAddrs[i], probes[i].addr, sizeof(DeviceAddress));

    for (uint8_t i = 0; i < n; i++)
        probes[i] = found[i];
    numProbes = n;

    // ...and arrivals once the new set is live, so probeName() can name them.
    for (uint8_t i = 0; i < numProbes; i++) {
        bool isNew = true;
        for (uint8_t j = 0; j < prevCount; j++)
            if (memcmp(probes[i].addr, prevAddrs[j], sizeof(DeviceAddress)) == 0)
                isNew = false;

        if (!probes[i].configured) {
            // 12-bit: 0.0625 C steps. Worth the 750 ms on a sensor read every 30 s.
            sensors.setResolution(probes[i].addr, 12);
            probes[i].configured = true;
        }

        if (isNew) {
            const uint8_t *a = probes[i].addr;
            LOG_INFO("MoonFridge: probe %s JOINED, slot %u, rom %02x%02x%02x%02x%02x%02x%02x%02x", probeName(i), i + 1, a[0],
                     a[1], a[2], a[3], a[4], a[5], a[6], a[7]);
        }
    }

    if (numProbes != prevCount)
        LOG_INFO("MoonFridge: %u probe(s) on GPIO %d", numProbes, MOONHUT_ONEWIRE_PIN);

    if (numProbes != 0)
        return;

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
}

// --------------------------------------------------------------------------
// Probe names
//
// A name is stored against a probe's 64-bit ROM address, never against its slot.
// Slots are sorted by ROM and therefore renumber whenever a probe is added or
// removed - so a slot-keyed name would silently relabel "Fridge" as "Freezer" the
// first time someone hangs a second probe on the bus.
// --------------------------------------------------------------------------

int8_t MoonFridgeModule::nameSlotForRom(const DeviceAddress addr) const
{
    for (uint8_t i = 0; i < MOONHUT_FRIDGE_MAX_PROBES; i++)
        if (names[i].used && memcmp(names[i].addr, addr, sizeof(DeviceAddress)) == 0)
            return (int8_t)i;
    return -1;
}

const char *MoonFridgeModule::probeName(uint8_t idx) const
{
    if (idx >= numProbes)
        return probeLabel(idx);
    int8_t slot = nameSlotForRom(probes[idx].addr);
    return (slot >= 0 && names[slot].name[0]) ? names[slot].name : probeLabel(idx);
}

void MoonFridgeModule::loadNames()
{
    if (namesLoaded)
        return;
    namesLoaded = true;

    auto f = FSCom.open(NAMES_PATH, FILE_O_READ);
    if (!f)
        return; // nothing mapped yet - probes fall back to P1/P2/...

    // One "<16 hex rom>=<name>" per line.
    char buf[256] = {};
    size_t n = f.read((uint8_t *)buf, sizeof(buf) - 1);
    f.close();
    if (n == 0)
        return;
    buf[n] = 0;

    uint8_t slot = 0;
    for (char *line = strtok(buf, "\n"); line && slot < MOONHUT_FRIDGE_MAX_PROBES; line = strtok(nullptr, "\n")) {
        char *eq = strchr(line, '=');
        if (!eq || (eq - line) != 16)
            continue;
        *eq = 0;

        DeviceAddress addr;
        bool ok = true;
        for (uint8_t i = 0; i < sizeof(DeviceAddress); i++) {
            char pair[3] = {line[i * 2], line[i * 2 + 1], 0};
            char *endp = nullptr;
            long v = strtol(pair, &endp, 16);
            if (endp != pair + 2) {
                ok = false;
                break;
            }
            addr[i] = (uint8_t)v;
        }
        if (!ok)
            continue;

        memcpy(names[slot].addr, addr, sizeof(DeviceAddress));
        strncpy(names[slot].name, eq + 1, MOONHUT_FRIDGE_NAME_LEN - 1);
        names[slot].name[MOONHUT_FRIDGE_NAME_LEN - 1] = 0;
        names[slot].used = true;
        slot++;
    }
    LOG_INFO("MoonFridge: loaded %u probe name(s)", slot);
}

void MoonFridgeModule::saveNames()
{
    char buf[256];
    size_t len = 0;
    for (uint8_t i = 0; i < MOONHUT_FRIDGE_MAX_PROBES; i++) {
        if (!names[i].used || !names[i].name[0])
            continue;
        const uint8_t *a = names[i].addr;
        int w = snprintf(buf + len, sizeof(buf) - len, "%02x%02x%02x%02x%02x%02x%02x%02x=%s\n", a[0], a[1], a[2], a[3], a[4],
                         a[5], a[6], a[7], names[i].name);
        if (w <= 0 || (size_t)w >= sizeof(buf) - len)
            break;
        len += (size_t)w;
    }

    auto f = FSCom.open(NAMES_PATH, FILE_O_WRITE);
    if (!f) {
        LOG_ERROR("MoonFridge: could not write %s", NAMES_PATH);
        return;
    }
    if (len)
        f.write((const uint8_t *)buf, len);
    f.close();
}

bool MoonFridgeModule::setProbeName(uint8_t idx, const char *name)
{
    loadNames();
    if (idx >= numProbes)
        return false;

    int8_t slot = nameSlotForRom(probes[idx].addr);
    if (slot < 0) {
        for (uint8_t i = 0; i < MOONHUT_FRIDGE_MAX_PROBES && slot < 0; i++)
            if (!names[i].used)
                slot = (int8_t)i;
    }
    if (slot < 0)
        return false;

    memcpy(names[slot].addr, probes[idx].addr, sizeof(DeviceAddress));
    if (name && name[0]) {
        strncpy(names[slot].name, name, MOONHUT_FRIDGE_NAME_LEN - 1);
        names[slot].name[MOONHUT_FRIDGE_NAME_LEN - 1] = 0;
        names[slot].used = true;
    } else {
        names[slot].name[0] = 0;
        names[slot].used = false;
    }

    saveNames();
    shownCount = 0xFF; // force the panel to repaint with the new label
    LOG_INFO("MoonFridge: slot %u is now \"%s\"", idx + 1, probeName(idx));
    return true;
}

const char *MoonFridgeModule::handleNameCommand(const char *body)
{
    static char reply[96];
    loadNames();

    while (*body == ' ')
        body++;

    if (strncmp(body, "list", 4) == 0 || !*body) {
        int len = snprintf(reply, sizeof(reply), "%u probe(s):", numProbes);
        for (uint8_t i = 0; i < numProbes && len > 0 && (size_t)len < sizeof(reply); i++)
            len += snprintf(reply + len, sizeof(reply) - len, " %u=%s", i + 1, probeName(i));
        return reply;
    }

    if (strncmp(body, "clear", 5) == 0) {
        for (uint8_t i = 0; i < MOONHUT_FRIDGE_MAX_PROBES; i++) {
            names[i].used = false;
            names[i].name[0] = 0;
        }
        saveNames();
        shownCount = 0xFF;
        snprintf(reply, sizeof(reply), "all probe names cleared");
        return reply;
    }

    // "<slot>=<name>", slot being the 1-based number shown on the panel.
    char *eq = (char *)strchr(body, '=');
    if (!eq) {
        snprintf(reply, sizeof(reply), "use fridgename:<slot>=<name>, or list/clear");
        return reply;
    }

    long slot = strtol(body, nullptr, 10);
    if (slot < 1 || slot > numProbes) {
        snprintf(reply, sizeof(reply), "no probe in slot %ld (%u present)", slot, numProbes);
        return reply;
    }

    if (!setProbeName((uint8_t)(slot - 1), eq + 1)) {
        snprintf(reply, sizeof(reply), "could not name slot %ld", slot);
        return reply;
    }
    snprintf(reply, sizeof(reply), "slot %ld is now \"%s\"", slot, probeName((uint8_t)(slot - 1)));
    return reply;
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
            LOG_WARN("MoonFridge: %s bad read (%.1f C)", probeName(i), c);
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
                LOG_INFO("MoonFridge: %s above %.1f C (%.1f) - dwell started", probeName(i), (double)MOONHUT_FRIDGE_HIGH_C,
                         p.tempC);
            }
            if ((now - p.aboveSinceMs) >= (MOONHUT_FRIDGE_DWELL_S * 1000UL))
                anyLatched = true;
        } else if (p.tempC < clearAt) {
            if (p.aboveSinceMs != 0)
                LOG_INFO("MoonFridge: %s back below %.1f C (%.1f) - cleared", probeName(i), (double)clearAt, p.tempC);
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

    // Every probe gets a vote. Comparing only probe 0 (as this did) meant a second
    // probe's number was repainted solely when the FIRST one happened to drift past
    // the delta - so on a stable fridge it looked frozen for minutes at a time and
    // only caught up on a reboot or an unrelated refresh.
    bool changed = (alarm != shownAlarm) || (numProbes != shownCount);

    for (uint8_t i = 0; i < numProbes && !changed; i++) {
        float c = 0.0f;
        if (getTempC(i, c)) {
            if (isnan(shownC[i]) || fabsf(c - shownC[i]) >= REDRAW_DELTA_C)
                changed = true;
        } else if (!isnan(shownC[i])) {
            changed = true; // reading was lost - stop showing a stale number
        }
    }

    if (!changed)
        return;

    for (uint8_t i = 0; i < MOONHUT_FRIDGE_MAX_PROBES; i++) {
        float c = 0.0f;
        shownC[i] = (i < numProbes && getTempC(i, c)) ? c : NAN;
    }
    shownCount = numProbes;
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

    // Keep scanning even once probes are present, so adding or losing one no longer
    // needs a reboot. Only between sample cycles: a bus search during a conversion
    // would corrupt the reading being collected.
    if (phase == Phase::Convert && (int32_t)(now - nextEnumerateAt) >= 0) {
        enumerate();
        nextEnumerateAt = now + RESCAN_PRESENT_MS;
        if (numProbes == 0)
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
