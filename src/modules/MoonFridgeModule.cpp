#include "MoonFridgeModule.h"

#ifdef MOONHUT_FRIDGE

#include "FSCommon.h" // littlefs: the persistent probe roster
#include "MeshService.h"
#include "NodeDB.h"
#include "configuration.h"
#include "mesh/Channels.h"
#include "mesh/Router.h"
#include "mesh/mesh-pb-constants.h"
#include <string.h>

#if HAS_SCREEN
#include "graphics/Screen.h"
#endif

// Only repaint the e-ink when a reading actually moves. A panel refresh is slow and
// visibly flashes on every 11th update (EINK_LIMIT_FASTREFRESH=10), and a fridge
// drifts by hundredths of a degree - so repainting every sample would be both ugly
// and pointless.
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
// the next reboot, and one that dropped out could never come back.
static constexpr uint32_t RESCAN_PRESENT_MS = 30 * 1000UL;

// A probe must miss this many consecutive scans before it counts as gone. One missed
// scan is normal on a marginal joint and should not move anything on the panel.
static constexpr uint8_t MISSED_SCANS_TO_DROP = 2;

// Glitch rejection. A DS18B20 on a long or busy bus throws the occasional corrupted
// read - a single flipped bit in a timing slot. Those must not reach the panel, the
// alarm or the mesh, but they also must not be mistaken for a probe that has failed.
//
// So: retry once immediately, then hold the last good reading through a few more
// failures before admitting there is no reading. Only a probe that stays silent is
// actually reported as faulted.
static constexpr uint8_t BAD_READS_TO_INVALIDATE = 3;
static constexpr uint32_t HOLD_LAST_GOOD_MS = 60 * 1000UL;

// A corrupted read is often a PLAUSIBLE number - -127 is easy to spot, but a flipped
// bit can land at a perfectly reasonable-looking temperature. What gives it away is
// that it is impossible: a sealed probe in a thermal mass cannot move this fast.
// Generous enough never to reject a real change, tight enough to catch a bit-flip
// that lands tens of degrees away.
// 2.0 C/s was far too generous: it permits 120 C per MINUTE, so corrupted reads could
// walk from 25 C to 80 C in easy 12 C steps and every one of them passed the gate.
// Observed live - P2 read 79.9 C five seconds after 28.1 C and was back at 28.0 C
// sixteen seconds later, which no probe in a stainless sheath can physically do.
//
// But RATE ALONE CANNOT TELL corruption from a real fast change. A probe dipped in
// boiling water to identify it - which is exactly how this bench got labelled - really
// does go from 28 C to 80 C in five seconds: a thin stainless sheath has a time constant
// of seconds in water, whatever it does in air. Rejecting on rate alone would blind the
// panel during the one procedure where you most need to watch a number move.
//
// The honest discriminator is PERSISTENCE, not speed. A real change stays; corruption
// does not. So a jump past the slew gate is held as "pending" and accepted the moment
// the NEXT sample agrees with it - costing one sample of delay - while a lone excursion
// that snaps back is dropped and counted as a glitch.
static constexpr float MAX_SLEW_C_PER_S = 0.2f;
static constexpr float SPIKE_FLOOR_C = 2.0f;
static constexpr uint32_t SPIKE_WINDOW_MS = 60 * 1000UL;

// A present probe that has not produced a good reading for this long counts as faulted.
static constexpr uint32_t FAULT_AFTER_MS = 5 * 60 * 1000UL;

// Heartbeat report, and the temperature change that forces an early one.
static constexpr uint32_t REPORT_PERIOD_MS = 5 * 60 * 1000UL;
static constexpr float REPORT_DELTA_C = 0.5f;

// The full configuration goes out on a slower cycle, and immediately on any change.
// Names and bands are far too big to repeat every heartbeat once there are a dozen
// probes, but a listener that misses a change must still converge without asking.
static constexpr uint32_t CFG_REPORT_PERIOD_MS = 30 * 60 * 1000UL;

// An alarm is re-announced this often while it stays latched. Announcing once and
// trusting the radio is how a warm fridge goes unnoticed all night: we have already
// watched single packets go missing on this link.
static constexpr uint32_t ALARM_REPEAT_MS = 5 * 60 * 1000UL;

static constexpr uint32_t BEEP_PERIOD_MS = 3000;
static constexpr uint16_t BEEP_FREQ_HZ = 2400;
static constexpr uint32_t BEEP_LEN_MS = 250;

// Where the persistent probe roster lives, and the v1 name-only file it replaces.
static const char *PROBES_PATH = "/fridgeprobes";
static const char *LEGACY_NAMES_PATH = "/fridgenames";

// Reports BROADCAST by default (see sendLine for the measurement behind that). Setting
// "fridge:dest=!<hex>" overrides it with a directed DM to that node.
static constexpr NodeNum DEFAULT_REPORT_DEST = 0x8fa66864; // the fleet gateway, for reference

// Channel to report on.
#ifndef MOONHUT_FRIDGE_CHANNEL
#define MOONHUT_FRIDGE_CHANNEL "MoonFleet"
#endif

// Resolve it to a real index by POSITION, never by reading meshtastic_Channel::index.
// That field is only preinitialised for channels the firmware sets up itself
// (Channels.cpp: "ch.index = chIndex"); a channel added later over the CLI carries 0.
// Trusting it sent every fridge report and every command reply out on channel 0 - the
// default, unencrypted, public channel - instead of the fleet's private one.
static bool findFridgeChannel(ChannelIndex &out)
{
    for (ChannelIndex i = 0; i < channels.getNumChannels(); i++) {
        if (strcasecmp(channels.getGlobalId(i), MOONHUT_FRIDGE_CHANNEL) == 0) {
            out = i;
            return true;
        }
    }
    out = channels.getPrimaryIndex(); // unprovisioned node: better than nothing
    return false;
}

static ChannelIndex fridgeChannelIndex()
{
    ChannelIndex i = 0;
    findFridgeChannel(i);
    return i;
}

MoonFridgeModule::MoonFridgeModule()
    : concurrency::OSThread("MoonFridge"), wire(MOONHUT_ONEWIRE_PIN), sensors(&wire)
#ifdef MOONHUT_ONEWIRE_PIN2
      ,
      wire2(MOONHUT_ONEWIRE_PIN2), sensors2(&wire2)
#endif
{
    for (uint8_t b = 0; b < busCount(); b++) {
        busFor(b).begin();
        // Collect conversions on a later tick instead of blocking for 750 ms.
        busFor(b).setWaitForConversion(false);
    }
    // NOT loadProbes()/enumerate() here. Modules are constructed before littlefs is
    // reliably mounted, and a load that silently fails is catastrophic: enumerate()
    // then sees every probe as new, sets rosterGrew, and saveProbes() OVERWRITES the
    // stored roster - destroying every name and threshold on the node.
    //
    // That is not hypothetical; it ate the name "Bench2" and had to be diagnosed from
    // the gateway's message history. The first runOnce() happens well after init, so
    // the roster is loaded there instead.
}

uint8_t MoonFridgeModule::busCount()
{
#ifdef MOONHUT_ONEWIRE_PIN2
    return 2;
#else
    return 1;
#endif
}

uint8_t MoonFridgeModule::busPin(uint8_t bus)
{
#ifdef MOONHUT_ONEWIRE_PIN2
    if (bus == 1)
        return MOONHUT_ONEWIRE_PIN2;
#else
    (void)bus;
#endif
    return MOONHUT_ONEWIRE_PIN;
}

DallasTemperature &MoonFridgeModule::busFor(uint8_t bus)
{
#ifdef MOONHUT_ONEWIRE_PIN2
    if (bus == 1)
        return sensors2;
#else
    (void)bus;
#endif
    return sensors;
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

// True if this reading is physically impossible given how recently we had a good one,
// and therefore a bus glitch rather than a temperature. Never judges without a recent
// reference: with no valid previous reading, any value has to be taken at face value.
bool MoonFridgeModule::isSpike(const Probe &p, float c, uint32_t now)
{
    if (!p.valid || p.lastGoodMs == 0)
        return false;
    const uint32_t dtMs = now - p.lastGoodMs;
    if (dtMs > SPIKE_WINDOW_MS)
        return false; // too long since the reference to judge against it
    const float maxJump = (MAX_SLEW_C_PER_S * (dtMs / 1000.0f)) + SPIKE_FLOOR_C;
    return fabsf(c - p.tempC) > maxJump;
}

// --------------------------------------------------------------------------
// The roster
//
// Sticky and append-only. A probe that has ever been seen keeps its slot, its name,
// its thresholds and its screen frame, whether or not it is currently answering. Two
// things fall out of that, both deliberate:
//
//   * The panel stops reflowing. A probe that drops off reads "--" in place instead
//     of its whole column vanishing - which is what used to happen, and which said
//     nothing about WHICH probe had gone.
//   * Slot numbers are permanent. The old code sorted by ROM address, so adding a
//     probe could renumber the existing ones and silently relabel Fridge as Freezer.
//     Discovery order never changes, so "slot 2" means the same probe for good.
//
// Removal is only ever explicit, via "fridge:forget".
// --------------------------------------------------------------------------

int8_t MoonFridgeModule::findRom(const DeviceAddress addr) const
{
    for (uint8_t i = 0; i < numProbes; i++)
        if (memcmp(probes[i].addr, addr, sizeof(DeviceAddress)) == 0)
            return (int8_t)i;
    return -1;
}

void MoonFridgeModule::scanBus(uint8_t bus, bool *seen, bool &rosterGrew)
{
    DallasTemperature &dallas = busFor(bus);
    dallas.begin(); // re-runs the search on this bus

    uint8_t count = dallas.getDeviceCount();
    for (uint8_t i = 0; i < count; i++) {
        DeviceAddress addr;
        if (!dallas.getAddress(addr, i))
            continue;

        int8_t slot = findRom(addr);
        if (slot < 0) {
            if (numProbes >= MOONHUT_FRIDGE_MAX_PROBES) {
                LOG_WARN("MoonFridge: more than %u probes across all buses - ignoring the extra",
                         MOONHUT_FRIDGE_MAX_PROBES);
                continue;
            }
            slot = (int8_t)numProbes++;
            probes[slot] = Probe();
            memcpy(probes[slot].addr, addr, sizeof(DeviceAddress));
            probes[slot].bus = bus;
            rosterGrew = true;

            const uint8_t *a = probes[slot].addr;
            LOG_INFO("MoonFridge: probe %s JOINED, slot %u, GPIO %u, rom %02x%02x%02x%02x%02x%02x%02x%02x",
                     probeName(slot), slot + 1, busPin(bus), a[0], a[1], a[2], a[3], a[4], a[5], a[6], a[7]);
        }

        // A probe physically moved to the other bus keeps its slot, name and thresholds.
        probes[slot].bus = bus;
        seen[slot] = true;
        probes[slot].missedScans = 0;
        if (!probes[slot].present) {
            probes[slot].present = true;
            probes[slot].warmingUp = true; // its first read will be the 85 C power-on value
            LOG_INFO("MoonFridge: probe %s is back, on GPIO %u", probeName(slot), busPin(bus));
        }
        if (!probes[slot].configured) {
            // 12-bit: 0.0625 C steps. Worth the 750 ms on a sensor read.
            dallas.setResolution(probes[slot].addr, 12);
            probes[slot].configured = true;
        }
    }
}

void MoonFridgeModule::enumerate()
{
    bool seen[MOONHUT_FRIDGE_MAX_PROBES] = {};
    bool rosterGrew = false;

    for (uint8_t b = 0; b < busCount(); b++)
        scanBus(b, seen, rosterGrew);

    for (uint8_t i = 0; i < numProbes; i++) {
        if (seen[i] || !probes[i].present)
            continue;
        // One missed scan is normal on a marginal joint. Do not move anything on the
        // panel until it has missed several in a row.
        if (++probes[i].missedScans >= MISSED_SCANS_TO_DROP) {
            probes[i].present = false;
            probes[i].valid = false;
            probes[i].configured = false;
            LOG_WARN("MoonFridge: probe %s LEFT the bus", probeName(i));
        }
    }

    if (rosterGrew)
        saveProbes();

    // Unconditional, not just when the roster grew. A probe restored from littlefs is
    // not "new", so gating this on discovery meant a node that booted with a saved
    // roster never built the per-probe frames - the button advanced once and then had
    // nowhere left to go. rebuildFrames() is a no-op when the count already matches,
    // and retries later if the screen does not exist yet.
    rebuildFrames();

    if (numProbes != 0)
        return;

    for (uint8_t b = 0; b < busCount(); b++)
        LOG_WARN("MoonFridge: no DS18B20 found on GPIO %u - check the pullup and the data wire", busPin(b));

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
        Probe &p = probes[i];
        if (!p.present) {
            p.valid = false;
            continue;
        }

        DallasTemperature &dallas = busFor(p.bus);
        float c = dallas.getTempC(p.addr);
        if (!plausible(c)) {
            // One immediate retry. A -127 on a marginal bus is usually a single
            // corrupted slot, and the very next read comes back clean - which is far
            // cheaper than waiting a whole sample period to find out.
            c = dallas.getTempC(p.addr);
        }

        bool usable = plausible(c) && !isSpike(p, c, now);

        // A jump the slew gate rejected, confirmed by the very next sample, is a real
        // physical change - a probe moved, or dipped in hot water - not a bus glitch.
        if (!usable && plausible(c) && !isnan(p.pendingC) && fabsf(c - p.pendingC) <= SPIKE_FLOOR_C) {
            LOG_INFO("MoonFridge: %s confirmed a fast change to %.1f C (was %.1f)", probeName(i), c, p.tempC);
            usable = true;
        }

        if (usable) {
            p.tempC = c;
            p.valid = true;
            p.warmingUp = false;
            p.lastGoodMs = now;
            p.badReads = 0;
            p.pendingC = NAN;
            continue;
        }

        // A probe that has just joined has not finished its first conversion, so 85 C
        // is expected exactly once and is not worth counting against it.
        if (p.warmingUp && c == 85.0f)
            continue;

        // Remember it, so the next sample can confirm it as a real change.
        p.pendingC = plausible(c) ? c : NAN;
        p.badReads++;
        p.glitches++;
        if (p.valid && p.badReads < BAD_READS_TO_INVALIDATE && (now - p.lastGoodMs) < HOLD_LAST_GOOD_MS) {
            // Ride it out on the last good value. This is the whole point: a glitch
            // must not blank the panel or fire a fault for one bad slot.
            LOG_WARN("MoonFridge: %s glitch %.1f C ignored (%u of %u, holding %.1f)", probeName(i), c, p.badReads,
                     BAD_READS_TO_INVALIDATE, p.tempC);
            continue;
        }

        if (p.valid)
            LOG_WARN("MoonFridge: %s no usable reading after %u tries (last %.1f C)", probeName(i), p.badReads, c);
        p.valid = false;
    }
}

void MoonFridgeModule::evaluate(uint32_t now)
{
    uint32_t latched = 0;
    uint32_t faulted = 0;

    for (uint8_t i = 0; i < numProbes; i++) {
        Probe &p = probes[i];

        if (!p.valid) {
            // A failed read is not evidence of warmth - hold the timer rather than either
            // arming or clearing on missing data.
            if (p.aboveSinceMs != 0 && (now - p.aboveSinceMs) >= (p.dwellS * 1000UL))
                latched |= (1UL << i);
        } else {
            const bool tooWarm = p.tempC > p.hiC;
            const bool tooCold = p.tempC < p.loC;
            // Hysteresis applies inward from whichever limit was crossed.
            const bool backInBand = (p.tempC < (p.hiC - MOONHUT_FRIDGE_HYSTERESIS_C)) &&
                                    (p.tempC > (p.loC + MOONHUT_FRIDGE_HYSTERESIS_C));

            if (tooWarm || tooCold) {
                if (p.aboveSinceMs == 0) {
                    p.aboveSinceMs = now;
                    LOG_INFO("MoonFridge: %s %s (%.1f, band %.1f..%.1f) - dwell started", probeName(i),
                             tooWarm ? "too warm" : "too cold", p.tempC, (double)p.loC, (double)p.hiC);
                }
                if ((now - p.aboveSinceMs) >= (p.dwellS * 1000UL))
                    latched |= (1UL << i);
            } else if (backInBand) {
                if (p.aboveSinceMs != 0)
                    LOG_INFO("MoonFridge: %s back in band (%.1f) - cleared", probeName(i), p.tempC);
                p.aboveSinceMs = 0;
            }
            // Between the limits and their hysteresis margins: hold the current state.
        }

        // A silent probe is a different problem from a warm one, and the operator needs to
        // know which - so faults are tracked and announced separately from alarms.
        if (!p.present || (p.lastGoodMs != 0 && (now - p.lastGoodMs) > FAULT_AFTER_MS))
            faulted |= (1UL << i);
    }

    // Per-probe flags for the panel.
    for (uint8_t i = 0; i < numProbes; i++) {
        probes[i].alarmReported = (latched & (1UL << i)) != 0;
        probes[i].faultReported = (faulted & (1UL << i)) != 0;
    }

    // --- one message for the whole node, not one per probe ---------------------
    //
    // Six probes alarming used to mean six packets inside eight seconds, repeating every
    // five minutes; sixteen would mean sixteen. That crowds the node's own heartbeats off
    // the air - we watched exactly that happen - and it turns one event into six Telegram
    // notifications. One packet listing every affected probe is both better radio manners
    // and a better alert.
    const bool alarmSetChanged = (latched != alarmMask);
    const bool alarmRepeatDue = latched && (now - alarmMsgAt) >= ALARM_REPEAT_MS;
    if (alarmSetChanged || alarmRepeatDue) {
        char body[400];
        size_t len = 0;
        for (uint8_t i = 0; i < numProbes && len < sizeof(body) - 1; i++) {
            if (!(latched & (1UL << i)))
                continue;
            // "name=temp>limit" for too warm, "name=temp<limit" for too cold - so the
            // message says which way it broke without needing the config to interpret it.
            const bool warm = !probes[i].valid || probes[i].tempC > probes[i].hiC;
            int w = snprintf(body + len, sizeof(body) - len, "%s%s=%.1f%c%.1f", len ? "\x1f" : "", probeName(i),
                             probes[i].tempC, warm ? '>' : '<', (double)(warm ? probes[i].hiC : probes[i].loC));
            if (w <= 0)
                break;
            len += (size_t)w;
        }

        char prefix[32];
        if (latched) {
            snprintf(prefix, sizeof(prefix), "FRIDGE ALARM v%u%s", (unsigned)configEpoch, alarmSetChanged ? "" : " rpt");
            sendSegmented(prefix, body);
            if (alarmSetChanged)
                alarmMuted = false; // any NEW probe alarming re-arms the buzzer
        } else if (alarmMask) {
            snprintf(prefix, sizeof(prefix), "FRIDGE CLEAR v%u", (unsigned)configEpoch);
            sendLine(prefix);
        }
        alarmMask = latched;
        alarmMsgAt = now;
    }

    // Faults are coalesced the same way, and for the same reason: a whole bus dropping out
    // is precisely when a per-probe storm would be worst.
    const bool faultSetChanged = (faulted != faultMask);
    const bool faultRepeatDue = faulted && (now - faultMsgAt) >= ALARM_REPEAT_MS;
    if (faultSetChanged || faultRepeatDue) {
        char body[400];
        size_t len = 0;
        for (uint8_t i = 0; i < numProbes && len < sizeof(body) - 1; i++) {
            if (!(faulted & (1UL << i)))
                continue;
            int w = snprintf(body + len, sizeof(body) - len, "%s%s", len ? "\x1f" : "", probeName(i));
            if (w <= 0)
                break;
            len += (size_t)w;
        }

        char prefix[32];
        if (faulted) {
            snprintf(prefix, sizeof(prefix), "FRIDGE FAULT v%u%s", (unsigned)configEpoch, faultSetChanged ? "" : " rpt");
            sendSegmented(prefix, body);
        } else if (faultMask) {
            snprintf(prefix, sizeof(prefix), "FRIDGE OK v%u", (unsigned)configEpoch);
            sendLine(prefix);
        }
        faultMask = faulted;
        faultMsgAt = now;
    }

    const bool anyLatched = latched != 0;
    if (anyLatched != alarm) {
        alarm = anyLatched;
        LOG_WARN("MoonFridge: ALARM %s", alarm ? "ON" : "OFF");
        if (!alarm) {
            alarmMuted = false;
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

    // Never drive a pin that carries a probe bus. tone() on a 1-Wire line corrupts every
    // read on it for as long as the alarm sounds - and an alarm is exactly when you most
    // need the readings. This has already happened once: the default buzzer pin was 48,
    // which is also the obvious choice for a second bus.
    for (uint8_t b = 0; b < busCount(); b++) {
        if (pin && pin == busPin(b)) {
            static bool moaned = false;
            if (!moaned) {
                moaned = true;
                LOG_ERROR("MoonFridge: buzzer GPIO %u is the bus %u data pin - refusing to drive it. "
                          "Move the buzzer (device.buzzer_gpio) to a free pin.",
                          pin, b);
            }
            return 0;
        }
    }
    return pin;
}

// Commands change alarm thresholds, so accepting them from anywhere means anyone in RF
// range of the default public channel could raise a limit and silently disable the alarm
// on someone's freezer. The channel PSK is the shared secret that gates them - the same
// model Meshtastic uses for admin messages.
//
// If the private channel is not configured there is nothing to gate against, and refusing
// everything would strand a fresh node with no way to set it up, so that case stays open.
bool MoonFridgeModule::acceptsCommand(ChannelIndex ch, bool pkiEncrypted) const
{
    // A PKI-encrypted DM is authenticated against this node's own key pair - a stronger
    // claim than knowing a shared channel PSK, so it is always honoured. It has to be
    // special-cased because Router.cpp stamps channel 0 on a packet when it encrypts it
    // with Curve25519, so a PKI DM is indistinguishable from a primary-channel packet by
    // index alone. Gating on the index alone would have locked out the very path the API
    // is meant to use.
    if (pkiEncrypted)
        return true;

    ChannelIndex want = 0;
    if (!findFridgeChannel(want))
        return true;
    return ch == want;
}

void MoonFridgeModule::muteAlarm()
{
    if (!alarm || alarmMuted)
        return;
    alarmMuted = true;
    uint8_t pin = buzzerPin();
    if (pin)
        noTone(pin);
    LOG_INFO("MoonFridge: buzzer muted by hand - the alarm is still active and still reported");
}

void MoonFridgeModule::serviceBuzzer(uint32_t now)
{
    if (!alarm || alarmMuted)
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

// --------------------------------------------------------------------------
// Reporting
//
// This module sends its OWN packets rather than feeding EnvironmentTelemetryModule.
// That module disables itself on its first run when the I2C sensor list is empty,
// which a DS18B20 never joins - so the fork's hook there was never called and the
// temperature never left the node. Owning the send also sidesteps the 30-minute floor
// NodeDB imposes on environment_update_interval while a default channel exists, and is
// the only way to report more than the two probes that fit in environmentMetrics.
// --------------------------------------------------------------------------

void MoonFridgeModule::sendLine(const char *text)
{
    if (!router || !service)
        return;

    meshtastic_MeshPacket *p = router->allocForSending();
    if (!p)
        return;
    p->decoded.portnum = meshtastic_PortNum_TEXT_MESSAGE_APP;
    // BROADCAST, not a DM to the gateway. Measured on this link over 40 packets: 27
    // broadcasts arrived and the DMs were patchy - it explains every gap we chased,
    // the two lost command replies, the config-only stretch and the missing heartbeats.
    //
    // A DM needs a route ("Setting next hop for dest ... to 0"); a broadcast floods and
    // any relay carries it, so on a marginal link the flood wins. The tank node, which
    // broadcasts, has not missed a report from a worse antenna position.
    //
    // The cost is real and accepted: channel-PSK encryption instead of end-to-end PKI,
    // and a little more airtime. For a device whose whole job is to say when a fridge
    // has failed, arriving beats arriving privately - and MOONHUT_FRIDGE_CHANNEL is the
    // fleet's private channel, not the public one.
    //
    // "fridge:dest=" still forces a DM for anyone who wants one.
    p->to = reportDest ? reportDest : NODENUM_BROADCAST;
    p->channel = fridgeChannelIndex();
    p->want_ack = false;
    p->decoded.payload.size = snprintf((char *)p->decoded.payload.bytes, sizeof(p->decoded.payload.bytes), "%s", text);
    service->sendToMesh(p, RX_SRC_LOCAL, false);
    LOG_INFO("MoonFridge: sent %s", text);
}

void MoonFridgeModule::sendEnvironmentMetrics()
{
    if (!router || !service)
        return;

    // Only the first two readable probes fit the standard packet. Sent anyway so the
    // Pi's existing environmentMetrics ingestion and its Grafana history keep working
    // unchanged; the FRIDGE| line is what carries every probe, by name.
    int8_t first = -1, second = -1;
    for (uint8_t i = 0; i < numProbes; i++) {
        if (!probes[i].valid)
            continue;
        if (first < 0)
            first = (int8_t)i;
        else if (second < 0)
            second = (int8_t)i;
    }
    if (first < 0)
        return;

    meshtastic_Telemetry t = meshtastic_Telemetry_init_zero;
    t.which_variant = meshtastic_Telemetry_environment_metrics_tag;
    t.variant.environment_metrics.has_temperature = true;
    t.variant.environment_metrics.temperature = probes[first].tempC;
    if (second >= 0) {
        t.variant.environment_metrics.has_soil_temperature = true;
        t.variant.environment_metrics.soil_temperature = probes[second].tempC;
    }

    meshtastic_MeshPacket *p = router->allocForSending();
    if (!p)
        return;
    p->decoded.portnum = meshtastic_PortNum_TELEMETRY_APP;
    p->to = NODENUM_BROADCAST;
    p->channel = fridgeChannelIndex();
    p->want_ack = false;
    p->decoded.payload.size =
        pb_encode_to_bytes(p->decoded.payload.bytes, sizeof(p->decoded.payload.bytes), &meshtastic_Telemetry_msg, &t);
    service->sendToMesh(p, RX_SRC_LOCAL, false);
}

// Split a long body across as many packets as it needs. A LoRa text payload tops out
// around 230 bytes; sixteen probes' worth of names and bands does not fit in one, and
// silently truncating configuration would be worse than sending nothing.
void MoonFridgeModule::sendSegmented(const char *prefix, char *body)
{
    // A LoRa text payload tops out around 230 bytes; sixteen probes' worth of names and
    // bands does not fit, and silently truncating configuration would be worse than
    // sending nothing.
    //
    // Segments carry "i/n" after the prefix. Without it a continuation is indistinguishable
    // from a whole new report - same prefix, same epoch - and a parser would have to guess
    // with a timing heuristic. A single-segment message omits the marker entirely, so the
    // common case stays clean.
    static constexpr size_t SEG_MAX = 200;

    // Pass one: how many segments will this take?
    uint8_t total = 1;
    size_t used = strlen(prefix);
    const size_t headLen = used;
    for (const char *f = body; *f;) {
        const char *end = strchr(f, '\x1f');
        const size_t flen = end ? (size_t)(end - f) : strlen(f);
        if (used + flen + 1 >= SEG_MAX) {
            total++;
            used = headLen;
        }
        used += flen + 1;
        f = end ? end + 1 : f + flen;
    }

    // Pass two: emit them.
    char packet[240];
    uint8_t index = 1;
    int wrote = (total == 1) ? snprintf(packet, sizeof(packet), "%s", prefix)
                             : snprintf(packet, sizeof(packet), "%s %u/%u", prefix, index, total);
    used = (wrote > 0) ? (size_t)wrote : 0;
    size_t segHead = used;

    for (char *field = strtok(body, "\x1f"); field; field = strtok(nullptr, "\x1f")) {
        const size_t need = strlen(field) + 1;
        if (used + need >= SEG_MAX && used > segHead) {
            sendLine(packet);
            index++;
            wrote = snprintf(packet, sizeof(packet), "%s %u/%u", prefix, index, total);
            used = (wrote > 0) ? (size_t)wrote : 0;
            segHead = used;
        }
        int w = snprintf(packet + used, sizeof(packet) - used, "|%s", field);
        if (w > 0)
            used += (size_t)w;
    }
    if (used > segHead)
        sendLine(packet);
}

void MoonFridgeModule::sendConfigReport()
{
    if (numProbes == 0)
        return;

    // name=hi/lo/dwell/glitches. The glitch count rides along because a joint that is
    // slowly degrading shows up as a rising rate here long before it fails outright -
    // which the read filter would otherwise hide completely.
    char body[768];
    size_t len = 0;
    for (uint8_t i = 0; i < numProbes && len < sizeof(body) - 1; i++) {
        int w = snprintf(body + len, sizeof(body) - len, "%s%s=%.1f/%.1f/%u/%u", len ? "\x1f" : "", probeName(i),
                         (double)probes[i].hiC, (double)probes[i].loC, (unsigned)probes[i].dwellS,
                         (unsigned)probes[i].glitches);
        if (w <= 0)
            break;
        len += (size_t)w;
    }

    char prefix[24];
    snprintf(prefix, sizeof(prefix), "FRIDGECFG v%u", (unsigned)configEpoch);
    sendSegmented(prefix, body);
    nextCfgReportAt = millis() + CFG_REPORT_PERIOD_MS;
}

void MoonFridgeModule::bumpEpoch()
{
    configEpoch++;
    saveProbes();
    // Report the new state at once rather than waiting for the next heartbeat. This is
    // what makes a lost command reply harmless: the caller learns the change landed by
    // seeing the state, not by catching the answer.
    report(millis(), true);
    sendConfigReport();
}

void MoonFridgeModule::report(uint32_t now, bool force)
{
    if (!force && (int32_t)(now - nextReportAt) < 0) {
        // Not due - but a meaningful move reports early, so a fridge left open is not
        // sat on for five minutes.
        bool moved = false;
        for (uint8_t i = 0; i < numProbes && !moved; i++)
            if (probes[i].valid && (isnan(reportedC[i]) || fabsf(probes[i].tempC - reportedC[i]) >= REPORT_DELTA_C))
                moved = true;
        if (!moved)
            return;
    }

    // Every heartbeat is a complete statement of state, not a delta: name, reading, and
    // a suffix saying whether that probe is alarming (!) or has no reading (?). A
    // listener that missed every previous packet is fully caught up by this one.
    char body[768];
    size_t len = 0;
    for (uint8_t i = 0; i < numProbes && len < sizeof(body) - 1; i++) {
        char value[24];
        if (probes[i].valid)
            snprintf(value, sizeof(value), "%.1f%s", probes[i].tempC, probes[i].alarmReported ? "!" : "");
        else
            snprintf(value, sizeof(value), "?");
        int w = snprintf(body + len, sizeof(body) - len, "%s%s=%s", len ? "\x1f" : "", probeName(i), value);
        if (w <= 0)
            break;
        len += (size_t)w;
    }

    char prefix[24];
    snprintf(prefix, sizeof(prefix), "FRIDGE v%u", (unsigned)configEpoch);
    sendSegmented(prefix, body);
    sendEnvironmentMetrics();

    for (uint8_t i = 0; i < MOONHUT_FRIDGE_MAX_PROBES; i++)
        reportedC[i] = (i < numProbes && probes[i].valid) ? probes[i].tempC : NAN;
    nextReportAt = now + REPORT_PERIOD_MS;

    if ((int32_t)(now - nextCfgReportAt) >= 0)
        sendConfigReport();
}

// --------------------------------------------------------------------------
// Persistence
// --------------------------------------------------------------------------

void MoonFridgeModule::loadProbes()
{
    if (probesLoaded)
        return;

    char buf[1024] = {};
    size_t n = 0;

    auto f = FSCom.open(PROBES_PATH, FILE_O_READ);
    if (f) {
        n = f.read((uint8_t *)buf, sizeof(buf) - 1);
        f.close();
        probesLoaded = true;
    } else {
        // v1 migration: "<rom>=<name>" with no thresholds.
        auto old = FSCom.open(LEGACY_NAMES_PATH, FILE_O_READ);
        if (!old) {
            // Neither file opened. On a genuinely fresh node that is correct and saving
            // is safe; if the filesystem simply is not up yet it is NOT, and we cannot
            // tell the two apart from here. Assume mounted-and-empty only once the FS
            // has proven itself by listing its root.
            auto root = FSCom.open("/");
            if (root) {
                root.close();
                probesLoaded = true; // filesystem is alive, the file really is absent
            } else {
                LOG_WARN("MoonFridge: filesystem not ready - will retry the roster load");
            }
            return;
        }
        n = old.read((uint8_t *)buf, sizeof(buf) - 1);
        old.close();
        probesLoaded = true;
        LOG_INFO("MoonFridge: migrating %s to %s", LEGACY_NAMES_PATH, PROBES_PATH);
    }
    if (n == 0)
        return;
    buf[n] = 0;

    char *lineSave = nullptr;
    for (char *line = strtok_r(buf, "\n", &lineSave); line && numProbes < MOONHUT_FRIDGE_MAX_PROBES;
         line = strtok_r(nullptr, "\n", &lineSave)) {
        // "v<n>" header line: the config epoch, so a reboot does not make a listener
        // think the settings rolled back to the beginning.
        if (line[0] == 'v' || line[0] == 'V') {
            configEpoch = (uint32_t)strtoul(line + 1, nullptr, 10);
            continue;
        }
        if (strlen(line) < 17)
            continue;

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

        Probe p;
        memcpy(p.addr, addr, sizeof(DeviceAddress));

        // v2 is '|' separated; v1 used '=' and carried only a name.
        char *rest = line + 16;
        const char sep = *rest;
        rest++;

        // Split on '|' by hand. strtok_r cannot be used here: it treats a run of
        // delimiters as one, so an UNNAMED probe (an empty first field) silently shifts
        // every later field left - the roster came back with a probe literally named
        // "8.0", and its high limit parsed out of the low one.
        char *field[4] = {nullptr, nullptr, nullptr, nullptr};
        uint8_t nf = 0;
        field[nf++] = rest;
        for (char *c = rest; *c && nf < 4; c++) {
            if (*c == '|') {
                *c = 0;
                field[nf++] = c + 1;
            }
        }

        if (field[0] && field[0][0]) {
            strncpy(p.name, field[0], MOONHUT_FRIDGE_NAME_LEN - 1);
            p.name[MOONHUT_FRIDGE_NAME_LEN - 1] = 0;
        }
        if (sep == '|') { // v2 carries the alarm band; v1 was name-only
            if (field[1] && field[1][0])
                p.hiC = strtof(field[1], nullptr);
            if (field[2] && field[2][0])
                p.loC = strtof(field[2], nullptr);
            if (field[3] && field[3][0])
                p.dwellS = (uint32_t)strtoul(field[3], nullptr, 10);
        }
        probes[numProbes++] = p;
    }
    LOG_INFO("MoonFridge: roster restored, %u probe(s) known", numProbes);
}

void MoonFridgeModule::saveProbes()
{
    // Belt and braces for the failure above: never write over a roster we did not
    // manage to read. Losing the names silently is far worse than not persisting a
    // change - the change can be redone, the names cannot be recovered.
    if (!probesLoaded) {
        LOG_ERROR("MoonFridge: refusing to save - the roster was never loaded, and "
                  "overwriting it would destroy the stored names and thresholds");
        return;
    }

    char buf[1024];
    size_t len = 0;
    {
        int w = snprintf(buf, sizeof(buf), "v%u\n", (unsigned)configEpoch);
        if (w > 0)
            len = (size_t)w;
    }
    for (uint8_t i = 0; i < numProbes; i++) {
        const uint8_t *a = probes[i].addr;
        int w = snprintf(buf + len, sizeof(buf) - len, "%02x%02x%02x%02x%02x%02x%02x%02x|%s|%.1f|%.1f|%u\n", a[0], a[1],
                         a[2], a[3], a[4], a[5], a[6], a[7], probes[i].name, (double)probes[i].hiC,
                         (double)probes[i].loC, (unsigned)probes[i].dwellS);
        if (w <= 0 || (size_t)w >= sizeof(buf) - len)
            break;
        len += (size_t)w;
    }

    auto f = FSCom.open(PROBES_PATH, FILE_O_WRITE);
    if (!f) {
        LOG_ERROR("MoonFridge: could not write %s", PROBES_PATH);
        return;
    }
    f.write((const uint8_t *)buf, len);
    f.close();
}

// --------------------------------------------------------------------------
// Remote control
// --------------------------------------------------------------------------

int8_t MoonFridgeModule::resolveProbe(const char *token) const
{
    if (!token || !*token)
        return -1;

    // A bare number is a slot; anything else is matched against the names, so the API
    // can address "Freezer" without tracking slot numbers.
    char *endp = nullptr;
    long n = strtol(token, &endp, 10);
    if (endp && *endp == '\0' && n >= 1 && n <= (long)numProbes)
        return (int8_t)(n - 1);

    for (uint8_t i = 0; i < numProbes; i++)
        if (probes[i].name[0] && strcasecmp(probes[i].name, token) == 0)
            return (int8_t)i;
    return -1;
}

const char *MoonFridgeModule::handleCommand(const char *body)
{
    static char reply[200];
    loadProbes();

    while (*body == ' ')
        body++;

    char work[160];
    strncpy(work, body, sizeof(work) - 1);
    work[sizeof(work) - 1] = 0;

    char *save = nullptr;
    char *verb = strtok_r(work, " =", &save);
    if (!verb) {
        snprintf(reply, sizeof(reply), "usage: fridge:list | name N=X | hi N=C | lo N=C | dwell N=S | forget N");
        return reply;
    }

    if (strcasecmp(verb, "list") == 0) {
        int len = snprintf(reply, sizeof(reply), "v%u %u probe(s)", (unsigned)configEpoch, numProbes);
        for (uint8_t i = 0; i < numProbes && len > 0 && (size_t)len < sizeof(reply); i++) {
            char t[16];
            if (probes[i].valid)
                snprintf(t, sizeof(t), "%.1f", probes[i].tempC);
            else
                snprintf(t, sizeof(t), "%s", probes[i].present ? "?" : "gone");
            len += snprintf(reply + len, sizeof(reply) - len, " | %u=%s %s hi%.0f lo%.0f g%u", i + 1, probeName(i),
                            t, (double)probes[i].hiC, (double)probes[i].loC, (unsigned)probes[i].glitches);
        }
        return reply;
    }

    if (strcasecmp(verb, "dest") == 0) {
        const char *v = strchr(body, '=');
        if (v) {
            v++;
            if (*v == '!')
                v++;
            reportDest = (NodeNum)strtoul(v, nullptr, 16);
        }
        snprintf(reply, sizeof(reply), "reports go to !%08x", (unsigned)(reportDest ? reportDest : DEFAULT_REPORT_DEST));
        return reply;
    }

    char *arg = save;
    while (arg && *arg == ' ')
        arg++;
    if (!arg || !*arg) {
        snprintf(reply, sizeof(reply), "usage: fridge:%s <probe>=<value>", verb);
        return reply;
    }

    char *eq = strchr(arg, '=');
    if (!eq && strcasecmp(verb, "forget") != 0) {
        snprintf(reply, sizeof(reply), "missing '=' - try fridge:%s <probe>=<value>", verb);
        return reply;
    }
    const char *value = "";
    if (eq) {
        *eq = 0;
        value = eq + 1;
    }

    // Trim a trailing space off the probe token so "hi 2 = -15" works too.
    for (char *e = arg + strlen(arg); e > arg && e[-1] == ' '; e--)
        e[-1] = 0;

    int8_t idx = resolveProbe(arg);
    if (idx < 0) {
        snprintf(reply, sizeof(reply), "no probe '%s' (%u known)", arg, numProbes);
        return reply;
    }
    Probe &p = probes[idx];

    if (strcasecmp(verb, "name") == 0) {
        strncpy(p.name, value, MOONHUT_FRIDGE_NAME_LEN - 1);
        p.name[MOONHUT_FRIDGE_NAME_LEN - 1] = 0;
    } else if (strcasecmp(verb, "hi") == 0) {
        p.hiC = strtof(value, nullptr);
    } else if (strcasecmp(verb, "lo") == 0) {
        p.loC = strtof(value, nullptr);
    } else if (strcasecmp(verb, "dwell") == 0) {
        p.dwellS = (uint32_t)strtoul(value, nullptr, 10);
    } else if (strcasecmp(verb, "forget") == 0) {
        LOG_INFO("MoonFridge: forgetting probe %s", probeName(idx));
        for (uint8_t i = idx; i + 1 < numProbes; i++)
            probes[i] = probes[i + 1];
        numProbes--;
        shownCount = 0xFF;
        rebuildFrames();
        bumpEpoch();
        snprintf(reply, sizeof(reply), "forgotten - %u probe(s) left", numProbes);
        return reply;
    } else {
        snprintf(reply, sizeof(reply), "unknown command '%s'", verb);
        return reply;
    }

    // An inverted band would put the probe permanently outside it - a silent way to
    // turn a monitor into a device that screams forever, or never.
    if (p.hiC <= p.loC) {
        snprintf(reply, sizeof(reply), "rejected: hi %.1f must be above lo %.1f", (double)p.hiC, (double)p.loC);
        loadProbes();
        return reply;
    }

    p.aboveSinceMs = 0; // a changed band must re-serve its dwell
    shownCount = 0xFF;  // force a repaint with the new label/band
    bumpEpoch();
    snprintf(reply, sizeof(reply), "%u=%s hi%.1f lo%.1f dwell%us", idx + 1, probeName(idx), (double)p.hiC,
             (double)p.loC, (unsigned)p.dwellS);
    LOG_INFO("MoonFridge: %s", reply);
    return reply;
}

// --------------------------------------------------------------------------
// Display
// --------------------------------------------------------------------------

void MoonFridgeModule::rebuildFrames()
{
#if HAS_SCREEN
    if (!screen || framesBuiltFor == numProbes)
        return;
    framesBuiltFor = numProbes;
    // One overview frame plus one detail frame per probe.
    screen->setFrames(graphics::Screen::FOCUS_PRESERVE);
#endif
}

void MoonFridgeModule::maybeRefreshDisplay()
{
#if HAS_SCREEN
    if (!screen)
        return;

    // Every probe gets a vote. Comparing only probe 0 meant a second probe's number
    // was repainted solely when the FIRST one drifted past the delta, so on a stable
    // fridge it looked frozen and only caught up on a reboot.
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

bool MoonFridgeModule::probeBand(uint8_t idx, float &hiC, float &loC) const
{
    if (idx >= numProbes)
        return false;
    hiC = probes[idx].hiC;
    loC = probes[idx].loC;
    return true;
}

bool MoonFridgeModule::probeAlarming(uint8_t idx) const
{
    return idx < numProbes && probes[idx].alarmReported;
}

bool MoonFridgeModule::probeFault() const
{
    uint32_t now = millis();
    for (uint8_t i = 0; i < numProbes; i++) {
        if (!probes[i].present)
            return true;
        if (probes[i].lastGoodMs != 0 && (now - probes[i].lastGoodMs) > FAULT_AFTER_MS)
            return true;
    }
    return false;
}

const char *MoonFridgeModule::probeLabel(uint8_t idx)
{
    static const char *labels[] = {"P1", "P2",  "P3",  "P4",  "P5",  "P6",  "P7",  "P8",
                                   "P9", "P10", "P11", "P12", "P13", "P14", "P15", "P16"};
    return idx < (sizeof(labels) / sizeof(labels[0])) ? labels[idx] : "P?";
}

const char *MoonFridgeModule::probeName(uint8_t idx) const
{
    if (idx >= numProbes)
        return probeLabel(idx);
    return probes[idx].name[0] ? probes[idx].name : probeLabel(idx);
}

int32_t MoonFridgeModule::runOnce()
{
    uint32_t now = millis();

    if (!started) {
        started = true;
        loadProbes();
        enumerate();
    }

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
    }

    switch (phase) {
    case Phase::Convert:
        if ((int32_t)(now - nextSampleAt) >= 0) {
            for (uint8_t b = 0; b < busCount(); b++)
                busFor(b).requestTemperatures(); // non-blocking: setWaitForConversion(false)
            readReadyAt = now + CONVERSION_MS;
            phase = Phase::Read;
        }
        break;
    case Phase::Read:
        if ((int32_t)(now - readReadyAt) >= 0) {
            readAll();
            evaluate(millis());
            maybeRefreshDisplay();
            report(millis(), false);
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
    if (alarm && !alarmMuted) {
        int32_t beepDelay = (int32_t)(nextBeepAt - now);
        if (beepDelay < 0)
            beepDelay = 0;
        if (beepDelay < delay)
            delay = beepDelay;
    }
    return delay > 0 ? delay : 10;
}

#endif // MOONHUT_FRIDGE
