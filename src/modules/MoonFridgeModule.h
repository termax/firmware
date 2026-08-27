#pragma once

#include "configuration.h"

#ifdef MOONHUT_FRIDGE

#include "concurrency/OSThread.h"
#include "mesh/MeshTypes.h"
#include "mesh/Channels.h"
#include <Arduino.h>
#include <DallasTemperature.h>
#include <OneWire.h>

// ---------------------------------------------------------------------------
// MoonHut fridge monitor: N DS18B20 probes on one 1-Wire bus + a local alarm.
//
// Every DS18B20 carries a factory-burned 64-bit ROM address, so all probes share
// a single data pin and are told apart by address. Adding probe #2 is therefore
// a wiring job, never a firmware or GPIO change - which is why this is written
// for N probes even though only one is fitted today.
// ---------------------------------------------------------------------------

// Data pin for the shared 1-Wire bus. Verified free on the E290 (see
// devices/vision-master-e290/README.md); override per-env in platformio.ini.
#ifndef MOONHUT_ONEWIRE_PIN
#define MOONHUT_ONEWIRE_PIN 47
#endif

// Optional SECOND 1-Wire bus. Splitting a room-sized install across two shorter buses
// beats one long one: 1-Wire degrades with total bus length and with stub legs, so two
// buses of four probes are far more reliable than one of eight.
// Undefine to go back to a single bus.
// Defined per-env in platformio.ini; leave undefined for a single-bus board.

#ifndef MOONHUT_FRIDGE_MAX_PROBES
#define MOONHUT_FRIDGE_MAX_PROBES 4
#endif

// Status LEDs, visible through the enclosure. Green means every probe is inside its
// band and answering; red means something needs a human. Undefine either to omit it.
//   solid green  - all well
//   solid red    - at least one probe is in alarm
//   blinking red - a probe has stopped answering (a different problem from a warm
//                  fridge, and one you would otherwise only notice on the panel)
#ifndef MOONHUT_LED_FAULT_BLINK_MS
#define MOONHUT_LED_FAULT_BLINK_MS 700
#endif

// Longest operator-assigned probe name, including the terminator. Kept short on
// purpose: it is drawn above the temperature in a column that is only a quarter of
// a 296 px panel wide when four probes are fitted.
#ifndef MOONHUT_FRIDGE_NAME_LEN
#define MOONHUT_FRIDGE_NAME_LEN 14
#endif

// DEFAULT alarm thresholds in degrees C, used by a probe that has not been given its
// own. A fridge at +4 and a freezer at -18 cannot share one limit, so every threshold
// here is only a starting point - the real ones are per probe, keyed by ROM, and are
// set remotely (see handleCommand).
#ifndef MOONHUT_FRIDGE_HIGH_C
#define MOONHUT_FRIDGE_HIGH_C 8.0f
#endif

// Too COLD is a real fault too: a fridge icing up, or a probe reading a freezer that
// has been left open onto the coils. Default is far enough below anything real that it
// never fires until someone sets it.
#ifndef MOONHUT_FRIDGE_LOW_C
#define MOONHUT_FRIDGE_LOW_C -40.0f
#endif

// How long the temperature must stay above the threshold before the buzzer
// sounds. A fridge legitimately passes +8 C on every defrost cycle and every
// time the door is held open; without a dwell the alarm would cry wolf several
// times a day and get ignored - which is the only real failure mode of an alarm.
#ifndef MOONHUT_FRIDGE_DWELL_S
#define MOONHUT_FRIDGE_DWELL_S 900 // 15 min
#endif

// How far BELOW the threshold it must fall to clear. Stops the alarm chattering
// on and off while the temperature sits exactly on the limit.
#ifndef MOONHUT_FRIDGE_HYSTERESIS_C
#define MOONHUT_FRIDGE_HYSTERESIS_C 1.5f
#endif

// Seconds between samples. A fridge has a thermal time constant in the tens of
// minutes; sampling faster only burns battery.
#ifndef MOONHUT_FRIDGE_POLL_S
#define MOONHUT_FRIDGE_POLL_S 30
#endif

class MoonFridgeModule : public concurrency::OSThread
{
  public:
    MoonFridgeModule();

    /// Number of probes found on the bus at the last enumeration.
    uint8_t probeCount() const { return numProbes; }

    /// Latest good reading for a probe. False if the probe is missing or its
    /// last read failed, in which case `outC` is untouched.
    bool getTempC(uint8_t idx, float &outC) const;

    /// True while the local over-temperature alarm is latched (threshold
    /// exceeded continuously for the dwell period).
    bool alarmActive() const { return alarm; }

    /// True when a probe was enumerated at boot but is no longer answering.
    /// Deliberately does NOT buzz - a silent node is what meshhub escalates on,
    /// and a screaming box because a connector wiggled is worse than useless.
    bool probeFault() const;

    /// Short fallback label for a probe: "P1", "P2"... Probes are sorted by ROM
    /// address so the numbering is stable across reboots.
    static const char *probeLabel(uint8_t idx);

    /// What to call this probe: the operator's name for it if one has been mapped,
    /// otherwise the positional label. This is what the panel and the logs use.
    const char *probeName(uint8_t idx) const;

    /// True if this probe answered the most recent bus scan.
    bool probePresent(uint8_t idx) const { return idx < numProbes && probes[idx].present; }

    /// Per-probe alarm band, for the display.
    bool probeBand(uint8_t idx, float &hiC, float &loC) const;

    /// True while THIS probe is outside its band and has served its dwell.
    bool probeAlarming(uint8_t idx) const;

    /// True if a "fridge:" command may be acted on. Commands change alarm thresholds, so
    /// they need either a PKI-authenticated DM or the private channel's PSK.
    bool acceptsCommand(ChannelIndex ch, bool pkiEncrypted) const;

    /// Handle a "fridge:" command body, e.g. "name 2=Freezer", "hi Freezer=-15",
    /// "list", "forget 2". Returns a short human-readable result for the reply.
    const char *handleCommand(const char *body);

    /// Silence a sounding buzzer until the next distinct alarm event. Bound to a long
    /// press; deliberately does NOT clear the alarm state or stop reporting it.
    void muteAlarm();

  protected:
    int32_t runOnce() override;

  private:
    enum class Phase { Convert, Read };

    // One entry per probe we have EVER seen. Deliberately sticky: a probe that misses a
    // bus scan keeps its slot, its name, its thresholds and its screen frame, and simply
    // reads as absent. Removing it on a missed scan is what made the panel reflow 2->1->2
    // and made a whole column vanish with nothing logged.
    struct Probe {
        DeviceAddress addr = {};
        char name[MOONHUT_FRIDGE_NAME_LEN] = {};
        float tempC = NAN;
        float hiC = MOONHUT_FRIDGE_HIGH_C;
        float loC = MOONHUT_FRIDGE_LOW_C;
        uint32_t dwellS = MOONHUT_FRIDGE_DWELL_S;
        bool valid = false;        // last read produced a usable temperature
        bool present = false;      // answered the most recent bus scan
        bool configured = false;   // 12-bit resolution has been pushed to this probe
        bool warmingUp = false;    // joined but has not completed its first conversion
        bool faultReported = false;
        bool alarmReported = false;
        uint8_t bus = 0;           // which 1-Wire bus this probe answers on
        uint8_t missedScans = 0;
        uint8_t badReads = 0;      // consecutive rejected readings
        uint32_t glitches = 0;     // lifetime rejected readings - a degrading joint shows
                                   // up here as a rising rate long before it fails
        float pendingC = NAN;      // a fast jump awaiting confirmation by the next sample
        uint32_t alarmSentMs = 0;  // last time this probe's alarm was announced
        uint32_t lastGoodMs = 0;
        uint32_t aboveSinceMs = 0; // 0 = not currently outside its band
    };

    void enumerate();
    void scanBus(uint8_t bus, bool *seen, bool &rosterGrew);
    void loadProbes();
    void saveProbes();
    int8_t findRom(const DeviceAddress addr) const;
    int8_t resolveProbe(const char *token) const; // slot number or name
    void scanCandidatePins();
    void report(uint32_t now, bool force);
    void sendConfigReport();
    void sendSegmented(const char *prefix, char *body);
    void bumpEpoch();
    void sendLine(const char *text);
    void sendEnvironmentMetrics();
    void rebuildFrames();
    void readAll();
    void evaluate(uint32_t now);
    void serviceBuzzer(uint32_t now);
    void serviceLeds(uint32_t now);
    void serviceButton(uint32_t now);
    void testBeep();
    void maybeRefreshDisplay();
    static uint8_t buzzerPin();
    static bool plausible(float c);
    static bool isSpike(const Probe &p, float c, uint32_t now);

    OneWire wire;
    DallasTemperature sensors;
#ifdef MOONHUT_ONEWIRE_PIN2
    OneWire wire2;
    DallasTemperature sensors2;
#endif

    /// The Dallas driver for a bus index, and how many buses are fitted.
    DallasTemperature &busFor(uint8_t bus);
    static uint8_t busCount();
    static uint8_t busPin(uint8_t bus);

    Probe probes[MOONHUT_FRIDGE_MAX_PROBES];
    uint8_t numProbes = 0;

    Phase phase = Phase::Convert;
    uint32_t nextSampleAt = 0;
    uint32_t readReadyAt = 0;
    uint32_t nextEnumerateAt = 0;

    bool alarm = false;
    uint32_t nextBeepAt = 0;

    // What the panel is currently displaying, per probe. Tracking only probe 0 here
    // was a real bug: the renderer draws every probe, so a screen that only repaints
    // when probe 0 moves leaves every other probe's number visibly stale.
    float shownC[MOONHUT_FRIDGE_MAX_PROBES] = {};
    uint8_t shownCount = 0xFF; // 0xFF = nothing painted yet
    bool shownAlarm = false;

    bool probesLoaded = false;
    bool started = false;   // first runOnce() does the load; the constructor is too early
    bool alarmMuted = false;
    bool btnInit = false;
    bool btnDown = false;
    bool btnLongSent = false;
    uint32_t testBeepAt = 0;       // when the second pulse of a test beep is due
    uint32_t btnChangedAt = 0;
    bool ledsInit = false;
    bool ledBlinkOn = false;
    uint32_t ledBlinkAt = 0;
    uint8_t framesBuiltFor = 0xFF; // roster size the frameset was last built for
    uint32_t nextReportAt = 0;
    uint32_t nextCfgReportAt = 0;
    // Which probes are currently latched / faulted, as bit masks. Messaging is driven off
    // these rather than per probe: one packet describing the whole node beats one packet
    // per probe, which at sixteen probes would be a sixteen-packet burst every repeat.
    uint32_t alarmMask = 0;
    uint32_t faultMask = 0;
    // Whether THIS boot has ever announced its fault/alarm state. Without these, a reboot
    // silently orphans whatever the mesh was last told - see the note in evaluate().
    bool faultAnnounced = false;
    bool alarmAnnounced = false;
    uint32_t alarmMsgAt = 0;
    uint32_t faultMsgAt = 0;
    // Bumped on every configuration change and carried in every heartbeat, so a listener
    // can tell at a glance whether the settings it believes in are the ones in force -
    // without needing to have caught the reply that changed them.
    uint32_t configEpoch = 0;
    float reportedC[MOONHUT_FRIDGE_MAX_PROBES] = {};
    NodeNum reportDest = 0; // 0 = the default gateway
};

extern MoonFridgeModule *moonFridgeModule;

#endif // MOONHUT_FRIDGE
