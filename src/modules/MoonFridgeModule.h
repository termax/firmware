#pragma once

#include "configuration.h"

#ifdef MOONHUT_FRIDGE

#include "concurrency/OSThread.h"
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

#ifndef MOONHUT_FRIDGE_MAX_PROBES
#define MOONHUT_FRIDGE_MAX_PROBES 4
#endif

// Longest operator-assigned probe name, including the terminator. Kept short on
// purpose: it is drawn above the temperature in a column that is only a quarter of
// a 296 px panel wide when four probes are fitted.
#ifndef MOONHUT_FRIDGE_NAME_LEN
#define MOONHUT_FRIDGE_NAME_LEN 14
#endif

// Alarm threshold in degrees C.
#ifndef MOONHUT_FRIDGE_HIGH_C
#define MOONHUT_FRIDGE_HIGH_C 8.0f
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

    /// Map a name onto the probe currently in slot `idx` (0-based). The mapping is
    /// stored against that probe's ROM address, not its slot, so it follows the
    /// physical probe even when adding another one renumbers the slots. Persisted to
    /// littlefs; survives reboots and reflashes. Empty name clears the mapping.
    bool setProbeName(uint8_t idx, const char *name);

    /// Handle a "fridgename:" command body, e.g. "2=Freezer" or "clear".
    /// Returns a short human-readable result for the reply/log.
    const char *handleNameCommand(const char *body);

  protected:
    int32_t runOnce() override;

  private:
    enum class Phase { Convert, Read };

    struct Probe {
        DeviceAddress addr = {};
        float tempC = NAN;
        bool valid = false;
        bool configured = false;   // 12-bit resolution has been pushed to this probe
        uint32_t lastGoodMs = 0;
        uint32_t aboveSinceMs = 0; // 0 = not currently above the threshold
    };

    void enumerate();
    void loadNames();
    void saveNames();
    int8_t nameSlotForRom(const DeviceAddress addr) const;
    void scanCandidatePins();
    void readAll();
    void evaluate(uint32_t now);
    void serviceBuzzer(uint32_t now);
    void maybeRefreshDisplay();
    static uint8_t buzzerPin();
    static bool plausible(float c);

    OneWire wire;
    DallasTemperature sensors;

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

    // ROM -> operator-assigned name. Kept separate from `probes` because a name must
    // outlive the probe being unplugged, and must not move when the slots renumber.
    struct NameMap {
        DeviceAddress addr = {};
        char name[MOONHUT_FRIDGE_NAME_LEN] = {};
        bool used = false;
    };
    NameMap names[MOONHUT_FRIDGE_MAX_PROBES];
    bool namesLoaded = false;
};

extern MoonFridgeModule *moonFridgeModule;

#endif // MOONHUT_FRIDGE
