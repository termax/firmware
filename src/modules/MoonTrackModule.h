#pragma once
#ifdef MOONHUT_TRACKER
#include "Observer.h"
#include "SinglePortModule.h"
#include "concurrency/OSThread.h"
#include <vector>

/**
 * MoonHut tracker: records the GPS track to littlefs while out of mesh range and
 * backfills the gap (Douglas-Peucker-decimated, chunked, acked) once the home
 * gateway is heard again. Live POSITION broadcasts are untouched — this covers
 * only what stock Meshtastic loses. Spec: mashtastic repo, docs/tracker.md.
 *
 * Storage (power-cut safe): append-only /moontrack.log (16B records) + tiny
 * /moontrack.st state file holding the synced offset (rewritten atomically).
 *
 * Wire protocol (PortNum 260, over the private channel):
 *   tracker -> home: 'T','K', uint8 seq, uint8 count, count x 16-byte records
 *   home -> tracker: 'T','A', uint8 seq   (ack, sent by meshhub)
 */
class MoonTrackModule : public SinglePortModule, private concurrency::OSThread
{
  public:
    MoonTrackModule();

  protected:
    virtual ProcessMessage handleReceived(const meshtastic_MeshPacket &mp) override;
    virtual int32_t runOnce() override;

  private:
    struct __attribute__((packed)) Rec {
        uint32_t time;
        int32_t lat; // deg * 1e7
        int32_t lon;
        int16_t alt;
        uint8_t speed; // reserved (0)
        uint8_t flags; // reserved (0)
    };

    void maybeRecord();
    bool gatewayHeard();
    void syncTick();
    void sendChunk();
    void loadBatch();

    uint32_t logSize();
    void loadState();
    void saveState();

    uint32_t synced = 0;      // byte offset of first unsynced record in the log
    int32_t lastLat = 0, lastLon = 0;
    int32_t anchorLat = 0, anchorLon = 0; // park-jitter anchor (real-movement gate)
    uint32_t lastRecTime = 0;

    // In-flight sync batch (decimated points covering log bytes synced..batchEnd)
    std::vector<Rec> batch;
    uint32_t batchEnd = 0;
    size_t batchNext = 0;    // next batch index to send
    uint8_t seqInFlight = 0;
    bool awaitingAck = false;
    uint32_t lastSendMs = 0;
    int retries = 0;

    uint32_t presenceSince = 0; // millis when gateway first heard in current streak
    uint32_t gwSeenMs = 0;      // own bookkeeping: nodedb lastHeard ignores API-originated
                                // packets, so we track gateway contact ourselves (port 260)
    uint32_t probeGapMs = 2 * 60 * 1000UL; // current gap between unanswered gateway probes

    // Parked/riding power state machine (P5): parked = GPS off + periodic peek.
    // Light sleep comes from provisioning (is_power_saving=true, role CLIENT).
    // RESERVE (2026-09-22): battery at/below RESERVE_PCT with no external power -> GPS off, no
    // peeks, light sleep allowed. Left only by external power (which is also the only way the
    // cell can charge). Keeps the cell from being run flat by tracking.
    enum PowerMode { RIDING, PARKED, PEEKING, RESERVE };
    PowerMode mode = RIDING;
    bool hadFirstFix = false; // no parking before the GPS proves it can fix (2026-07-13)
    uint32_t lastMoveMs = 0;
    uint32_t parkedCycleMs = 0;
    uint32_t peekStartMs = 0;
    uint32_t unparkFirstMs = 0; // first beyond-threshold peek fix; a 2nd must confirm
    int32_t parkLat = 0, parkLon = 0;
    uint8_t uneventfulPeeks = 0;   // consecutive parked peeks that ended still-parked (no lock,
                                   // or lock at the park spot) -> cadence backoff (2026-07-24)
    uint32_t lastLockMs = 0;    // last GPS lock (warm-peek window sizing)
    int32_t pendLat = 0, pendLon = 0; // first-fix quarantine (rogue cold-fix guard)
    uint32_t pendMs = 0;
    bool firstFixVetted = false;
    float parkGwSnr = 0;        // gateway SNR snapshot at park time (RF-change unpark assist)
    bool parkGwValid = false;   // snapshot taken (gateway was in NodeDB at park)
    bool prevGwHeard = false;   // presence edge detection while parked
    void powerTick();
    void toRiding();
    void toParked();
    void toReserve();
    bool homeNear();
    int preflightSleepCb(void *unused); // veto light sleep while RIDING/PEEKING (battery only matters)
    CallbackObserver<MoonTrackModule, void *> preflightSleepObserver =
        CallbackObserver<MoonTrackModule, void *>(this, &MoonTrackModule::preflightSleepCb);
    uint8_t lowBattTicks = 0; // consecutive ticks at/below RESERVE_PCT on battery (TX sag guard)
    // External power on a Heltec V4 (2026-09-24, the Udon ride that never recorded): the board
    // has no VBUS sense, so the core's getHasUSB() is just "cell above 4.20 V" - true only with a
    // FULL cell, false for the whole of a charge. A charger is recognised instead by the cell
    // CLIMBING: median of the newest 4 ticks vs the 4 ticks EXT_TREND_WINDOW earlier (10 min).
    static const uint8_t MV_HIST_N = 44;   // 11 min of 15 s ticks
    int16_t mvHist[MV_HIST_N] = {0};
    uint8_t mvHead = 0, mvCount = 0;
    bool extByTrend = false;               // set on a rise, cleared once the cell stops climbing
    uint32_t loadChangeMs = 0;             // last GPS on/off; the trend ignores the step transient
    void trendTick();
    bool externalPower();                  // getHasUSB() || extByTrend
    void noteLoadStep() { loadChangeMs = millis(); }
    void sendHeartbeat();
};

extern MoonTrackModule *moonTrackModule;
#endif
