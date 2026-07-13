#pragma once
#ifdef MOONHUT_TRACKER
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

    // Parked/riding power state machine (P5): parked = GPS off + periodic peek.
    // Light sleep comes from provisioning (is_power_saving=true, role CLIENT).
    enum PowerMode { RIDING, PARKED, PEEKING };
    PowerMode mode = RIDING;
    bool hadFirstFix = false; // no parking before the GPS proves it can fix (2026-07-13)
    uint32_t lastMoveMs = 0;
    uint32_t parkedCycleMs = 0;
    uint32_t peekStartMs = 0;
    uint32_t unparkFirstMs = 0; // first beyond-threshold peek fix; a 2nd must confirm
    int32_t parkLat = 0, parkLon = 0;
    void powerTick();
    void toRiding();
    void toParked();
    void sendHeartbeat();

  public:
    void forceRiding(); // PRG button: deterministic "recording now" (2026-07-13)
};

extern MoonTrackModule *moonTrackModule;
#endif
