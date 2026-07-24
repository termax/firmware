#include "configuration.h"
#ifdef MOONHUT_TRACKER
#include "MoonTrackModule.h"
#include "FSCommon.h"
#include "GPSStatus.h"
#include "MeshService.h"
#include "NodeDB.h"
#include "airtime.h"
#include "gps/GPS.h"
#include "gps/RTC.h"
#include <cmath>
#include <cstring>

MoonTrackModule *moonTrackModule = nullptr;

#define TRACK_PORT ((meshtastic_PortNum)260)
#define TRACK_LOG "/moontrack.log"
#define TRACK_STATE "/moontrack.st"
#define REC_SZ ((uint32_t)sizeof(Rec))
#define MIN_DIST_M 15.0
#define MIN_INTERVAL_S 20 // 2026-07-13: denser raw recording for map resolution
#define KEEPALIVE_S 600
#define GATEWAY_NODE 0x8fa66864 // gateway V4 (was WSMX 0x62ec2a74) — hearing home is what makes syncing possible
#define PRESENCE_WINDOW_S 600
#define PRESENCE_STABLE_MS 120000
#define CHUTIL_MAX 40.0f // 25 proved too shy on a chatty day; 1pkt/30s is the real limiter
#define RECS_PER_PKT 10 // 168B payload — 232B rode the frame-size edge and vanished on air
#define MAX_BATCH 512
#define CHUNK_GAP_MS 30000
#define ACK_TIMEOUT_MS 60000
#define MAX_RETRIES 2
#define LOG_COMPACT_BYTES (1024 * 1024)
#define DP_EPSILON_M 8.0 // 2026-07-13: 20m shed too much curve detail on rides; ~2x points, still ~4 packets/hour-of-trip
#define PARK_AFTER_MS (10 * 60 * 1000UL)  // no movement this long -> parked
#define PARK_JITTER_M 60.0                // stationary open-sky GPS scatter radius (2026-07-23: 30-75m
                                          // orbit outside beat MIN_DIST_M all night -> rode at 4.3%/h)
#define MOVING_WINDOW_MS (2 * 60 * 1000UL) // full 15m-resolution recording while the anchor moved this recently
#define FIRSTFIX_HUNT_MS (30 * 60 * 1000UL) // keep GPS hunting after boot until FIRST fix (battery cap)
// Walk-detection tuning (2026-07-12): two field walks never unparked — the 15-min
// peek + 120s fix window + 50m radius gauntlet loses to walking pace with the node
// in a pocket. Denser peeks + longer fix window + tighter radius; parked cost is
// ~3x peek current, affordable on a healthy cell. (Engine-sense divider on the
// scooter remains the definitive fix per docs/tracker.md.)
#define PEEK_EVERY_MS (5 * 60 * 1000UL)   // parked GPS peek cadence
#define PEEK_TIMEOUT_MS (180 * 1000UL)    // give up waiting for a fix
#define UNPARK_DIST_M 25.0                // moved this far from parking spot -> riding
#define UNPARK_CONFIRM_MS (20 * 1000UL)   // second fix this much later must agree

MoonTrackModule::MoonTrackModule()
    : SinglePortModule("moontrack", TRACK_PORT), concurrency::OSThread("MoonTrack")
{
    loadState();
    lastMoveMs = millis(); // arm the park timer at boot — 0 meant "never park until first move"
    LOG_INFO("MoonTrack: log=%u bytes, synced=%u", (unsigned)logSize(), (unsigned)synced);
}

// ---- storage -------------------------------------------------------------

uint32_t MoonTrackModule::logSize()
{
    auto f = FSCom.open(TRACK_LOG, FILE_O_READ);
    if (!f)
        return 0;
    uint32_t s = f.size();
    f.close();
    return s;
}

void MoonTrackModule::loadState()
{
    auto f = FSCom.open(TRACK_STATE, FILE_O_READ);
    if (f) {
        f.read((uint8_t *)&synced, sizeof(synced));
        f.close();
    }
    uint32_t sz = logSize();
    if (synced > sz)
        synced = sz; // state ahead of log (compaction or corruption) — clamp
}

void MoonTrackModule::saveState()
{
    auto f = FSCom.open(TRACK_STATE, FILE_O_WRITE);
    if (f) {
        f.write((uint8_t *)&synced, sizeof(synced));
        f.close();
    }
}

void MoonTrackModule::maybeRecord()
{
    if (!gpsStatus || !gpsStatus->getHasLock())
        return;
    int32_t lat = gpsStatus->getLatitude();
    int32_t lon = gpsStatus->getLongitude();
    uint32_t now = getValidTime(RTCQualityDevice, false) /* UTC — last_heard and record times must share the epoch */;
    if (now == 0)
        return; // no usable clock yet

    // First-fix quarantine: a cold GPS's first fix can land km off (phantom points
    // that Traccar draws straight lines through). Hold the first fix after boot;
    // record only once a second, independent fix agrees within 300 m. A stale
    // quarantine (>5 min old) restarts — the rogue never escapes.
    if (!firstFixVetted) {
        double pdLat = (lat - pendLat) * 1e-7 * 111320.0;
        double pdLon = (lon - pendLon) * 1e-7 * 111320.0 * cos(lat * 1e-7 * M_PI / 180.0);
        if (!pendMs || (millis() - pendMs) > 5 * 60 * 1000UL) {
            pendLat = lat; pendLon = lon; pendMs = millis();
            return;
        }
        if ((millis() - pendMs) < 10 * 1000UL)
            return; // same fix burst — wait for an independent sample
        if (sqrt(pdLat * pdLat + pdLon * pdLon) > 300.0) {
            LOG_INFO("MoonTrack: first-fix disagreement — discarding as rogue");
            pendLat = lat; pendLon = lon; pendMs = millis();
            return;
        }
        firstFixVetted = true; // two fixes agree — trust GPS from here on
    }

    // Equirectangular distance from the last recorded point
    double dLat = (lat - lastLat) * 1e-7 * 111320.0;
    double dLon = (lon - lastLon) * 1e-7 * 111320.0 * cos(lat * 1e-7 * M_PI / 180.0);
    double dist = sqrt(dLat * dLat + dLon * dLon);

    // Park-anchor jitter gate: real movement = escaping a PARK_JITTER_M circle
    // around a slow anchor; only that arms the park timer. Raw 15m record jumps
    // must NOT — a stationary receiver's scatter resets the timer forever and the
    // node never parks (first outdoor-parked night, 2026-07-23). Recording keeps
    // its full resolution while the anchor is in motion (MOVING_WINDOW_MS), so
    // ride detail is untouched; a stationary orbit degrades to keepalives only.
    double aLat = (lat - anchorLat) * 1e-7 * 111320.0;
    double aLon = (lon - anchorLon) * 1e-7 * 111320.0 * cos(lat * 1e-7 * M_PI / 180.0);
    bool realMove = sqrt(aLat * aLat + aLon * aLon) >= PARK_JITTER_M;
    if (realMove) {
        anchorLat = lat;
        anchorLon = lon;
        lastMoveMs = millis();
    }
    bool moving = realMove || (millis() - lastMoveMs) < MOVING_WINDOW_MS;
    bool moved = moving && dist >= MIN_DIST_M && (now - lastRecTime) >= MIN_INTERVAL_S;
    bool keepalive = (now - lastRecTime) >= KEEPALIVE_S;
    if (!moved && !keepalive)
        return;

    // flags bit0 = keepalive (stationary repeat) — lets the hub feed Traccar only
    // real movement instead of inferring it from exact coordinate repeats
    Rec r = {now, lat, lon, (int16_t)(gpsStatus->getAltitude()), 0, (uint8_t)(moved ? 0 : 1)};
    auto f = FSCom.open(TRACK_LOG, "a");
    if (f) {
        f.write((uint8_t *)&r, REC_SZ);
        f.close();
        lastLat = lat;
        lastLon = lon;
        lastRecTime = now;
        // lastMoveMs is armed only by the anchor gate above — never by raw records
    }
}

// ---- parked/riding power state machine ------------------------------------

void MoonTrackModule::toRiding()
{
    if (gps)
        gps->enable();
    mode = RIDING;
    lastMoveMs = millis();
    anchorLat = lastLat; // measure real movement from where we unparked; a false
    anchorLon = lastLon; // alarm stays inside the jitter circle and re-parks
    uneventfulPeeks = 0; // fresh situation — reset the peek backoff ladder
    LOG_INFO("MoonTrack: RIDING");
}

void MoonTrackModule::toParked()
{
    parkLat = lastLat;
    parkLon = lastLon;
    if (gps)
        gps->disable(); // FULL off. Softsleep reverted 2026-07-23: the module's STANDBY
                        // line is (evidently) unwired, so GPS_SOFTSLEEP held the module
                        // at acquisition power all night — measured 4.4%/h parked drain.
                        // Peek backoff + RF departure detection make cold peeks affordable.
    mode = PARKED;
    parkedCycleMs = millis();
    // RF snapshot for the peek-on-change assist
    const meshtastic_NodeInfoLite *gw = nodeDB ? nodeDB->getMeshNode(GATEWAY_NODE) : nullptr;
    parkGwValid = (gw != nullptr);
    parkGwSnr = gw ? gw->snr : 0;
    prevGwHeard = gatewayHeard();
    LOG_INFO("MoonTrack: PARKED (gwSnr %.1f)", parkGwSnr);
}

void MoonTrackModule::sendHeartbeat()
{
    // Theft canary: tiny port-260 beacon each parked peek; silence = jammed/gone
    meshtastic_MeshPacket *p = allocDataPacket();
    p->to = NODENUM_BROADCAST;
    p->decoded.payload.bytes[0] = 'T';
    p->decoded.payload.bytes[1] = 'H';
    p->decoded.payload.size = 2;
    service->sendToMesh(p, RX_SRC_LOCAL, false);
}

void MoonTrackModule::powerTick()
{
    if (!hadFirstFix && gpsStatus && gpsStatus->getHasLock()) {
        hadFirstFix = true;
        LOG_INFO("MoonTrack: first GPS fix since boot");
    }
    switch (mode) {
    case RIDING:
        // 2026-07-13: an indoor boot burned the whole RIDING window fixless, then moving
        // peeks (cold GPS + Doppler) failed all trip -> nothing recorded. Until the GPS
        // proves it can fix, keep hunting instead of parking (capped for battery).
        if (lastMoveMs && (millis() - lastMoveMs) > PARK_AFTER_MS) {
            if (hadFirstFix || millis() > FIRSTFIX_HUNT_MS)
                toParked();
        }
        break;
    case PARKED: {
        // Peek-on-RF-change: a parked node whose radio world shifts hard has almost
        // certainly been moved (carried home in a bag, 2026-07-19) — peek NOW instead
        // of waiting for the drumbeat. Triggers: gateway SNR far from the park
        // snapshot, or the gateway reappearing after being lost.
        bool rfShift = false;
        const meshtastic_NodeInfoLite *gw = nodeDB ? nodeDB->getMeshNode(GATEWAY_NODE) : nullptr;
        if (parkGwValid && gw && fabsf(gw->snr - parkGwSnr) >= 8.0f)
            rfShift = true;
        bool gwNow = gatewayHeard();
        if (gwNow && !prevGwHeard)
            rfShift = true; // home reappeared — world changed
        if (!gwNow && prevGwHeard) {
            // Departure hunt (2026-07-20, lost outbound legs to Nathon/Chaweng):
            // home's radio just went quiet while we're parked — almost certainly
            // driving away. Peeks can't fix from inside a moving car; go RIDING
            // so GPS hunts continuously and the outbound leg records from
            // mid-route instead of starting at the destination. Self-bounding:
            // fixless/stationary riding re-parks after PARK_AFTER_MS.
            prevGwHeard = gwNow;
            LOG_INFO("MoonTrack: home lost while parked -> departure hunt");
            toRiding();
            return;
        }
        prevGwHeard = gwNow;
        if (rfShift)
            LOG_INFO("MoonTrack: RF shift while parked -> early peek");
        // Adaptive backoff (2026-07-22, tracker died in <48h parked in a car):
        // fixless peeks under bad sky ran GPS 60% of parked time forever. After 3
        // consecutive lockless peeks, stretch the cadence — RF-based departure
        // detection now covers "we left", so blind peeking is redundant.
        uint32_t cadence = PEEK_EVERY_MS;                 // 5 min
        if (uneventfulPeeks >= 7) cadence = 60 * 60 * 1000UL;
        else if (uneventfulPeeks >= 5) cadence = 30 * 60 * 1000UL;
        else if (uneventfulPeeks >= 3) cadence = 15 * 60 * 1000UL;
        if (rfShift || (millis() - parkedCycleMs) > cadence) {
            if (rfShift) {
                if (gw)
                    parkGwSnr = gw->snr; // rebase so one shift = one extra peek, not a storm
                uneventfulPeeks = 0;        // world changed — earn the full retry ladder again
            }
            if (gps)
                gps->enable();
            mode = PEEKING;
            peekStartMs = millis();
        }
        break;
    }
    case PEEKING: {
        bool timeout = (millis() - peekStartMs) > PEEK_TIMEOUT_MS; // full window: cold starts need it (softsleep reverted)
        if (gpsStatus && gpsStatus->getHasLock()) {
            lastLockMs = millis();
            double dLat = (gpsStatus->getLatitude() - parkLat) * 1e-7 * 111320.0;
            double dLon = (gpsStatus->getLongitude() - parkLon) * 1e-7 * 111320.0 *
                          cos(parkLat * 1e-7 * M_PI / 180.0);
            // 2026-07-13: single drifty fixes faked 25-110 m "moves" while parked
            // (scatter bursts in Traccar). A real departure keeps the distance past
            // the threshold; drift settles back. Demand a second agreeing fix
            // UNPARK_CONFIRM_MS later before unparking.
            if (sqrt(dLat * dLat + dLon * dLon) > UNPARK_DIST_M) {
                if (!unparkFirstMs) {
                    unparkFirstMs = millis();
                } else if ((millis() - unparkFirstMs) >= UNPARK_CONFIRM_MS) {
                    unparkFirstMs = 0;
                    toRiding();
                    return;
                }
            } else {
                if (unparkFirstMs)
                    LOG_INFO("MoonTrack: unpark canceled (GPS drift)");
                unparkFirstMs = 0;
                timeout = true; // solid fix, still parked — wrap up the peek
            }
        }
        if (unparkFirstMs && (millis() - unparkFirstMs) < (UNPARK_CONFIRM_MS + 60000UL))
            timeout = false; // hold the peek open until the confirm resolves
        if (timeout) {
            // Verification night 2 (2026-07-24): ONLY fixless peeks climbed the
            // ladder, so an outdoor park (every peek locks, every lock is a cold
            // start at full acquisition power) peeked at 5 min forever = 5.6%/h,
            // worse than not parking. Any peek that ends still-parked is
            // uneventful — no lock and lock-at-park-spot climb the same ladder.
            // Departure coverage: RF hunt near home; away-parked accepts the
            // late-start tradeoff (same call as the 2026-07-22 backoff).
            if (uneventfulPeeks < 250)
                uneventfulPeeks++; // uneventful peek — climb the backoff ladder
            unparkFirstMs = 0;
            sendHeartbeat();
            if (gps)
                gps->disable(); // full off between peeks (softsleep reverted, see toParked)
            mode = PARKED;
            parkedCycleMs = millis();
        }
        break;
    }
    }
}

// ---- presence ------------------------------------------------------------

bool MoonTrackModule::gatewayHeard()
{
    // Own bookkeeping (gwSeenMs, set on any port-260 packet from the gateway) — the
    // nodedb's last_heard never updates for API-originated packets, which is ALL a
    // quiet serial-gateway ever sends. Discovered the hard way (lastHeard=0 forever).
    bool heard = gwSeenMs && (millis() - gwSeenMs) < (PRESENCE_WINDOW_S * 1000UL);
    if (!heard) {
        presenceSince = 0;
        return false;
    }
    if (!presenceSince)
        presenceSince = millis();
    return (millis() - presenceSince) >= PRESENCE_STABLE_MS; // hysteresis vs edge flapping
}

// ---- sync ----------------------------------------------------------------

void MoonTrackModule::loadBatch()
{
    uint32_t sz = logSize();
    uint32_t pending = sz > synced ? sz - synced : 0;
    uint32_t n = pending / REC_SZ;
    if (n == 0)
        return;
    if (n > MAX_BATCH)
        n = MAX_BATCH;

    std::vector<Rec> raw(n);
    auto f = FSCom.open(TRACK_LOG, FILE_O_READ);
    if (!f)
        return;
    f.seek(synced);
    f.read((uint8_t *)raw.data(), n * REC_SZ);
    f.close();
    batchEnd = synced + n * REC_SZ;

    // Douglas-Peucker (iterative, stack of index ranges), epsilon in meters
    std::vector<bool> keep(n, false);
    keep[0] = keep[n - 1] = true;
    std::vector<std::pair<int, int>> stack;
    stack.push_back({0, (int)n - 1});
    while (!stack.empty()) {
        auto [a, b] = stack.back();
        stack.pop_back();
        if (b - a < 2)
            continue;
        double ax = raw[a].lon * 1e-7, ay = raw[a].lat * 1e-7;
        double bx = raw[b].lon * 1e-7, by = raw[b].lat * 1e-7;
        double scale = 111320.0, cx = cos(ay * M_PI / 180.0);
        double vx = (bx - ax) * scale * cx, vy = (by - ay) * scale;
        double vlen = sqrt(vx * vx + vy * vy);
        double maxd = 0;
        int maxi = -1;
        for (int i = a + 1; i < b; i++) {
            double px = (raw[i].lon * 1e-7 - ax) * scale * cx, py = (raw[i].lat * 1e-7 - ay) * scale;
            double d = vlen < 1e-9 ? sqrt(px * px + py * py) : fabs(px * vy - py * vx) / vlen;
            if (d > maxd) {
                maxd = d;
                maxi = i;
            }
        }
        if (maxd > DP_EPSILON_M && maxi > 0) {
            keep[maxi] = true;
            stack.push_back({a, maxi});
            stack.push_back({maxi, b});
        }
    }
    batch.clear();
    for (uint32_t i = 0; i < n; i++)
        if (keep[i])
            batch.push_back(raw[i]);
    batchNext = 0;
    awaitingAck = false;
    retries = 0;
    LOG_INFO("MoonTrack: batch %u raw -> %u points (bytes %u..%u)", (unsigned)n, (unsigned)batch.size(),
             (unsigned)synced, (unsigned)batchEnd);
}

void MoonTrackModule::sendChunk()
{
    size_t remaining = batch.size() - batchNext;
    size_t count = remaining > RECS_PER_PKT ? RECS_PER_PKT : remaining;
    meshtastic_MeshPacket *p = allocDataPacket();
    p->to = NODENUM_BROADCAST;
    uint8_t *b = p->decoded.payload.bytes;
    b[0] = 'T';
    b[1] = 'K';
    b[2] = ++seqInFlight;
    b[3] = (uint8_t)count;
    // v2-ready header: origin nodeid — with gossip mules, sender != track owner.
    // For own data origin == us; a future mule fills in the cargo's true origin.
    uint32_t origin = nodeDB->getNodeNum();
    memcpy(b + 4, &origin, 4);
    memcpy(b + 8, &batch[batchNext], count * REC_SZ);
    p->decoded.payload.size = 8 + count * REC_SZ;
    service->sendToMesh(p, RX_SRC_LOCAL, false);
    awaitingAck = true;
    lastSendMs = millis();
    LOG_INFO("MoonTrack: chunk seq=%u count=%u sent", seqInFlight, (unsigned)count);
}

void MoonTrackModule::syncTick()
{
    if (batch.empty()) {
        // Active presence probe: a quiet gateway may not transmit for long stretches,
        // so passive listening starves. With a backlog and no recent contact, ASK —
        // meshhub answers 'TP' probes, and the answer updates last_heard (= presence).
        if (logSize() > synced && !gatewayHeard()) {
            static uint32_t lastProbeMs = 0;
            if (millis() - lastProbeMs > 120000) {
                lastProbeMs = millis();
                meshtastic_MeshPacket *p = allocDataPacket();
                p->to = GATEWAY_NODE;
                p->decoded.payload.bytes[0] = 'T';
                p->decoded.payload.bytes[1] = 'P';
                p->decoded.payload.size = 2;
                service->sendToMesh(p, RX_SRC_LOCAL, false);
                LOG_INFO("MoonTrack: probing gateway");
            }
        }
        if (logSize() > synced && gatewayHeard() && airTime) {
            if (airTime->channelUtilizationPercent() < CHUTIL_MAX) {
                loadBatch();
            } else {
                static uint32_t lastYellMs = 0;
                if (millis() - lastYellMs > 300000) { // yield visibly, not silently
                    lastYellMs = millis();
                    LOG_INFO("MoonTrack: sync yielding, chUtil=%.1f%%", airTime->channelUtilizationPercent());
                }
            }
        }
        return;
    }
    if (awaitingAck) {
        if (millis() - lastSendMs < ACK_TIMEOUT_MS)
            return;
        if (retries < MAX_RETRIES) {
            retries++;
            sendChunk(); // resend same window (batchNext unchanged)
        } else {
            LOG_WARN("MoonTrack: batch abandoned (no acks); will retry on next presence");
            batch.clear();
            presenceSince = 0;
        }
        return;
    }
    // Next chunk (rate-limited, utilization-yielding)
    if (millis() - lastSendMs < CHUNK_GAP_MS)
        return;
    if (airTime && airTime->channelUtilizationPercent() >= CHUTIL_MAX)
        return;
    if (batchNext >= batch.size()) {
        // Whole batch acked — advance and persist
        synced = batchEnd;
        saveState();
        batch.clear();
        // Compact once everything is synced and the log has grown
        if (synced >= logSize() && synced > LOG_COMPACT_BYTES) {
            FSCom.remove(TRACK_LOG);
            synced = 0;
            saveState();
            LOG_INFO("MoonTrack: log compacted");
        }
        return;
    }
    sendChunk();
}

ProcessMessage MoonTrackModule::handleReceived(const meshtastic_MeshPacket &mp)
{
    if (mp.from == GATEWAY_NODE)
        gwSeenMs = millis(); // presence proof: home spoke to us on our port
    auto &d = mp.decoded;
    // Remote wake ("TW" from home): one deliberate riding stint — GPS on until a real
    // fix, records + syncs, then re-parks. The couch-side unstick (PRG is wake-only).
    if (d.payload.size >= 2 && d.payload.bytes[0] == 'T' && d.payload.bytes[1] == 'W' && mp.from == GATEWAY_NODE) {
        LOG_INFO("MoonTrack: remote wake from home -> RIDING");
        toRiding();
        return ProcessMessage::STOP;
    }
    if (d.payload.size >= 3 && d.payload.bytes[0] == 'T' && d.payload.bytes[1] == 'A' &&
        d.payload.bytes[2] == seqInFlight && awaitingAck) {
        awaitingAck = false;
        retries = 0;
        batchNext += RECS_PER_PKT;
        if (batchNext > batch.size())
            batchNext = batch.size();
        LOG_INFO("MoonTrack: ack seq=%u", d.payload.bytes[2]);
    }
    return ProcessMessage::STOP;
}

int32_t MoonTrackModule::runOnce()
{
    maybeRecord();
    powerTick();
    syncTick();
    static uint32_t lastStateMs = 0;
    if (millis() - lastStateMs > 60000) { // heartbeat state line for bench debugging
        lastStateMs = millis();
        LOG_INFO("MoonTrack: state log=%u synced=%u gwHeard=%d mode=%d batch=%u lock=%d",
                 (unsigned)logSize(), (unsigned)synced, (int)gatewayHeard(), (int)mode,
                 (unsigned)batch.size(), gpsStatus ? (int)gpsStatus->getHasLock() : -1);
    }
    return 15 * 1000;
}
#endif
