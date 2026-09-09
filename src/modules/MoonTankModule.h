#pragma once

#include "configuration.h"

#ifdef MOONHUT_TANK

#include "concurrency/OSThread.h"
#include <Arduino.h>

// --------------------------------------------------------------------------
// MoonHut tank level: an HC-SR04P ultrasonic ranger pointed down at a water
// surface. Level = tank depth - measured distance.
//
// This is deliberately a PROOF OF CONCEPT and knows nothing about tanks yet: it
// measures distance, filters it, and shows it. The open question it exists to
// answer is whether an HC-SR04 can reach the bottom of a 3 m tank at all -
// nominal range is 4 m but usable range on a weak return (a distant, rippled
// water surface) is closer to 2-3 m, and it degrades exactly when the tank is
// nearly empty and the reading matters most.
//
// The JSN-SR04T waterproof unit uses an identical trigger/echo interface, so
// everything here carries over to it unchanged.
// --------------------------------------------------------------------------

#ifndef MOONHUT_TANK_TRIG_PIN
#define MOONHUT_TANK_TRIG_PIN 17
#endif
#ifndef MOONHUT_TANK_ECHO_PIN
#define MOONHUT_TANK_ECHO_PIN 16
#endif

// --- Second ranger, for the A/B bench ------------------------------------
//
// Optional: define BOTH pins to fit a second trigger/echo sensor alongside the first
// and carry its reading in the same report, so two sensors can be compared on the same
// board, the same water and the same minute.
//
// The second sensor is deliberately PASSIVE. It never feeds the level-rate fit, the
// panel, the fast-drain alert or the report-triggering logic - it is a measurement
// channel for deciding which sensor to keep, not a second source of truth. When the
// winner is known, swap the primary pin defines to it and drop these.
//
// The JSN-SR04T goes on 47/48 because that is the pair the final V4 node has free, so
// the firmware carries over to that board unchanged.
#if defined(MOONHUT_TANK_TRIG_PIN2) && defined(MOONHUT_TANK_ECHO_PIN2)
#define MOONHUT_TANK_DUAL 1
#endif

// Silence between the two bursts. MUST stay above ECHO_TIMEOUT_US so the first sensor's
// ping is fully dead before the second speaks.
//
// Overlap does not present as noise, which is what makes it dangerous: each sensor hears
// the OTHER's burst and reports the flight time to it - a short, stable, entirely
// plausible distance that looks exactly like a real reading of a full tank.
#ifndef MOONHUT_TANK_INTERLEAVE_MS
#define MOONHUT_TANK_INTERLEAVE_MS 60
#endif

// Width of the trigger pulse, microseconds.
//
// 50, not the 10 every HC-SR04 example uses. MEASURED on a JSN-SR04T with the sweep
// build on 2026-09-05: 10 us NEVER produced an echo (0/5 on every attempt), 20 us was
// marginal (one burst rejected at 0.706 m spread, the next clean), and 30 us and above
// were solid 5/5 at 2.116 m with a spread of 0-9 mm.
//
// The JSN has its own MCU sampling that line and simply ignores a pulse it does not
// latch - no error, no echo, indistinguishable from a dead sensor or a wiring fault.
// That cost most of an evening: the sensor was alive and correctly wired the whole time.
//
// 50 satisfies the HC-SR04 datasheet too (it asks for 10 us MINIMUM), so one value
// drives both parts, and it is nothing against the settling gap below.
#ifndef MOONHUT_TANK_TRIG_US
#define MOONHUT_TANK_TRIG_US 50
#endif

// Quiet time after each ping so the previous burst has died away.
#ifndef MOONHUT_TANK_PING_GAP_MS
#define MOONHUT_TANK_PING_GAP_MS 60
#endif

// --- Sensor profiles -----------------------------------------------------
//
// The tested sensors are all trigger/echo parts, but they are NOT interchangeable:
// they disagree on how long to wait for the echo, how fast they think sound travels,
// and how close they can see. Getting one of those wrong does not produce a slightly
// wrong reading - it produces NO reading, which looks exactly like a dead sensor or a
// wiring fault, and that is where the evenings go.
//
// This was compile-time (-D flags) until 2026-09-06. It is runtime now because the
// wrong choice is only discovered with the sensor in hand, and a rebuild-and-reflash
// at the tank is the worst possible place to find out. Set it when provisioning:
//
//     tank:sensor=a02        (over MoonFleet or a PKI DM, like height= and offset=)
//
// The choice persists in the same littlefs file as the calibration.
struct MoonTankSensor {
    const char *name;
    uint16_t trigUs;         // trigger pulse width
    uint32_t echoTimeoutUs;  // pulseIn() bound: covers the WAIT for the pulse plus its length
    float speedMs;           // speed of sound this part assumes
    float minValidM;         // below this the reading is the part's own dead zone
    float maxValidM;
    uint32_t deadPulseUs;    // 0 = none; a fixed width some parts emit to mean "no target"
    uint16_t pingGapMs;      // quiet time the part needs between triggers to answer again
    bool trigActiveLow;      // true = line IDLES HIGH and the trigger is a dip to LOW
    uint16_t echoBlankUs;    // deafen this long after the trigger; 0 = listen immediately
    // RINGDOWN band: the fixed, sub-floor width this part returns when the target is
    // deep inside its near field (the transducer hears its own burst). A burst of NOTHING
    // BUT sub-floor pings no shorter than ringLoM is positive evidence the sensor is
    // alive and the surface is close - that is what tells a FULL tank from a DEAD sensor.
    // Only ringLoM gates (between ringHiM and the floor the part returns REAL echoes of a
    // surface that is still inside the dead zone - measured, see judgeBurst); ringHiM is
    // descriptive. 0,0 = unmeasured on this part, and an unmeasured part can never claim
    // FULL. See MOONHUT_TANK_RUNT_M.
    float ringLoM;
    float ringHiM;
    // NOISE-TRIP band: the fixed distance this part reports when the real echo is LOST
    // (receiver noise crossing the threshold at one point on the gain ramp). MEASURED on the
    // JSN 2026-09-08 with the transducer physically unplugged: a tight 0.729 m, 5/5, in 35 %
    // of pings, plus scatter beyond it - and the SAME 0.73 m on both units, on four days,
    // whenever splash or bad aim killed the echo. A surface genuinely at 0.73 m reads clean
    // burst after burst; the artifact never does. So a reading inside this band is only
    // accepted after MOONHUT_TANK_NOISE_CLEAN_N clean bursts. 0,0 = none known.
    float noiseLoM;
    float noiseHiM;
    const char *desc;
};

extern const MoonTankSensor MOONHUT_TANK_SENSORS[];
extern const uint8_t MOONHUT_TANK_SENSOR_COUNT;

// Which profile a fresh node starts on, before anything is provisioned.
#ifndef MOONHUT_TANK_SENSOR_DEFAULT
#define MOONHUT_TANK_SENSOR_DEFAULT "jsn"
#endif

// DIAGNOSTIC: cycle the trigger width across bursts and log which one the sensor
// answers. One flash instead of four, and it names the working width outright.
#ifdef MOONHUT_TANK_TRIG_SWEEP
#define MOONHUT_TANK_TRIG_WIDTHS {10, 20, 30, 50, 100, 200}
#endif

// Seconds between measurements - the COMPILE-TIME DEFAULT only. The live value is
// `pollS`, settable over the air with `tank:poll=<s>` and persisted, so a deployed
// node's cadence can be changed without a reflash. See MOONHUT_TANK_POLL_MIN_S below.
//
// The tank build deliberately ships at 3 s, which is far faster than a water tank
// needs: MIN_REPORT_S is 120, so ~40 bursts happen between the two earliest possible
// reports and 39 of them cannot affect anything that leaves the node. It is kept fast
// on purpose - this module is the fleet's fast-response ranger testbed, and other
// projects (pump-lamp detection, anything reacting to an object crossing the beam)
// want sub-10 s latency. It is NOT a sensible default for a battery node: at 3 s the
// ESP32-S3 never sleeps, which dominates the power budget by far.
#ifndef MOONHUT_TANK_POLL_S
#define MOONHUT_TANK_POLL_S 2
#endif

// Bounds on the runtime value. The floor is 1 s rather than 0 because a burst itself
// occupies SAMPLES * pingGapMs (~500 ms at the tank build's settings) and a 0 would
// mean runOnce() re-arming with no gap at all - a busy loop wearing a scheduler's
// clothes. The ceiling is an hour; anything longer wants the sensor's power gated and
// the MCU actually asleep, which is a different firmware shape, not a bigger number.
#ifndef MOONHUT_TANK_POLL_MIN_S
#define MOONHUT_TANK_POLL_MIN_S 1
#endif
#ifndef MOONHUT_TANK_POLL_MAX_S
#define MOONHUT_TANK_POLL_MAX_S 3600
#endif

// How many raw pings go into one reported figure. Ultrasonics throw spurious
// echoes off walls, pipes and ripples constantly; a median discards them without
// the lag of an average. Odd numbers only.
#ifndef MOONHUT_TANK_SAMPLES
#define MOONHUT_TANK_SAMPLES 5
#endif

// A burst must return at least this many echoes to count. Below it there is no
// median to speak of - a "median" of one sample is a single unverified ping wearing
// the same confidence as a five-of-five result, which is exactly how the fridge bus
// glitches got through before they were filtered.
#ifndef MOONHUT_TANK_MIN_ECHOES
#define MOONHUT_TANK_MIN_ECHOES 3
#endif

// ...and the samples must agree to within this, in metres. A wide spread means the
// sensor picked a different target on each ping, so the median is not a measurement,
// it is the least-bad guess among several different objects.
#ifndef MOONHUT_TANK_MAX_SPREAD_M
#define MOONHUT_TANK_MAX_SPREAD_M 0.10f
#endif

// --- Consensus, not range --------------------------------------------------
//
// The original gate demanded every sample fall within MAX_SPREAD_M min-to-max. Ping
// dumps from a JSN showed why that is too brittle: a real target at 1.25 m arrives
// alongside a fixed ~0.247 m RINGDOWN artifact (the transducer hearing its own burst -
// it recurs to the millimetre across unrelated bursts) and a tail of later multipath
// reflections at 1.4-2.1 m. One ringdown sample destroys a min-to-max range on its own,
// so a perfectly good median was being thrown away every time.
//
// Instead: take the median, then count how many samples agree with it within a
// tolerance. A burst is trusted when enough of them do. That ignores one low artifact
// and one high reflection without ever averaging them in.
#ifndef MOONHUT_TANK_AGREE_M
#define MOONHUT_TANK_AGREE_M 0.10f
#endif
#ifndef MOONHUT_TANK_AGREE_MIN
#define MOONHUT_TANK_AGREE_MIN 3
#endif

// --- Cluster selection (2026-09-08) ----------------------------------------
//
// The median-then-consensus filter above assumes ONE target plus noise. In a narrow tank
// below ~1.1 m there are TWO: the surface, and the walls the beam is now wide enough to
// hit. MEASURED with the probe untouched: bursts alternated between 1.29 m (surface) and
// 0.93 m (wall) and half of them failed consensus outright - including bursts that
// contained a perfect 8 mm hit on the real surface, discarded because it arrived in bad
// company. The median of a two-population burst lands on whichever population won 3/5.
//
// So: group the sorted pings into clusters (a gap wider than AGREE_M starts a new one),
// then CHOOSE a cluster rather than take the middle of everything:
//   1. if the node has a belief about the level (stableM, the cross-burst median), the
//      cluster nearest it within TRACK_GATE_M - this is what recovers the good ping;
//   2. else the largest cluster;
//   3. ties -> the FARTHEST. A downward-looking tank sensor's spurious targets (walls,
//      floats, inlet streams, the tube lip) are all NEARER than the water. Never the
//      other way round.
// The chosen cluster then faces the same MIN_ECHOES / AGREE_MIN gates as before, so a
// lone ping still cannot become a reading. `nc=` in the report says how many clusters
// the burst had - two or more is the multi-target diagnosis, on the wire, per burst.
#ifndef MOONHUT_TANK_TRACK_GATE_M
#define MOONHUT_TANK_TRACK_GATE_M 0.15f
#endif

// Runtime OVERRIDE of the profile's near-field floor. 0 = use the profile value.
//
// Added 2026-09-07 for tank 1, where a float-switch ball hangs ~0.34 m below the probe -
// ABOVE the jsn profile's 0.30 m floor, so it was accepted as a valid reading with 5/5
// echoes and 4 mm spread. On a 2 m tank that publishes as "almost full", permanently, and
// no filter catches it because the reading is internally consistent.
//
// The geometry is what makes a single cutoff work: a FLOAT can only ever be CLOSER than
// the water it sits above - as the tank fills it rises, so its distance shrinks. It never
// appears beyond its resting distance. So everything nearer than the cutoff is the float
// and everything beyond it is water.
//
// The cost is real and must be stated: this BLINDS the tank above the cutoff. When the
// water is genuinely that high the node reports d=? rather than a number. That is the
// honest failure - a float switch is the right instrument for "full", and this is the
// right instrument for everything below it.
//
// Prefer PHYSICAL separation when the tank allows it (>=50 cm clear of the beam cone);
// that keeps the whole range. This is the fallback for when geometry will not cooperate.
#ifndef MOONHUT_TANK_FLOOR_MAX_M
#define MOONHUT_TANK_FLOOR_MAX_M 2.0f
#endif

// --- Blind-vs-fault evidence (2026-09-08) ------------------------------------
//
// A full tank and a dead sensor used to leave the node as the same line: `d=?|why=no
// echo`. The sensor itself knows the difference and the code was throwing it away:
// water inside the near field returns a rock-steady RINGDOWN pulse (MEASURED on tank 1,
// 2026-09-08: 204 pings, 0 timeouts, 0.246-0.255 m, for the whole time the surface was
// inside the 0.30 m floor), while a dead or unwired sensor returns NO PULSE AT ALL.
//
// So every ping now carries a CLASS out of pingOnce(), and a burst is judged on the
// whole set:
//   BURST_BLIND  = zero valid echoes, zero silent pings, >= BLIND_MIN_NEAR sub-floor pings
//                  no shorter than the ringdown's lower edge, agreeing to BLIND_MAX_SPREAD_M
//   BURST_SILENT = only timeouts and runts - nothing came back at all
//   BURST_MIXED  = everything else (3 ringdowns + 2 timeouts is NOT evidence of anything)
// and the module state is a run of them: FAULT is tested FIRST (STATE_RUN silent bursts),
// then FULL (STATE_RUN perfect blind bursts with no bad burst in the last BLIND_CLEAN_
// BURSTS, and the geometry and trend not vetoing). Entering FULL is hard; ONE imperfect
// burst leaves it. A healthy transducer rings every single time it is fired -
// intermittent silence is a dying transducer, which is exactly what must never read full.
//
// RUNT is a SAFETY class, not tidiness: with NO sensor attached, the trigger edge couples
// 1-12 us of crosstalk onto the adjacent echo pin (measured, see pingOnce). Without a
// minimum width an unplugged sensor could satisfy the near-field test and read FULL.
#ifndef MOONHUT_TANK_RUNT_M
#define MOONHUT_TANK_RUNT_M 0.05f
#endif
#ifndef MOONHUT_TANK_BLIND_MIN_NEAR
#define MOONHUT_TANK_BLIND_MIN_NEAR 4
#endif
// SECOND BOUNCE. With the surface inside the near field the direct echo lands in the
// ringing and is reported as ringdown - but the sound also goes sensor -> water -> sensor ->
// water -> sensor and comes back as a clean pulse at TWICE the true distance. MEASURED on
// tank 1, 2026-09-09 01:27-02:50 TH, tank full, surface 0.22-0.245 m: bursts of 3-4 ringdown
// pings plus 1-2 "valid" echoes at 0.44-0.49 m. Those passed as readings (three of them
// even made consensus at 0.445 m), kept the FULL verdict from forming for 25 minutes, and
// taught the Pi a level of 0.488 m that it then used to call the correct FULL a fault.
//
// So when a burst is MOSTLY near-field, a valid ping nearer than ECHO2_MAX_X times the
// floor is the second bounce: it leaves the sample set, counts as near-field evidence,
// and is reported as e2=. The test is against the FLOOR, not the ringdown value: the
// ringdown reads the transducer's fixed ring time (~0.23 m on this part) whatever the true
// distance is, while the bounce sits at twice the TRUE distance - on 2026-09-09 15:20 TH,
// surface ~0.18 m, the bounces came back at 0.34-0.38 m, only 1.5x the ringdown, and a
// ratio test built on the night before (surface 0.23 m, bounce 0.44-0.49 m, ratio ~2)
// let them through. Geometry: ringdown-majority means the surface is inside the near
// field (< floor), so its second bounce is < 2 x floor. A burst with only one or two
// ringdown pings is left alone - at the zone edge a real surface at 0.32-0.45 m does come
// with a stray ringdown ping, and that reading must survive.
// 2, not 3: live 15:40 TH a burst of 2 ringdown + 3 bounces at 0.35 m made consensus and
// dropped FULL to a 97 % reading. Two pings under the floor already say near-field.
#ifndef MOONHUT_TANK_ECHO2_MIN_NEAR
#define MOONHUT_TANK_ECHO2_MIN_NEAR 2
#endif
#ifndef MOONHUT_TANK_ECHO2_MAX_X
#define MOONHUT_TANK_ECHO2_MAX_X 2.2f
#endif
// 80 mm, the width of the whole ringdown band, not 20. MEASURED 2026-09-09 15:30 TH with the
// surface ~0.18 m from the face: 9 of 24 all-near-field bursts spread 21-45 mm (the direct
// echo arriving inside the ring moves its threshold crossing), and at 20 mm each one counted
// as imperfect, kept the clean window dirty, and FULL never formed on a plainly full tank.
// Every ping in such a burst is under the floor; scatter among them is still near-field.
#ifndef MOONHUT_TANK_BLIND_MAX_SPREAD_M
#define MOONHUT_TANK_BLIND_MAX_SPREAD_M 0.080f
#endif
#ifndef MOONHUT_TANK_STATE_RUN
#define MOONHUT_TANK_STATE_RUN 3
#endif
#ifndef MOONHUT_TANK_BLIND_CLEAN_BURSTS
#define MOONHUT_TANK_BLIND_CLEAN_BURSTS 16
#endif
// Trend corroboration - ONE-DIRECTIONAL. The last good reading can only VETO a FULL
// (blindness that began while the surface was still far from the floor is an object in
// the beam, not water), never assert one, and it is unarmed after a boot or past its TTL.
#ifndef MOONHUT_TANK_CONF_TTL_S
#define MOONHUT_TANK_CONF_TTL_S 1800
#endif
#ifndef MOONHUT_TANK_CONF_NEAR_M
#define MOONHUT_TANK_CONF_NEAR_M 0.15f
#endif
#ifndef MOONHUT_TANK_CONF_ODD_M
#define MOONHUT_TANK_CONF_ODD_M 0.30f
#endif
// Geometry guard: FULL is only claimable when the blind edge IS the 100 % line. On a node
// whose floor was RAISED (tank:floor=, to reject a float ball), going blind means the
// float is in the beam, not that the tank is full.
#ifndef MOONHUT_TANK_GEOM_SLACK_M
#define MOONHUT_TANK_GEOM_SLACK_M 0.05f
#endif
// LEAVING full takes this many consecutive imperfect bursts (entering stays hard). Live,
// 2026-09-08 19:27-20:50: with the float ball in the beam one stray ball ping knocked FULL to
// UNKNOWN within seconds, and FULL came back a minute later - every flip a LoRa packet.
#ifndef MOONHUT_TANK_FULL_LEAVE_N
#define MOONHUT_TANK_FULL_LEAVE_N 3
#endif
// The same edge announcement (FULL / FAULT / UNK) is not repeated on air within this window.
#ifndef MOONHUT_TANK_EDGE_MIN_S
#define MOONHUT_TANK_EDGE_MIN_S 600
#endif
// Clean bursts (single target, spread <= AGREE_M/2) required before a reading inside the
// profile's noise-trip band is believed. See MoonTankSensor::noiseLoM.
#ifndef MOONHUT_TANK_NOISE_CLEAN_N
#define MOONHUT_TANK_NOISE_CLEAN_N 3
#endif

// Near-field floor MOVED to the sensor profile (MoonTankSensor::minValidM), because it
// is a property of the part, not of the build: on a JSN it is 0.30 m to clear RINGDOWN
// (a rock-steady 0.227-0.247 m from that part is the transducer still ringing, not a
// target), while an A02 claims a 3 cm blind zone and would be needlessly blinded by it.
// A global here silently applied one sensor's artifact to another's readings.

// Screen policy. On external power the panel never blanks - the whole point of the box
// is a reading you can see. On battery it is allowed to sleep, but e-ink holds its last
// image without power, so a sleeping panel keeps SHOWING a number that is quietly going
// stale. Waking it briefly once an hour keeps that number honest for the cost of one
// refresh.
#ifndef MOONHUT_TANK_BATT_SCREEN_ON_S
#define MOONHUT_TANK_BATT_SCREEN_ON_S 60
#endif
#ifndef MOONHUT_TANK_BATT_REFRESH_S
#define MOONHUT_TANK_BATT_REFRESH_S 3600
#endif

// Report on a change this big, or on the heartbeat below, whichever comes first.
#ifndef MOONHUT_TANK_REPORT_DELTA_M
#define MOONHUT_TANK_REPORT_DELTA_M 0.02f
#endif
// --- Rate of level change -------------------------------------------------
//
// Reported as LEVEL metres per hour: positive = filling, negative = draining.
// Note the sign flip against the raw measurement - the sensor reads distance DOWN to the
// surface, so a rising distance means a FALLING level. Publishing the raw derivative would
// invert every reading a human looks at.
//
// Least-squares slope over a window, not a first-to-last difference: this sensor's noise is
// tens of millimetres, and a two-point difference would report that noise divided by the
// interval as if it were a flow rate.
#ifndef MOONHUT_TANK_RATE_SAMPLES
#define MOONHUT_TANK_RATE_SAMPLES 16 // ring depth
#endif

#ifndef MOONHUT_TANK_RATE_DECIMATE_S
#define MOONHUT_TANK_RATE_DECIMATE_S 60 // keep at most one sample a minute
#endif

// Below this span the slope is not published at all. A rate fitted over a couple of minutes
// of a noisy signal is a random number with units attached.
#ifndef MOONHUT_TANK_RATE_MIN_SPAN_S
#define MOONHUT_TANK_RATE_MIN_SPAN_S 300 // 5 min
#endif

// --- Cross-burst median ----------------------------------------------------
//
// The consensus filter inside a burst checks that the five pings AGREE. It cannot tell
// agreement from truth: an object crossing the beam gives five pings that agree
// perfectly on the wrong answer.
//
// MEASURED 2026-09-06, MoonTank1, 82-burst soak on a static target: 81 bursts at
// 2.104-2.111 m and ONE burst at 1.246 m with a 1 mm spread - something crossed the beam
// for ~3 s. That burst is exactly the shape the filter is built to TRUST. On a tank, a
// bird, a leaf, a hand or a rat produces a confident, tight-spread, wrong level.
//
// Only comparing ACROSS bursts can catch it, so the rate fit is fed the median of the
// last few burst medians rather than whichever burst happened to land on the decimation
// boundary. An odd window so the median is a real sample; 5 bursts is ~18 s at a 3.5 s
// cadence, and a transient has to dominate 3 of 5 to move the answer.
//
// The PANEL and the reported distance deliberately keep the INSTANTANEOUS value - a
// display at the tank should react to what is in front of it right now. It is the rate
// fit and the alert built on it that must not.
#ifndef MOONHUT_TANK_STABLE_WINDOW
#define MOONHUT_TANK_STABLE_WINDOW 5
#endif

// Fast-drain alert. DISABLED by default (0) and it should stay that way until this tank's
// normal draw has been observed: a threshold guessed before the first day of real data is
// just a source of false alarms, and an alert nobody trusts is worse than no alert.
#ifndef MOONHUT_TANK_FAST_DRAIN_MPH
#define MOONHUT_TANK_FAST_DRAIN_MPH 0.0f
#endif

// How much the distance must move before the PANEL is repainted. Unrelated to the
// reporting delta: this one only protects the e-ink from being burned through.
#ifndef MOONHUT_TANK_REDRAW_DELTA_M
#define MOONHUT_TANK_REDRAW_DELTA_M 0.01f
#endif

// The floor under EVERY report. Without one, a change-triggered report bypassed the
// heartbeat interval completely and could fire at the poll rate.
#ifndef MOONHUT_TANK_MIN_REPORT_S
#define MOONHUT_TANK_MIN_REPORT_S 120
#endif

#ifndef MOONHUT_TANK_REPORT_S
#define MOONHUT_TANK_REPORT_S 60
#endif

// --- Live aiming mode ------------------------------------------------------
//
// Mounting a probe is a two-person job otherwise: one at the tank moving it, one at a
// console reading the effect. `tank:live=<minutes>` collapses that to one person with a
// phone - the node broadcasts every burst on the fleet channel while you aim.
//
// Three properties matter and each exists because the obvious version is wrong:
//
//  * It reports FAILURES, not just readings. While aiming, "0.229 ringdown" and "2.1 m
//    ok" carry equal information - a mode that only speaks when the burst succeeds is
//    silent during exactly the part you are trying to fix. The normal report path sends
//    a bare `d=?`, which cannot tell ringdown from a timeout.
//  * It is RATE LIMITED, and not to the poll rate. A packet is ~968 ms of airtime at
//    LONG_FAST; broadcasting every 3 s is ~30 % duty on a shared channel, and this node
//    already logs "Ch. util >25%. Skip send". 15 s is frequent enough to aim by.
//  * It EXPIRES BY ITSELF. A live mode left on by someone who walked away is a node
//    quietly flooding the mesh for as long as it has power. It is a bounded window with
//    a hard ceiling, not a flag.
#ifndef MOONHUT_TANK_LIVE_PERIOD_S
#define MOONHUT_TANK_LIVE_PERIOD_S 15
#endif
#ifndef MOONHUT_TANK_LIVE_MAX_MIN
#define MOONHUT_TANK_LIVE_MAX_MIN 30
#endif
#ifndef MOONHUT_TANK_LIVE_DEFAULT_MIN
#define MOONHUT_TANK_LIVE_DEFAULT_MIN 10
#endif

// FAILSAFE: live for this long after EVERY boot, with no command needed.
//
// The command path needs a text packet from ANOTHER node, so a person at a remote tank
// with no laptop and no second radio has no way to ask the node anything. If the command
// path is broken - and as of 2026-09-07 it has never been proven end to end - they get
// no feedback at all and the trip is wasted.
//
// Power-cycling is the one input everybody has. So a boot gives a bounded window of
// readings for free, and positioning a probe becomes: switch it off and on, then watch
// the phone. 0 disables. Bounded, so a node that reboots in service costs the channel one
// window and then goes quiet.
#ifndef MOONHUT_TANK_LIVE_ON_BOOT_MIN
#define MOONHUT_TANK_LIVE_ON_BOOT_MIN 15
#endif
// Do not speak the instant the module starts: channels and the radio are still coming up,
// and the first line would be sent to nobody. Also keeps it out of the boot log noise.
#ifndef MOONHUT_TANK_LIVE_ON_BOOT_DELAY_S
#define MOONHUT_TANK_LIVE_ON_BOOT_DELAY_S 30
#endif

// --- Stall detection -------------------------------------------------------
//
// A node in an enclosure at a tank has no serial cable, so "it stopped showing a
// distance" has to be diagnosable from the mesh alone. Two things make that possible:
// every report carries uptime and the consecutive-failure count, and a run of failures
// long enough to matter announces itself instead of going quiet.
//
// The distinction that matters in the log: a node that REBOOTED comes back with a small
// uptime, a node that STALLED keeps counting up. Without uptime in the line the two are
// indistinguishable after the fact, which is exactly the hole this closes.
// How long the last fitted rate keeps riding on no-value reports after the numbers stop.
#ifndef MOONHUT_TANK_RATE_HOLD_S
#define MOONHUT_TANK_RATE_HOLD_S 900
#endif

#ifndef MOONHUT_TANK_STALL_S
#define MOONHUT_TANK_STALL_S 600
#endif

#ifndef MOONHUT_TANK_HEIGHT_DEFAULT
#define MOONHUT_TANK_HEIGHT_DEFAULT 0.0f
#endif
#ifndef MOONHUT_TANK_OFFSET_DEFAULT
#define MOONHUT_TANK_OFFSET_DEFAULT 0.0f
#endif

// --- Calibration -----------------------------------------------------------
//
// The node needs a tank height locally to show a percentage on its own panel, which is
// the whole point of having a panel at the tank. It is deliberately RUNTIME-SETTABLE and
// persisted, never a build flag: re-plumbing a tank must be a command, not a reflash.
//
// This does NOT make the node the system of record. The Pi still owns litres, history
// and alerting from the raw distance; the percentage here is a convenience readout for
// whoever is standing in front of the box. Two consumers of one geometry, deliberately -
// so if they ever disagree, the Pi wins.
//
//   tank:height=3.20    metres from the sensor face to the tank floor
//   tank:offset=0.35    optional dead space at the top the sensor cannot see
//   tank:show           report the current calibration
//   tank:clear          forget it and go back to showing distance only
#ifndef MOONHUT_TANK_CFG_PATH
#define MOONHUT_TANK_CFG_PATH "/tankcfg"
#endif

// What one ping came back as. pingOnce() is side-effect free on purpose - it is called
// for both rangers under MOONHUT_TANK_DUAL - so the classification travels in the result.
enum PingClass : uint8_t { PING_OK = 0, PING_TIMEOUT, PING_RUNT, PING_NEAR, PING_FAR, PING_SENTINEL };
struct PingResult {
    float m;        // metres, valid for OK/NEAR/FAR; NAN otherwise
    uint32_t us;    // raw pulse width
    PingClass cls;
};

// Tally of one burst, by class, plus the extent of the sub-floor pings.
struct BurstEvidence {
    uint8_t ok = 0, timeouts = 0, runts = 0, nears = 0, fars = 0, sentinels = 0;
    uint8_t echo2 = 0;   // second-bounce pings reclassified out of `ok` at burst time - see measure()
    float nearLo = NAN, nearHi = NAN;
    void reset() { *this = BurstEvidence(); }
    void add(const PingResult &r)
    {
        switch (r.cls) {
        case PING_OK: ok++; break;
        case PING_TIMEOUT: timeouts++; break;
        case PING_RUNT: runts++; break;
        case PING_FAR: fars++; break;
        case PING_SENTINEL: sentinels++; break;
        case PING_NEAR:
            nears++;
            if (isnan(nearLo) || r.m < nearLo) nearLo = r.m;
            if (isnan(nearHi) || r.m > nearHi) nearHi = r.m;
            break;
        }
    }
    /// Nothing but timeouts and runts: the sensor did not answer at all.
    bool allSilent() const { return ok == 0 && nears == 0 && fars == 0 && sentinels == 0; }
    float nearSpread() const { return nears ? nearHi - nearLo : NAN; }
};

enum BurstVerdict : uint8_t { BURST_OK = 0, BURST_SILENT, BURST_BLIND, BURST_MIXED };
enum TankState : uint8_t { TANK_UNKNOWN = 0, TANK_OK, TANK_BLIND_FULL, TANK_FAULT };

class MoonTankModule : public concurrency::OSThread
{
  public:
    MoonTankModule();

    /// The module's verdict on the tank, from runs of bursts - see the evidence note above.
    TankState state() const { return tankState; }
    /// Wire/panel name: unk, ok, full, fault. Matches what the gateway's parser accepts.
    const char *stateName() const;
    /// The last burst's evidence, for the panel and the report.
    const BurstEvidence &evidence() const { return lastEv; }

    /// Last filtered distance in metres, or NAN if the last burst found nothing.
    float distanceM() const { return lastM; }

    /// Spread of the samples behind the last reading, in metres. This is the number
    /// that says whether to trust it: a tight cluster is a real surface, a wide one
    /// means the sensor is guessing.
    float spreadM() const { return lastSpreadM; }

    /// How many of the last burst returned an echo at all.
    uint8_t validSamples() const { return lastValid; }

    /// Why the last burst was rejected, or nullptr if it was accepted.
    const char *rejectReason() const { return reject; }

    /// Session min/max, for the bench range test.
    float minM() const { return sessionMinM; }
    float maxM() const { return sessionMaxM; }

    /// True once a tank height has been set. Until then the panel shows distance only
    /// and says so, rather than inventing a percentage from a guessed height.
    bool isCalibrated() const { return tankHeightM > 0.0f; }
    float tankHeight() const { return tankHeightM; }
    float deadTopM() const { return tankOffsetM; }

    /// What the last burst measured, kept even when the gates rejected it. The panel
    /// shows this so a rejected reading is diagnosable without a serial cable.
    float rawM() const { return lastRawM; }

    /// Water depth in metres, NAN if uncalibrated or the last burst failed.
    float levelM() const;
    /// Fill percentage 0-100, NAN if uncalibrated or the last burst failed.
    float levelPct() const;
    /// Level change rate, metres/hour: + filling, - draining. NAN until the fit has span.
    float levelRateMphPublic() const { return levelRateMph(); }

    /// "tank:<command>" surface. Returns a short human-readable result to DM back.
    const char *handleCommand(const char *body);
    /// Commands are honoured on the fleet channel or a PKI DM, same rule as the fridge.
    bool acceptsCommand(uint8_t channelIndex, bool pkiEncrypted) const;

    void loadCalibration();
    void saveCalibration();

    // Active sensor profile. Never null - falls back to the default if a stored name
    // no longer matches a known profile (e.g. a profile was renamed in a later build).
    const MoonTankSensor *sensor = nullptr;
    const MoonTankSensor *lookupSensor(const char *name) const;
    const MoonTankSensor *activeSensor();

#ifdef MOONHUT_TANK_DUAL
    /// The comparison sensor's last filtered distance, NAN if its burst was rejected.
    float distance2M() const { return lastM2; }
    float spread2M() const { return lastSpread2; }
    uint8_t validSamples2() const { return lastValid2; }
    const char *rejectReason2() const { return reject2; }
#endif

  protected:
    int32_t runOnce() override;

  private:
    PingResult pingOnce(uint8_t trigPin, uint8_t echoPin, uint16_t trigUs); // one cycle, classified

    // Blind-vs-fault: judge the burst as a whole, then advance the state machine.
    BurstVerdict judgeBurst(bool accepted);
    void updateState(BurstVerdict v);
    const char *confidence();     // boot / stale / rise / near / none / odd - see CONF_* above
    bool blindEdgeIsFull();       // geometry guard - see GEOM_SLACK_M
    float activeRingLo() { return ringHiM > ringLoM ? ringLoM : activeSensor()->ringLoM; }
    float activeRingHi() { return ringHiM > ringLoM ? ringHiM : activeSensor()->ringHiM; }
    BurstEvidence evA;            // being collected this burst
    BurstEvidence lastEv;         // the burst just judged
    TankState tankState = TANK_UNKNOWN;
    TankState shownState = TANK_UNKNOWN; // panel repaint gate - NAN->NAN transitions were invisible
    uint8_t silentRun = 0;        // consecutive all-silent bursts
    uint8_t blindRun = 0;         // consecutive perfect blind bursts
    uint16_t badMask = 0;         // last 16 bursts, 1 = mixed or silent
    float lastGoodM = NAN;        // for the trend corroboration
    float prevGoodM = NAN;
    uint32_t lastKnownAtMs = 0;   // last time the module KNEW something: a reading, or blind-full
    float ringLoM = 0.0f, ringHiM = 0.0f; // runtime ringdown band; 0,0 = profile. `tank:ring=lo,hi`
    uint8_t badRun = 0;           // consecutive imperfect bursts while FULL (hysteresis on leaving)
    uint32_t lastEdgeMs[4] = {};  // per TankState: when its edge line last went on air (0 = never)
    uint8_t cleanRun = 0;         // consecutive clean bursts (single target, tight) - noise-band gate
    uint8_t screenMode = 0;       // 0 = auto (battery policy), 1 = always on. `tank:screen=`, persisted
    float sosMs = 0.0f;           // runtime speed of sound, m/s; 0 = profile. `tank:sos=` / `tank:temp=`
    float activeSpeedMs() { return sosMs > 0.0f ? sosMs : activeSensor()->speedMs; }
    // Ringdown band actually SEEN while a live window was open - emitted when it closes.
    // This is the phase-3 measurement, for free: aim, watch, and the band prints itself.
    float liveRingLo = NAN, liveRingHi = NAN;
    uint32_t liveRingN = 0, liveTimeouts = 0, livePings = 0;

    /// Judge a collected sample set. Sorts `s` in place, sets median/spread, and sets
    /// `why` when the result should not be trusted. `median` is still set for the log
    /// unless nothing answered at all.
    bool evaluate(float *s, uint8_t n, float &median, float &spread, const char *&why);

    void measure();
    void acceptReading(float median); // the accepted-burst half of measure()
    void diagnose();   // runs after repeated silence: says WHY there is no echo
    void report(bool force);
    void serviceScreen(uint32_t now);
    void sendLine(const char *text);
    void recordLevel(uint32_t now, float metres);

    // Ring of recent accepted burst medians, and their median. See the note above: this
    // is what defends the rate fit from a single beam-crossing burst.
    void pushBurst(float metres);
    float stableM() const; // median of the ring, NAN until it has filled
    float stableBuf[MOONHUT_TANK_STABLE_WINDOW] = {};
    uint8_t stableCount = 0;
    uint8_t stableHead = 0;
    float levelRateMph() const; // metres/hour, + = filling, NAN if not enough span

    // Ring of decimated (time, distance) samples, for the least-squares rate fit.
    struct RateSample {
        uint32_t atMs;
        float m;
    };
    RateSample rateBuf[MOONHUT_TANK_RATE_SAMPLES] = {};
    uint8_t rateCount = 0;
    uint8_t rateHead = 0;
    uint32_t lastRateAt = 0;
    bool drainAlarm = false;

    // --- Sampling state machine ------------------------------------------------
    //
    // ONE ping per scheduler pass, not five in a row. Five back-to-back delay()s is
    // ~600 ms of a 3 s cycle spent inside delay(), and Meshtastic's scheduler is
    // COOPERATIVE - nothing else runs during that. It starved the button thread, whose
    // library needs regular ticks to recognise a press, so PRG presses landing inside a
    // burst were never seen at all: several presses produced one event.
    //
    // The fridge module already avoids this deliberately (setWaitForConversion(false));
    // the lesson simply never got carried across. MVT1's button works and the tank
    // nodes' did not, which is the A/B that identified it.
    //
    // Now runOnce() blocks only for a single pulseIn (25 ms worst case) and returns to
    // the scheduler between pings. Same cadence, same samples, no starvation - and the
    // LoRa timing stops sharing a core with half a second of delay().
    enum SamplePhase : uint8_t { PHASE_A = 0, PHASE_B = 1, PHASE_EVAL = 2 };
    SamplePhase phase = PHASE_A;
    uint8_t pingIdx = 0;
    float sampA[MOONHUT_TANK_SAMPLES] = {};
    uint8_t nA = 0;
#ifdef MOONHUT_TANK_DUAL
    float sampB[MOONHUT_TANK_SAMPLES] = {};
    uint8_t nB = 0;
#endif

    bool calLoaded = false;     // littlefs is not mounted when modules are constructed
    // Build defaults so a node comes up CALIBRATED even if the stored file is missing or
    // stale. Added 2026-09-07: tank 1's node-side height did not survive a reflash (cause
    // still unexplained) and the panel then read 0 %, which at a tank is worse than
    // useless - it says "empty" to anyone walking past while the data is perfectly fine.
    // A stored value still wins; these only fill in when there is nothing valid to load.
    // NOTE: `tank:clear` therefore returns to THIS value, not to 0.
    float tankHeightM = MOONHUT_TANK_HEIGHT_DEFAULT;   // 0 = uncalibrated
    float tankOffsetM = MOONHUT_TANK_OFFSET_DEFAULT; // dead space at the top, off usable depth
    uint16_t pollS = MOONHUT_TANK_POLL_S; // live measurement cadence; `tank:poll=`, persisted
    // Live aiming mode. Deliberately NOT persisted: it must never survive a reboot, or a
    // node that reset while live comes back flooding the channel with nobody listening.
    // Reporting cadence, runtime-settable 2026-09-07. These drive the ANALYTICS
    // resolution (fill/drain rates), not the measurement rate - `pollS` is that. They were
    // build flags, which meant the data resolution of a node in a box at a tank could only
    // be changed by opening the box. 0 = use the compile-time default.
    uint32_t reportS = 0;      // heartbeat; 0 = MOONHUT_TANK_REPORT_S
    uint32_t minReportS = 0;   // floor under every report; 0 = MOONHUT_TANK_MIN_REPORT_S
    float reportDeltaM = 0.0f; // report early on a change this big; 0 = the build default
    uint32_t activeReportS() const { return reportS ? reportS : MOONHUT_TANK_REPORT_S; }
    uint32_t activeMinReportS() const { return minReportS ? minReportS : MOONHUT_TANK_MIN_REPORT_S; }
    float activeDeltaM() const { return reportDeltaM > 0.0f ? reportDeltaM : MOONHUT_TANK_REPORT_DELTA_M; }
    float floorM = 0.0f;        // runtime near-field floor; 0 = use the sensor profile's
    /// The floor actually in force: the runtime override when set, else the profile's.
    // Not const: activeSensor() resolves the profile lazily and is itself non-const.
    float activeFloorM() { return floorM > 0.0f ? floorM : activeSensor()->minValidM; }
    uint32_t liveUntilMs = 0;   // 0 = off
    uint32_t nextLiveAtMs = 0;
    void sendLive();
    void serviceLive(); // called every burst; expires the window and paces the broadcasts

    float lastM = NAN;
    float lastSpreadM = NAN;
    // What the last burst actually saw, kept even when the gates rejected it. Throwing
    // the number away is what made a rejected reading undiagnosable from the mesh.
    float lastRawM = NAN;
    uint32_t consecFails = 0;
    uint32_t lastGoodAtMs = 0;
    bool stallAnnounced = false;
    uint8_t lastValid = 0;
    uint8_t lastClusters = 0;      // how many distinct targets the last burst contained
    float sessionMinM = NAN;
    float sessionMaxM = NAN;
    uint32_t bursts = 0;
    uint32_t timeouts = 0;
    bool diagnosed = false;
    const char *reject = nullptr;
#ifdef MOONHUT_TANK_DUAL
    float lastM2 = NAN;
    float lastSpread2 = NAN;
    uint8_t lastValid2 = 0;
    const char *reject2 = nullptr;
#endif
    float reportedM = NAN;
    float pendingM = NAN;          // a candidate change, not yet confirmed by a second reading
    float shownM = NAN;            // what the panel is currently displaying
    uint32_t nextReportAt = 0;
    uint32_t nextMinReportAt = 0;  // floor under every report, change-triggered ones included
    uint32_t screenOnSince = 0;
    uint32_t screenOffSince = 0;   // 0 = the panel is lit
    bool wasOnUsb = true;
};

extern MoonTankModule *moonTankModule;

#endif // MOONHUT_TANK
