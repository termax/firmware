#include "TextMessageModule.h"
#include "Channels.h"
#include "MeshService.h"
#include "gps/RTC.h"
#include "MessageStore.h"
#include "NodeDB.h"
#include "PowerFSM.h"
#include "buzz.h"
#include "configuration.h"
#include "graphics/Screen.h"
#include "graphics/SharedUIDisplay.h"
#include "graphics/draw/MessageRenderer.h"
#include "main.h"
TextMessageModule *textMessageModule;

ProcessMessage TextMessageModule::handleReceived(const meshtastic_MeshPacket &mp)
{
#if defined(DEBUG_PORT) && !defined(DEBUG_MUTE)
    auto &p = mp.decoded;
    LOG_INFO("Received text msg from=0x%0x, id=0x%x, msg=%.*s", mp.from, mp.id, p.payload.size, p.payload.bytes);
#endif
    // add packet ID to the rolling list of packets
    textPacketList[textPacketListIndex] = mp.id;
    textPacketListIndex = (textPacketListIndex + 1) % TEXT_PACKET_LIST_SIZE;

    // We only store/display messages destined for us.
    devicestate.rx_text_message = mp;
    devicestate.has_rx_text_message = true;
    // MoonHut message hierarchy: MoonPaper = the persistent sign message; a DM to us =
    // 10s flash; anything else = 3s flash. The stock "New Message" popup is suppressed
    // entirely (it carries no information on a dedicated sign).
    // NOTE: Screen::handleTextMessage is dead code in 2.7.26, so all of this lives here.
    enum MoonKind { MOON_NONE, MOON_PERSIST, MOON_FLASH_DM, MOON_FLASH_PUB };
    MoonKind moonKind = MOON_NONE;
#ifdef MOONHUT_SIGN
    size_t moonSkip = 0;         // bytes of a leading "@target#ackid " to strip before display
    char moonAckPending[24] = ""; // receipt id parsed from the prefix ("" = no receipt)
    if (mp.from != 0) {
        if (strcmp(channels.getByIndex(mp.channel).settings.name, "MoonPaper") == 0) {
            moonKind = MOON_PERSIST;
            // Multi-sign targeting (2026-07-16, fleet grew beyond one sign): "@SHORT msg"
            // on the sign channel is for the sign whose owner short name matches (case-
            // insensitive); "@all msg" or no prefix = every sign. Non-matching signs stay
            // completely quiet (no flash, no wake, no qr-ack). Runtime owner.short_name =
            // re-targeting a sign is a config change, not a reflash.
            const char *t = reinterpret_cast<const char *>(mp.decoded.payload.bytes);
            if (t[0] == '@') {
                char tgt[17] = {0};
                size_t n = 0;
                while (n < sizeof(tgt) - 1 && t[1 + n] && t[1 + n] != ' ' && t[1 + n] != ':' && t[1 + n] != '#')
                    tgt[n] = t[1 + n], n++;
                const char *rest = t + 1 + n;
                // Optional receipt id "#<ackid>" right after the target (2026-07-24):
                // gateway mints it when a send requests a read-receipt. Parsed here,
                // stripped with the rest of the prefix so it never reaches the screen.
                char ackid[24] = {0};
                if (*rest == '#') {
                    rest++;
                    size_t a = 0;
                    while (a < sizeof(ackid) - 1 && *rest && *rest != ' ' && *rest != ':') {
                        char c = *rest;
                        if ((c >= '0' && c <= '9') || (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z'))
                            ackid[a++] = c;
                        rest++;
                    }
                    ackid[a] = 0;
                }
                while (*rest == ' ' || *rest == ':')
                    rest++;
                if (n == 0 || !*rest)
                    moonKind = MOON_NONE; // "@" alone / empty body — nothing to show
                else if (strcasecmp(tgt, "all") == 0 || strcasecmp(tgt, owner.short_name) == 0) {
                    moonSkip = (size_t)(rest - t);
                    if (ackid[0])
                        strcpy(moonAckPending, ackid); // armed after render (see below)
                } else
                    moonKind = MOON_NONE; // addressed to another sign
            }
        } else if (!isBroadcast(mp.to) && isToUs(&mp))
            // DM = this sign's own content (2026-07-16): with the Paper's USB dead and
            // unflashable, DMs are the per-device targeting path that needs no channel
            // convention — a text DM to a sign IS its new sign text, not a 10s flash.
            moonKind = MOON_PERSIST;
        else
            moonKind = MOON_FLASH_PUB;
    }
#endif
    (void)moonKind;
    IF_SCREEN(
        // Guard against running in MeshtasticUI or with no screen
        if (config.display.displaymode != meshtastic_Config_DisplayConfig_DisplayMode_COLOR) {
            // Store in the central message history
            const StoredMessage &sm = messageStore.addFromPacket(mp);
#ifndef MOONHUT_SIGN
            // Pass message to renderer (banner + thread switching + scroll reset)
            // Use the global Screen singleton to retrieve the current OLED display
            auto *display = screen ? screen->getDisplayDevice() : nullptr;
            graphics::MessageRenderer::handleNewMessage(display, sm, mp);
#else
            (void)sm;
#endif
        })
#if defined(MOONHUT_SIGN) && HAS_SCREEN
    if (moonKind != MOON_NONE && screen) {
        // Attribution: sender short name + HH:MM local time (if RTC valid), prefixed by
        // "DM" or the channel name for flashes so the reader knows what they're seeing.
        const meshtastic_NodeInfoLite *snode = nodeDB->getMeshNode(mp.from);
        const char *sname = (snode && snode->has_user && snode->user.short_name[0]) ? snode->user.short_name : "???";
        char prefix[24] = "";
        if (moonKind == MOON_FLASH_DM) {
            snprintf(prefix, sizeof(prefix), "DM ");
        } else if (moonKind == MOON_FLASH_PUB) {
            const char *chName = channels.getByIndex(mp.channel).settings.name;
            snprintf(prefix, sizeof(prefix), "#%s ", chName[0] ? chName : "LongFast");
        }
        char attr[64];
        uint32_t when = getValidTime(RTCQualityDevice, true);
        if (when > 0) {
            uint32_t ds = when % 86400;
            snprintf(attr, sizeof(attr), "%s%s  %02u:%02u", prefix, sname, (unsigned)(ds / 3600), (unsigned)((ds % 3600) / 60));
        } else {
            snprintf(attr, sizeof(attr), "%s%s", prefix, sname);
        }
        const char *text = reinterpret_cast<const char *>(mp.decoded.payload.bytes) + moonSkip;
        if (moonKind == MOON_PERSIST) {
            // "sethome:<content>" persists the boot/home screen (per-paper login QR) to
            // littlefs and shows it; a reboot restores it. Otherwise a normal sign message.
            if (strncmp(text, "sethome:", 8) == 0)
                graphics::MessageRenderer::setMoonHome(text + 8);
            else
                graphics::MessageRenderer::setMoonSignMessage(text, attr);
        } else
            graphics::MessageRenderer::setMoonFlashMessage(text, attr, moonKind == MOON_FLASH_DM ? 10000 : 3000);

        // Read-receipt (2026-07-24): a persistent sign message carrying "@target#ackid"
        // arms the PRG-hold ack and auto-DMs "rcv:<ackid>" now as delivery confirmation.
        // A persistent message with no ackid disarms any stale pending id. Flashes and the
        // DM/QR paths never arm a receipt.
        if (moonKind == MOON_PERSIST) {
            moonSetAckContext(moonAckPending, mp.from);
            if (moonAckPending[0]) {
                meshtastic_MeshPacket *rcv = allocDataPacket();
                rcv->to = mp.from;
                rcv->decoded.payload.size = snprintf((char *)rcv->decoded.payload.bytes,
                                                     sizeof(rcv->decoded.payload.bytes), "rcv:%s", moonAckPending);
                service->sendToMesh(rcv, RX_SRC_LOCAL, false);
            }
        }

        screen->setOn(true); // the sign must show incoming messages even from the screensaver
        UIFrameEvent e;
        e.action = UIFrameEvent::Action::SWITCH_TO_TEXTMESSAGE;
        screen->handleUIFrameEvent(&e); // jump to the sign frame (drawTextMessageFrame renders it) + fast refresh

        // Render confirmation for the QR rotation pipeline: "qr:<data>|<caption>|<ackid>"
        // gets a "qr-ok:<ackid>" DM back to the sender (or "qr-err:" if the payload can't
        // encode), so the rotation script can retry on silence. No ackid = no reply, so
        // manual sends stay quiet. "qr-ok:" itself never matches the qr: prefix (no loop).
        if ((text[0] == 'q' || text[0] == 'Q') && (text[1] == 'r' || text[1] == 'R') && text[2] == ':') {
            const char *p1 = strchr(text + 3, '|');
            const char *p2 = p1 ? strchr(p1 + 1, '|') : nullptr;
            if (p2 && p2[1]) {
                const size_t dataLen = (size_t)(p1 - (text + 3));
                const bool fits = dataLen > 0 && dataLen <= 134; // QR v6 byte-mode capacity at ECC LOW
                meshtastic_MeshPacket *reply = allocDataPacket();
                reply->to = mp.from;
                reply->decoded.payload.size = snprintf((char *)reply->decoded.payload.bytes,
                                                       sizeof(reply->decoded.payload.bytes), "qr-%s:%s",
                                                       fits ? "ok" : "err", p2 + 1);
                service->sendToMesh(reply, RX_SRC_LOCAL, false);
            }
        }

#ifdef LED_POWER
        // Attention blink for the sign message and DMs; a brief public-channel flash stays silent.
        if (moonKind != MOON_FLASH_PUB) {
            for (int i = 0; i < 10; i++) {
                digitalWrite(LED_POWER, LED_STATE_OFF);
                delay(150);
                digitalWrite(LED_POWER, LED_STATE_ON);
                delay(150);
            }
        }
#endif
    }
#endif
    // Only trigger screen wake if configuration allows it
    if (shouldWakeOnReceivedMessage()) {
        powerFSM.trigger(EVENT_RECEIVED_MSG);
    }

    // Notify any observers (e.g. external modules that care about packets)
    notifyObservers(&mp);

    return ProcessMessage::CONTINUE; // Let others look at this message also if they want
}

bool TextMessageModule::wantPacket(const meshtastic_MeshPacket *p)
{
    return MeshService::isTextPayload(p);
}

bool TextMessageModule::recentlySeen(uint32_t id)
{
    for (size_t i = 0; i < TEXT_PACKET_LIST_SIZE; i++) {
        if (textPacketList[i] != 0 && textPacketList[i] == id) {
            return true;
        }
    }
    return false;
}

#ifdef MOONHUT_SIGN
void TextMessageModule::moonSetAckContext(const char *ackid, NodeNum from)
{
    strncpy(moonAckId, ackid ? ackid : "", sizeof(moonAckId) - 1);
    moonAckId[sizeof(moonAckId) - 1] = 0;
    moonAckFrom = from;
}

bool TextMessageModule::moonSignAck()
{
    if (!moonAckId[0])
        return false; // nothing armed — let the caller fall through (e.g. no-op on a plain message)
    meshtastic_MeshPacket *reply = allocDataPacket();
    reply->to = moonAckFrom ? moonAckFrom : 0x8fa66864; // originator, else the gateway
    reply->decoded.payload.size =
        snprintf((char *)reply->decoded.payload.bytes, sizeof(reply->decoded.payload.bytes), "ack:%s", moonAckId);
    service->sendToMesh(reply, RX_SRC_LOCAL, false);
    graphics::MessageRenderer::setMoonFlashMessage("ACKNOWLEDGED", "", 3000); // plain ASCII: font has no U+2713
    moonAckId[0] = '\0'; // one-shot: repeated holds on the same message don't re-ack
    return true;
}
#endif