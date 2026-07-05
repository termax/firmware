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
    if (mp.from != 0) {
        if (strcmp(channels.getByIndex(mp.channel).settings.name, "MoonPaper") == 0)
            moonKind = MOON_PERSIST;
        else if (!isBroadcast(mp.to) && isToUs(&mp))
            moonKind = MOON_FLASH_DM;
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
        const char *text = reinterpret_cast<const char *>(mp.decoded.payload.bytes);
        if (moonKind == MOON_PERSIST)
            graphics::MessageRenderer::setMoonSignMessage(text, attr);
        else
            graphics::MessageRenderer::setMoonFlashMessage(text, attr, moonKind == MOON_FLASH_DM ? 10000 : 3000);

        screen->setOn(true); // the sign must show incoming messages even from the screensaver
        UIFrameEvent e;
        e.action = UIFrameEvent::Action::SWITCH_TO_TEXTMESSAGE;
        screen->handleUIFrameEvent(&e); // jump to the sign frame (drawTextMessageFrame renders it) + fast refresh

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