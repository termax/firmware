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
    bool moonSign = false;
#ifdef MOONHUT_SIGN
    // MoonHut: a MoonPaper-channel message takes over the e-ink fullscreen via the sign
    // frame. NOTE: Screen::handleTextMessage is dead code in 2.7.26 (nothing observes it),
    // so the sign must be driven from here.
    if (mp.from != 0 && strcmp(channels.getByIndex(mp.channel).settings.name, "MoonPaper") == 0)
        moonSign = true;
#endif
    (void)moonSign;
    IF_SCREEN(
        // Guard against running in MeshtasticUI or with no screen
        if (config.display.displaymode != meshtastic_Config_DisplayConfig_DisplayMode_COLOR) {
            // Store in the central message history
            const StoredMessage &sm = messageStore.addFromPacket(mp);

            // Pass message to renderer (banner + thread switching + scroll reset)
            // Use the global Screen singleton to retrieve the current OLED display
            if (!moonSign) {
                auto *display = screen ? screen->getDisplayDevice() : nullptr;
                graphics::MessageRenderer::handleNewMessage(display, sm, mp);
            }
        })
#if defined(MOONHUT_SIGN) && HAS_SCREEN
    if (moonSign && screen) {
        // Attribution: sender short name + HH:MM local time (if RTC valid)
        const meshtastic_NodeInfoLite *snode = nodeDB->getMeshNode(mp.from);
        const char *sname = (snode && snode->has_user && snode->user.short_name[0]) ? snode->user.short_name : "???";
        char attr[64];
        uint32_t when = getValidTime(RTCQualityDevice, true);
        if (when > 0) {
            uint32_t ds = when % 86400;
            snprintf(attr, sizeof(attr), "%s  %02u:%02u", sname, (unsigned)(ds / 3600), (unsigned)((ds % 3600) / 60));
        } else {
            snprintf(attr, sizeof(attr), "%s", sname);
        }
        graphics::MessageRenderer::setMoonSignMessage(reinterpret_cast<const char *>(mp.decoded.payload.bytes), attr);

        screen->setOn(true); // a sign must always show the new message, even from the screensaver
        UIFrameEvent e;
        e.action = UIFrameEvent::Action::SWITCH_TO_TEXTMESSAGE;
        screen->handleUIFrameEvent(&e); // jump to the sign frame (drawTextMessageFrame renders it) + fast refresh
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