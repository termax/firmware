#pragma once
#include "Observer.h"
#include "SinglePortModule.h"
#define TEXT_PACKET_LIST_SIZE 50

/**
 * Text message handling for Meshtastic.
 *
 * This module is responsible for receiving and storing incoming text messages
 * from the mesh. It updates device state and notifies observers so that other
 * components (such as the MessageRenderer) can later display or process them.
 *
 * Rendering of messages on screen is no longer done here.
 */
class TextMessageModule : public SinglePortModule, public Observable<const meshtastic_MeshPacket *>
{
  public:
    /** Constructor
     * name is for debugging output
     */
    TextMessageModule() : SinglePortModule("text", meshtastic_PortNum_TEXT_MESSAGE_APP) {}

    bool recentlySeen(uint32_t id);

#ifdef MOONHUT_SIGN
    // Sign read-receipt: an incoming "@target#ackid msg" arms a pending ack. The sign
    // auto-DMs "rcv:<ackid>" on display (delivery) and, when a human HOLDS the PRG
    // button, "ack:<ackid>" (read). See TextMessageModule.cpp and Screen.cpp SELECT.
    void moonSetAckContext(const char *ackid, NodeNum from); // arm (non-empty) / disarm ("")
    bool moonSignAck(); // fire ack:<id> from a PRG hold; false if nothing armed
#endif

  protected:
    /** Called to handle a particular incoming message
     *
     * @return ProcessMessage::STOP if you've guaranteed you've handled this
     *         message and no other handlers should be considered for it.
     */
    virtual ProcessMessage handleReceived(const meshtastic_MeshPacket &mp) override;
    virtual bool wantPacket(const meshtastic_MeshPacket *p) override;

  private:
    uint32_t textPacketList[TEXT_PACKET_LIST_SIZE] = {0};
    size_t textPacketListIndex = 0;
#ifdef MOONHUT_SIGN
    char moonAckId[24] = ""; // pending receipt id for the current sign message ("" = none)
    NodeNum moonAckFrom = 0; // originator to DM the receipt back to
#endif
};

extern TextMessageModule *textMessageModule;