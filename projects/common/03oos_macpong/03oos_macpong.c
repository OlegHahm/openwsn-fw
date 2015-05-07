/**
\brief A simple application to test MAC connectivity.

\author Thomas Watteyne <watteyne@eecs.berkeley.edu>, August 2014.
*/

// stack initialization
#include "opendefs.h"
#include "board.h"
#include "scheduler.h"
#include "openstack.h"
// needed for spoofing
#include "openqueue.h"
#include "opentimers.h"
#include "IEEE802154E.h"
#include "openserial.h"
#include "packetfunctions.h"
#include "sixtop.h"
#include "idmanager.h"
#include "neighbors.h"

#define LEN_PAYLOAD 100

//=========================== variables =======================================

typedef struct {
   opentimer_id_t  timerId;
} macpong_vars_t;

macpong_vars_t macpong_vars;

//=========================== prototypes ======================================

void macpong_initSend(opentimer_id_t id);
void macpong_send(uint8_t payloadCtr);

//=========================== initialization ==================================

int mote_main(void) {
   board_init();
   scheduler_init();
   openstack_init();
   if (idmanager_getMyID(ADDR_64B)->addr_64b[7]==0x62) {
       idmanager_setIsDAGroot(TRUE);

       open_addr_t     temp_neighbor;
       memset(&temp_neighbor,0,sizeof(temp_neighbor));
       temp_neighbor.type             = ADDR_16B;
       temp_neighbor.addr_16b[0]      = 0x99;
       temp_neighbor.addr_16b[1]      = 0x83;

       schedule_addActiveSlot(SCHEDULE_MINIMAL_6TISCH_SLOTOFFSET + SCHEDULE_MINIMAL_6TISCH_ACTIVE_CELLS + 1,
               CELLTYPE_TX,
               FALSE,
               SCHEDULE_MINIMAL_6TISCH_CHANNELOFFSET,
               &temp_neighbor
               );
       schedule_addActiveSlot(SCHEDULE_MINIMAL_6TISCH_SLOTOFFSET + SCHEDULE_MINIMAL_6TISCH_ACTIVE_CELLS + 2,
               CELLTYPE_RX,
               FALSE,
               SCHEDULE_MINIMAL_6TISCH_CHANNELOFFSET,
               &temp_neighbor
              );
   }
   else if ((idmanager_getMyID(ADDR_64B)->addr_64b[6]==0x99) && (idmanager_getMyID(ADDR_64B)->addr_64b[7]==0x83)) {
       open_addr_t     temp_neighbor;
       memset(&temp_neighbor,0,sizeof(temp_neighbor));
       temp_neighbor.type             = ADDR_16B;
       temp_neighbor.addr_16b[0]      = 0x21;
       temp_neighbor.addr_16b[1]      = 0x62;

       schedule_addActiveSlot(SCHEDULE_MINIMAL_6TISCH_SLOTOFFSET + SCHEDULE_MINIMAL_6TISCH_ACTIVE_CELLS + 1,
               CELLTYPE_RX,
               FALSE,
               SCHEDULE_MINIMAL_6TISCH_CHANNELOFFSET,
               &temp_neighbor
               );
       schedule_addActiveSlot(SCHEDULE_MINIMAL_6TISCH_SLOTOFFSET + SCHEDULE_MINIMAL_6TISCH_ACTIVE_CELLS + 2,
               CELLTYPE_TX,
               FALSE,
               SCHEDULE_MINIMAL_6TISCH_CHANNELOFFSET,
               &temp_neighbor
               );
   }

   scheduler_start();
   return 0; // this line should never be reached
}

void macpong_initSend(opentimer_id_t id) {
    if (idmanager_getIsDAGroot()==TRUE) {
        return;
    }
    if (ieee154e_isSynch()==TRUE && neighbors_getNumNeighbors()==1) {
        // send packet
        //poipoimacpong_send(0);   
        // cancel timer
        opentimers_stop(macpong_vars.timerId);
    }
}

void macpong_send(uint8_t payloadCtr) {
    OpenQueueEntry_t* pkt;
    uint8_t i;

    pkt = openqueue_getFreePacketBuffer(COMPONENT_UECHO);
    if (pkt==NULL) {
        openserial_printError(
                COMPONENT_IPHC,
                ERR_NO_FREE_PACKET_BUFFER,
                (errorparameter_t)0,
                (errorparameter_t)0
                );
        return;
    }
    pkt->creator                   = COMPONENT_IPHC;
    pkt->owner                     = COMPONENT_IPHC;

    neighbors_getNeighbor(&pkt->l2_nextORpreviousHop,ADDR_16B,0);
    packetfunctions_reserveHeaderSize(pkt,LEN_PAYLOAD);
    ((uint8_t*)pkt->payload)[0]    = payloadCtr;
    for (i=1;i<LEN_PAYLOAD;i++){
        ((uint8_t*)pkt->payload)[i]  = i;
    }
    sixtop_send(pkt);
}

//=========================== stubbing ========================================

//===== IPHC

void iphc_init(void) {
    macpong_vars.timerId    = opentimers_start(
            5000,
            TIMER_PERIODIC,TIME_MS,
            macpong_initSend
            );
}

void iphc_sendDone(OpenQueueEntry_t* msg, owerror_t error) {
    msg->owner = COMPONENT_IPHC;
    openqueue_freePacketBuffer(msg);
}

void iphc_receive(OpenQueueEntry_t* msg) {
    msg->owner = COMPONENT_IPHC;
    macpong_send(++msg->payload[0]);
    openqueue_freePacketBuffer(msg);
}

//===== L3

void forwarding_init(void)        { return; }
void openbridge_init(void)        { return; }
void openbridge_triggerData(void) { return; }

//===== L4

void icmpv6_init(void)            { return; }

void icmpv6echo_init(void)        { return; }
void icmpv6echo_trigger(void)     { return; }

void icmpv6router_init(void)      { return; }
void icmpv6router_trigger(void)   { return; }

void icmpv6rpl_init(void)         { return; }
void icmpv6rpl_trigger(void)      { return; }
void icmpv6rpl_writeDODAGid(uint8_t* dodagid) { return; }

void opentcp_init(void)           { return; }

void openudp_init(void)           { return; }

void opencoap_init(void)          { return; }

//===== L7

void openapps_init(void)          { return; }

void ohlone_init(void)            { return; }

void tcpecho_init(void)           { return; }

void tcpinject_init(void)         { return; }
void tcpinject_trigger(void)      { return; }

void tcpprint_init(void)          { return; }

void c6t_init(void)               { return; }
void cinfo_init(void)             { return; }
void cleds__init(void)            { return; }
void cwellknown_init(void)        { return; }
// TCP
void techo_init(void)             { return; }
// UDP
void uecho_init(void)             { return; }

