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

open_addr_t *myId;

//=========================== prototypes ======================================

void macpong_initSend(opentimer_id_t id);
void macpong_send(uint8_t payloadCtr, open_addr_t *dst);
void macpong_addToFixedSchedule(uint8_t id[], uint8_t rx_cell, uint8_t tx_cell);

static inline unsigned _getNodeNumber(void);

//=========================== initialization ==================================

#ifdef SIMU
#warning SIMULATION build
#define NUMBER_OF_NODES (1)
uint8_t rootId[4] = {0, 0, 0, 1};
uint8_t node_ids[NUMBER_OF_NODES][4] = {{0, 0, 0, 2}};
#else
/*
#define NUMBER_OF_NODES (8)
uint8_t rootId[4] = {0x03, 0xdb, 0xa4, 0x83};
uint8_t node_ids[NUMBER_OF_NODES][4] = {
   // {0x03, 0xdb, 0x99, 0x83},
	{0x03, 0xd5, 0x98, 0x76},
	{0x03, 0xd9, 0xc2, 0x77},
	{0x03, 0xd8, 0x89, 0x82},
	{0x03, 0xd6, 0xb1, 0x68},
	{0x02, 0xd7, 0x16, 0x60},
	{0x03, 0xd7, 0xb6, 0x68},
	{0x02, 0xdb, 0x18, 0x60},
	{0x03, 0xda, 0xa5, 0x80},
};
*/
#define NUMBER_OF_NODES (2)
uint8_t rootId[4] = {0x03, 0xda, 0xc0, 0x81};
uint8_t node_ids[NUMBER_OF_NODES][4] = {
	{0x02, 0xdb, 0x37, 0x61},
	{0x03, 0xdd, 0xb3, 0x80},
    };
#endif

int mote_main(void) {
   board_init();
   scheduler_init();
   openstack_init();

   myId = idmanager_getMyID(ADDR_64B);

   if ((myId->addr_64b[4] == rootId[0]) && (myId->addr_64b[5] == rootId[1]) &&
           (myId->addr_64b[6] == rootId[2]) && (myId->addr_64b[7] == rootId[3])) {
       idmanager_setIsDAGroot(TRUE);
       for (int i = 0; i < NUMBER_OF_NODES; i++) {
           macpong_addToFixedSchedule(node_ids[i], 2*i+1, 2*i+2);
       }
   }
   else {
       extern neighbors_vars_t neighbors_vars;
       neighbors_vars.myDAGrank = 1;
       for (int i = 0; i < NUMBER_OF_NODES; i++) {
           if ((myId->addr_64b[4] == node_ids[i][0]) && (myId->addr_64b[5] == node_ids[i][1]) &&
                   (myId->addr_64b[6] == node_ids[i][2]) && (myId->addr_64b[7] == node_ids[i][3])) {
               if (i == 0) {
                   macpong_addToFixedSchedule(rootId, 2*i+2, 2*i+1);
                   macpong_addToFixedSchedule(node_ids[i+1], 2*i+4, 2*i+3);
               }
               else {
                   macpong_addToFixedSchedule(node_ids[i-1], 2*(i-1)+3, 2*(i-1)+4);
               }
           }
       }
   }
   scheduler_start();
   return 0; // this line should never be reached
}

static inline unsigned _getNodeNumber(void) {
    for (int i = 0; i < NUMBER_OF_NODES; i++) {
        if ((myId->addr_64b[4] == node_ids[i][0]) && (myId->addr_64b[5] == node_ids[i][1]) &&
                (myId->addr_64b[6] == node_ids[i][2]) && (myId->addr_64b[7] == node_ids[i][3])) {
            return i;
        }
    }
    return 0;
}

void macpong_setPrefix(open_addr_t *addr) {
    open_addr_t *myID;
    memset(addr, 0, sizeof(open_addr_t));
    myID = idmanager_getMyID(ADDR_64B);
    memcpy(&(addr->addr_64b), myID->addr_64b, 4);
}

void macpong_addToFixedSchedule(uint8_t id[], uint8_t rx_cell, uint8_t tx_cell) {
    open_addr_t     temp_neighbor;

    macpong_setPrefix(&temp_neighbor);
    temp_neighbor.type             = ADDR_64B;
    memcpy(&(temp_neighbor.addr_64b[4]), id, 4);

    schedule_addActiveSlot(SCHEDULE_MINIMAL_6TISCH_SLOTOFFSET + SCHEDULE_MINIMAL_6TISCH_ACTIVE_CELLS + tx_cell,
            CELLTYPE_TX,
            FALSE,
            SCHEDULE_MINIMAL_6TISCH_CHANNELOFFSET,
            &temp_neighbor
            );
    schedule_addActiveSlot(SCHEDULE_MINIMAL_6TISCH_SLOTOFFSET + SCHEDULE_MINIMAL_6TISCH_ACTIVE_CELLS + rx_cell,
            CELLTYPE_RX,
            FALSE,
            SCHEDULE_MINIMAL_6TISCH_CHANNELOFFSET,
            &temp_neighbor
            );
}

void macpong_initSend(opentimer_id_t id) {
    if (idmanager_getIsDAGroot()==TRUE) {
        return;
    }

   if (ieee154e_isSynch()==TRUE && neighbors_getNumNeighbors()>=1) {
       open_addr_t temp_neighbor;
       macpong_setPrefix(&temp_neighbor);
       temp_neighbor.type             = ADDR_64B;

       unsigned myNumber = _getNodeNumber();
       openserial_printError(COMPONENT_ICN,
               ERR_DEBUG1,
               (errorparameter_t) myNumber,
               (errorparameter_t) id);

       if (myNumber == 1) {
           memcpy(&(temp_neighbor.addr_64b[4]), rootId, 4);
       }
       else {
           memcpy(&(temp_neighbor.addr_64b[4]), node_ids[myNumber-1], 4);
       }
       // send packet
       macpong_send(0, &temp_neighbor);
       // cancel timer
       //opentimers_stop(macpong_vars.timerId);
   }
}

void macpong_send(uint8_t payloadCtr, open_addr_t *dst) {
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
    pkt->l2_nextORpreviousHop.type = ADDR_64B;

    memcpy(&pkt->l2_nextORpreviousHop, dst, sizeof(open_addr_t));

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
    if (idmanager_getIsDAGroot()==TRUE) {
        macpong_send(++msg->payload[0], &(msg->l2_nextORpreviousHop));
    }
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

