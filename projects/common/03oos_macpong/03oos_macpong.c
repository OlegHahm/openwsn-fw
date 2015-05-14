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

extern neighbors_vars_t neighbors_vars;

//=========================== variables =======================================

typedef struct {
   opentimer_id_t  timerId;
} macpong_vars_t;

typedef struct {
    open_addr_t *sender;
    open_addr_t *receiver;
} macpong_scheduled_link_t;

macpong_vars_t macpong_vars;

open_addr_t *myId;

unsigned slot0isActive = 1;

//=========================== prototypes ======================================

void macpong_initSend(opentimer_id_t id);
void macpong_send(uint8_t payloadCtr, open_addr_t *dst);
void macpong_addToFixedSchedule(open_addr_t *addr, int16_t rx_cell, int16_t tx_cell, int16_t shared);

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
#define NUMBER_OF_NODES     (5)

//#define NODE_01     {.type = ADDR_64B, .addr_64b = {0x05, 0x43, 0x32, 0xff, 0x03, 0xdb, 0xa3, 0x78}} // m3-210
#define NODE_01     {.type = ADDR_64B, .addr_64b = {0x05, 0x43, 0x32, 0xff, 0x03, 0xda, 0xc0, 0x81}} // m3-210
#define NODE_02     {.type = ADDR_64B, .addr_64b = {0x05, 0x43, 0x32, 0xff, 0x03, 0xdd, 0x94, 0x81}} // m3-234
#define NODE_03     {.type = ADDR_64B, .addr_64b = {0x05, 0x43, 0x32, 0xff, 0x03, 0xdb, 0xa4, 0x78}} // m3-260
#define NODE_04     {.type = ADDR_64B, .addr_64b = {0x05, 0x43, 0x32, 0xff, 0x02, 0xd7, 0x38, 60}} // m3-222
#define NODE_05     {.type = ADDR_64B, .addr_64b = {0x05, 0x43, 0x32, 0xff, 0x03, 0xd9, 0xb3, 83}} // m3-248

//	{0x02, 0xdb, 0x37, 0x61}, // m3-250
//	{0x03, 0xdd, 0xb3, 0x80}, // m3-270
//
open_addr_t node_ids[NUMBER_OF_NODES] = {
    NODE_01,
    NODE_02,
    NODE_03,
    NODE_04,
    NODE_05
};

#define SCHEDULE_SIZE   (5 + (7*2)) // NUMBER_OF_NODES + (NUMBER_OF_LINKS * 2)

macpong_scheduled_link_t  mySchedule[SCHEDULE_SIZE] = {
    /* broadcast cell for NODE_01 */
    {&(node_ids[0]), NULL},
    /* link from 01 to 02 */
    {&(node_ids[1]), &(node_ids[0])},
    {&(node_ids[0]), &(node_ids[1])},
    /* link from 01 to 03 */
    {&(node_ids[2]), &(node_ids[0])},
    {&(node_ids[0]), &(node_ids[2])},

    /* broadcast cell for NODE_02 */
    {&(node_ids[1]), NULL},
    /* link from 02 to 03 */
    {&(node_ids[2]), &(node_ids[1])},
    {&(node_ids[1]), &(node_ids[2])},
    /* link from 02 to 04 */
    {&(node_ids[3]), &(node_ids[1])},
    {&(node_ids[1]), &(node_ids[3])},
    /* link from 02 to 05 */
    {&(node_ids[4]), &(node_ids[1])},
    {&(node_ids[1]), &(node_ids[4])},

    /* broadcast cell for NODE_03 */
    {&(node_ids[2]), NULL},
    /* link from 02 to 05 */
    {&(node_ids[4]), &(node_ids[2])},
    {&(node_ids[2]), &(node_ids[4])},

    /* broadcast cell for NODE_04 */
    {&(node_ids[3]), NULL},
    /* link from 04 to 05 */
    {&(node_ids[4]), &(node_ids[3])},
    {&(node_ids[3]), &(node_ids[4])},
    
    /* broadcast cell for NODE_05 */
    {&(node_ids[4]), NULL},
};
#endif

int mote_main(void) {
   board_init();
   scheduler_init();
   openstack_init();

   myId = idmanager_getMyID(ADDR_64B);

   /* check for root node */
   if (memcmp(myId, &(node_ids[0]), sizeof(open_addr_t)) == 0) {
       idmanager_setIsDAGroot(TRUE);
   }
   else {
       neighbors_vars.myDAGrank = 44;
   }
   /* iterate over the full schedule and make my reservations*/
   for (int i = 0; i < SCHEDULE_SIZE; i++) {
       /*  if I am the sender... */
       if (memcmp(myId, mySchedule[i].sender, sizeof(open_addr_t)) == 0) {
           /* without a particular receiver, schedule a shared cell */
           if (mySchedule[i].receiver == NULL) {
               macpong_addToFixedSchedule(NULL, -1, -1, i+NUMSERIALRX);
           }
           /* or for this particular receiver */
           else {
               macpong_addToFixedSchedule(mySchedule[i].receiver, -1, i+NUMSERIALRX, -1);
           }
       }
       /* if I am the receiver or another node has shared cell, schedule an RX cell */
       else if ((memcmp(myId, mySchedule[i].receiver, sizeof(open_addr_t)) == 0) ||
               (mySchedule[i].receiver == NULL)) {
           {
               macpong_addToFixedSchedule(mySchedule[i].sender, i+NUMSERIALRX, -1, -1);
           }
       }
   }

   scheduler_start();
   return 0; // this line should never be reached
}

void macpong_addToFixedSchedule(open_addr_t *addr, int16_t rx_cell, int16_t tx_cell, int16_t shared) {
    if (tx_cell >= 0) {
        schedule_addActiveSlot(SCHEDULE_MINIMAL_6TISCH_SLOTOFFSET + SCHEDULE_MINIMAL_6TISCH_ACTIVE_CELLS + tx_cell,
                CELLTYPE_TX,
                FALSE,
                SCHEDULE_MINIMAL_6TISCH_CHANNELOFFSET,
                addr 
                );
    }
    if (rx_cell >= 0) {
        if (rx_cell == 0) {
            rx_cell = -1;
        }
        schedule_addActiveSlot(SCHEDULE_MINIMAL_6TISCH_SLOTOFFSET + SCHEDULE_MINIMAL_6TISCH_ACTIVE_CELLS + rx_cell,
                CELLTYPE_RX,
                FALSE,
                SCHEDULE_MINIMAL_6TISCH_CHANNELOFFSET,
                addr 
                );
    }
    if (shared >= 0) {
        open_addr_t     temp_neighbor;
        memset(&temp_neighbor,0,sizeof(temp_neighbor));
        temp_neighbor.type             = ADDR_ANYCAST;
        schedule_addActiveSlot(SCHEDULE_MINIMAL_6TISCH_SLOTOFFSET + SCHEDULE_MINIMAL_6TISCH_ACTIVE_CELLS + shared,
                CELLTYPE_TXRX,
                TRUE,
                SCHEDULE_MINIMAL_6TISCH_CHANNELOFFSET,
                &temp_neighbor
                );
    }
}

void macpong_initSend(opentimer_id_t id) {
    if (idmanager_getIsDAGroot()==TRUE) {
        return;
    }

   if (ieee154e_isSynch()==TRUE && neighbors_getNumNeighbors()>=1) {
       open_addr_t dst;
       neighbors_getNeighbor(&dst, ADDR_64B, 0);

       // send packet
       macpong_send(0, &dst);
       if (slot0isActive) {
           /*  remove default slot  */
           memset(&dst,0,sizeof(dst));
           dst.type             = ADDR_ANYCAST;
           schedule_removeActiveSlot(0, &dst);
           slot0isActive = 0;
           macpong_addToFixedSchedule(&(node_ids[0]), 0, -1, -1);
       }
       // cancel timer
       //opentimers_stop(macpong_vars.timerId);
   }
   else {
       if (!slot0isActive) {
           macpong_addToFixedSchedule(&(node_ids[0]), -1, -1, 0);
           slot0isActive = 1;
       }
   }
}

void macpong_send(uint8_t payloadCtr, open_addr_t *dst) {
    OpenQueueEntry_t* pkt;
    uint8_t i;

    pkt = openqueue_getFreePacketBuffer(COMPONENT_ICN);
    if (pkt==NULL) {
        openserial_printError(COMPONENT_ICN, ERR_NO_FREE_PACKET_BUFFER,
                (errorparameter_t)0, (errorparameter_t)0);
        return;
    }
    pkt->creator                   = COMPONENT_ICN;
    pkt->owner                     = COMPONENT_ICN;
    pkt->l2_nextORpreviousHop.type = ADDR_64B;

    memcpy(&pkt->l2_nextORpreviousHop, dst, sizeof(open_addr_t));

    openserial_printInfo(COMPONENT_ICN, ERR_ICN_SEND,
                          1, (errorparameter_t) dst->addr_64b[7]);

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
    msg->owner = COMPONENT_ICN;
    openserial_printInfo(COMPONENT_ICN, ERR_ICN_SEND,
                          2, (errorparameter_t) msg->l2_nextORpreviousHop.addr_64b[7]);
    openqueue_freePacketBuffer(msg);
}

void iphc_receive(OpenQueueEntry_t* msg) {
    msg->owner = COMPONENT_ICN;
    openserial_printInfo(COMPONENT_ICN, ERR_ICN_RECV,
                          1, (errorparameter_t) msg->l2_nextORpreviousHop.addr_64b[7]);
    if (idmanager_getIsDAGroot()==TRUE) {
        openserial_printInfo(COMPONENT_ICN, ERR_ICN_SEND,
                44, (errorparameter_t) msg->l2_nextORpreviousHop.addr_64b[7]);
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

void HardFault_Handler(void) {
    openserial_printCritical(COMPONENT_NULL, ERR_FAULT_HANDLER,
            0, 0);
    while(1)
    {
    }
}
void NMI_Handler(void) {
    openserial_printCritical(COMPONENT_NULL, ERR_FAULT_HANDLER,
            1, 1);
    while(1)
    {
    }
}
void MemManage_Handler(void) {
    openserial_printCritical(COMPONENT_NULL, ERR_FAULT_HANDLER,
            2, 2);
    while(1)
    {
    }
}
void BusFault_Handler(void) {
    openserial_printCritical(COMPONENT_NULL, ERR_FAULT_HANDLER,
            3, 3);
    while(1)
    {
    }
}
