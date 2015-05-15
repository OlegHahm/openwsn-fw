/**
\brief A simple ICN like application over ICN.

\author Oliver Hahm <oliver.hahm@inria.fr>, May 2015.
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

char *interest = "/ndn/RIOT/sensor";

char *content = "Go! are you ready? \
                 Start the riot! start the riot! start the riot! start the riot now!";

extern neighbors_vars_t neighbors_vars;

//=========================== variables =======================================

typedef enum {
    ICN_INTEREST    = 1,
    ICN_CONTENT     = 2,
} icn_packet_type_t;

typedef struct {
   opentimer_id_t  timerId;
} icn_vars_t;

typedef struct {
    open_addr_t *sender;
    open_addr_t *receiver;
} icn_link_t;

typedef struct {
    open_addr_t *id;
    open_addr_t *dst;
    open_addr_t *nextHop;
} icn_routing_entry_t;

typedef struct {
    icn_packet_type_t type;
} icn_hdr_t;

icn_vars_t icn_vars;

open_addr_t *myId;

open_addr_t pit_entry = { .type = ADDR_NONE};

unsigned slot0isActive = 1;
uint16_t send_counter = 0;

//=========================== prototypes ======================================

void icn_initInterest(opentimer_id_t id);
void icn_send(open_addr_t *dst, OpenQueueEntry_t *pkt);
void icn_addToFixedSchedule(open_addr_t *addr, int16_t rx_cell, int16_t tx_cell, int16_t shared);
unsigned _linkIsScheduled(open_addr_t *dst);
open_addr_t* _routeLookup(open_addr_t *dst);

//=========================== initialization ==================================

#define ADDR_LEN_64B    (sizeof(uint8_t) + 8)

#define CONTENT_STORE   (&node_ids[0])
#define HAS_CONTENT     (memcmp(myId, CONTENT_STORE, ADDR_LEN_64B) == 0)
#define WANT_CONTENT    (memcmp(myId, &(node_ids[2]), ADDR_LEN_64B) == 0)

#ifdef SIMU
#warning SIMULATION build
#define NUMBER_OF_NODES     (5)
#define NODE_01     {.type = ADDR_64B, .addr_64b = {0x14, 0x15, 0x92, 0xcc, 0x00, 0x00, 0x00, 0x01}}
#define NODE_02     {.type = ADDR_64B, .addr_64b = {0x14, 0x15, 0x92, 0xcc, 0x00, 0x00, 0x00, 0x02}}
#define NODE_03     {.type = ADDR_64B, .addr_64b = {0x14, 0x15, 0x92, 0xcc, 0x00, 0x00, 0x00, 0x03}}
#define NODE_04     {.type = ADDR_64B, .addr_64b = {0x14, 0x15, 0x92, 0xcc, 0x00, 0x00, 0x00, 0x04}}
#define NODE_05     {.type = ADDR_64B, .addr_64b = {0x05, 0x15, 0x92, 0xcc, 0x00, 0x00, 0x00, 0x05}}

open_addr_t node_ids[NUMBER_OF_NODES] = {
    NODE_01,
    NODE_02,
    NODE_03,
    NODE_04,
    NODE_05,
};


#define SSF_INT_SIZE   (NUMBER_OF_NODES + (7*2)) // NUMBER_OF_NODES + (NUMBER_OF_LINKS * 2)

icn_link_t  ssf_int[SSF_INT_SIZE] = {
    /* broadcast cell for NODE_01 */
    {&(node_ids[0]), NULL},
    /* link from 01 to 02 */
    {&(node_ids[1]), &(node_ids[0])},
    {&(node_ids[0]), &(node_ids[1])},
    /* link from 01 to 04 */
    {&(node_ids[3]), &(node_ids[0])},
    {&(node_ids[0]), &(node_ids[3])},

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

#define RRT_SIZE        (6)
icn_routing_entry_t routing_table[RRT_SIZE] = {
    /* default route for 02 over 01 */
    {&(node_ids[1]), NULL, &(node_ids[0])},
    /* default route for 03 over 02 */
    {&(node_ids[2]), NULL, &(node_ids[1])},
    /* default route for 04 over 01 */
    {&(node_ids[3]), NULL, &(node_ids[0])},
    /* default route for 05 over 02 */
    {&(node_ids[4]), NULL, &(node_ids[1])},

    /* route for 01 to 03 over 02 */
    {&(node_ids[0]), &(node_ids[2]), &(node_ids[1])},
    /* route for 01 to 05 over 02 */
    {&(node_ids[0]), &(node_ids[4]), &(node_ids[1])},
};


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
//#define NODE_04     {.type = ADDR_64B, .addr_64b = {0x05, 0x43, 0x32, 0xff, 0x02, 0xd7, 0x38, 0x60}} // m3-222
//#define NODE_05     {.type = ADDR_64B, .addr_64b = {0x05, 0x43, 0x32, 0xff, 0x03, 0xd9, 0xb3, 0x83}} // m3-248
#define NODE_04 	{.type = ADDR_64B, .addr_64b = {0x05, 0x43, 0x32, 0xff, 0x03, 0xd7, 0xb1, 0x80}} // m3-224
#define NODE_05 	{.type = ADDR_64B, .addr_64b = {0x05, 0x43, 0x32, 0xff, 0x03, 0xde, 0xb8, 0x81}} // m3-246
#define NODE_06 	{.type = ADDR_64B, .addr_64b = {0x05, 0x43, 0x32, 0xff, 0x02, 0xdc, 0x28, 0x61}} // m3-272
#define NODE_07 	{.type = ADDR_64B, .addr_64b = {0x05, 0x43, 0x32, 0xff, 0x03, 0xda, 0x98, 0x75}} // m3-284
#define NODE_08 	{.type = ADDR_64B, .addr_64b = {0x05, 0x43, 0x32, 0xff, 0x03, 0xdc, 0xa6, 0x82}} // m3-6

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

#define SSF_INT_SIZE    (5 + (7*2) + 4) // NUMBER_OF_NODES + (NUMBER_OF_LINKS * 2)
#define SSF_cs_SIZE     ((7*2) + 4) // (NUMBER_OF_LINKS * 2)

icn_link_t ssf_int[SSF_INT_SIZE] = {
    /* broadcast cell for NODE_05 */
    {&(node_ids[4]), NULL},

    /* broadcast cell for NODE_04 */
    {&(node_ids[3]), NULL},
    /* link from 04 to 05 */
    {&(node_ids[4]), &(node_ids[3])},

    /* broadcast cell for NODE_03 */
    {&(node_ids[2]), NULL},
    /* link from 02 to 05 */
    {&(node_ids[4]), &(node_ids[2])},
    {&(node_ids[2]), &(node_ids[4])},

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

   {&(node_ids[3]), &(node_ids[4])},

    /* broadcast cell for NODE_01 */
    {&(node_ids[0]), NULL},
    /* link from 01 to 02 */
    {&(node_ids[1]), &(node_ids[0])},
    {&(node_ids[0]), &(node_ids[1])},
    {&(node_ids[1]), &(node_ids[0])},
    {&(node_ids[0]), &(node_ids[1])},
    {&(node_ids[1]), &(node_ids[0])},
    {&(node_ids[0]), &(node_ids[1])},
    /* link from 01 to 04 */
    {&(node_ids[3]), &(node_ids[0])},
    {&(node_ids[0]), &(node_ids[3])},
};

icn_link_t ssf_cs[SSF_INT_SIZE] = {
    /* link from 01 to 02 */
    {&(node_ids[1]), &(node_ids[0])},
    {&(node_ids[0]), &(node_ids[1])},
    {&(node_ids[1]), &(node_ids[0])},
    {&(node_ids[0]), &(node_ids[1])},
    {&(node_ids[1]), &(node_ids[0])},
    {&(node_ids[0]), &(node_ids[1])},

    /* link from 01 to 04 */
    {&(node_ids[3]), &(node_ids[0])},
    {&(node_ids[0]), &(node_ids[3])},

    /* link from 02 to 03 */
    {&(node_ids[2]), &(node_ids[1])},
    {&(node_ids[1]), &(node_ids[2])},
    /* link from 02 to 04 */
    {&(node_ids[3]), &(node_ids[1])},
    {&(node_ids[1]), &(node_ids[3])},
    /* link from 02 to 05 */
    {&(node_ids[4]), &(node_ids[1])},
    {&(node_ids[1]), &(node_ids[4])},

    /* link from 03 to 05 */
    {&(node_ids[4]), &(node_ids[2])},
    {&(node_ids[2]), &(node_ids[4])},

    /* link from 04 to 05 */
    {&(node_ids[4]), &(node_ids[3])},
    {&(node_ids[3]), &(node_ids[4])},
 };

#define RRT_SIZE        (6)
icn_routing_entry_t routing_table[RRT_SIZE] = {
    /* default route for 02 over 01 */
    {&(node_ids[1]), NULL, &(node_ids[0])},
    /* default route for 03 over 02 */
    {&(node_ids[2]), NULL, &(node_ids[1])},
    /* default route for 04 over 01 */
    {&(node_ids[3]), NULL, &(node_ids[0])},
    /* default route for 05 over 02 */
    {&(node_ids[4]), NULL, &(node_ids[1])},

    /* route for 01 to 03 over 02 */
    {&(node_ids[0]), &(node_ids[2]), &(node_ids[1])},
    /* route for 01 to 05 over 02 */
    {&(node_ids[0]), &(node_ids[4]), &(node_ids[1])},
};

#endif

int mote_main(void) {
   board_init();
   scheduler_init();
   openstack_init();

   myId = idmanager_getMyID(ADDR_64B);

   /* check for root node */
   if (memcmp(myId, &(node_ids[0]), ADDR_LEN_64B) == 0) {
       idmanager_setIsDAGroot(TRUE);
   }
   else {
       neighbors_vars.myDAGrank = 44;
   }
   /* iterate over the full schedule and make my reservations*/
   for (int i = 0; i < SSF_INT_SIZE; i++) {
       /*  if I am the sender... */
       if (memcmp(myId, ssf_int[i].sender, ADDR_LEN_64B) == 0) {
           /* without a particular receiver, schedule a shared cell */
           if (ssf_int[i].receiver == NULL) {
               icn_addToFixedSchedule(NULL, -1, -1, i+NUMSERIALRX);
           }
           /* or for this particular receiver */
           else {
               icn_addToFixedSchedule(ssf_int[i].receiver, -1, i+NUMSERIALRX, -1);
           }
       }
       /* if I am the receiver or another node has shared cell, schedule an RX cell */
       else if ((memcmp(myId, ssf_int[i].receiver, ADDR_LEN_64B) == 0) ||
               (ssf_int[i].receiver == NULL)) {
           {
               icn_addToFixedSchedule(ssf_int[i].sender, i+NUMSERIALRX, -1, -1);
           }
       }
   }

   scheduler_start();
   return 0; // this line should never be reached
}

unsigned _linkIsScheduled(open_addr_t *dst) {
   for (int i = 0; i < SSF_INT_SIZE; i++) {
       if ((memcmp(myId, ssf_int[i].sender, ADDR_LEN_64B) == 0) && 
               (memcmp(dst, ssf_int[i].receiver, ADDR_LEN_64B) == 0)) {
           return 1;
       }
   }
   return 0;
}

open_addr_t* _routeLookup(open_addr_t *dst) {
    open_addr_t *next = NULL;
    /* iterate over routing table  */
   for (int i = 0; i < RRT_SIZE; i++) {
       /* find entries for this node */
       if (memcmp(myId, routing_table[i].id, ADDR_LEN_64B) == 0) {
           /* dedicated route found, use it */
           if (memcmp(dst, routing_table[i].dst, ADDR_LEN_64B) == 0) {
               return routing_table[i].nextHop;
           }
           /* default route found, remember */
           if (routing_table[i].dst == NULL) {
               next = routing_table[i].nextHop;
           }
       }
   }
   /* return either default route or NULL */
   return next;
}

void icn_addToFixedSchedule(open_addr_t *addr, int16_t rx_cell, int16_t tx_cell, int16_t shared) {
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

void icn_initInterest(opentimer_id_t id) {
    if (!ieee154e_isSynch()) {
        if (!slot0isActive) {
            icn_addToFixedSchedule(&(node_ids[0]), -1, -1, 0);
            slot0isActive = 1;
        }
        return;
    }
    if ((neighbors_getNumNeighbors()>=1) && WANT_CONTENT) {
        /* create packet */
        OpenQueueEntry_t* pkt;
        pkt = openqueue_getFreePacketBuffer(COMPONENT_ICN);
        if (pkt==NULL) {
            openserial_printError(COMPONENT_ICN, ERR_NO_FREE_PACKET_BUFFER,
                    (errorparameter_t)0, (errorparameter_t)0);
            return;
        }

        pkt->creator                   = COMPONENT_ICN;
        pkt->owner                     = COMPONENT_ICN;
        pkt->l2_nextORpreviousHop.type = ADDR_64B;

        // send interest packet
        openserial_printInfo(COMPONENT_ICN, ERR_ICN_SEND,
                send_counter, (errorparameter_t) node_ids[0].addr_64b[7]);
        packetfunctions_reserveHeaderSize(pkt, sizeof(icn_hdr_t) + strlen(interest) + 1);
        ((icn_hdr_t*)pkt->payload)->type = ICN_INTEREST;
        memcpy((pkt->payload + sizeof(icn_hdr_t)), interest, strlen(interest) + 1);

        icn_send(CONTENT_STORE, pkt);
    }

    if (!idmanager_getIsDAGroot() && slot0isActive) {
        /*  remove default slot  */
        open_addr_t tmp;
        memset(&tmp,0,ADDR_LEN_64B);
        tmp.type             = ADDR_ANYCAST;
        schedule_removeActiveSlot(0, &tmp);
        slot0isActive = 0;
        icn_addToFixedSchedule(&(node_ids[0]), 0, -1, -1);
    }
}

void icn_send(open_addr_t *dst, OpenQueueEntry_t *pkt) {
    /* find next hop */
    if (!_linkIsScheduled(dst)) {
        open_addr_t *tmp;
        tmp = _routeLookup(dst);
        if (tmp == NULL) {
            openserial_printError(COMPONENT_ICN, ERR_NO_NEXTHOP,
                    (errorparameter_t) dst->addr_64b[6],
                    (errorparameter_t) dst->addr_64b[7]);
            openqueue_freePacketBuffer(pkt);
            return;
        }
        dst = tmp;
    }

    memcpy(&pkt->l2_nextORpreviousHop, dst, ADDR_LEN_64B);

    sixtop_send(pkt);
}

//=========================== stubbing ========================================

//===== IPHC

void iphc_init(void) {
    icn_vars.timerId    = opentimers_start(
            5000,
            TIMER_PERIODIC,TIME_MS,
            icn_initInterest
            );
}

void iphc_sendDone(OpenQueueEntry_t* msg, owerror_t error) {
    msg->owner = COMPONENT_ICN;
    //openserial_printInfo(COMPONENT_ICN, ERR_ICN_SEND, 2, (errorparameter_t) msg->l2_nextORpreviousHop.addr_64b[7]);
    openqueue_freePacketBuffer(msg);
}

void iphc_receive(OpenQueueEntry_t* msg) {
    msg->owner = COMPONENT_ICN;
    if (HAS_CONTENT) {
        openserial_printInfo(COMPONENT_ICN, ERR_ICN_RECV1,
                (errorparameter_t) msg->l2_nextORpreviousHop.addr_64b[6],
                (errorparameter_t) msg->l2_nextORpreviousHop.addr_64b[7]);
        openserial_printInfo(COMPONENT_ICN, ERR_ICN_RECV2,
                (errorparameter_t) msg->payload[0],
                (errorparameter_t) msg->payload[1]);
            msg->creator = COMPONENT_ICN;
            icn_send(CONTENT_STORE, msg);
        openqueue_freePacketBuffer(msg);
    }
    else {
        /*
        openserial_printInfo(COMPONENT_ICN, ERR_ICN_FWD1,
                (errorparameter_t) msg->l2_nextORpreviousHop.addr_64b[6],
                (errorparameter_t) msg->l2_nextORpreviousHop.addr_64b[7]);
        openserial_printInfo(COMPONENT_ICN, ERR_ICN_FWD2,
                (errorparameter_t) node_ids[0].addr_64b[6],
                (errorparameter_t) node_ids[0].addr_64b[7]);
        */
        icn_hdr_t *icn_pkt = (icn_hdr_t*) msg->payload;
        if (icn_pkt->type == ICN_INTEREST) {
            memcpy(&pit_entry, &msg->l2_nextORpreviousHop, sizeof(open_addr_t));
            /* forward to CS node */
            msg->creator = COMPONENT_ICN;
            icn_send(CONTENT_STORE, msg);
        }
        else if (icn_pkt->type == ICN_CONTENT) {
            if (pit_entry.type != ADDR_NONE) {
                msg->creator = COMPONENT_ICN;
                icn_send(&pit_entry, msg);
                pit_entry.type = ADDR_NONE;
            }
            else {
                openserial_printError(COMPONENT_ICN, ERR_ICN_FWD_NOT_FOUND,
                        (errorparameter_t) 47,
                        (errorparameter_t) 11);
                openqueue_freePacketBuffer(msg);
            }
        }
        else {
            openserial_printError(COMPONENT_ICN, ERR_MSG_UNKNOWN_TYPE,
                    (errorparameter_t) icn_pkt->type,
                    (errorparameter_t) icn_pkt->type);
            openqueue_freePacketBuffer(msg);
        }
    }
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
