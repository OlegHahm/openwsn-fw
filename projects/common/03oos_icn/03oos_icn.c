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
#include "bitfield.h"

char *interest = "/ndn/RIOT/sensor";

char *content = "Go! are you ready? Start the riot!";

extern neighbors_vars_t neighbors_vars;

//=========================== variables =======================================

typedef enum {
    ICN_INTEREST    = 1,
    ICN_CONTENT     = 2,
    ICN_BACKGROUND  = 3
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
    uint16_t seq;
} icn_hdr_t;

icn_vars_t icn_vars;

open_addr_t *myId;

open_addr_t pit_entry = { .type = ADDR_NONE};
unsigned pit_ctr = 0;

unsigned slot0isActive = 1;
uint16_t send_counter = 0;
uint16_t receive_counter = 0;
uint16_t fail_counter = 0;
unsigned csSlotsActive = 0;

//=========================== prototypes ======================================

void icn_makeReservation(icn_link_t *schedule, size_t len, size_t offset);
void icn_makeTXReservation(icn_link_t *schedule, open_addr_t *neighbor, size_t len, size_t offset);
void icn_makeRXReservation(icn_link_t *schedule, open_addr_t *neighbor, size_t len, size_t offset);
void icn_removeRXReservation(icn_link_t *schedule, open_addr_t *neighbor, size_t len, size_t offset);
void icn_removeTXReservation(icn_link_t *schedule, open_addr_t *neighbor, size_t len, size_t offset);
void icn_initContent(open_addr_t *lastHop);
void icn_initInterest(opentimer_id_t id);
void icn_send(open_addr_t *dst, OpenQueueEntry_t *pkt);
void icn_addToFixedSchedule(open_addr_t *addr, int16_t rx_cell, int16_t tx_cell, int16_t shared);
unsigned _linkIsScheduled(open_addr_t *dst);
open_addr_t* _routeLookup(open_addr_t *dst);

//=========================== initialization ==================================

#define ADAPTIVE_SCHEDULE   (0)
#define TIMED_SENDING       (1)
#define INTEREST_INTERVAL   (1500)
#define BACKGROUND_INTERVAL (50)
#define FLOW_CONTROL        (1)
#define FLOW_THR            (40)
#define USE_CSMA            (0)
#define STRONG_LINKS        (0)
#define UNUSED_LINKS        (1)
#define BACKGROUND          (1)
#define LILLE               (0)

#define NUMBER_OF_CHUNKS    (100)
#define ADDR_LEN_64B    (sizeof(uint8_t) + 8)

#define CONTENT_STORE   (&node_ids[0])
#define HAS_CONTENT     (memcmp(myId, CONTENT_STORE, ADDR_LEN_64B) == 0)
#define WANT_CONTENT    (memcmp(myId, &(node_ids[9]), ADDR_LEN_64B) == 0)
#define BACKGROUND_NODE ((memcmp(myId, &(node_ids[3]), ADDR_LEN_64B) == 0) || \
        (memcmp(myId, &(node_ids[5]), ADDR_LEN_64B) == 0) || (memcmp(myId, \
                &(node_ids[6]), ADDR_LEN_64B) == 0))

#define NUMBER_OF_NODES     (10)

#ifdef SIMU
#warning SIMULATION build
#define NODE_01     {.type = ADDR_64B, .addr_64b = {0x14, 0x15, 0x92, 0xcc, 0x00, 0x00, 0x00, 0x01}}
#define NODE_02     {.type = ADDR_64B, .addr_64b = {0x14, 0x15, 0x92, 0xcc, 0x00, 0x00, 0x00, 0x02}}
#define NODE_03     {.type = ADDR_64B, .addr_64b = {0x14, 0x15, 0x92, 0xcc, 0x00, 0x00, 0x00, 0x03}}
#define NODE_04     {.type = ADDR_64B, .addr_64b = {0x14, 0x15, 0x92, 0xcc, 0x00, 0x00, 0x00, 0x04}}
#define NODE_05     {.type = ADDR_64B, .addr_64b = {0x05, 0x15, 0x92, 0xcc, 0x00, 0x00, 0x00, 0x05}}
#elif LILLE
#define NODE_01	{.type = ADDR_64B, .addr_64b = {0x05, 0x43, 0x32, 0xff, 0x03, 0xdb, 0xa7, 0x90 }} // m3-229
#define NODE_02	{.type = ADDR_64B, .addr_64b = {0x05, 0x43, 0x32, 0xff, 0x03, 0xdd, 0xb0, 0x73 }} // m3-228
#define NODE_03	{.type = ADDR_64B, .addr_64b = {0x05, 0x43, 0x32, 0xff, 0x02, 0xd0, 0x09, 0x59 }} // m3-227
#define NODE_04	{.type = ADDR_64B, .addr_64b = {0x05, 0x43, 0x32, 0xff, 0x02, 0xdb, 0x24, 0x50 }} // m3-226
#define NODE_05	{.type = ADDR_64B, .addr_64b = {0x05, 0x43, 0x32, 0xff, 0x02, 0xd7, 0x13, 0x57 }} // m3-225
#define NODE_06	{.type = ADDR_64B, .addr_64b = {0x05, 0x43, 0x32, 0xff, 0x03, 0xd8, 0xb0, 0x70 }} // m3-224
#define NODE_07	{.type = ADDR_64B, .addr_64b = {0x05, 0x43, 0x32, 0xff, 0x02, 0xdb, 0x25, 0x52 }} // m3-223
#define NODE_08	{.type = ADDR_64B, .addr_64b = {0x05, 0x43, 0x32, 0xff, 0x03, 0xd8, 0xb2, 0x88 }} // m3-222
#define NODE_09	{.type = ADDR_64B, .addr_64b = {0x05, 0x43, 0x32, 0xff, 0x03, 0xdc, 0xb8, 0x73 }} // m3-221
#define NODE_10	{.type = ADDR_64B, .addr_64b = {0x05, 0x43, 0x32, 0xff, 0x03, 0xd9, 0xc2, 0x70 }} // m3-220
#else
//#define NODE_01     {.type = ADDR_64B, .addr_64b = {0x05, 0x43, 0x32, 0xff, 0x03, 0xdb, 0xa3, 0x78}} // m3-210
#define NODE_01     {.type = ADDR_64B, .addr_64b = {0x05, 0x43, 0x32, 0xff, 0x03, 0xda, 0xc0, 0x81}} // m3-210
#define NODE_02     {.type = ADDR_64B, .addr_64b = {0x05, 0x43, 0x32, 0xff, 0x03, 0xdd, 0x94, 0x81}} // m3-234
#define NODE_03     {.type = ADDR_64B, .addr_64b = {0x05, 0x43, 0x32, 0xff, 0x03, 0xdb, 0xa4, 0x78}} // m3-260
//#define NODE_04     {.type = ADDR_64B, .addr_64b = {0x05, 0x43, 0x32, 0xff, 0x02, 0xd7, 0x38, 0x60}} // m3-222
//#define NODE_05     {.type = ADDR_64B, .addr_64b = {0x05, 0x43, 0x32, 0xff, 0x03, 0xd9, 0xb3, 0x83}} // m3-248
#define NODE_04 	{.type = ADDR_64B, .addr_64b = {0x05, 0x43, 0x32, 0xff, 0x03, 0xd7, 0xb1, 0x80}} // m3-224
#define NODE_05 	{.type = ADDR_64B, .addr_64b = {0x05, 0x43, 0x32, 0xff, 0x03, 0xde, 0xb8, 0x81}} // m3-246
//#define NODE_06 	{.type = ADDR_64B, .addr_64b = {0x05, 0x43, 0x32, 0xff, 0x03, 0xd6, 0xb5, 0x82}} // m3-218
#define NODE_06 	{.type = ADDR_64B, .addr_64b = {0x05, 0x43, 0x32, 0xff, 0x03, 0xd5, 0x95, 0x67}} // m3-214
//#define NODE_07	    {.type = ADDR_64B, .addr_64b = {0x05, 0x43, 0x32, 0xff, 0x02, 0xdb, 0x37, 0x61}} // m3-240
#define NODE_07	    {.type = ADDR_64B, .addr_64b = {0x05, 0x43, 0x32, 0xff, 0x02, 0xd6, 0x38, 0x61}} // m3-230
//#define NODE_08	    {.type = ADDR_64B, .addr_64b = {0x05, 0x43, 0x32, 0xff, 0x02, 0xdc, 0x28, 0x61}} // m3-272
//#define NODE_08 	{.type = ADDR_64B, .addr_64b = {0x05, 0x43, 0x32, 0xff, 0x03, 0xdc, 0xa6, 0x82}} // m3-6
#define NODE_08 	{.type = ADDR_64B, .addr_64b = {0x05, 0x43, 0x32, 0xff, 0x03, 0xdb, 0x99, 0x83}} // m3-8
#define NODE_09	    {.type = ADDR_64B, .addr_64b = {0x05, 0x43, 0x32, 0xff, 0x03, 0xd9, 0xa9, 0x68}} // m3-278
#define NODE_10	    {.type = ADDR_64B, .addr_64b = {0x05, 0x43, 0x32, 0xff, 0x03, 0xda, 0x98, 0x75}} // m3-284

#define NODE_11 	{.type = ADDR_64B, .addr_64b = {0x05, 0x43, 0x32, 0xff, 0x02, 0xdc, 0x28, 0x61}} // m3-272
#define NODE_12 	{.type = ADDR_64B, .addr_64b = {0x05, 0x43, 0x32, 0xff, 0x03, 0xda, 0x87, 0x72}} // m3-10
#endif

open_addr_t node_ids[NUMBER_OF_NODES] = {
    NODE_01,
    NODE_02,
    NODE_03,
    NODE_04,
    NODE_05,
    NODE_06,
    NODE_07,
    NODE_08,
    NODE_09,
    NODE_10
};

#define NUMBER_OF_LINKS (4)

#define SSF_INT_SIZE    (NUMBER_OF_NODES - 1 + NUMBER_OF_LINKS + STRONG_LINKS) 
#define SSF_INT_OFFSET  (NUMSERIALRX + SCHEDULE_MINIMAL_6TISCH_SLOTOFFSET + SCHEDULE_MINIMAL_6TISCH_ACTIVE_CELLS)

#define SSF_CS_SIZE     ((NUMBER_OF_LINKS * 2 + STRONG_LINKS + (UNUSED_LINKS * 13))) // (NUMBER_OF_LINKS * 2)
#define SSF_CS_OFFSET   (SSF_INT_OFFSET + SSF_INT_SIZE + 1)

icn_link_t ssf_int[SSF_INT_SIZE] = {
    /* broadcast cell for NODE_08 */
    {&(node_ids[7]), NULL},
    /* broadcast cell for NODE_10 */
    {&(node_ids[9]), NULL},
    /* broadcast cell for NODE_03 */
    {&(node_ids[2]), NULL},
    /* broadcast cell for NODE_02 */
    {&(node_ids[1]), NULL},

    /* link from 08 to 10 */
    {&(node_ids[7]), &(node_ids[9])}, // 2

    /* link from 10 to 03 */
    {&(node_ids[9]), &(node_ids[2])}, // 15

    /* link from 03 to 02 */
    {&(node_ids[2]), &(node_ids[1])}, // 17

    /* link from 02 to 01 */
    {&(node_ids[1]), &(node_ids[0])}, // 27

#if (STRONG_LINKS == 16)
    /* link from 08 to 10 */
    {&(node_ids[7]), &(node_ids[9])}, // 2
    /* link from 10 to 03 */
    {&(node_ids[9]), &(node_ids[2])}, // 15
    /* link from 03 to 02 */
    {&(node_ids[2]), &(node_ids[1])}, // 17
    /* link from 02 to 01 */
    {&(node_ids[1]), &(node_ids[0])}, // 27
    /* link from 08 to 10 */
    {&(node_ids[7]), &(node_ids[9])}, // 2
    /* link from 10 to 03 */
    {&(node_ids[9]), &(node_ids[2])}, // 15
    /* link from 03 to 02 */
    {&(node_ids[2]), &(node_ids[1])}, // 17
    /* link from 02 to 01 */
    {&(node_ids[1]), &(node_ids[0])}, // 27
    /* link from 08 to 10 */
    {&(node_ids[7]), &(node_ids[9])}, // 2
    /* link from 10 to 03 */
    {&(node_ids[9]), &(node_ids[2])}, // 15
    /* link from 03 to 02 */
    {&(node_ids[2]), &(node_ids[1])}, // 17
    /* link from 02 to 01 */
    {&(node_ids[1]), &(node_ids[0])}, // 27
    /* link from 08 to 10 */
    {&(node_ids[7]), &(node_ids[9])}, // 2
    /* link from 10 to 03 */
    {&(node_ids[9]), &(node_ids[2])}, // 15
    /* link from 03 to 02 */
    {&(node_ids[2]), &(node_ids[1])}, // 17
    /* link from 02 to 01 */
    {&(node_ids[1]), &(node_ids[0])}, // 27
#endif
#if (STRONG_LINKS == 32)
    /* link from 08 to 10 */
    {&(node_ids[7]), &(node_ids[9])}, // 2
    /* link from 10 to 03 */
    {&(node_ids[9]), &(node_ids[2])}, // 15
    /* link from 03 to 02 */
    {&(node_ids[2]), &(node_ids[1])}, // 17
    /* link from 02 to 01 */
    {&(node_ids[1]), &(node_ids[0])}, // 27
    /* link from 08 to 10 */
    {&(node_ids[7]), &(node_ids[9])}, // 2
    /* link from 10 to 03 */
    {&(node_ids[9]), &(node_ids[2])}, // 15
    /* link from 03 to 02 */
    {&(node_ids[2]), &(node_ids[1])}, // 17
    /* link from 02 to 01 */
    {&(node_ids[1]), &(node_ids[0])}, // 27
    /* link from 08 to 10 */
    {&(node_ids[7]), &(node_ids[9])}, // 2
    /* link from 10 to 03 */
    {&(node_ids[9]), &(node_ids[2])}, // 15
    /* link from 03 to 02 */
    {&(node_ids[2]), &(node_ids[1])}, // 17
    /* link from 02 to 01 */
    {&(node_ids[1]), &(node_ids[0])}, // 27
    /* link from 08 to 10 */
    {&(node_ids[7]), &(node_ids[9])}, // 2
    /* link from 10 to 03 */
    {&(node_ids[9]), &(node_ids[2])}, // 15
    /* link from 03 to 02 */
    {&(node_ids[2]), &(node_ids[1])}, // 17
    /* link from 02 to 01 */
    {&(node_ids[1]), &(node_ids[0])}, // 27
#endif

    /* broadcast cell for NODE_09 */
    {&(node_ids[8]), NULL},

    /* broadcast cell for NODE_07 */
    {&(node_ids[6]), NULL},

    /* broadcast cell for NODE_06 */
    {&(node_ids[5]), NULL},

    /* broadcast cell for NODE_05 */
    {&(node_ids[4]), NULL},

    /* broadcast cell for NODE_04 */
    {&(node_ids[3]), NULL},
};

icn_link_t ssf_cs[SSF_CS_SIZE] = {
    /* link from 01 to 02 */
    {&(node_ids[0]), &(node_ids[1])}, // 28

    /* link from 02 to 03 */
    {&(node_ids[1]), &(node_ids[2])}, // 18

    /* link from 03 to 10 */
    {&(node_ids[2]), &(node_ids[9])}, // 16

    /* link from 08 to 10 */
    {&(node_ids[9]), &(node_ids[7])}, // 1

    /* link from 01 to 02 */
    {&(node_ids[0]), &(node_ids[1])}, // 28

    /* link from 02 to 03 */
    {&(node_ids[1]), &(node_ids[2])}, // 18

    /* link from 03 to 10 */
    {&(node_ids[2]), &(node_ids[9])}, // 16

    /* link from 08 to 10 */
    {&(node_ids[9]), &(node_ids[7])}, // 1

#if (STRONG_LINKS == 16)
    /* link from 01 to 02 */
    {&(node_ids[0]), &(node_ids[1])}, // 28
    /* link from 02 to 03 */
    {&(node_ids[1]), &(node_ids[2])}, // 18
    /* link from 03 to 10 */
    {&(node_ids[2]), &(node_ids[9])}, // 16
    /* link from 08 to 10 */
    {&(node_ids[9]), &(node_ids[7])}, // 1
    /* link from 01 to 02 */
    {&(node_ids[0]), &(node_ids[1])}, // 28
    /* link from 02 to 03 */
    {&(node_ids[1]), &(node_ids[2])}, // 18
    /* link from 03 to 10 */
    {&(node_ids[2]), &(node_ids[9])}, // 16
    /* link from 08 to 10 */
    {&(node_ids[9]), &(node_ids[7])}, // 1
    /* link from 01 to 02 */
    {&(node_ids[0]), &(node_ids[1])}, // 28
    /* link from 02 to 03 */
    {&(node_ids[1]), &(node_ids[2])}, // 18
    /* link from 03 to 10 */
    {&(node_ids[2]), &(node_ids[9])}, // 16
    /* link from 08 to 10 */
    {&(node_ids[9]), &(node_ids[7])}, // 1
    /* link from 01 to 02 */
    {&(node_ids[0]), &(node_ids[1])}, // 28
    /* link from 02 to 03 */
    {&(node_ids[1]), &(node_ids[2])}, // 18
    /* link from 03 to 10 */
    {&(node_ids[2]), &(node_ids[9])}, // 16
    /* link from 08 to 10 */
    {&(node_ids[9]), &(node_ids[7])}, // 1
#endif

#if (STRONG_LINKS == 32)
    /* link from 01 to 02 */
    {&(node_ids[0]), &(node_ids[1])}, // 28
    /* link from 02 to 03 */
    {&(node_ids[1]), &(node_ids[2])}, // 18
    /* link from 03 to 10 */
    {&(node_ids[2]), &(node_ids[9])}, // 16
    /* link from 08 to 10 */
    {&(node_ids[9]), &(node_ids[7])}, // 1
    /* link from 01 to 02 */
    {&(node_ids[0]), &(node_ids[1])}, // 28
    /* link from 02 to 03 */
    {&(node_ids[1]), &(node_ids[2])}, // 18
    /* link from 03 to 10 */
    {&(node_ids[2]), &(node_ids[9])}, // 16
    /* link from 08 to 10 */
    {&(node_ids[9]), &(node_ids[7])}, // 1
    /* link from 01 to 02 */
    {&(node_ids[0]), &(node_ids[1])}, // 28
    /* link from 02 to 03 */
    {&(node_ids[1]), &(node_ids[2])}, // 18
    /* link from 03 to 10 */
    {&(node_ids[2]), &(node_ids[9])}, // 16
    /* link from 08 to 10 */
    {&(node_ids[9]), &(node_ids[7])}, // 1
    /* link from 01 to 02 */
    {&(node_ids[0]), &(node_ids[1])}, // 28
    /* link from 02 to 03 */
    {&(node_ids[1]), &(node_ids[2])}, // 18
    /* link from 03 to 10 */
    {&(node_ids[2]), &(node_ids[9])}, // 16
    /* link from 08 to 10 */
    {&(node_ids[9]), &(node_ids[7])}, // 1
#endif


#if UNUSED_LINKS
    {&(node_ids[4]), &(node_ids[2])},
    {&(node_ids[4]), &(node_ids[3])},
    {&(node_ids[8]), &(node_ids[2])},
    {&(node_ids[3]), &(node_ids[1])},
    {&(node_ids[4]), &(node_ids[1])},
    {&(node_ids[5]), &(node_ids[1])},
    {&(node_ids[6]), &(node_ids[1])},
    {&(node_ids[3]), &(node_ids[0])},
    {&(node_ids[0]), &(node_ids[3])},
    {&(node_ids[5]), &(node_ids[0])},
    {&(node_ids[0]), &(node_ids[5])},
    {&(node_ids[6]), &(node_ids[0])},
#endif
};

#define RRT_SIZE        (11)
icn_routing_entry_t routing_table[RRT_SIZE] = {
    /* default route for 02 over 01 */
    {&(node_ids[1]), NULL, &(node_ids[0])},
    /* default route for 03 over 02 */
    {&(node_ids[2]), NULL, &(node_ids[1])},
    /* default route for 04 over 01 */
    {&(node_ids[3]), NULL, &(node_ids[0])},
    /* default route for 05 over 02 */
    {&(node_ids[4]), NULL, &(node_ids[1])},
    /* default route for 06 over 01 */
    {&(node_ids[5]), NULL, &(node_ids[0])},
    /* default route for 07 over 01 */
    {&(node_ids[6]), NULL, &(node_ids[0])},
    /* default route for 08 over 10 */
    {&(node_ids[7]), NULL, &(node_ids[9])},
    /* default route for 09 over 03 */
    {&(node_ids[8]), NULL, &(node_ids[2])},
    /* default route for 10 over 03 */
    {&(node_ids[9]), NULL, &(node_ids[2])},

    /* route for 01 to 03 over 02 */
    {&(node_ids[0]), &(node_ids[2]), &(node_ids[1])},
    /* route for 01 to 05 over 02 */
    {&(node_ids[0]), &(node_ids[4]), &(node_ids[1])},
};

BITFIELD(128, received_chunks);

int mote_main(void) {
   board_init();
   scheduler_init();
   openstack_init();

   memset(received_chunks, 0, sizeof(received_chunks));

   myId = idmanager_getMyID(ADDR_64B);

   /* check for root node */
   if (memcmp(myId, &(node_ids[0]), ADDR_LEN_64B) == 0) {
       idmanager_setIsDAGroot(TRUE);
   }
   else {
       neighbors_vars.myDAGrank = 44;
   }

#if (USE_CSMA == 0)
   icn_makeReservation(ssf_int, SSF_INT_SIZE, SSF_INT_OFFSET);
#if ADAPTIVE_SCHEDULE == 0
   icn_makeReservation(ssf_cs, SSF_CS_SIZE, SSF_CS_OFFSET);
#endif
#endif
   scheduler_start();
   return 0; // this line should never be reached
}

void icn_makeReservation(icn_link_t *schedule, size_t len, size_t offset) {
    /* iterate over the full schedule and make my reservations*/
   for (int i = 0; i < len; i++) {
#if USE_CSMA
       icn_addToFixedSchedule(NULL, -1, -1, i+offset);
#else
       /*  if I am the sender... */
       if (memcmp(myId, schedule[i].sender, ADDR_LEN_64B) == 0) {
           /* without a particular receiver, schedule a shared cell */
           if (schedule[i].receiver == NULL) {
               icn_addToFixedSchedule(NULL, -1, -1, i+offset);
           }
           /* or for this particular receiver */
           else {
               icn_addToFixedSchedule(schedule[i].receiver, -1, i+offset, -1);
           }
       }
       /* if I am the receiver or another node has shared cell, schedule an RX cell */
       else if ((memcmp(myId, schedule[i].receiver, ADDR_LEN_64B) == 0) ||
               (schedule[i].receiver == NULL)) {
           {
               icn_addToFixedSchedule(schedule[i].sender, i+offset, -1, -1);
           }
       }
#endif
   }
}

void icn_makeTXReservation(icn_link_t *schedule, open_addr_t *neighbor, size_t len, size_t offset) {
    slotinfo_element_t slot_info;
    uint16_t tmp = 0;
    /* iterate over the full schedule and make my reservations*/
   for (int i = 0; i < len; i++) {
       /*  if I am the sender for this particular receiver */
       if (packetfunctions_sameAddress(myId, schedule[i].sender) &&
               packetfunctions_sameAddress(neighbor, schedule[i].receiver)) {
           if (tmp++ < csSlotsActive) {
               continue;
           }

           schedule_getSlotInfo(i+offset, schedule[i].receiver, &slot_info);
           if (slot_info.link_type != CELLTYPE_OFF) {
               openserial_printError(COMPONENT_ICN, ERR_WRONG_CELLTYPE,
                       i+offset, slot_info.link_type);
           }
           openserial_printError(COMPONENT_ICN, ERR_DEBUG3,
                        i+offset, 1);
           icn_addToFixedSchedule(schedule[i].receiver, -1, i+offset, -1);
           return;
       }
   }
}

void icn_removeTXReservation(icn_link_t *schedule, open_addr_t *neighbor, size_t len, size_t offset) {
    uint16_t tmp = 0;
    openserial_printError(COMPONENT_ICN, ERR_DEBUG1, csSlotsActive, 0);
    /* iterate over the full schedule and remove my reservations*/
   for (int i = 0; i < len; i++) {
       /* if I am the sender */
       if (packetfunctions_sameAddress(myId, schedule[i].sender) &&
               packetfunctions_sameAddress(neighbor, schedule[i].receiver)) {
           if (tmp++ < (csSlotsActive - 1)) {
               continue;
           }
           openserial_printError(COMPONENT_ICN, ERR_DEBUG3,
                        i+offset, 0);
           schedule_removeActiveSlot(i+offset, schedule[i].receiver);
           return;
       }
   }
}

void icn_makeRXReservation(icn_link_t *schedule, open_addr_t *neighbor, size_t len, size_t offset) {
    slotinfo_element_t slot_info;
    uint16_t tmp = 0;
    /* iterate over the full schedule and make my reservations*/
   for (int i = 0; i < len; i++) {
       /* if I am the receiver for the given sender */
       if (packetfunctions_sameAddress(myId, schedule[i].receiver) &&
               packetfunctions_sameAddress(neighbor, schedule[i].sender)) {
           if (tmp++ < csSlotsActive) {
               continue;
           }

           schedule_getSlotInfo(i+offset, schedule[i].sender, &slot_info);
           if (slot_info.link_type != CELLTYPE_OFF) {
               openserial_printError(COMPONENT_ICN, ERR_WRONG_CELLTYPE,
                        slot_info.link_type, i+offset);
               return;
           }
           openserial_printError(COMPONENT_ICN, ERR_DEBUG4,
                        i+offset, 1);
           icn_addToFixedSchedule(schedule[i].sender, i+offset, -1, -1);
           return;
       }
   }
}

void icn_removeRXReservation(icn_link_t *schedule, open_addr_t *neighbor, size_t len, size_t offset) {
    uint16_t tmp = 0;
    openserial_printError(COMPONENT_ICN, ERR_DEBUG2, csSlotsActive, 0);
    /* iterate over the full schedule and remove my reservations*/
   for (int i = 0; i < len; i++) {
       /* if I am the receiver */
       if (packetfunctions_sameAddress(myId, schedule[i].receiver) &&
               packetfunctions_sameAddress(neighbor, schedule[i].sender)) {
           if (tmp++ < (csSlotsActive - 1)) {
               continue;
           }
           openserial_printError(COMPONENT_ICN, ERR_DEBUG4,
                        i+offset, 0);
           schedule_removeActiveSlot(i+offset, schedule[i].sender);
           return;
       }
   }
}

unsigned _linkIsScheduled(open_addr_t *dst) {
#if 0
   if (memcmp(myId, &(node_ids[0]), ADDR_LEN_64B) == 0) {
       openserial_printError(COMPONENT_ICN, ERR_DEBUG4,
               (errorparameter_t) 44, dst->type);
       openserial_printError(COMPONENT_ICN, ERR_DEBUG4,
               (errorparameter_t) dst->addr_64b[0],
               (errorparameter_t) dst->addr_64b[1]);
       openserial_printError(COMPONENT_ICN, ERR_DEBUG4,
               (errorparameter_t) dst->addr_64b[2],
               (errorparameter_t) dst->addr_64b[3]);
       openserial_printError(COMPONENT_ICN, ERR_DEBUG4,
               (errorparameter_t) dst->addr_64b[4],
               (errorparameter_t) dst->addr_64b[5]);
       openserial_printError(COMPONENT_ICN, ERR_DEBUG4,
               (errorparameter_t) dst->addr_64b[6],
               (errorparameter_t) dst->addr_64b[7]);
   }
#endif

   for (int i = 0; i < SSF_INT_SIZE; i++) {
       if ((memcmp(myId, ssf_int[i].sender, ADDR_LEN_64B) == 0) &&
               (memcmp(dst, ssf_int[i].receiver, ADDR_LEN_64B) == 0)) {
           return 1;
       }
   }
   for (int i = 0; i < SSF_CS_SIZE; i++) {
       if ((memcmp(myId, ssf_cs[i].sender, ADDR_LEN_64B) == 0) &&
               (memcmp(dst, ssf_cs[i].receiver, ADDR_LEN_64B) == 0)) {
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
        schedule_addActiveSlot(tx_cell,
                CELLTYPE_TX,
                FALSE,
                SCHEDULE_MINIMAL_6TISCH_CHANNELOFFSET,
                addr
                );
    }
    if (rx_cell >= 0) {
        schedule_addActiveSlot(rx_cell,
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
        schedule_addActiveSlot(shared,
                CELLTYPE_TXRX,
                TRUE,
                SCHEDULE_MINIMAL_6TISCH_CHANNELOFFSET,
                &temp_neighbor
                );
    }
}

void icn_initContent(open_addr_t *lastHop) {
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
    packetfunctions_reserveHeaderSize(pkt, sizeof(icn_hdr_t) + strlen(content) + 1);
    ((icn_hdr_t*)pkt->payload)->type = ICN_CONTENT;
    ((icn_hdr_t*)pkt->payload)->seq = send_counter;
    memcpy((pkt->payload + sizeof(icn_hdr_t)), content, strlen(content) + 1);

    openserial_printError(COMPONENT_ICN, ERR_ICN_SEND,
            (errorparameter_t)send_counter, (errorparameter_t)0);
    icn_send(lastHop, pkt);
}

void icn_initInterest(opentimer_id_t id) {
    if (fail_counter) {
        openserial_printInfo(COMPONENT_ICN, ERR_ICN_TX_FAIL_COUNT, fail_counter, 0);
    }
    if (!ieee154e_isSynch()) {
        if (!slot0isActive) {
            icn_addToFixedSchedule(&(node_ids[0]), -1, -1, 0);
            slot0isActive = 1;
        }
        return;
    }
    if ((neighbors_getNumNeighbors()>=1) && WANT_CONTENT) {
#if FLOW_CONTROL
        if (send_counter > (receive_counter + FLOW_THR)) {
            openserial_printError(COMPONENT_ICN, ERR_ICN_FLOW_CTRL,
                    send_counter, receive_counter);
            return;
        }
#endif
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

#if ADAPTIVE_SCHEDULE
        /* make reservation for CS */
        open_addr_t *nextHop = _routeLookup(CONTENT_STORE);
        icn_makeRXReservation(ssf_cs, nextHop, SSF_CS_SIZE, SSF_CS_OFFSET);
        csSlotsActive++;
#endif

        // send interest packet
        openserial_printInfo(COMPONENT_ICN, ERR_ICN_SEND,
                send_counter, (errorparameter_t) node_ids[0].addr_64b[7]);
        packetfunctions_reserveHeaderSize(pkt, sizeof(icn_hdr_t) + strlen(interest) + 1);
        ((icn_hdr_t*)pkt->payload)->type = ICN_INTEREST;
        ((icn_hdr_t*)pkt->payload)->seq = send_counter++;
        memcpy((pkt->payload + sizeof(icn_hdr_t)), interest, strlen(interest) + 1);

        if (send_counter < NUMBER_OF_CHUNKS) {
            icn_send(CONTENT_STORE, pkt);
        }
#if (TIMED_SENDING == 0)
            opentimers_stop(icn_vars.timerId);
#else
        if (send_counter >= NUMBER_OF_CHUNKS) {
            opentimers_stop(icn_vars.timerId);
        }
#endif
    }

#if BACKGROUND
    if (BACKGROUND_NODE) {
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

        packetfunctions_reserveHeaderSize(pkt, sizeof(icn_hdr_t) + strlen(interest) + 1);
        ((icn_hdr_t*)pkt->payload)->type = ICN_BACKGROUND;
        ((icn_hdr_t*)pkt->payload)->seq = send_counter++;
        memcpy((pkt->payload + sizeof(icn_hdr_t)), interest, strlen(interest) + 1);

        icn_send(CONTENT_STORE, pkt);
    }
#endif
    if (!idmanager_getIsDAGroot() && slot0isActive) {
        /*  remove default slot  */
        open_addr_t tmp;
        memset(&tmp,0,sizeof(open_addr_t));
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
    if (BACKGROUND_NODE) {
    icn_vars.timerId    = opentimers_start(
            BACKGROUND_NODE,
            TIMER_PERIODIC,TIME_MS,
            icn_initInterest
            );
    }
    else {
        icn_vars.timerId    = opentimers_start(
                INTEREST_INTERVAL,
                TIMER_PERIODIC,TIME_MS,
                icn_initInterest
                );
    }
}

void iphc_sendDone(OpenQueueEntry_t* msg, owerror_t error) {
    msg->owner = COMPONENT_ICN;
    fail_counter += msg->l2_numTxAttempts - 1;
#if ADAPTIVE_SCHEDULE
    if (error == E_SUCCESS) {
        if (HAS_CONTENT) {
            if (csSlotsActive) {
                icn_removeTXReservation(ssf_cs, &(msg->l2_nextORpreviousHop), SSF_CS_SIZE, SSF_CS_OFFSET);
                csSlotsActive--;
            }
        }
        else if (!WANT_CONTENT && ((pit_ctr <= 1) && (csSlotsActive))) {
            icn_removeRXReservation(ssf_cs, &(msg->l2_nextORpreviousHop), SSF_CS_SIZE, SSF_CS_OFFSET);
            icn_removeTXReservation(ssf_cs, &pit_entry, SSF_CS_SIZE, SSF_CS_OFFSET);
            csSlotsActive--;
            }
    }
#endif
    openqueue_freePacketBuffer(msg);
}

void iphc_receive(OpenQueueEntry_t* msg) {
    msg->owner = COMPONENT_ICN;
    icn_hdr_t *icn_pkt = (icn_hdr_t*) msg->payload;
    switch (icn_pkt->type) {
        case ICN_INTEREST:
            if (HAS_CONTENT) {
#if ADAPTIVE_SCHEDULE
                /* for next hop, to receive potential content */
                icn_makeTXReservation(ssf_cs, &(msg->l2_nextORpreviousHop), SSF_CS_SIZE, SSF_CS_OFFSET);
                csSlotsActive++;
//                    openserial_printInfo(COMPONENT_ICN, ERR_DEBUG1, msg->l2_nextORpreviousHop.addr_64b[6], msg->l2_nextORpreviousHop.addr_64b[7]);
#endif
//                openserial_printInfo(COMPONENT_ICN, ERR_ICN_RECV1,
//                        (errorparameter_t) msg->l2_nextORpreviousHop.addr_64b[6],
//                        (errorparameter_t) msg->l2_nextORpreviousHop.addr_64b[7]);
                openserial_printInfo(COMPONENT_ICN, ERR_ICN_RECV_INT,
                        msg->l2_asn.bytes0and1,
//                        msg->l2_asn.byte4);
//                openserial_printInfo(COMPONENT_ICN, ERR_ICN_RECV_INT,
                        (errorparameter_t) icn_pkt->seq);
//                        0);
                send_counter = icn_pkt->seq;
                icn_initContent(&(msg->l2_nextORpreviousHop));
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
                memcpy(&pit_entry, &msg->l2_nextORpreviousHop, ADDR_LEN_64B);
                pit_ctr++;

#if ADAPTIVE_SCHEDULE
                /* make reservations in CS */
                /* for next hop, to receive potential content */
                open_addr_t *nextHop = _routeLookup(CONTENT_STORE);
                icn_makeRXReservation(ssf_cs, nextHop, SSF_CS_SIZE, SSF_CS_OFFSET);

                /* for previous hop to send back the potential content */
                icn_makeTXReservation(ssf_cs, &pit_entry, SSF_CS_SIZE, SSF_CS_OFFSET);
                csSlotsActive++;
#endif

                /* forward to CS node */
                msg->creator = COMPONENT_ICN;
                icn_send(CONTENT_STORE, msg);
            }
            break;
        case ICN_CONTENT:
            if (pit_entry.type != ADDR_NONE) {
                msg->creator = COMPONENT_ICN;
                icn_send(&pit_entry, msg);
                if (--pit_ctr <= 0) {
                    pit_entry.type = ADDR_NONE;
                    pit_ctr = 0;
                }
            }
            else if (WANT_CONTENT) {
                openserial_printInfo(COMPONENT_ICN, ERR_ICN_RECV_CONT,
                        msg->l2_asn.bytes0and1,
                        (errorparameter_t) icn_pkt->seq);
                if (bf_isset(received_chunks, icn_pkt->seq)) {
                    openserial_printInfo(COMPONENT_ICN, ERR_ICN_DUPLICATE,
                            icn_pkt->seq, 0);
                }
                else {
                //    openserial_printInfo(COMPONENT_ICN, ERR_ICN_RECV_CONT,
                //            (errorparameter_t) icn_pkt->seq,
                //            (errorparameter_t) send_counter);
                    bf_set(received_chunks, icn_pkt->seq);
                }

                receive_counter++;
#if ADAPTIVE_SCHEDULE
                if (csSlotsActive) {
                    icn_removeRXReservation(ssf_cs, &(msg->l2_nextORpreviousHop), SSF_CS_SIZE, SSF_CS_OFFSET);
                    csSlotsActive--;
                }
#endif
                openqueue_freePacketBuffer(msg);
#if (TIMED_SENDING == 0)
                if (send_counter < NUMBER_OF_CHUNKS) {
                    icn_initInterest(0);
                }
#endif
            }
            else {
                openserial_printError(COMPONENT_ICN, ERR_ICN_FWD_NOT_FOUND,
                        (errorparameter_t) 47,
                        (errorparameter_t) 11);
                openqueue_freePacketBuffer(msg);
            }
            break;
        case ICN_BACKGROUND:
            openqueue_freePacketBuffer(msg);
            break;
        default:
            openserial_printError(COMPONENT_ICN, ERR_MSG_UNKNOWN_TYPE,
                    (errorparameter_t) icn_pkt->type,
                    (errorparameter_t) icn_pkt->type);
            openqueue_freePacketBuffer(msg);
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
