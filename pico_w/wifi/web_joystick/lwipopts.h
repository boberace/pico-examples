#ifndef _LWIPOPTS_H
#define _LWIPOPTS_H

// Generally you would define your own explicit list of lwIP options
// (see https://www.nongnu.org/lwip/2_1_x/group__lwip__opts.html)
//
// This example uses a common include to avoid repetition


#define LWIP_MDNS_RESPONDER 1
#define LWIP_IGMP 1 //  mdns.c
#define LWIP_NUM_NETIF_CLIENT_DATA 1 // netif.c

#define MEMP_NUM_SYS_TIMEOUT            (LWIP_NUM_SYS_TIMEOUT_INTERNAL+3) //https://github.com/raspberrypi/pico-sdk/issues/1281

#include "lwipopts_examples_common.h"

#endif
