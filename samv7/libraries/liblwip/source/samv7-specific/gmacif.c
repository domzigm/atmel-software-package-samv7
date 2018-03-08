/* ----------------------------------------------------------------------------
 *         SAM Software Package License 
 * ----------------------------------------------------------------------------
 * Copyright (c) 2013, Atmel Corporation
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the disclaimer below.
 *
 * Atmel's name may not be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * DISCLAIMER: THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * ----------------------------------------------------------------------------
 */

/*
 * This file is a skeleton for developing Ethernet network interface
 * drivers for lwIP. Add code to the low_level functions and do a
 * search-and-replace for the word "gmacif" to replace it with
 * something that better describes your network interface.
 */

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include <board.h>

#include "lwip/opt.h"
#include "gmacif.h"

#include "lwip/def.h"
#include "lwip/mem.h"
#include "lwip/pbuf.h"
#include "lwip/sys.h"
#include "lwip/stats.h"
#include "netif/etharp.h"
#include "string.h"
/*----------------------------------------------------------------------------
 *        Definitions
 *----------------------------------------------------------------------------*/

#define _mem_barrier_   __DSB();

/* Define those to better describe your network interface. */
#define IFNAME0 'e'
#define IFNAME1 'n'

#define TX_BUFFERS        32     /** Must be a power of 2 */
#define RX_BUFFERS        32     /** Must be a power of 2 */
#define BUFFER_SIZE        512
#define DUMMY_SIZE              2
#define DUMMY_BUFF_SIZE         512

#define GMAC_CAF_DISABLE  0
#define GMAC_CAF_ENABLE   1
#define GMAC_NBC_DISABLE  0
#define GMAC_NBC_ENABLE   1

/* The GMAC driver instance */
sGmacd gGmacd;

/* The GMACB driver instance */
GMacb gGmacb;

static struct gmacif Gmacif_config;

/** The PINs for GMAC */
const Pin gmacPins[] = {BOARD_GMAC_RUN_PINS};

/** TX descriptors list */
COMPILER_SECTION(".ram_nocache")
COMPILER_ALIGNED(8) static sGmacTxDescriptor gTxDs[TX_BUFFERS], gDummyTxDs[DUMMY_SIZE];


/** TX callbacks list */
COMPILER_ALIGNED(8) static fGmacdTransferCallback gTxCbs[TX_BUFFERS], gDummyTxCbs[DUMMY_SIZE];

/** RX descriptors list */
COMPILER_SECTION(".ram_nocache")
COMPILER_ALIGNED(8) static sGmacRxDescriptor gRxDs[RX_BUFFERS], gDummyRxDs[DUMMY_SIZE];

/** Send Buffer */
/* Section 3.6 of AMBA 2.0 spec states that burst should not cross 1K Boundaries.
   Receive buffer manager writes are burst of 2 words => 3 lsb bits of the address
   shall be set to 0 */
COMPILER_ALIGNED(32) static uint8_t pTxBuffer[TX_BUFFERS * BUFFER_SIZE], pTxDummyBuffer[DUMMY_SIZE * DUMMY_BUFF_SIZE];

/** Receive Buffer */
COMPILER_ALIGNED(32) static uint8_t pRxBuffer[RX_BUFFERS * BUFFER_SIZE], pRxDummyBuffer[DUMMY_SIZE * DUMMY_BUFF_SIZE];


/*---------------------------------------------------------------------------
 *         Local functions
 *---------------------------------------------------------------------------*/


/* Forward declarations. */
static void  gmacif_input(struct netif *netif);
static err_t gmacif_output(struct netif *netif, struct pbuf *p, struct ip_addr *ipaddr);

static void glow_level_init(struct netif *netif)
{
    struct gmacif *gmacif = netif->state;
    sGmacd *pGmacd = &gGmacd;
    GMacb  *pGmacb = &gGmacb;
    sGmacInit Que0, Que;

    /* set MAC hardware address length */
    netif->hwaddr_len = ETHARP_HWADDR_LEN;
    /* set MAC hardware address */
    netif->hwaddr[0] = gmacif->ethaddr.addr[0];
    netif->hwaddr[1] = gmacif->ethaddr.addr[1];
    netif->hwaddr[2] = gmacif->ethaddr.addr[2];
    netif->hwaddr[3] = gmacif->ethaddr.addr[3];
    netif->hwaddr[4] = gmacif->ethaddr.addr[4];
    netif->hwaddr[5] = gmacif->ethaddr.addr[5];
    /* maximum transfer unit */
    netif->mtu = 1500;
   
    /* device capabilities */
    netif->flags = NETIF_FLAG_BROADCAST;
    _mem_barrier_
    /* Init GMAC driver structure */
    memset(&Que0, 0, sizeof(Que0));
    Que0.bIsGem = 1;
    Que0.bDmaBurstLength = 4;
    Que0.pRxBuffer =pRxBuffer;
    Que0.pRxD = gRxDs;
    Que0.wRxBufferSize = BUFFER_SIZE;
    Que0.wRxSize = RX_BUFFERS;
    Que0.pTxBuffer = pTxBuffer;
    Que0.pTxD = gTxDs;
    Que0.wTxBufferSize = BUFFER_SIZE;
    Que0.wTxSize = TX_BUFFERS;
    Que0.pTxCb = gTxCbs;

    memset(&Que, 0, sizeof(Que));
    Que.bIsGem = 1;
    Que.bDmaBurstLength = 4;
    Que.pRxBuffer =pRxDummyBuffer;
    Que.pRxD = gDummyRxDs;
    Que.wRxBufferSize = DUMMY_BUFF_SIZE;
    Que.wRxSize = DUMMY_SIZE;
    Que.pTxBuffer = pTxDummyBuffer;
    Que.pTxD = gDummyTxDs;
    Que.wTxBufferSize = DUMMY_BUFF_SIZE;
    Que.wTxSize = DUMMY_SIZE;
    Que.pTxCb = gDummyTxCbs;
    GMACD_Init(pGmacd, GMAC, ID_GMAC, GMAC_CAF_ENABLE, GMAC_NBC_DISABLE);

    for(gmacQueList_t q = GMAC_QUE_1; q <= GMAC_QUE_MAX; q++)
    {
      /* All queues must be initialized regardless if they're used or not */
      GMACD_InitTransfer(pGmacd, &Que,  q);
    }
  
    GMACD_InitTransfer(pGmacd, &Que0, GMAC_QUE_0);
    
    GMAC_SetAddress(gGmacd.pHw, 0, Gmacif_config.ethaddr.addr);
    
     /* Setup interrupts */  
    NVIC_ClearPendingIRQ(GMAC_IRQn);
    NVIC_EnableIRQ(GMAC_IRQn);
    /* Init GMACB driver */
    GMACB_Init(pGmacb, pGmacd, BOARD_GMAC_PHY_ADDR);
    GMACB_ResetPhy(pGmacb);
    /* PHY initialize */
    if (!GMACB_InitPhy(pGmacb, BOARD_MCK,  0, 0,  gmacPins, PIO_LISTSIZE(gmacPins)))
    {
        printf("P: PHY Initialize ERROR!\n\r");
        return ;
    }
   
    /* Auto Negotiate, work in RMII mode */
    if (!GMACB_AutoNegotiate(pGmacb))
    {
        printf( "P: Auto Negotiate ERROR!\n\r" ) ;
        return;
    }
    printf( "P: Link detected \n\r" ) ;
}

/**
 * This function should do the actual transmission of the packet. The packet is
 * contained in the pbuf that is passed to the function. This pbuf
 * might be chained.
 *
 * @param netif the lwip network interface structure for this gmacif
 * @param p the MAC packet to send (e.g. IP packet including MAC addresses and type)
 * @return ERR_OK if the packet could be sent
 *         an err_t value if the packet couldn't be sent
 */
static err_t glow_level_output(struct netif *netif, struct pbuf *p)
{
    struct pbuf *q;
    uint8_t buf[1514];
    uint8_t *bufptr = &buf[0];
    uint8_t rc;

#if ETH_PAD_SIZE
    pbuf_header(p, -ETH_PAD_SIZE);    /* drop the padding word */
#endif

    for(q = p; q != NULL; q = q->next) {
        /* Send the data from the pbuf to the interface, one pbuf at a
        time. The size of the data in each pbuf is kept in the ->len
        variable. */

        /* send data from(q->payload, q->len); */
        memcpy(bufptr, q->payload, q->len);
        bufptr += q->len;
    }

    /* signal that packet should be sent(); */
//    rc = GMACD_Send(&gGmacd, buf, 0, NULL, GMAC_QUE_2);
//    rc = GMACD_Send(&gGmacd, buf, 0, NULL, GMAC_QUE_1);
    rc = GMACD_Send(&gGmacd, buf, p->tot_len, NULL, GMAC_QUE_0);
    if (rc != GMACD_OK) {
        return ERR_BUF;
    }
#if ETH_PAD_SIZE
    pbuf_header(p, ETH_PAD_SIZE);     /* reclaim the padding word */
#endif

    LINK_STATS_INC(link.xmit);
    return ERR_OK;
}

/**
 * Should allocate a pbuf and transfer the bytes of the incoming
 * packet from the interface into the pbuf.
 *
 * @param netif the lwip network interface structure for this gmacif
 * @return a pbuf filled with the received packet (including MAC header)
 *         NULL on memory error
 */
static struct pbuf *glow_level_input(struct netif *netif)
{                    
    struct pbuf *p, *q;
    u16_t len;
    uint8_t buf[1514];
    uint8_t *bufptr = &buf[0];
    
    uint32_t frmlen;
    uint8_t rc;
    
    /* Obtain the size of the packet and put it into the "len"
       variable. */
    rc = GMACD_Poll(&gGmacd, buf, (uint32_t)sizeof(buf), (uint32_t*)&frmlen, GMAC_QUE_0);
    if (rc != GMACD_OK)
    { 
      return NULL;
    }
    len = frmlen;
    
#if ETH_PAD_SIZE
    len += ETH_PAD_SIZE;      /* allow room for Ethernet padding */
#endif

    /* We allocate a pbuf chain of pbufs from the pool. */
    p = pbuf_alloc(PBUF_RAW, len, PBUF_POOL);
    
    if (p != NULL) {
#if ETH_PAD_SIZE
        pbuf_header(p, -ETH_PAD_SIZE);          /* drop the padding word */
#endif
        /* We iterate over the pbuf chain until we have read the entire
         * packet into the pbuf. */
        for(q = p; q != NULL; q = q->next) {
            /* Read enough bytes to fill this pbuf in the chain. The
             * available data in the pbuf is given by the q->len
             * variable. */
            /* read data into(q->payload, q->len); */
            memcpy(q->payload, bufptr, q->len);
            bufptr += q->len;
            memory_sync();
        }
        /* acknowledge that packet has been read(); */

#if ETH_PAD_SIZE
        pbuf_header(p, ETH_PAD_SIZE);           /* reclaim the padding word */
#endif
        LINK_STATS_INC(link.recv);
    } else {
        /* drop packet(); */
        LINK_STATS_INC(link.memerr);
        LINK_STATS_INC(link.drop);
    }
    return p;
}

/**
 * This function is called by the TCP/IP stack when an IP packet
 * should be sent. It calls the function called glow_level_output() to
 * do the actual transmission of the packet.
 *
 */
static err_t gmacif_output(struct netif *netif, struct pbuf *p, struct ip_addr *ipaddr)
{
    /* resolve hardware address, then send (or queue) packet */
    return etharp_output(netif, p, ipaddr);
}

/**
 * This function should be called when a packet is ready to be read
 * from the interface. It uses the function low_level_input() that
 * should handle the actual reception of bytes from the network
 * interface. Then the type of the received packet is determined and
 * the appropriate input function is called.
 *
 * @param netif the lwip network interface structure for this gmacif
 */

static void gmacif_input(struct netif *netif)
{
    struct gmacif *gmacif;
    struct eth_hdr *ethhdr;
    struct pbuf *p;
    gmacif = netif->state;
    
    /* move received packet into a new pbuf */
    p = glow_level_input(netif);
    /* no packet could be read, silently ignore this */
    if (p == NULL) return;
    /* points to packet payload, which starts with an Ethernet header */
    ethhdr = p->payload;
    
    switch (htons(ethhdr->type)) {
        /* IP packet? */
        case ETHTYPE_IP:
            /* skip Ethernet header */
            pbuf_header(p, (s16_t)(-sizeof(struct eth_hdr)));
            /* pass to network layer */
            netif->input(p, netif);
            break;
        
        case ETHTYPE_ARP:
            /* pass p to ARP module  */
            etharp_arp_input(netif, &gmacif->ethaddr, p);
            break;
        default:
            pbuf_free(p);
            p = NULL;
            break;
        }
}

/*----------------------------------------------------------------------------
 *        Exported functions
 *----------------------------------------------------------------------------*/


/**
 * Set the MAC address of the system.
 * Should only be called before gmacif_init is called.
 * The stack calls gmacif_init after the user calls netif_add
 *
 */

void gmacif_setmac(u8_t *addr)
{
    Gmacif_config.ethaddr.addr[0] = addr[0];
    Gmacif_config.ethaddr.addr[1] = addr[1];
    Gmacif_config.ethaddr.addr[2] = addr[2];
    Gmacif_config.ethaddr.addr[3] = addr[3];
    Gmacif_config.ethaddr.addr[4] = addr[4];
    Gmacif_config.ethaddr.addr[5] = addr[5];
}

/**
 * Should be called at the beginning of the program to set up the
 * network interface. It calls the function glow_level_init() to do the
 * actual setup of the hardware.
 *
 */
err_t gmacif_init(struct netif *netif)
{
    struct gmacif *gmacif;
    gmacif = &Gmacif_config;
    if (gmacif == NULL)
    {
        LWIP_DEBUGF(NETIF_DEBUG, ("gmacif_init: out of memory\n"));
        return ERR_MEM;
    }
    netif->state = gmacif;
    netif->name[0] = IFNAME0;
    netif->name[1] = IFNAME1;
    netif->output = gmacif_output;
    netif->linkoutput = glow_level_output;
    _mem_barrier_
    glow_level_init(netif);
    etharp_init();
    return ERR_OK;
}

/**
 * Polling task
 * Should be called periodically
 *
 */
void gmacif_poll(struct netif *netif)
{
    gmacif_input(netif);
}

