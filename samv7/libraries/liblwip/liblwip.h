#ifndef _LIB_LWIP 
#define _LIB_LWIP 


/* Define attribute */
#if defined   ( __CC_ARM   ) /* Keil µVision 4 */
    #define WEAK __attribute__ ((weak))
#elif defined ( __ICCARM__ ) /* IAR Ewarm 5.41+ */
    #define WEAK __weak
#elif defined (  __GNUC__  ) /* GCC CS3 2009q3-68 */
    #define WEAK __attribute__ ((weak))
#endif

/* Define NO_INIT attribute */
#if defined   ( __CC_ARM   )
    #define NO_INIT
#elif defined ( __ICCARM__ )
    #define NO_INIT __no_init
#elif defined (  __GNUC__  )
    #define NO_INIT
#endif

/*
 * drivers
 */
 
#ifdef libwip_1_4_1
#include "source/lwip_1_4_1/src/include/lwip/opt.h"
#include "source/lwip_1_4_1/src/include/lwip/tcp.h"
#include "source/lwip_1_4_1/src/include/lwip/tcp_impl.h"
#include "source/lwip_1_4_1/src/include/lwip/init.h"
#include "source/lwip_1_4_1/src/include/lwip/memp.h"
#include "source/lwip_1_4_1/src/include/lwip/udp.h"
#include "source/lwip_1_4_1/src/include/lwip/dhcp.h"
#else
#include "source/lwip_1.3.2/src/include/lwip/opt.h"
#include "source/lwip_1.3.2/src/include/lwip/tcp.h"
#include "source/lwip_1.3.2/src/include/lwip/init.h"
#include "source/lwip_1.3.2/src/include/lwip/memp.h"
#include "source/lwip_1.3.2/src/include/lwip/udp.h"
#include "source/lwip_1.3.2/src/include/lwip/dhcp.h"

#endif

#include "source/samv7-specific/arch/sys_arch.h"
#include "source/samv7-specific/gmacif.h"
#endif /* _LIB_LWIP */
