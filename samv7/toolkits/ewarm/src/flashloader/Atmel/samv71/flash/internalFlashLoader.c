/* ----------------------------------------------------------------------------
 *         SAM Software Package License
 * ----------------------------------------------------------------------------
 * Copyright (c) 2011, Atmel Corporation
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

/** \addtogroup flash_flashloader Embedded Flash Flashloader
 *
 * It describes how to write and debug your own flash loader.
 * Finally, the flash loader framework API functions are described in detail.
 *
 * Related files :\n
 * \ref flashd.c\n
 * \ref flashd.h.\n
 * \ref efc.c\n
 * \ref efc.h.\n
*/
/*@{*/
/*@}*/


/**
 * \file
 *
 * The flash flashloader provides the embedded flash flashloader framework.
 *
 */

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/
#include <samv71.h>
#include <efc.h>
#include <flashd.h>
#include "interface.h"
#include "flash_loader.h"
#include "flash_loader_extra.h"

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

/*----------------------------------------------------------------------------
 *        Local definitions
 *----------------------------------------------------------------------------*/
/* Software version */
#define VERSION "1.0"

/* Debug enable/disable */
#define DEBUG

/* Boot mode bit for GPNMV */
#define GPNMV_BOOT_MODE_BIT      1

/* Bank selection bit for GPNMV */
#define GPNMV_BANK_SELECTION_BIT 2

/*----------------------------------------------------------------------------
 *        Local Varible
 *----------------------------------------------------------------------------*/
static char message[80];
static int8_t SectorErased[IFLASH_SIZE / IFLASH_SECTOR_SIZE];

/*----------------------------------------------------------------------------
 *        Export functions
 *----------------------------------------------------------------------------*/

/**
 * \brief Initializes the flash driver.
 *
 * \param base_of_flash  base address of flash memory.
 * \param image_size The size of the whole image that is to be written in bytes.
 * \param link_address The original link address of the first byte of the image.
 * \param flags Contains optional flags.
 * \param  argc Number of arguments.
 * \param  An array of argument string pointers.
 * \return 0 if successful; otherwise returns an error code.
 */
uint32_t FlashInit(void *base_of_flash,
                   uint32_t image_size,
                   uint32_t link_address,
                   uint32_t flags,
                   int argc,
                   char const *argv[])

{
    uint32_t i;

    /* stop warning */
    base_of_flash = base_of_flash;
    image_size = image_size;
    link_address = link_address;
    flags = flags;

    if(!findOption( "--flash", 0, argc, argv )){
        return RESULT_ERROR;
    }

#ifdef DEBUG
    sprintf(message, "%s", "-I- Internal flash loader");
    cSpyMessageLog(message);
#endif

    if (findOption( "--boot", 0, argc, argv )) {
#ifdef DEBUG
        sprintf(message, "-I- Boot from Flash ");
        cSpyMessageLog(message);
#endif
        FLASHD_SetGPNVM(GPNMV_BOOT_MODE_BIT);
    }
    else {
        FLASHD_ClearGPNVM(GPNMV_BOOT_MODE_BIT);
    }

	for (i = 0; i < (IFLASH_SIZE / IFLASH_SECTOR_SIZE); i++) {
        SectorErased[i] = 0;
    }

    return RESULT_OK;
}

/**
 * \brief Writes a data buffer in the internal flash.
 *
 * \param block_start  address of the start of the flash memory block affected by the write operation.
 * \param offset_into_block  How far into the current block that this write operation shall start.
 * \param count  Size of data buffer in bytes.
 * \param buffer Points to the buffer containing the bytes to write.
 * \return 0 if successful; otherwise returns an error code.
 */

uint32_t FlashWrite(void *block_start, uint32_t offset_into_block, uint32_t count, char const *buffer)
{
#ifdef DEBUG
    sprintf(message, "-I- Write arguments: address 0x%08x,  offset 0x%x of 0x%x Bytes", (unsigned int)block_start, offset_into_block,count );
    cSpyMessageLog(message);
#endif
    // Write data
    if (FLASHD_Write((unsigned int)block_start + offset_into_block, buffer, count) != 0) {
#ifdef DEBUG
        sprintf(message, "-E- Failed to write!");
        cSpyMessageLog(message);
#endif
        return RESULT_ERROR;
    }
    sprintf(message, "-I- Write Done!");
    cSpyMessageLog(message);
    return RESULT_OK;
}

/**
 * \brief  Unlock the flash in giving address. (auto erased before program).
 *
 * \param block_start Address of the start of the flash memory block affected by the write operation.
 * \param block_size  The size of the block, in bytes.
 * \return 0 if successful; otherwise returns an error code.
 */
uint32_t FlashErase(void *block_start, uint32_t block_size)
{
    uint32_t id;
    uint32_t startAddr;
    uint32_t endAddr;


#ifdef DEBUG
    sprintf(message, "-I- Erase arguments: address 0x%08x of 0x%x Bytes", (unsigned int)block_start, block_size);
    cSpyMessageLog(message);
#endif

    id = ((uint32_t)block_start - IFLASH_ADDR) / IFLASH_SECTOR_SIZE;
    if (SectorErased[id] == 0) {
        startAddr = IFLASH_ADDR + id * IFLASH_SECTOR_SIZE;
        endAddr = startAddr + IFLASH_SECTOR_SIZE;
#ifdef DEBUG
        sprintf(message, "-I- Unlock arguments: address 0x%08x of 0x%x Bytes", startAddr, IFLASH_SECTOR_SIZE);
        cSpyMessageLog(message);
#endif
        if ( FLASHD_Unlock(startAddr, endAddr, 0, 0) != 0) {
#ifdef DEBUG
            sprintf(message, "-E- Failed to unlock!");
            cSpyMessageLog(message);
#endif
            return RESULT_ERROR;
        }

        /* Special erase for first sector */
        if (id == 0) {
            FLASHD_EraseSector(IFLASH_ADDR);             /* Small sector 0: 8K */
            FLASHD_EraseSector(IFLASH_ADDR + 1024 * 8);  /* Small sector 1: 8K */
            FLASHD_EraseSector(IFLASH_ADDR + 1024 * 16); /* Large sector  : 48K */

        } else {
            /* Common erase Other sectors */
            FLASHD_EraseSector(startAddr);
        }
#ifdef DEBUG
        sprintf(message, "-I- erase Done!");
        cSpyMessageLog(message);
#endif
        SectorErased[id] = 1;
    }

    return RESULT_OK;
}


/**
 * \brief  UThis is an optional function. You can implement it if you need to perform
 * some cleanup after flash loading has finished.
 */
uint32_t FlashSignoff()
{
    return RESULT_OK;
}

