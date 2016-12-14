
/**
* \file
*
* Implementation of low level library.
*
*/

#if defined ( __ICCARM__ ) /* IAR Ewarm 5.41+ */

#include "ff.h"

#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <yfuns.h>

extern size_t __write( int handle, const unsigned char *buf, size_t bufSize )
{
    size_t nChars = 0 ;

    /* Check for the command to flush all handles */
    if ( handle == -1 )
    {
        return 0 ;
    }

    /* Check for stdout and stderr (only necessary if FILE descriptors are enabled.) */
    if ( handle != 1 && handle != 2 )
    {
        /* remove warnings */
        return 0xfffffff ;
    }

    f_write ((FIL *)handle, buf, (UINT)bufSize, (UINT *)&nChars);
    
    return nChars ;
}


extern size_t __read( int handle, unsigned char *buf, size_t bufSize ) 
{
  size_t nChars = 0 ;

  /* Check for stdin (only necessary if FILE descriptors are enabled) */
  if ( handle != 0 )
  {
    /* remove warnings */
    return 0xfffffff ;
  }

  f_read ((FIL *)handle, buf, (UINT)bufSize,(UINT *) nChars);

  return nChars ;
}

extern long __lseek(int handle, long offset, int whence)
{
  return -1;
}

extern int __close(int handle)
{
  return 0;
}

extern int remove(const char * filename)
{
  return 0;
}
#endif
