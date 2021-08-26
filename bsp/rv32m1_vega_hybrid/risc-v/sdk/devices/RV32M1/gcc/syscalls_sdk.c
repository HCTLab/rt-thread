#include <stdio.h>
#include <sys/stat.h>

#include <fsl_lpuart.h>

int _fstat (int fd, struct stat *pStat)
{
    pStat->st_mode = S_IFCHR;
    return 0;
}

int _close(int fd)
{
    return -1;
}

int _write (int fd, char *pBuffer, int size)
{
    LPUART_WriteBlocking( LPUART0, (uint8_t *) pBuffer, size );
    return size;
}

int _isatty (int fd)
{
    return 1;
}

int _lseek(int fd, int a, int b)
{
    return -1;
}

int _read (int fd, char *pBuffer, int size)
{
    LPUART_ReadBlocking( LPUART0, (uint8_t *) pBuffer, size );
    return size;
}

caddr_t _sbrk(int increment)
{
    extern char end   asm("__StackLimit");
    extern char base  asm("__StackTop");

    static char *s_pHeapEnd, *s_pHeapBase;

    s_pHeapEnd  = &end;
    s_pHeapBase = &base;

    if (s_pHeapEnd + increment > s_pHeapBase)
        return (caddr_t) -1;

    return (caddr_t) s_pHeapEnd;
}
