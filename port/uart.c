/* CMSDK UART on the MPS2-AN385, register level, no HAL.
 * UART0 base 0x40004000:  DATA +0x00, STATE +0x04, CTRL +0x08
 * STATE bit 0 = TX buffer full. */
#include <stdint.h>
#include <stddef.h>

#define UART0_BASE  0x40004000u
#define UART_DATA   (*(volatile uint32_t *)(UART0_BASE + 0x00))
#define UART_STATE  (*(volatile uint32_t *)(UART0_BASE + 0x04))
#define UART_CTRL   (*(volatile uint32_t *)(UART0_BASE + 0x08))
#define UART_BAUDDIV (*(volatile uint32_t *)(UART0_BASE + 0x10))
#define STATE_TX_FULL 0x01u

void uart_init(void)
{
    UART_BAUDDIV = 16u;   /* minimum legal divisor for the CMSDK UART */
    UART_CTRL    = 0x01u; /* TX enable */
}

void uart_putc(char ch)
{
    while (UART_STATE & STATE_TX_FULL) { }
    UART_DATA = (uint32_t)ch;
}

void uart_puts(const char *s)
{
    while (*s) {
        if (*s == '\n') uart_putc('\r');
        uart_putc(*s++);
    }
}

/* newlib calls this for printf. Retargeting _write is the whole of "make
 * printf work on bare metal". */
int _write(int fd, const char *buf, int len)
{
    (void)fd;
    for (int i = 0; i < len; i++) {
        if (buf[i] == '\n') uart_putc('\r');
        uart_putc(buf[i]);
    }
    return len;
}

/* newlib syscall stubs: without these the link fails with undefined
 * references, which is the first thing every bare-metal newlib build hits. */
int   _close(int f)                       { (void)f; return -1; }
int   _fstat(int f, void *st)             { (void)f; (void)st; return 0; }
int   _isatty(int f)                      { (void)f; return 1; }
int   _lseek(int f, int p, int w)         { (void)f; (void)p; (void)w; return 0; }
int   _read(int f, char *b, int l)        { (void)f; (void)b; (void)l; return 0; }
void  _exit(int c)                        { (void)c; for (;;) { } }
int   _kill(int p, int s)                 { (void)p; (void)s; return -1; }
int   _getpid(void)                       { return 1; }

extern char end;
static char *heap = 0;
void *_sbrk(int incr)
{
    char *prev;
    if (heap == 0) heap = &end;
    prev = heap;
    heap += incr;
    return prev;
}
