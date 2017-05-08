/*************************************************
 * Anthor  : LuoZhongYao@gmail.com
 * Modified: 2017/05/06
 ************************************************/
#ifndef __SLIP_H__
#define __SLIP_H__

#include <stdint.h>
void slip_send_packet(uint8_t *p, int len, int (*putc)(void *ctx, uint8_t c), void *ctx);
int slip_read_packet(uint8_t *p, int (*getc)(void *ctx, uint8_t *c), void *ctx) ;



#endif

