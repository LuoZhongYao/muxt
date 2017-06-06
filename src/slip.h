/*************************************************
 * Anthor  : LuoZhongYao@gmail.com
 * Modified: 2017/05/06
 ************************************************/
#ifndef __SLIP_H__
#define __SLIP_H__

#include <stdint.h>
#include "pt/pt.h"

void *slip_create(uint16_t len,
                  void *ctx,
                  void (*recv)(void *ctx, const uint8_t *payload, uint16_t len),
                  void (*putch)(void *ctx, uint8_t ch));
void slip_destroy(void *slip);
void slip_encode(void *_slip, const uint8_t *payload, uint16_t len);
PT_THREAD(slip_decode)(void *_slip, const uint8_t *payload, uint16_t len);

#endif

