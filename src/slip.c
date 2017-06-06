#define LOG_TAG "slip"

#include <stdlib.h>

#include "slip.h"
#include "log.h"

#define SLIP_END             0xc0    /* indicates end of packet */
#define SLIP_ESC             0xdb    /* indicates byte stuffing */
#define SLIP_ESC_END         0xdc    /* ESC ESC_END means END data byte */
#define SLIP_ESC_ESC         0xdd    /* ESC ESC_ESC means ESC data byte */

struct  slip 
{
    uint16_t  size;
    uint16_t  len;
    void *ctx;
    struct pt pt;
    void (*recv)(void *ctx, const uint8_t *payload, uint16_t len);
    void (*putch)(void *ctx, uint8_t ch);
    uint8_t  buffer[0];
};

void *slip_create(uint16_t size,
                  void *ctx,
                  void (*recv)(void *ctx, const uint8_t *payload, uint16_t len),
                  void (*putch)(void *ctx, uint8_t ch))
{
    struct slip *slip = (struct slip*)malloc(sizeof(struct slip) + size);
    slip->size = size;
    slip->len = 0;
    slip->recv = recv;
    slip->ctx = ctx;
    slip->putch = putch;
    PT_INIT(&slip->pt);
    return (void *)slip;
}

void slip_encode(void *_slip, const uint8_t *payload, uint16_t len)
{
    struct slip *slip = (struct slip *)_slip;
    slip->putch(slip->ctx, SLIP_END);
    for(uint16_t i = 0;i < len;i++) {
        if(payload[i] == SLIP_END) {
            slip->putch(slip->ctx, SLIP_ESC);
            slip->putch(slip->ctx, SLIP_ESC_END);
        } else if(payload[i] == SLIP_ESC) {
            slip->putch(slip->ctx, SLIP_ESC);
            slip->putch(slip->ctx, SLIP_ESC_ESC);
        } else {
            slip->putch(slip->ctx, payload[i]);
        }
    }
    slip->putch(slip->ctx, SLIP_END);
}

PT_THREAD(slip_decode)(void *_slip, const uint8_t *payload, uint16_t len)
{
    struct slip *slip = (struct slip *)_slip;
    int i = 0;
    PT_BEGIN(&slip->pt);

__try_agin:
    slip->len = 0;
    for(;;i++) {
        PT_WAIT_UNTIL(&slip->pt, i < len);
        //LOGD("%02x ", payload[i]);
        if(payload[i] == SLIP_END) {
            if(slip->len) {
                slip->recv(slip->ctx, slip->buffer, slip->len);
            }
            i++;
            goto __try_agin;
        }

        if(slip->len >= slip->size)
            goto __try_agin;

        if(payload[i] == SLIP_ESC) {
            i++;
            PT_WAIT_UNTIL(&slip->pt, i < len);
            switch(payload[i]) {
            case SLIP_ESC_END: slip->buffer[slip->len++] = SLIP_END; break;
            case SLIP_ESC_ESC: slip->buffer[slip->len++] = SLIP_ESC; break;
            default: goto __try_agin;
            }
        } else {
            slip->buffer[slip->len++] = payload[i];
        }
    }
    PT_END(&slip->pt);
}
