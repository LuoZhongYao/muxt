/*************************************************
 * Anthor  : LuoZhongYao@gmail.com
 * Modified: 2017/05/30
 ************************************************/
#ifndef __CHANNEL_H__
#define __CHANNEL_H__
#include <stdint.h>
#include <sys/time.h>
#include <stdlib.h>
#include <poll.h>
#include "log.h"

#define __unused __attribute__((unused))
#define ffs(n) __builtin_ffs(n)
#define __packed __attribute__((packed))

#define STREAM_TO_U16(p) ({\
        uint8_t *__ptr = (uint8_t*)p;\
        (uint16_t)__ptr[0] | ((uint16_t)__ptr[1]) << 8; })

#define STREAM_TO_U32(p) ({\
        uint8_t *__ptr = (uint8_t*)p;\
        (uint32_t)__ptr[0] | ((uint32_t)__ptr[1]) << 8 | ((uint32_t)__ptr[2]) << 16 | ((uint32_t)__ptr[3]) << 24; })

#define U16_TO_STREAM(p,v) do{\
    uint16_t __v = v; \
    uint8_t *__ptr = (uint8_t*)p;\
    __ptr[0] = __v & 0xff;\
    __ptr[1] = (__v >> 8) & 0xff;\
} while(0)

#define U32_TO_STREAM(p,v) do{\
    uint32_t __v = v; \
    uint8_t *__ptr = (uint8_t*)p;\
    __ptr[0] = __v & 0xff;\
    __ptr[1] = (__v >> 8) & 0xff;\
    __ptr[2] = (__v >> 16) & 0xff;\
    __ptr[3] = (__v >> 24) & 0xff;\
} while(0)

struct chmgr;
struct ftq;
void mgr_print(struct chmgr *chmgr, const char *fmt, ...);

struct progress
{
    struct chmgr *chmgr;
    unsigned long pr_total;
    unsigned long pr_cursize;
    unsigned long pr_time;
    unsigned long pr_start;
};

static inline void progress_init(struct progress *pr, unsigned long total, struct chmgr *chmgr)
{
    pr->pr_total = total;
    pr->pr_cursize = 0;
    pr->pr_time = 0;
    pr->pr_start = 0;
    pr->chmgr = chmgr;
}

static inline void progress(struct progress *pr, unsigned long size, const char *prompt)
{
    int prog;
    struct  timeval    tv;
    uint64_t now, interval;
    gettimeofday(&tv, NULL);
    now = tv.tv_sec * 1000000 + tv.tv_usec;
    pr->pr_cursize += size;
    prog = pr->pr_cursize * 100 / pr->pr_total;
    if(pr->pr_start == 0)
        pr->pr_start = now;

    interval = now - pr->pr_time;
    if(pr->pr_time != 0) {
        mgr_print(pr->chmgr,
        //LOGI(
                "%s %3d%% [%.*s%.*s] %ld B/s %ld/%ld KB    \r", 
                prompt,
                prog,
                prog / 4, "#########################",
                25 - (prog / 4), "                         ",
                (size * 1000000) / interval,
                pr->pr_cursize / 1024,
                pr->pr_total / 1024);
    }

    if(pr->pr_cursize == pr->pr_total) {
        interval = now - pr->pr_start;
        mgr_print(pr->chmgr, "\nTotal: %ld KB, %ld S, %ld B/s\n", pr->pr_total / 1024, interval / 1000000, (pr->pr_total * 1000000) / interval);
    }
    pr->pr_time = now;
}

enum 
{
    CTRL_MGR       = 0,
    /* 1 - 30 */        /* logical channel */
    /* 32 */            /* physical channel */
    CTRL_BLK_SEQ0  = 33,     /* 64 - 96 blk seq */
    CTRL_BLK_SEQ31 = 64,
    CTRL_BLK_AEQ0  = 65,
    CTRL_BLK_AEQ31 = 96,

    CTRL_BLK_COMP,
    CTRL_FSTAT_REQ,         /* file stat */
    CTRL_FSTAT_RSP,
    CTRL_HEARTBEAT,
    CTRL_HEARTBEAT_ACK,
    CTRL_LOGICAL_MGR,
};

#define SEQ_TO_AEQ(seq) (seq - CTRL_BLK_SEQ0 + CTRL_BLK_AEQ0)

enum
{
    CTRL_STATUS_SUCCESS,
    CTRL_STATUS_FAILED,
    CTRL_STATUS_ALREADY_EXISTS,     /* */
    CTRL_STATUS_BUSY,
};

//struct blk 
//{
//    uint32_t offset;
//};
//
//struct bln
//{
//    uint8_t status;
//};
//
//
//struct fstat
//{
//    uint32_t mode;
//    unsigned char name[0];
//};
//
//struct fm
//{
//    uint8_t  fn;
//    uint16_t len;
//    unsigned char payload[0];
//    /*uint8_t crc[2]; */ 
//};

struct chmgr
{
    int fd;
    int baudrate;
    int heartbeat;
    void *slip;
    struct ftq *ftq;
    const char *shell;
    const char *prefix;
    const char *server;
    int (*cbs[32])(struct chmgr *, int id);
    struct pollfd fds[32];
};


#define FM_HDR_SIZE     3
#define FSTAT_HDR_SIZE  4
#define BLK_HDR_SIZE    4
#define BLN_HDR_SIZE    1

#define SLIP_MTU        512
#define FM_MTU          (SLIP_MTU - 2)
#define BLK_MTU         (FM_MTU - FM_HDR_SIZE)

extern int fm_write(struct chmgr *chmgr, uint8_t fn, const void *buf, uint16_t size);

#endif

