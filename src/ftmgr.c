#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include "channel.h"
#include "ftmgr.h"
#include "log.h"

#define FTQ_NR 8
struct __ftb
{
    int8_t tts;
    uint8_t *blk;
    uint16_t size;
};

struct ftmgr
{
    int fd;
    int8_t tts;
    uint8_t bitmap;
    uint32_t offset;
    struct progress pr;
    struct __ftb ftq[FTQ_NR];
    struct chmgr *chmgr;
};

void ftmgr_fill(struct ftmgr *ftmgr)
{
    if(ftmgr->fd == -1)
        return;
    while(ftmgr->bitmap) {
        int rn;
        uint8_t seq;
        seq = ffs(ftmgr->bitmap) - 1;

        rn =read(ftmgr->fd, ftmgr->ftq[seq].blk + BLK_HDR_SIZE, BLK_MTU - BLK_HDR_SIZE);
        if(rn <= 0) {
            close(ftmgr->fd);
            ftmgr->fd = -1;
            break;
        }

        ftmgr->bitmap &= ~(1 << seq);
        U32_TO_STREAM(ftmgr->ftq[seq].blk, ftmgr->offset);
        ftmgr->offset += rn;
        ftmgr->ftq[seq].size = rn + BLK_HDR_SIZE;
        ftmgr->ftq[seq].tts = ftmgr->tts++;
    }
}

void ftmgr_timedout(struct ftmgr *ftmgr)
{
    int seq = -1, tts = 0;
    for(int i = 0;i < FTQ_NR;i++) {
        ftmgr->ftq[i].tts--;
        if(!(ftmgr->bitmap & (1 << i)) && ftmgr->ftq[i].tts < tts) {
            seq = i;
            tts = ftmgr->ftq[i].tts;
        }
    }

    if(seq != -1) {
        ftmgr->ftq[seq].tts = ftmgr->tts;
        fm_write(ftmgr->chmgr, CTRL_BLK_SEQ0 + seq, ftmgr->ftq[seq].blk, ftmgr->ftq[seq].size);
    }
}

int ftmgr_aeq_handle(struct ftmgr *ftmgr, uint8_t aeq, const uint8_t offset[4])
{
    if(!(ftmgr->bitmap & (1 << aeq)) && !memcmp(ftmgr->ftq[aeq].blk, offset, 4)) {
        ftmgr->tts--;
        ftmgr->bitmap |= (1 << aeq);
        progress(&ftmgr->pr, ftmgr->ftq[aeq].size - BLK_HDR_SIZE, "send");
        ftmgr_fill(ftmgr);
        ftmgr_timedout(ftmgr);
    } else if(memcmp(ftmgr->ftq[aeq].blk, offset, 4)) {
        LOGE("packted resend seq = %02x\n", aeq);
    }

    if(ftmgr->bitmap == (__typeof__(ftmgr->bitmap))-1) {
        fm_write(ftmgr->chmgr, CTRL_BLK_COMP, NULL, 0);
        return 0;
    }
    return 1;
}

void free_ftmgr(struct ftmgr **ftmgr)
{
    struct ftmgr *ft = *ftmgr;
    *ftmgr = NULL;
    for(int i = 0;i < FTQ_NR;i++) {
        free(ft->ftq[i].blk);
    }
    if(ft->fd != -1)
        close(ft->fd);
    free(ft);
}

struct ftmgr *alloc_ftmgr(const char *name, struct chmgr *chmgr, uint32_t *mode)
{
    struct stat statbuf;
    struct ftmgr *ftmgr = (struct ftmgr*)malloc(sizeof(struct ftmgr));
    if(stat(name, &statbuf) < 0)
        goto _err;
    *mode = statbuf.st_mode;
    progress_init(&ftmgr->pr, statbuf.st_size, chmgr);

    ftmgr->fd = open(name, O_RDONLY);
    if(ftmgr->fd < 0)
        goto _err;

    ftmgr->offset = 0;
    ftmgr->chmgr = chmgr;
    ftmgr->tts = 0;
    ftmgr->bitmap = (__typeof__(ftmgr->bitmap))-1;
    for(int i = 0;i < FTQ_NR;i++) {
        ftmgr->ftq[i].blk = malloc(BLK_MTU);
    }
    return ftmgr;
_err:
    free(ftmgr);
    return NULL;
}

