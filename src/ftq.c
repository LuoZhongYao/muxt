#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include "channel.h"
#include "ftq.h"
#include "log.h"

#define FTQ_NR 8
struct task_chain
{
    int (*fn)(struct ftq *ftq); /* 1: complete, 0:agin, -1: error */
    struct task_chain *next;
};

struct __ftb
{
    int8_t tts;
    uint8_t *blk;
    uint16_t size;
};

struct ftq
{
    int fd;
    int8_t tts;
    uint8_t bitmap;
    uint32_t offset;
    struct progress pr;
    struct __ftb ftb[FTQ_NR];
    struct chmgr *chmgr;
    struct task_chain *task_chain;
};

static int ftq_load_file(struct ftq *ftq)
{
    int ret;
    uint8_t *blk, seq;
    if(ftq->fd < 0 || !(ftq->bitmap))
        return 0;
    while((ftq->bitmap)) {
        blk = (uint8_t*)malloc(BLK_MTU);

        ret = read(ftq->fd, blk + BLK_HDR_SIZE, BLK_MTU - BLK_HDR_SIZE);
        if(ret <= 0 )
            goto __quit;

        seq = ffs(ftq->bitmap) - 1;
        ftq->bitmap &= ~(1 << seq);
        U32_TO_STREAM(blk, ftq->offset);
        ftq->offset += ret;
        ftq->ftb[seq].size = ret + BLK_HDR_SIZE;
        ftq->ftb[seq].blk = blk;
        ftq->ftb[seq].tts = ftq->tts;
        ftq->tts++;
    }

    return 0;
__quit:
    free(blk);
    return ret == 0 ? 1 : -1;
}

static int ftq_send_block(struct ftq *ftq)
{
    if(ftq->bitmap == (__typeof__(ftq->bitmap))-1)
        return 1;
    for(int i = 0;i < FTQ_NR;i++) {
        //LOGD("ftq->ftb[%d] = %d\n", i, ftq->ftb[i].tts);
        if(!(ftq->bitmap & (1 << i)) && (ftq->ftb[i].tts--) <= 0) {
            ftq->ftb[i].tts = ftq->tts;
            fm_write(ftq->chmgr, CTRL_BLK_SEQ0 + i, ftq->ftb[i].blk, ftq->ftb[i].size);
        }
    }
    return 0;
}

static int ftq_transm_complete(struct ftq *ftq)
{
    uint8_t status = CTRL_STATUS_SUCCESS;
    if(ftq->bitmap == (__typeof__(ftq->bitmap))-1) {
        fm_write(ftq->chmgr, CTRL_BLK_COMP, &status, 1);
        return 1;
    }
    return 0;
}

static struct task_chain *alloc_task_chain(int (*fn)(struct ftq *), struct task_chain *next)
{
    struct task_chain *tc = (struct task_chain*)malloc(sizeof(struct task_chain));
    tc->fn = fn;
    tc->next = next;
    return tc;
}
static int _wait_remote_reply(struct ftq *ftq __unused)
{
    return 0;
}

static struct task_chain wait_remote_reply = 
{
    .fn = _wait_remote_reply,
    .next = NULL
};

void ftq_rm(struct ftq *ftq, uint8_t seq)
{
    if(ftq) {
        ftq->tts--;
        if(!(ftq->bitmap & (1 << seq))) {
            ftq->bitmap |= (1 << seq);
            progress(&ftq->pr, ftq->ftb[seq].size - BLK_HDR_SIZE, "send");
            free(ftq->ftb[seq].blk);
        }
    }
}

struct ftq *alloc_ftq(const char *name, struct chmgr *chmgr, uint32_t *mode)
{
    struct stat statbuf;
    struct ftq *ftq = (struct ftq*)malloc(sizeof(struct ftq));
    if(stat(name, &statbuf) < 0)
        goto _err;
    *mode = statbuf.st_mode;
    progress_init(&ftq->pr, statbuf.st_size, chmgr);

    ftq->fd = open(name, O_RDONLY);
    if(ftq->fd < 0)
        goto _err;
    ftq->bitmap = (__typeof__(ftq->bitmap))-1;
    ftq->offset = 0;
    ftq->chmgr = chmgr;
    ftq->tts = 0;
    ftq->task_chain = &wait_remote_reply;
    return ftq;
_err:
    free(ftq);
    return NULL;
}

void ftq_alloc_task_chain(struct ftq *ftq)
{
    ftq->task_chain = alloc_task_chain(ftq_load_file, alloc_task_chain(ftq_send_block, alloc_task_chain(ftq_transm_complete, NULL)));
}

void ftq_reset(struct ftq *ftq, uint8_t seq)
{
    if(ftq) {
        ftq->ftb[seq].tts = 0;
    }
}

void ftq_free(struct ftq **ftq)
{
    close((*ftq)->fd);
    free(*ftq);
    *ftq = NULL;
}

bool ftq_task_process(struct ftq *ftq)
{
    int res;
    struct task_chain **tc;
    if(ftq == NULL)
        return false;
    tc = &ftq->task_chain;
    while(*tc) {
        res = (*tc)->fn(ftq);
        if(res == 1) {
            struct task_chain *cur = *tc;
            *tc = (*tc)->next;
            free(cur);
        } else  {
            if(res == -1) {
                LOGE("task_process: %s\n", strerror(errno));
            }
            tc = &(*tc)->next;
        }
    }
    return ftq->task_chain == NULL;
}
