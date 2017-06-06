/*************************************************
 * Anthor  : LuoZhongYao@gmail.com
 * Modified: 2017/05/30
 ************************************************/
#ifndef __FTQ_H__
#define __FTQ_H__
#include <stdint.h>
#include <stdbool.h>
#include "channel.h"

struct ftq;
void ftq_rm(struct ftq *ftq, uint8_t seq);
struct ftq *alloc_ftq(const char *name, struct chmgr *chmgr, uint32_t *mode);
void ftq_free(struct ftq **ftq);
bool ftq_task_process(struct ftq *ftq);
void ftq_alloc_task_chain(struct ftq *ftq);
void ftq_reset(struct ftq *ftq, uint8_t seq);

#endif

