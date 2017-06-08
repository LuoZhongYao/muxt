/*************************************************
 * Anthor  : LuoZhongYao@gmail.com
 * Modified: 2017/05/30
 ************************************************/
#ifndef __FTMGR_H__
#define __FTMGR_H__
#include <stdint.h>
#include <stdbool.h>
#include "channel.h"

struct ftmgr;
void free_ftmgr(struct ftmgr **ftmgr);
struct ftmgr *alloc_ftmgr(const char *name, struct chmgr *chmgr, uint32_t *mode);
void ftmgr_fill(struct ftmgr *ftmgr);
void ftmgr_timedout(struct ftmgr *ftmgr);
int ftmgr_aeq_handle(struct ftmgr *, uint8_t aeq, const uint8_t offset[4]);

#endif

