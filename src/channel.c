#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <stdbool.h>
#include <alloca.h>
#include <fcntl.h>
#include <poll.h>
#include <unistd.h>
#include <stdarg.h>
#include <arpa/inet.h>

#include "channel.h"
#include "slip.h"
#include "ftq.h"
#include "log.h"

int loglevel = LOGLEVEL_ERROR | LOGLEVEL_WARNING;
FILE *logfile = NULL;
extern int open_serialport(const char *dev, int baudrate);
extern void close_pty(const char *prefix, int fd, int idx);
extern int open_pty(const char *prefix, int idx);
extern int load_shell(const char *sh);

static uint16_t __calc_crc(uint8_t ch, uint16_t crc)
{
    /* Calculate the CRC using the above 16 entry lookup table */

    static const uint16_t crc_table[] = {
        0x0000, 0x1081, 0x2102, 0x3183,
        0x4204, 0x5285, 0x6306, 0x7387,
        0x8408, 0x9489, 0xa50a, 0xb58b,
        0xc60c, 0xd68d, 0xe70e, 0xf78f
    };

    /* Do this four bits at a time - more code, less space */

    crc = (crc >> 4) ^ crc_table[(crc ^ ch) & 0x000f];
    crc = (crc >> 4) ^ crc_table[(crc ^ (ch >> 4)) & 0x000f];

    return crc;
}

static uint16_t calc_crc(const uint8_t *payload, uint16_t length)
{
    uint16_t crc = 0;
    for(int i = 0;i < length;i++) {
        crc = __calc_crc(payload[i], crc);
    }
    return crc;
}

static void chmgr_init(struct chmgr *chmgr)
{
    memset(chmgr, 0, sizeof(struct chmgr));
    for(int i = 0;i < 32;i++) {
        chmgr->fds[i].fd = -1;
    }

    chmgr->baudrate = 460800;
    chmgr->ftq = NULL;
    chmgr->fd = -1;
    chmgr->server = "muxtd -s /tmp/mux";
    chmgr->prefix = "/tmp/mux";
    chmgr->shell = "/bin/sh";
    chmgr->heartbeat = 20;
}

static int alloc_channel(struct chmgr *chmgr, int (*cb)(struct chmgr *, int id), int id)
{
    if(chmgr->fds[id].fd > 0)
        return -1;

#if defined(MUXT)
    chmgr->fds[id].fd = open_pty(chmgr->prefix, id);
#else
    chmgr->fds[id].fd = load_shell(chmgr->shell);
#endif
    chmgr->cbs[id] = cb;
    if(chmgr->fds[id].fd < 0)
        return -1;

    chmgr->fds[id].events = POLLIN;
    return 0;
}

static void free_channel(struct chmgr *chmgr, int id)
{
    close_pty(chmgr->prefix, chmgr->fds[id].fd, id);
    chmgr->fds[id].fd = 0;
    chmgr->fds[id].events = 0;
}

static void handle_fstat_req(struct chmgr *chmgr, const uint8_t *buf, uint16_t len __unused)
{
    mode_t mode = STREAM_TO_U32(buf);
    uint8_t status = CTRL_STATUS_SUCCESS;
    if(chmgr->fd >= 0) {
        close(chmgr->fd);
        chmgr->fd = -1;
    }

    if(0 > (chmgr->fd = creat((const char*)(buf + FSTAT_HDR_SIZE), mode))) {
        status = CTRL_STATUS_FAILED;
        LOGE("Create %s: %s\n", buf + FSTAT_HDR_SIZE, strerror(errno));
    }
    fm_write(chmgr, CTRL_FSTAT_RSP, &status, 1);
}

static void handle_fstat_rsp(struct chmgr *chmgr, uint8_t status)
{
    if(status != CTRL_STATUS_SUCCESS)
        goto __err;
    ftq_alloc_task_chain(chmgr->ftq);
__err: ;
}


static void handle_blk_seq(struct chmgr *chmgr, uint8_t seq, const uint8_t *buf, uint16_t len)
{
    uint32_t offset = STREAM_TO_U32(buf);
    uint8_t status = CTRL_STATUS_FAILED;
    if(chmgr->fd >= 0) {
        status = CTRL_STATUS_SUCCESS;
        lseek(chmgr->fd, offset, SEEK_SET);
        write(chmgr->fd, buf + BLK_HDR_SIZE, len - BLK_HDR_SIZE);
    }
    fm_write(chmgr, SEQ_TO_AEQ(seq), &status, 1);
}

static void handle_blk_aeq(struct chmgr *chmgr, uint8_t aeq, uint8_t status)
{
    if(status == CTRL_STATUS_SUCCESS) {
        ftq_rm(chmgr->ftq, aeq - CTRL_BLK_AEQ0);
    } else {
        ftq_reset(chmgr->ftq, aeq - CTRL_BLK_AEQ0);
    }
}

static void handle_blk_comp(struct chmgr *chmgr)
{
    if(chmgr->fd)
        close(chmgr->fd);
    chmgr->fd = -1;
}

static int fstat_req(struct chmgr *chmgr, const char *lname, const char *rname)
{
    int ret = -1;
    uint32_t mode;
    const uint8_t nalen = strlen(rname) + 1;
    uint8_t *fs = NULL;
    chmgr->ftq = alloc_ftq(lname, chmgr, &mode);
    if(chmgr->ftq == NULL)
        goto __exit;

    fs = (uint8_t *)malloc(FSTAT_HDR_SIZE + nalen);
    U32_TO_STREAM(fs, mode);
    strcpy((char*)(fs + FSTAT_HDR_SIZE), rname);
    ret = fm_write(chmgr, CTRL_FSTAT_REQ, fs, FSTAT_HDR_SIZE + nalen);
    free(fs);

__exit:
    return ret;
}

static int logical_channel_cb(struct chmgr *chmgr, int idx)
{
    int readn;
    uint8_t *buf = (uint8_t*)malloc(FM_MTU - 10);
    if(0 < (readn = read(chmgr->fds[idx].fd, buf, FM_MTU - 10)))
        fm_write(chmgr, idx, buf, readn);
    free(buf);
    return readn;
}

static int handle_logical_mgr(struct chmgr *chmgr, uint8_t fn, const uint8_t *buf, uint16_t len)
{
    if(chmgr->fds[fn].fd <= 0 && alloc_channel(chmgr, logical_channel_cb, fn))
        return -1;
    return write(chmgr->fds[fn].fd, buf, len);
}

static void handle_heartbeat(struct chmgr *chmgr)
{
    fm_write(chmgr, CTRL_HEARTBEAT_ACK, NULL, 0);
}

static void chmgr_handler(struct chmgr *chmgr, uint8_t fn, uint16_t len, const uint8_t *buf)
{
    switch(fn) {
    case 1 ... 30           : handle_logical_mgr(chmgr, fn, buf, len); break;
    case CTRL_FSTAT_REQ     : handle_fstat_req(chmgr, buf, len); break;
    case CTRL_FSTAT_RSP     : handle_fstat_rsp(chmgr, buf[0]); break;
    case CTRL_BLK_COMP      : handle_blk_comp(chmgr); break;
    case CTRL_BLK_SEQ0 ... CTRL_BLK_SEQ31: handle_blk_seq(chmgr, fn, buf, len);break;
    case CTRL_BLK_AEQ0 ... CTRL_BLK_AEQ31: handle_blk_aeq(chmgr, fn, buf[0]);break;
    case CTRL_HEARTBEAT     : handle_heartbeat(chmgr); break;
    case CTRL_HEARTBEAT_ACK: break;
    }
}

static int chmgr_putc(struct chmgr *chmgr, uint8_t c)
{
    //LOGD("\e[32m%02x\e[0m ", c);
    return write(chmgr->fds[31].fd, &c, 1);
}

int fm_write(struct chmgr *chmgr, uint8_t fn, const void *payload, uint16_t len)
{
    uint16_t crc;
    uint8_t *fm = (uint8_t*)malloc(FM_HDR_SIZE + len + 2);

    fm[0] = fn;
    U16_TO_STREAM(fm + 1, len);
    memcpy(fm + 3, payload, len);
    crc = calc_crc(fm, FM_HDR_SIZE + len);
    U16_TO_STREAM(fm + FM_HDR_SIZE + len, crc);
    //slip_send_packet(fm, FM_HDR_SIZE + len + 2, (int (*) (void*, uint8_t))chmgr_putc, (void*)chmgr);
    slip_encode(chmgr->slip, fm, FM_HDR_SIZE + len + 2);
    return len;
}

static int fm_handler(struct chmgr *chmgr, const uint8_t *fm, uint16_t size)
{
    uint16_t len;
    uint16_t crc1 = 0, crc2 = 0;
    len = STREAM_TO_U16(fm + 1);
    if(len != size - 5) {
        LOGE("The data length does not match len = %d, size = %d\n", len, size);
        errno = EINVAL;
        return -1;
    }

    crc1 = STREAM_TO_U16(fm + FM_HDR_SIZE + len);
    crc2 = calc_crc(fm, FM_HDR_SIZE + len);
    if(crc1 !=  crc2) {
        LOGE("crc{%04x, %04x} does not match\n", crc1, crc2);
        errno = EINVAL;
        return -1;
    }

    chmgr_handler(chmgr, fm[0], len, fm + FM_HDR_SIZE);
    chmgr->heartbeat = 20;
    return 0;
}

static int phys_read(struct chmgr *chmgr, int idx __unused)
{
    uint8_t buf[16];
    int rn = read(chmgr->fds[31].fd, buf, 16);
    return slip_decode(chmgr->slip, buf, rn);
}

void mgr_print(struct chmgr *chmgr, const char *fmt, ...)
{
    int n;
    char buf[1024];
    va_list ap;
    va_start(ap, fmt);
    n = vsnprintf(buf, 1024, fmt, ap);
    write(chmgr->fds[0].fd, buf, n);
    va_end(ap);
}

static int mgr_process(struct chmgr *chmgr, int idx __unused)
{
    int ret = 0;
    char buf[1024];
    char *lname, *rname;

    if(0 < (ret = read(chmgr->fds[0].fd, buf, 1024))) {
        buf[ret] =  '\0';
        lname = strtok(buf, " \n");
        rname = strtok(NULL, " \n");

        if(rname == NULL)
            rname = lname;

        mgr_print(chmgr, "send %s to %s\n", lname, rname);

        if(chmgr->ftq != NULL)
            ftq_free(&chmgr->ftq);
        if(0 > (ret = fstat_req(chmgr, lname, rname))) {
            mgr_print(chmgr, "send %s to %s: %s\n", lname, rname, strerror(errno));
        }
    }
    return ret;
}

static void chmgr_sync_process(struct chmgr *chmgr)
{
#if defined(MUXT)
    int n;
    char buf[256];
    if(chmgr->heartbeat < 5) {
        n = snprintf(buf, 256, "\n\n\n%s -r %d\n", chmgr->server, chmgr->baudrate);
        write(chmgr->fds[31].fd, buf, n);
        chmgr->heartbeat = 20;
    } else
#endif
    if(chmgr->heartbeat < 10) {
        fm_write(chmgr, CTRL_HEARTBEAT, NULL, 0);
    } 
    chmgr->heartbeat--;
}

static int phys_open(struct chmgr *chmgr, const char *name)
{
    if(0 > (chmgr->fds[31].fd = open_serialport(name, chmgr->baudrate))) {
        LOGE("open %s: %s\n", name, strerror(errno));
        return -1;
    }

    if(0 > (chmgr->fds[0].fd = open_pty(chmgr->prefix, 0))) {
        LOGE("open_pty %s: %s\n", name, strerror(errno));
        close(chmgr->fds[31].fd);
        chmgr->fds[31].fd = 0;
        return -1;
    }

    chmgr->fds[31].events = POLLIN;
    chmgr->fds[0].events = POLLIN;
    chmgr->cbs[31] = phys_read;
    chmgr->cbs[0]  = mgr_process;
    chmgr->slip = slip_create(SLIP_MTU, chmgr, (void(*)(void *, const uint8_t *, uint16_t))fm_handler, (void (*)(void *ctx, uint8_t))chmgr_putc);
    return 0;
}

static volatile bool running = true;

int main(int argc, char **argv)
{
    int opt, ps, num_of_port = 4;
    struct chmgr chmgr;
    const char *dev = "/dev/ttyUSB0";

    chmgr_init(&chmgr);

    while(0 < (opt = getopt(argc, argv, "d:r:s:n:C:"))) {
        switch(opt) {
        case 'C': chmgr.server = optarg; break;
        case 's': chmgr.prefix = optarg; break;
        case 'd': dev = optarg; break;
        case 'r': chmgr.baudrate = atoi(optarg); break;
        case 'n': num_of_port = atoi(optarg); break;
        }
    }

#if defined(MUXT)
    daemon(0, 0);
    for(int i = 1;i <= num_of_port;i++) {
        if(-1 == alloc_channel(&chmgr, logical_channel_cb, i))
            LOGE("Create channel(%d): %s\n", i, strerror(errno));
    }
#endif

    if(phys_open(&chmgr, dev))
        goto __err;
    while(running) {
        if(0 < (ps = poll(chmgr.fds, 32, 30))) {
            for(int i = 0;i < 32;i++) {
                if(chmgr.fds[i].fd < 0)
                    continue;
                if((chmgr.fds[i].revents & POLLIN)) {
                    chmgr.cbs[i](&chmgr, i);
                }
                if(((chmgr.fds[i].revents & POLLHUP) == POLLHUP)) {
                    if(i == 31)
                        goto __exit;
                    close_pty(chmgr.prefix, chmgr.fds[i].fd, i);
                    chmgr.fds[i].fd = open_pty(chmgr.prefix, i);
                }
            }

        } else {
            chmgr_sync_process(&chmgr);
        }

        if(ftq_task_process(chmgr.ftq)) {
            close_pty(chmgr.prefix, chmgr.fds[0].fd, 0);
            chmgr.fds[0].fd = open_pty(chmgr.prefix, 0);
            ftq_free(&chmgr.ftq);
        }
    }
__exit:
    for(int i = 0; i < 31;i++) {
        if(chmgr.fds[i].fd > 0)
            close_pty(chmgr.prefix, chmgr.fds[i].fd, i);
    }
    close(chmgr.fds[31].fd);
    return 0;
__err:
    return -1;
}
