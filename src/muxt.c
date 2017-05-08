#if !defined(__QNX__) && !defined(_GNU_SOURCE)
# define _GNU_SOURCE
#endif
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <time.h>
#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <termios.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <string.h>
#include <paths.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/wait.h>

#if defined(__QNX__)
# include <unix.h>
#else
# include <pty.h>
#endif

#include "log.h"
#include "slip.h"

#define ffs(n) __builtin_ffs(n)
#define popcount(n) __builtin_popcount(n)
#define CHNAME(n)   n ? "Logica" : "Control"
#define CTRL_ACK(n) n ? "Response" : "Request"

#define MAX_CHANNELS   32
#define MAX_BUFF       210

enum {
    CHANNEL_STATUS_CLOSE,
    CHANNEL_STATUS_DISCONNECTED,
    CHANNEL_STATUS_OPEN,        /* physical connection */
    CHANNEL_STATUS_CONNECTED,   /* logical connection */
};

enum {
    CONTROL_LOGICAL_CONNECT,
    CONTROL_LOGICAL_DISCONNECT,
};

enum {
    CONTROL_STATUS_SUCCESS,
    CONTROL_STATUS_ERROR,
};

struct logical_channel
{
    uint8_t id;
    uint8_t status;
    int fd;
    struct logical_channel *dev;
};

struct frame
{
    uint8_t channel;    /* 0 channel is the control channel */
    uint8_t length;
    uint8_t payload[0];
    /* uint16 crc; */
};

struct control
{
    uint8_t ack:1;
    uint8_t cmd:7;
    uint8_t channel;
    uint8_t status;
    uint8_t payload[0];
};


int loglevel = LOGLEVEL_ERROR | LOGLEVEL_WARNING | LOGLEVEL_INFO;

static volatile int terminate = 0;
static char *devSymlinkPrefix = 0;
static char *serportdev;
static int numOfPorts = 0;
static int maxfd = 0;
static int baudrate = 0;
static uint32_t channel_bitmap = -1;


static struct logical_channel logical_channels[MAX_CHANNELS];
static int open_pty(char* devname, int idx);
static void close_pty(char* devname, int fd, int idx);
static int frame_write(struct logical_channel *ch, const uint8_t *payload, uint8_t length);

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

static uint16_t calc_crc(uint8_t *payload, uint16_t length)
{
    uint16_t crc = 0;
    for(int i = 0;i < length;i++) {
        crc = __calc_crc(payload[i], crc);
    }
    return crc;
}

static speed_t tcio_baud(int baud)
{
#if defined(__QNX__)
    return baud;
#else
#define CASE_BAUD(n) case n : return B ## n; break
    switch(baud) {
        CASE_BAUD(4000000);
        CASE_BAUD(3000000);
        CASE_BAUD(2000000);
        CASE_BAUD(1500000);
        CASE_BAUD(1000000);
        CASE_BAUD(921600);
        CASE_BAUD(460800);
        CASE_BAUD(230400);
        CASE_BAUD(115200);
        CASE_BAUD(57600);
        CASE_BAUD(38400);
        CASE_BAUD(19200);
        CASE_BAUD(1200);
        CASE_BAUD(600);
    }
#undef CASE_BAUD
    return B115200;
#endif
}

static int open_serialport(char *dev)
{
    int fd;

    fd = open(dev, O_RDWR | O_NOCTTY/* | O_NDELAY*/);
    if (fd != -1) {
        struct termios options;
        speed_t baud = tcio_baud(baudrate);
        LOGD("serial opened\n" );
        // The old way. Let's not change baud settings
        fcntl(fd, F_SETFL, 0);

        // get the parameters
        tcgetattr(fd, &options);

        cfsetispeed(&options, baud);
        cfsetospeed(&options, baud);

        cfmakeraw(&options);

        // Set the new options for the port...
        tcsetattr(fd, TCSANOW, &options);
    }
    return fd;
}

#if defined(MUXT)
static int open_pty(char* devname, int idx) 
{
    int fd, oflags;
    char symname[128];
    struct termios options;
#if __QNX__
    int slave;
    char slavename[128];
    if(openpty(&fd, &slave, slavename, NULL, NULL) < 0) {
        LOGE("openpty %s: %s", devname, strerror(errno));
        return -1;
    }
#else
    char *slavename;
    fd = open("/dev/ptmx", O_RDWR | O_NOCTTY | O_NONBLOCK);
    if(fd < 0) {
        LOGE("/dev/ptmx: %s", strerror(errno));
        return -1;
    }
	grantpt(fd); /* change permission of slave */
	unlockpt(fd); /* unlock slave */
	slavename = ptsname(fd); /* get name of slave */
#endif
    snprintf(symname, sizeof(symname), "%s%d", devname, idx);
    unlink(symname);
    if (symlink(slavename, symname) != 0) {
        LOGE("Symbolic link %s -> %s: %s.\n", symname, slavename, strerror(errno));
    }
    // get the parameters
    tcgetattr(fd, &options);
    cfmakeraw(&options);

    tcsetattr(fd, TCSANOW, &options);

    oflags = fcntl(fd, F_GETFL);
    oflags |= (O_NONBLOCK | O_NOCTTY);
    fcntl(fd, F_SETFL, oflags);

    return fd;
}

static void close_pty(char *devname, int fd, int idx)
{
    char symname[128];
    snprintf(symname, sizeof(symname), "%s%d", devname, idx);
    unlink(symname);
    close(fd);
}

#elif defined(MUXTD)
static int open_pty(char* devname, int idx) 
{
    pid_t pid;
    int fd, oflags;
    struct termios options;
    char slavename[128];
    if((pid = forkpty(&fd, slavename, NULL, NULL)) < 0) {
        LOGE("openpty %s: %s", devname, strerror(errno));
        return -1;
    } else if(pid == 0) {
        execl("/bin/sh", "sh", NULL);
    }
    /*
    // get the parameters
    tcgetattr(fd, &options);
    cfmakeraw(&options);

    tcsetattr(fd, TCSANOW, &options);

    oflags = fcntl(fd, F_GETFL);
    oflags |= (O_NONBLOCK | O_NOCTTY);
    fcntl(fd, F_SETFL, oflags);
    */

    return fd;
}

static void close_pty(char *devname, int fd, int idx)
{
    close(fd);
}
#endif


static struct logical_channel *alloc_logical_channel(int id, struct logical_channel *dev)
{
    struct logical_channel *ch;
    if(channel_bitmap == 0 || (id != -1 && (id >= MAX_CHANNELS || id < 0 || !((1 << id) & channel_bitmap)))) {
        errno = E2BIG;
        return NULL;
    }

    id = (id == -1) ? ffs(channel_bitmap) - 1 : id;

    ch = logical_channels + id;
    ch->id = id;
    ch->dev = dev ?: ch;
    if(id == 0)
        ch->fd = open_serialport(serportdev);
    else
        ch->fd = open_pty(devSymlinkPrefix, id);

    if(ch->fd < 0)
        return NULL;

    ch->status = CHANNEL_STATUS_OPEN;
    channel_bitmap &= ~(1 << id);
    return ch;
}

static struct logical_channel *get_logical_channel(int id)
{
    if(!(1 << id & channel_bitmap))
        return logical_channels + id;
    return NULL;
}

static void free_logical_channel(struct logical_channel *ch, void *_unused __attribute__((unused)))
{
    if(ch->id == 0)
        close(ch->fd);
    else
        close_pty(devSymlinkPrefix, ch->fd, ch->id);
    ch->status = CHANNEL_STATUS_CLOSE;
    channel_bitmap |= (1 << ch->id);
}

static void foreach_logical_channel(void (*fn)(struct logical_channel *ch, void *ctx), void *ctx)
{
    for(int i = MAX_CHANNELS - 1;i >= 0;i--) {
        if(!(1 << i & channel_bitmap)) {
            fn(logical_channels + i, ctx);
        }
    }
}

static int logical_channel_putc(struct logical_channel *ch, uint8_t c)
{
    int ret = write(ch->fd, &c, 1);
    return ret == 1 ? 0 : -1;
}

static int logical_channel_getc(struct logical_channel *ch, uint8_t *c)
{
    int ret = read(ch->fd, c, 1);
    return ret == 1 ? 0 : -1;
}

static int write_virtual_channel(uint8_t channel, uint8_t *buf, int n)
{
    struct logical_channel *ch;
    if(NULL == (ch = get_logical_channel(channel)))
        return -1;
    return write(ch->fd, buf, n);
}

static int write_control_channel(struct logical_channel *cch, struct control *ctrl)
{
    return frame_write(cch, (uint8_t*)ctrl, sizeof(struct control));
}

static int control_channel(struct logical_channel *cch, struct control *ctrl)
{
    uint8_t status = CONTROL_STATUS_ERROR;
    struct logical_channel *ch;
#define GETCH(n) ((n->ack || n->channel == 0)? get_logical_channel(n->channel) : alloc_logical_channel(n->channel, cch))
    switch(ctrl->cmd) {

    case CONTROL_LOGICAL_CONNECT: {
        LOGI("%s connect logical channel(%d): %d\n", CTRL_ACK(ctrl->ack), ctrl->channel, ctrl->status);
        if(NULL != (ch = GETCH(ctrl))) {
            if(ctrl->status == CONTROL_STATUS_SUCCESS) {      /* command status is always successful */
                ch->status = CHANNEL_STATUS_CONNECTED;
                status = CONTROL_STATUS_SUCCESS;
            } else if(ctrl->channel) {
                free_logical_channel(ch, NULL);
            }
        }
    } break;

    case CONTROL_LOGICAL_DISCONNECT: {
        LOGI("%s disconnect logical channel(%d): %d\n", CTRL_ACK(ctrl->ack), ctrl->channel, ctrl->status);
        if(NULL != (ch = get_logical_channel(ctrl->channel))) {
            if(ch->status == CHANNEL_STATUS_CONNECTED && !ctrl->ack) {
                ch->status = CHANNEL_STATUS_DISCONNECTED;
                status = CONTROL_STATUS_SUCCESS;
            }
            /* The control channel can not be released here, must wait for other channels are released before they can */
            if(ctrl->channel)
                free_logical_channel(ch, NULL);
        }
    } break;

    }

    if(!ctrl->ack) {
        write_control_channel(cch, &(struct control) { .ack = 1,
            .cmd = ctrl->cmd,
            .channel = ctrl->channel,
            .status  = status,
        });
    }

#if defined(MUXT)
    if(ctrl->channel == 0 &&
       ctrl->cmd == CONTROL_LOGICAL_CONNECT &&
       cch->status == CHANNEL_STATUS_CONNECTED &&
       ctrl->status == CONTROL_STATUS_SUCCESS) 
    {
        for(int i = 1;i <= numOfPorts;i++) {
            if(NULL != (ch = alloc_logical_channel(i, cch))) {
                write_control_channel(cch, &(struct control) { .ack = 0,
                    .cmd = CONTROL_LOGICAL_CONNECT,
                    .channel = i,
                    .status  = CONTROL_STATUS_SUCCESS,
                });
            }
        }
    }
#endif
    return 0;
#undef GETCH
}

static int frame_process(struct logical_channel *ch, struct frame *frame)
{
    if(frame->channel == 0)
        return control_channel(ch, (struct control*)frame->payload);
    LOGD("write virtual channel(%d): %d\n", frame->channel, frame->length);
    return write_virtual_channel(frame->channel, frame->payload, frame->length);
}

static int frame_write(struct logical_channel *ch, const uint8_t *payload, uint8_t length)
{
    uint16_t crc;
    struct frame *frame = (struct frame*)malloc(sizeof(struct frame) + length + 2);

    frame->channel = ch->id;
    frame->length = length;
    memcpy(frame->payload, payload, length);
    crc = calc_crc((uint8_t*)frame, sizeof(struct frame) + length);
    frame->payload[length] = crc & 0xff;
    frame->payload[length + 1] = (crc >> 8) & 0xff;
    slip_send_packet((uint8_t*)frame, sizeof(struct frame) + length + 2, (int (*) (void*, uint8_t))logical_channel_putc, (void*)ch->dev);
    return length;
}

static struct frame *frame_read(struct logical_channel *ch)
{
    uint16_t crc1 = 0, crc2 = 0;
    struct frame *frame = (struct frame*)calloc(1, sizeof(struct frame) + MAX_BUFF + 2);

    if(0 > slip_read_packet((uint8_t*)frame, (int (*)(void *,uint8_t *))logical_channel_getc, (void*)ch->dev)) {
        LOGE("slip read packet: %s\n", strerror(errno));
        goto __err;
    }

    crc1 = frame->payload[frame->length] | frame->payload[frame->length + 1] << 8;
    crc2 = calc_crc((uint8_t*)frame, frame->length + sizeof(struct frame));
    if(crc1 !=  crc2) {
        LOGE("CRC1(%04x) CRC2(%04x) does not match\n", crc1, crc2);
        goto __err;
    }

    return frame;
__err:
    free(frame);
    return NULL;
}

static void logical_channel_select_cb(struct logical_channel *ch, void *fds)
{
    fd_set *rfds = (fd_set*)fds;
    if(FD_ISSET(ch->fd, rfds)) {
        LOGD("channle(%d) data in\n", ch->id);
        if(ch->id == 0) {
            struct frame *frame;
            if(NULL == (frame = frame_read(ch))) {
                return;
            }
            frame_process(ch, frame);
            free(frame);
        } else {
            uint8_t buf[MAX_BUFF];
            int readn = read(ch->fd, buf, sizeof(buf));
            LOGD("read virtual channel: %d %.*s\n", readn, readn, buf);
            if(readn > 0) {
                frame_write(ch, buf, readn);
            }
        }
    }
}

static void logical_channel_disconnect(struct logical_channel *ch, struct logical_channel *cch)
{
    LOGI("Disconnect the logical channel %d.\n", ch->id);
    write_control_channel(cch, &(struct control){.ack = 0,
        .cmd = CONTROL_LOGICAL_DISCONNECT,
        .channel = ch->id,
        .status = CONTROL_STATUS_SUCCESS
    });
    ch->status = CHANNEL_STATUS_DISCONNECTED;
}

static void logical_channel_fd_set(struct logical_channel *ch, void *fds)
{
    fd_set *rfds = (fd_set*)fds;
    if(ch->fd > maxfd)
        maxfd = ch->fd;
    FD_SET(ch->fd, rfds);
}

// shows how to use this program
static void usage(char *_name)
{
    fprintf(stderr,"\nUsage: %s [options] <pty1> <pty2> ...\n",_name);
    fprintf(stderr,"  <ptyN>              : pty devices (e.g. /dev/ptya0)\n\n");
    fprintf(stderr,"options:\n");
    fprintf(stderr,"  -n <number>         : Number of logical serial ports\n");
    fprintf(stderr,"  -p <serport>        : Serial port device to connect to [/dev/modem]\n");
    fprintf(stderr,"  -d <loglevel>       : Set loglevel:\n");
    fprintf(stderr,"                          ERROR     0x01\n");
    fprintf(stderr,"                          WARNING   0x02\n");
    fprintf(stderr,"                          INFO      0x04\n");
    fprintf(stderr,"                          DEBUG     0x08\n");
    fprintf(stderr,"  -b <baudrate>       : MUX mode baudrate (0,9600,19200, ...)\n");
    fprintf(stderr,"  -s <symlink-prefix> : Prefix for the symlinks of slave devices (e.g. /dev/mux)\n");
    fprintf(stderr,"  -h                  : Show this help message\n");
}

//#if defined(MUXT)
//static void daemon(void)
//{
//    if(fork())
//        exit(-1);
//
//	umask(0);
//
//    if(setsid() < 0)
//		exit(-1);
//
//	if ((chdir("/")) < 0)
//		exit(-1);
//
//	close(STDIN_FILENO);
//	close(STDOUT_FILENO);
//	close(STDERR_FILENO);
//}
//#endif
static void signal_handler(int sig)
{
    switch(sig) {
    case SIGINT:
    case SIGKILL:
    case SIGTERM: terminate = 1; break;
    }
}

int main(int argc, char *argv[])
{
    int sel;
    fd_set rfds;
    struct timeval timeout;
    char *programName;
    struct logical_channel *cch;

    int opt;

    programName = argv[0];

    if(argc < 2) {
        usage(programName);
        exit(-1);
    }

    serportdev="/dev/ttyS0";

    while(0 < (opt = getopt(argc,argv,"n:p:h?d:b:s:"))) {
        switch(opt) {
        case 'n': numOfPorts = atoi(optarg); break;
        case 'p': serportdev = optarg; break;
        case 'd': loglevel = strtol(optarg, NULL, 16); break;
        case 'b': baudrate = atoi(optarg); break;
        case 's': devSymlinkPrefix = optarg; break;
        case '?': case 'h' : usage(programName); exit(0); break;
        default: break;
        }
    }

    programName = argv[0];
    signal(SIGINT, signal_handler);
    signal(SIGKILL, signal_handler);
    signal(SIGTERM, signal_handler);
#if defined(MUXT)
    daemon(0, 0);
#endif

    while(!terminate) {

        LOGI("Enter multiplexing mode.\n");
        if(NULL == (cch = alloc_logical_channel(0, NULL)))
            return -1;

        while (!terminate && cch->status >= CHANNEL_STATUS_OPEN) {

            FD_ZERO(&rfds);
            foreach_logical_channel(logical_channel_fd_set, (void *)&rfds);

            timeout.tv_sec = 1;
            timeout.tv_usec = 0;

            if(0 < (sel = select(maxfd + 1, &rfds, NULL, NULL, &timeout))) {
                foreach_logical_channel(logical_channel_select_cb, (void*)&rfds);
            }

            if(cch->status == CHANNEL_STATUS_OPEN)
                write_control_channel(cch, &(struct control){.ack = 0,
                    .cmd = CONTROL_LOGICAL_CONNECT,
                    .channel = 0,
                    .status = CONTROL_STATUS_SUCCESS});
        }

        LOGI("Disconnect all channel.\n");
        foreach_logical_channel((void (*)(struct logical_channel*, void *))logical_channel_disconnect, cch);
        LOGI("Release all channels.");
        foreach_logical_channel(free_logical_channel, NULL);
        LOGI("Leave multiplexing mode.");
    }

    LOGI("%s finished\n", programName);
    return 0;
}
