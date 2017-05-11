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
#include <sys/poll.h>
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
#define CTRL_ACK(n) n ? "response" : "request"

#define MAX_CHANNELS        32
#define MAX_BUFF            210
#define HEARTBEAT_TIMEOUT   10000
#define HEARTBEAT_THRESHOLD 5000

enum {
    CHANNEL_STATUS_CLOSE,
    CHANNEL_STATUS_DISCONNECTED,
    CHANNEL_STATUS_OPEN,        /* physical connection */
    CHANNEL_STATUS_CONNECTED,   /* logical connection */
};

enum {
    CONTROL_HEART_BEAT,
    CONTROL_LOGICAL_CONNECT,
    CONTROL_LOGICAL_DISCONNECT,
};

enum {
    CONTROL_STATUS_SUCCESS,
    CONTROL_STATUS_ERROR,
};

struct channel
{
    uint8_t id;
    uint8_t status;
    struct pollfd *pollfd;
    int fd;
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

int loglevel = LOGLEVEL_ERROR | LOGLEVEL_WARNING;

static volatile int terminate  = 0;
int heartbeat_timeout         = HEARTBEAT_TIMEOUT;
static char *serportdev        = "/dev/ttyS0";
static int maxfd               = 0;
static int baudrate            = 460800;
static uint32_t channel_bitmap = -1;
static struct channel *phy_channel = NULL;

#if defined(MUXT)
static char *remote_server     = "/bin/muxtd -p /dev/ttyS0";
static char *devSymlinkPrefix  = "/tmp/mux";
static int   numOfPorts        = 1;
#define OPTS "h?p:b:d:s:C:n:"
#else
static char *local_shell       = "/bin/sh";
#define OPTS "h?p:b:d:s:"
#endif


static struct pollfd fds[MAX_CHANNELS] = {{0,0,0}};
static struct channel channels[MAX_CHANNELS];
static int open_pty(int idx);
static void close_pty(int fd, int idx);
static int frame_write(struct channel *ch, const uint8_t *payload, uint8_t length);

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
    return B460800;
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
        // fcntl(fd, F_SETFL, 0);

        // get the parameters
        tcgetattr(fd, &options);

        cfsetispeed(&options, baud);
        cfsetospeed(&options, baud);

        cfmakeraw(&options);

        options.c_cc[VTIME] = 1;
        options.c_cc[VMIN]  = 0;

        // Set the new options for the port...
        tcsetattr(fd, TCSANOW, &options);
    }
    return fd;
}

#if defined(MUXT)
static int open_pty(int idx) 
{
    int fd, oflags;
    char symname[128];
    struct termios options;
#if __QNX__
    int slave;
    char slavename[128];
    if(openpty(&fd, &slave, slavename, NULL, NULL) < 0) {
        LOGE("openpty: %s", strerror(errno));
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

#if defined(MUXT)
    snprintf(symname, sizeof(symname), "%s%d", devSymlinkPrefix, idx);
    unlink(symname);
    if (symlink(slavename, symname) != 0) {
        LOGE("Symbolic link %s -> %s: %s.\n", symname, slavename, strerror(errno));
    }
#endif
    // get the parameters
    tcgetattr(fd, &options);
    cfmakeraw(&options);

    tcsetattr(fd, TCSANOW, &options);

    //oflags = fcntl(fd, F_GETFL);
    //oflags |= (O_NONBLOCK | O_NOCTTY);
    //fcntl(fd, F_SETFL, oflags);

    return fd;
}

#elif defined(MUXTD)

static int open_pty(int idx) 
{
    pid_t pid;
    int fd, oflags;
    struct termios options;
    char slavename[128];
    if((pid = forkpty(&fd, slavename, NULL, NULL)) < 0) {
        LOGE("forkpty: %s\n", strerror(errno));
        return -1;
    } else if(pid == 0) {
        execl(local_shell, "sh", NULL);
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
#endif

static void close_pty(int fd, int idx)
{
#if defined(MUXT)
    char symname[128];
    snprintf(symname, sizeof(symname), "%s%d", devSymlinkPrefix, idx);
    unlink(symname);
#else
    (void)idx;
#endif
    close(fd);
}

static struct channel *alloc_channel(int id)
{
    struct channel *ch;
    if(channel_bitmap == 0 || (id != -1 && (id >= MAX_CHANNELS || id < 0 || !((1 << id) & channel_bitmap)))) {
        errno = E2BIG;
        return NULL;
    }

    id = (id == -1) ? ffs(channel_bitmap) - 1 : id;

    ch = channels + id;
    ch->id = id;
    ch->fd = id ? open_pty(id) : open_serialport(serportdev);

    if(ch->fd < 0)
        return NULL;

    ch->status = CHANNEL_STATUS_OPEN;
    ch->pollfd = fds + id;
    ch->pollfd->fd = ch->fd;
    ch->pollfd->events = POLLIN;
    channel_bitmap &= ~(1 << id);
    return ch;
}

static struct channel *get_channel(int id)
{
    if(!(1 << id & channel_bitmap))
        return channels + id;
    return NULL;
}

static void free_channel(struct channel *ch, void *_unused __attribute__((unused)))
{
    if(ch->id == 0)
        close(ch->fd);
    else
        close_pty(ch->fd, ch->id);
    ch->status = CHANNEL_STATUS_CLOSE;
    ch->pollfd->fd = 0;
    ch->pollfd->events = 0;
    channel_bitmap |= (1 << ch->id);
}

static void foreach_channel(void (*fn)(struct channel *ch, void *ctx), void *ctx)
{
    for(int i = MAX_CHANNELS - 1;i >= 0;i--) {
        if(!(1 << i & channel_bitmap)) {
            fn(channels + i, ctx);
        }
    }
}

static int channel_putc(struct channel *ch, uint8_t c)
{
    int ret = -1;
    if(ch != NULL) {
        ret = write(ch->fd, &c, 1);
    }
    return ret == 1 ? 0 : -1;
}

static int channel_getc(struct channel *ch, uint8_t *c)
{ 
    int ret = -1;
    if(ch != NULL) {
        if(0 == (ret = read(ch->fd, c, 1)))
            errno = ETIMEDOUT;
    }
    return ret == 1 ? 0 : -1;
}

static int write_virtual_channel(uint8_t channel, uint8_t *buf, int n)
{
    struct channel *ch;
    if(NULL == (ch = get_channel(channel)))
        return -1;
    return write(ch->fd, buf, n);
}

static int write_control_channel(struct channel *cch, struct control *ctrl)
{
    return frame_write(cch, (uint8_t*)ctrl, sizeof(struct control));
}

static int control_channel(struct channel *cch, struct control *ctrl)
{
    uint8_t status = CONTROL_STATUS_ERROR;
    struct channel *ch;
    switch(ctrl->cmd) {

    case CONTROL_HEART_BEAT: 
        heartbeat_timeout = HEARTBEAT_TIMEOUT;
        status = CONTROL_STATUS_SUCCESS;
    break;

    case CONTROL_LOGICAL_CONNECT: {
        LOGI("connect channel(%d) %s: status = %d\n", ctrl->channel, CTRL_ACK(ctrl->ack), ctrl->status);
        if((NULL != (ch = get_channel(ctrl->channel))) ||
           (NULL != (ch = alloc_channel(ctrl->channel)))) {
            if(!ctrl->ack || ctrl->status == CONTROL_STATUS_SUCCESS) {
                ch->status = CHANNEL_STATUS_CONNECTED;
                status = CONTROL_STATUS_SUCCESS;
            } else if(ctrl->channel) {
                ch->status = CHANNEL_STATUS_DISCONNECTED;
                /* Blocking sleep is rubbish, but the code is simple */
                sleep(1);
                write_control_channel(cch, &(struct control){.ack = 0,
                    .cmd = CONTROL_LOGICAL_CONNECT,
                    .channel = ctrl->channel,
                    .status = CONTROL_STATUS_SUCCESS});
            }
        }
    } break;

    case CONTROL_LOGICAL_DISCONNECT: {
        LOGI("disconnect channel(%d) %s: status = %d\n", ctrl->channel, CTRL_ACK(ctrl->ack), ctrl->status);
        if(NULL != (ch = get_channel(ctrl->channel))) {
            ch->status = CHANNEL_STATUS_DISCONNECTED;
            status = CONTROL_STATUS_SUCCESS;
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

    return 0;
}

static int frame_process(struct channel *ch, struct frame *frame)
{
    if(frame->channel == 0)
        return control_channel(ch, (struct control*)frame->payload);
    LOGD("write virtual channel(%d): %d\n", frame->channel, frame->length);
    return write_virtual_channel(frame->channel, frame->payload, frame->length);
}

static int frame_write(struct channel *ch, const uint8_t *payload, uint8_t length)
{
    uint16_t crc;
    struct frame *frame = (struct frame*)malloc(sizeof(struct frame) + length + 2);

    frame->channel = ch->id;
    frame->length = length;
    memcpy(frame->payload, payload, length);
    crc = calc_crc((uint8_t*)frame, sizeof(struct frame) + length);
    frame->payload[length] = crc & 0xff;
    frame->payload[length + 1] = (crc >> 8) & 0xff;
    slip_send_packet((uint8_t*)frame, sizeof(struct frame) + length + 2, (int (*) (void*, uint8_t))channel_putc, (void*)phy_channel);
    return length;
}

static struct frame *frame_read(void)
{
    uint16_t crc1 = 0, crc2 = 0;
#define FRAME_MAX_SIZE (sizeof(struct frame) + MAX_BUFF + 2) /* crc 2 byte */
    struct frame *frame = (struct frame*)calloc(1, FRAME_MAX_SIZE);

    if(0 > slip_read_packet((uint8_t*)frame, FRAME_MAX_SIZE, (int (*)(void *,uint8_t *))channel_getc, (void*)phy_channel)) {
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

static void channel_connect(struct channel *ch, struct channel *cch)
{
    LOGI("Request connect channel(%d): status = %d\n", ch->id, ch->status);
    if(ch->status != CHANNEL_STATUS_CONNECTED) {
        write_control_channel(cch, &(struct control){.ack = 0,
            .cmd = CONTROL_LOGICAL_CONNECT,
            .channel = ch->id,
            .status = CONTROL_STATUS_SUCCESS
        });
    }
}

static void channel_disconnect(struct channel *ch, struct channel *cch)
{
    LOGI("Disconnect channel(%d): status = %d\n", ch->id, ch->status);
    write_control_channel(cch, &(struct control){.ack = 0,
        .cmd = CONTROL_LOGICAL_DISCONNECT,
        .channel = ch->id,
        .status = CONTROL_STATUS_SUCCESS
    });
    ch->status = CHANNEL_STATUS_DISCONNECTED;
}

static int process_control_channel(struct channel *ch)
{
    struct frame *frame;
    LOGD("control: fd %d, %d\n", ch->fd, ch->pollfd->fd);
    if(NULL == (frame = frame_read()))
        return -1;
    heartbeat_timeout = HEARTBEAT_TIMEOUT;
    frame_process(ch, frame);
    free(frame);
    return 0;
}

static int process_virtual_channel(struct channel *ch)
{
    int readn;
    uint8_t buf[MAX_BUFF];
    LOGD("virtual: fd %d, %d\n", ch->fd, ch->pollfd->fd);
    if(0 < (readn = read(ch->fd, buf, sizeof(buf))))
        frame_write(ch, buf, readn);
    return readn;
}

static void process_channel_poll(int channel)
{
    struct channel *ch = get_channel(channel);
    if(ch != NULL)
        channel ? process_virtual_channel(ch) : process_control_channel(ch);
}

static int channel_poll(time_t ms)
{
    int ps, w = 1;
    if(0 < (ps = poll(fds, MAX_CHANNELS, ms))) {
        for(int i = 0;i < MAX_CHANNELS;i++) {
            if(fds[i].events & fds[i].revents) {
                process_channel_poll(i);
                w = 0;
            }
        }
        if(w)
            usleep(ms * 1000);
    }

    return ps;
}


// shows how to use this program
static void usage(char *_name)
{
    fprintf(stderr,"\nUsage: %s [options]\n",_name);
    fprintf(stderr,"options:\n");
    fprintf(stderr,"  -p <serport>        : Serial port device to connect to [/dev/ttyS0]\n");
    fprintf(stderr,"  -b <baudrate>       : MUX mode baudrate (0,9600,19200, ...) [460800]\n");
    fprintf(stderr,"  -d <loglevel>       : Set loglevel: [ERROR | WARNING]\n");
    fprintf(stderr,"                          ERROR     0x01\n");
    fprintf(stderr,"                          WARNING   0x02\n");
    fprintf(stderr,"                          INFO      0x04\n");
    fprintf(stderr,"                          DEBUG     0x08\n");
#if defined(MUXT)
    fprintf(stderr,"  -n <number>         : Number of logical serial ports [1]\n");
    fprintf(stderr,"  -s <symlink-prefix> : Prefix for the symlinks of slave devices (e.g. /dev/mux)\n");
    fprintf(stderr,"  -C <server command> : Remote service start command [/bin/muxtd -p /dev/ttyS0]\n");
#else
    fprintf(stderr,"  -s <shell>          : Login shell [/bin/sh]\n");
#endif
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
    int opt;
    char *programName = argv[0];

    if(argc < 2) {
        usage(programName);
        exit(-1);
    }

    while(0 < (opt = getopt(argc,argv, OPTS))) {
        switch(opt) {
        case 'p': serportdev = optarg; break;
        case 'd': loglevel = strtol(optarg, NULL, 16); break;
        case 'b': baudrate = atoi(optarg); break;
#if defined(MUXT)
        case 's': devSymlinkPrefix = optarg; break;
        case 'n': numOfPorts = atoi(optarg); break;
        case 'C': remote_server = optarg; break;
#endif
        case '?': case 'h' : usage(programName); exit(0); break;
        default: break;
        }
    }

    programName = argv[0];
    signal(SIGINT, signal_handler);
    signal(SIGKILL, signal_handler);
    signal(SIGTERM, signal_handler);

    /* create phyical channel */
    if(NULL == (phy_channel = alloc_channel(0))) {
        LOGE("Create phyical channle %s: %s\n", serportdev, strerror(errno));
        return -1;
    }

#if defined(MUXT)
    daemon(0, 0);
    for(int i = 1;i <= numOfPorts;i++) {
        if(NULL == alloc_channel(i)) {
            LOGE("Create logical channle(%d): %s\n", i, strerror(errno));
        }
    }
#endif

    while(!terminate) {

        LOGI("Enter multiplexing mode.\n");
        heartbeat_timeout = HEARTBEAT_TIMEOUT;

        while(phy_channel->status != CHANNEL_STATUS_CONNECTED) {
            if(terminate)
                goto __exit;
#if defined(MUXT)
            /* remote run muxtd */
            write(phy_channel->fd, remote_server, strlen(remote_server));
            write(phy_channel->fd, "\n", 1);
#elif defined(MUXTD)
            channel_connect(phy_channel, phy_channel);
#endif
            channel_poll(5000);
        }

#if defined(MUXT)
        /* connect all logical channel */
        foreach_channel((void (*)(struct channel*, void *))channel_connect, phy_channel);
#endif

        while (!terminate && phy_channel->status == CHANNEL_STATUS_CONNECTED && heartbeat_timeout > 0) {
            if(!channel_poll(100))
                heartbeat_timeout--;
            if(heartbeat_timeout < HEARTBEAT_THRESHOLD)
                write_control_channel(phy_channel, &(struct control){.ack = 0,
                    .cmd = CONTROL_HEART_BEAT,
                    .channel = 0,
                    .status = CONTROL_STATUS_SUCCESS});
        }

        LOGI("Disconnect all channel: heartbeat_timeout = %d.\n", heartbeat_timeout);
        foreach_channel((void (*)(struct channel*, void *))channel_disconnect, phy_channel);
        LOGI("Leave multiplexing mode.\n");
    }

__exit:
    LOGI("Release all channels.\n");
    foreach_channel(free_channel, NULL);

    LOGI("%s finished\n", programName);
    return 0;
}
