#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


#ifdef __QNX__
#   include <unix.h>
#else
#   include <pty.h>
#endif

#include "log.h"

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

int open_serialport(const char *dev, int baudrate)
{
    int fd;

    fd = open(dev, O_RDWR | O_NOCTTY /*| O_NDELAY*/);
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

        //options.c_cflag |= (IXON | IXOFF);
        options.c_cc[VTIME] = 1;
        options.c_cc[VMIN]  = 0;

        // Set the new options for the port...
        tcsetattr(fd, TCSANOW, &options);
    }
    return fd;
}

int open_pty(const char *prefix, int idx) 
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

    snprintf(symname, sizeof(symname), "%s%d", prefix, idx);
    unlink(symname);
    if (symlink(slavename, symname) != 0) {
        LOGE("Symbolic link %s -> %s: %s.\n", symname, slavename, strerror(errno));
    }

    // get the parameters
    tcgetattr(fd, &options);
    cfmakeraw(&options);

    tcsetattr(fd, TCSANOW, &options);

    //oflags = fcntl(fd, F_GETFL);
    //oflags |= (O_NONBLOCK | O_NOCTTY);
    //fcntl(fd, F_SETFL, oflags);

    return fd;
}

int load_shell(const char *sh) 
{
    pid_t pid;
    int fd/*, oflags*/;
    //struct termios options;
    char slavename[128];
    if((pid = forkpty(&fd, slavename, NULL, NULL)) < 0) {
        LOGE("forkpty: %s\n", strerror(errno));
        return -1;
    } else if(pid == 0) {
        execl(sh, "sh", NULL);
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

void close_pty(const char *prefix, int fd, int idx)
{
    char symname[128];
    snprintf(symname, sizeof(symname), "%s%d", prefix, idx);
    unlink(symname);
    close(fd);
}

