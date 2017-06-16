#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>


int main(int argc, char **argv)
{
    int c , fd, n;
    const char *dev = "/tmp/mux0";
    const char *in = NULL;
    const char *out = NULL;
    char buf[1024];
    while(-1 != (c = getopt(argc, argv, "d:i:o:"))) {
        switch(c) {
        case 'd': dev = optarg; break;
        case 'i': in = optarg; break;
        case 'o': out = optarg; break;
__help:
        default:
        case 'h': printf("%s -d <dev> -i <in-file> -o <out-file>\n", argv[0]); exit(1); break;
        }
    }
    if(dev == NULL || in == NULL || out == NULL)
        goto __help;

    memset(buf, 0, sizeof(buf));
    if((*in != '/') && (*in != '~')) {
        getcwd(buf, sizeof(buf));
    }


    n = strlen(buf);
    n += snprintf(buf + n , sizeof(buf) - n, "/%s %s", in, out);
    fd = open(dev, O_RDWR);
    write(fd, buf, n);
    while(0 < (n = read(fd, buf, sizeof(buf)))) {
        write(2, buf, n);
    }
    return 0;
}

