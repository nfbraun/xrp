#include <stdio.h>
#include <inttypes.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/select.h>
#include <fcntl.h>

int do_read(int fd)
{
    fd_set fds;
    struct timeval tv;
    int res;
    char buf[1024];
    
    FD_ZERO(&fds);
    FD_SET(fd, &fds);
    
    tv.tv_sec = 0;
    tv.tv_usec = 0;
    
    res = select(fd+1, &fds, NULL, NULL, &tv);
    if(res < 0) {
        perror("select()");
        return -1;
    }
    
    if(!FD_ISSET(fd, &fds))
        return 0;
    
    res = read(fd, buf, 1024);
    if(res < 0) {
        perror("read()");
        return -1;
    } else {
        return res;
    }
}

int main()
{
    int fd;
    
    fd = open("softscope.fifo", O_RDONLY | O_NONBLOCK);
    if(fd < 0) {
        perror("Open softscope.fifo");
        return -1;
    }
    
    while(1) {
        printf("Read: %d\n", do_read(fd));
        usleep(10000);
    }

    return 0;
}
