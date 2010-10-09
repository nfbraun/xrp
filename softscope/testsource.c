#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <math.h>

int main()
{
    int fd;
    fd = open("softscope.fifo", O_WRONLY);
    if(fd < 0) {
        perror("Open softscope.fifo");
        return -1;
    }
    
    float data[4];
    int t;
    while(1) {
        data[0] = ((float) sin((float)t / 10.) * 10000.);
        data[1] = ((float) sin((float)t / 8.) * 5000.);
        data[2] = ((float) sin((float)t / 15.) * 14000.);
        data[3] = ((float) sin((float)t / 25.) * 8000.);
        
        if(write(fd, &data, sizeof(data)) != sizeof(data)) {
            perror("Write to file");
            break;
        }
        t++;
        usleep(10000); // 10 ms
    }
    
    if(close(fd) < 0) {
        perror("Close softscope.fifo");
        return -1;
    }
    
    return 0;
}
