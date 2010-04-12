#include "DataReader.h"
#include <iostream>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/select.h>
#include <fcntl.h>

RawDataReader::RawDataReader(const char* fname, int bufsize, int chunksize)
{
    // Note that read() may read incomplete chunks. We make the buffer one
    // chunk larger than requested to have space for this.

    fChunksize = chunksize;
    fNChunks = bufsize + 1;
    fBufsize = fNChunks * chunksize;
    fHead = -1;
    fBufPos = 0;
    fBuf = new char[fBufsize];
    
    fFd = open(fname, O_RDONLY | O_NONBLOCK);
    if(fFd < 0)
        perror("open()");
}

RawDataReader::~RawDataReader()
{
    delete[] fBuf;
}

void RawDataReader::reset(int bufsize)
{
    if(bufsize >= 0 && (bufsize + 1) != fNChunks) {
        delete[] fBuf;
        fNChunks = bufsize + 1;
        fBufsize = fNChunks * fChunksize;
        fBuf = new char[fBufsize];
    }
    fHead = -1;
    fBufPos = 0;
}

int RawDataReader::do_read(char* buf, int len)
{
    if(fFd < 0)
        return -1;
    
    fd_set fds;
    struct timeval tv;
    int res;
    
    FD_ZERO(&fds);
    FD_SET(fFd, &fds);
    
    tv.tv_sec = 0;
    tv.tv_usec = 0;
    
    res = select(fFd+1, &fds, NULL, NULL, &tv);
    if(res < 0) {
        perror("select()");
        return -1;
    }
    
    if(!FD_ISSET(fFd, &fds))
        return 0;
    
    res = read(fFd, buf, len);
    if(res < 0) {
        perror("read()");
        return -1;
    }
    
    return res;
}

bool RawDataReader::readAll()
{
    int n_read = 0;
    
    do {
        n_read = do_read(&fBuf[fBufPos], fBufsize - fBufPos);
        if(n_read < 0)
            break;
        
        fHead += (n_read + fBufPos % fChunksize) / fChunksize;
        fBufPos += n_read;
        fBufPos %= fBufsize;
    } while(n_read > 0);
    
    return (n_read >= 0);
}

char* RawDataReader::atRaw(int t)
{
    if(t > fHead || t <= (fHead - fNChunks + 1) || t < 0)
        return NULL;
    return &fBuf[(t % fNChunks)*fChunksize];
}
