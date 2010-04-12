#ifndef __RAWDATAREADER_H__
#define __RAWDATAREADER_H__

#include <QFile>

class RawDataReader {
  public:
    RawDataReader(const char* fname, int bufsize, int chunksize);
    ~RawDataReader();
    inline int head()  { return fHead; }
    void reset(int bufsize = -1);
    bool readAll();
    char* atRaw(int t);
    
  private:
    int do_read(char* buf, int len);
    
    int fFd;
    char* fBuf;
    int fBufsize;    // total buffer size in bytes
    int fNChunks;    // total buffer size in chunks
    int fChunksize;  // size of one chunk in bytes
    int fBufPos;     // buffer index for next byte to be read
    int fHead;       // id of newest available chunk
};

#endif
