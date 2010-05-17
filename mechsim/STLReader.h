#ifndef __STLREADER_H__
#define __STLREADER_H__

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <inttypes.h>

class STLReader
{
  public:
    STLReader(const char* fname);
    ~STLReader();
    inline operator bool() const { return fOK; }
    inline int getNTri() const { return fNumTri; }
    
    typedef float STLFloat_t;
    typedef uint16_t STLAttr_t;
    
    #pragma pack(push, 1)
    typedef struct {
        STLFloat_t norm[3];
        STLFloat_t v1[3], v2[3], v3[3];
        STLAttr_t attr;
    } STLTriRecord_t;
    #pragma pack(pop)
    
    STLTriRecord_t* getTriRecord(uint32_t id);

  private:
    int Read();
    
    int fHandle;
    bool fOK;
    uint32_t fNumTri;
    STLTriRecord_t* fBuf;
    unsigned int fBufHead, fBufSize;
    
    static const int STL_HEADLEN = 84;
    static const int BUF_MAXSIZE = 1024;
};

#endif
