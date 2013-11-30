#include "STLReader.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <iostream>

STLReader::STLReader(const char* fname)
    : fOK(true), fNumTri(0), fBufHead(0), fBufSize(0)
{
    fBuf = new STLTriRecord_t[BUF_MAXSIZE];
    
    fHandle = open(fname, O_RDONLY);
    if(fHandle < 0) {
        std::cerr << __func__ << ": open failed" << std::endl;
        fOK = false;
    }

    // Skip header
    if(fOK && lseek(fHandle, 80, SEEK_SET) != 80) {
        std::cerr << __func__ << ": seek failed" << std::endl;
        fOK = false;
    }
    
    // Get number of triangles
    if(fOK && read(fHandle, &fNumTri, 4) != 4) {
        std::cerr << __func__ << ": read failed" << std::endl;
        fOK = false;
    }
}

STLReader::~STLReader()
{
    if(fOK) close(fHandle);
    delete[] fBuf;
}

int STLReader::Read()
{
    int result;
    uint8_t* ptr = (uint8_t*) fBuf;
    size_t nread = 0;
    size_t count = sizeof(STLTriRecord_t)*BUF_MAXSIZE;
    
    do {
        result = read(fHandle, ptr, count);
        if(result <= 0)
            break;
        ptr += result;
        nread += result;
        count -= result;
    } while(nread < sizeof(STLTriRecord_t));
    
    if(result < 0)
        return result;
    else
        return nread / sizeof(STLTriRecord_t);
}

STLReader::STLTriRecord_t* STLReader::getTriRecord(uint32_t id)
{
    if(!fOK || id >= fNumTri)
        return NULL;
    
    if(id >= fBufHead && id < (fBufHead + fBufSize)) {
        return &fBuf[id - fBufHead];
    }
    
    if(lseek(fHandle, STL_HEADLEN + id*sizeof(STLTriRecord_t), SEEK_SET) < 0) {
        std::cerr << __func__ << ": seek failed" << std::endl;
        fOK = false;
        return NULL;
    }
    
    int result = Read();
    if(result < 1) {
        std::cerr << __func__ << ": read failed" << std::endl;
        fOK = false;
        return NULL;
    }
    fBufHead = id;
    fBufSize = result;
    
    return &fBuf[0];
}
