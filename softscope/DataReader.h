#ifndef SSCOPE_DATAREADER_H
#define SSCOPE_DATAREADER_H

#include "RawDataReader.h"

template <class T>
class DataReader : public RawDataReader
{
  public:
    DataReader(const char* fname, int bufsize, int channels)
        : RawDataReader(fname, bufsize, channels * sizeof(T)),
          fNChannels(channels)
        {}
    
    T at(int t, int ch)
    {
        if(ch < 0 || ch >= fNChannels) return 0;
        char* p = atRaw(t);
        if(!p) return 0;
        return ((T*)p)[ch];
    }
  
  private:
    int fNChannels;
};

#endif
