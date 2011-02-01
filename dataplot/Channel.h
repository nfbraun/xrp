#ifndef VC_CHANNEL_H
#define VC_CHANNEL_H

#include "Data.h"

class Channel
{
  public:
    static const double CH_GAIN_CHOICES[];
    static const int CH_GAIN_DEFAULT;
    static const unsigned int N_CH_GAIN_CHOICES;
    
    Channel() : fGainIdx(CH_GAIN_DEFAULT), fOffset(0.) {}
    Channel(const Data& data, const std::string& name)
        : fData(data), fName(name), fGainIdx(CH_GAIN_DEFAULT), fOffset(0.) {}
    inline const Data& data() const { return fData; }
    inline void setData(const Data& data) { fData = data; }
    const std::string& name() const { return fName; }
    inline void setName(const std::string name) { fName = name; }
    
    inline int gainIdx() const { return fGainIdx;}
    inline double gain() const { return CH_GAIN_CHOICES[fGainIdx]; }
    inline void setGainIdx(int i) { fGainIdx = i; }
    
    inline double offset() const { return fOffset; }
    inline void setOffset(double o) { fOffset = o; }
    
  private:
    Data fData;
    std::string fName;
    
    int fGainIdx;
    double fOffset;
};

#endif
