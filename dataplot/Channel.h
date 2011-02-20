#ifndef VC_CHANNEL_H
#define VC_CHANNEL_H

#include "Data.h"

class _Channel {
  public:
    static const double CH_GAIN_CHOICES[];
    static const int CH_GAIN_DEFAULT;
    static const unsigned int N_CH_GAIN_CHOICES;
};

class Channel;

class ChannelObj
{
  friend class Channel;
  
  private:
    ChannelObj(bool notnull=false)
        : fRefcount(0), fNotNull(notnull), fGainIdx(_Channel::CH_GAIN_DEFAULT),
          fOffset(0.) {}
    ChannelObj(const Data& data, const std::string& name)
        : fRefcount(0), fNotNull(true), fData(data), fName(name),
          fGainIdx(_Channel::CH_GAIN_DEFAULT), fOffset(0.) {}
    
    int fRefcount;
    bool fNotNull;
    
    Data fData;
    std::string fName;
    
    int fGainIdx;
    double fOffset;
};

class Channel: public _Channel
{
  public:
    typedef void* id_t;
    
    Channel(bool notnull=false);
    Channel(const Channel& src);
    Channel(const Data& data, const std::string& name);
    Channel& operator=(const Channel& src);
    ~Channel();
    
    inline id_t id() const { return (id_t) fObj; }
    
    inline operator bool() const { return fObj->fNotNull; }
    inline bool operator==(const Channel& other) { return (fObj == other.fObj); }
    inline bool operator!=(const Channel& other) { return !(*this == other); }
    
    inline const Data& data() const { return fObj->fData; }
    inline void setData(const Data& data) { fObj->fData = data; }
    const std::string& name() const { return fObj->fName; }
    inline void setName(const std::string name) { fObj->fName = name; }
    
    inline int gainIdx() const { return fObj->fGainIdx;}
    inline double gain() const { return CH_GAIN_CHOICES[fObj->fGainIdx]; }
    inline void setGainIdx(int i) { fObj->fGainIdx = i; }
    
    inline double offset() const { return fObj->fOffset; }
    inline void setOffset(double o) { fObj->fOffset = o; }
    
  private:
    ChannelObj *fObj;
};

#endif
