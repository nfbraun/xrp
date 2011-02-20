#ifndef VC_VCHANNEL_H
#define VC_VCHANNEL_H

#include <QColor>
#include "Channel.h"

class VChannel
{
  public:
    VChannel() {}
    VChannel(const QColor& color)
      : fColor(color), fEnabled(false) {}
    
    inline QColor color() const  { return fColor; }
    inline const Channel& channel() const   { return fChannel; }
    inline Channel& channel()     { return fChannel; }
    inline bool enabled() const   { return fChannel && fEnabled; }
    inline void setChannel(const Channel& ch)    { fChannel = ch; }
    inline void setEnabled(bool e)    { fEnabled = e; }
    
  private:
    QColor fColor;
    Channel fChannel;
    bool fEnabled;

};

#endif
