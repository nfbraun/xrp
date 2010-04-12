#ifndef __CHANNEL_H__
#define __CHANNEL_H__

#include <QColor>

class Channel {
  public:
    Channel() {}
    Channel(const QColor& color)
      : fColor(color), fUPerDiv(1.), fOffset(0.) {}
    
    inline QColor color() const  { return fColor; }
    inline float uPerDiv() const { return fUPerDiv; }
    inline float offset() const  { return fOffset; }
    inline bool enabled() const  { return fEnabled; }
    inline void setUPerDiv(float u)  { fUPerDiv = u; }
    inline void setOffset(float o)   { fOffset = o; }
    inline void setEnabled(bool e)   { fEnabled = e; }
    
  private:
    QColor fColor;
    float fUPerDiv;
    float fOffset;
    bool fEnabled;
};

#endif
