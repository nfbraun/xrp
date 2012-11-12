#ifndef SSCOPE_SCOPEWIDGET_H
#define SSCOPE_SCOPEWIDGET_H

#include "DataReader.h"
#include "Channel.h"
#include <QWidget>
#include <QPaintEvent>
#include <QTimer>
#include <QPixmap>

class ScopeWidget : public QWidget
{
    Q_OBJECT
    
  public:
    ScopeWidget(const char* sourcename, QWidget *parent = 0, int tperdiv = 1);
    
    QSize minimumSizeHint() const;
    QSize sizeHint() const;
    inline const Channel* channel(int ch) const
      { if(ch < 0 || ch >= N_CHANNELS) return NULL; return &fChannels[ch]; }
    inline Channel* channel(int ch)
      { if(ch < 0 || ch >= N_CHANNELS) return NULL; return &fChannels[ch]; }
    inline bool running()  { return fRunning; }
    
    void chParamsChanged();
    void setRunning(bool running);
    void setTicsPerDiv(int tperdiv);
    
    static const int N_CHANNELS = 4;

  public slots:
    void timestep();

  protected:
    inline int ticsPerDiv()    { return fTicsPerDiv; }
    inline int ticsPerScreen() { return 10 * fTicsPerDiv; }
  
    void paintEvent(QPaintEvent* ev);
    void resizeEvent(QResizeEvent* ev);
    void generateGrid();
    void updateDisplay();
    void redrawDisplay();
    void drawRegion(QPainter& painter, int t1, int t2);
    float t2x(int t);
    float u2y(float u, float u_per_div, float offset);
    
    Channel fChannels[N_CHANNELS];
    QTimer* fTimer;
    bool fRunning;
    int fTicsPerDiv;
    DataReader<float> fReader;
    int fLastHead;
    static const int UPDATE_INTERVAL;
    QPixmap fGridPixmap, fDisplayPixmap;
};

#endif
