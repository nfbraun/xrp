#ifndef VC_VCWIDGET_H
#define VC_VCWIDGET_H

#include <QWidget>
#include <QPaintEvent>
#include <QTimer>
#include <QPixmap>
#include <map>
#include <stdint.h>

#include "Channel.h"
#include "VChannel.h"
#include "VPTransform.h"
#include "Data.h"

class VCWidget : public QWidget
{
    Q_OBJECT
    
  public:
    VCWidget(QWidget *parent = 0);
    
    // bool loadFile(const char* fname);
    
    QSize minimumSizeHint() const;
    QSize sizeHint() const;
    
    inline const VChannel* vchannel(int ch) const
      { if(ch < 0 || ch >= N_VCHANNELS) return 0; return &fVChannels[ch]; }
    inline VChannel* vchannel(int ch)
      { if(ch < 0 || ch >= N_VCHANNELS) return 0; return &fVChannels[ch]; }
    
    static const int N_VCHANNELS = 4;
    
    void chParamsChanged();
    
  public slots:
    void scaleToFit();
    void scaleXToFit();
    void scaleYToFit();
    
  protected:
    void zoomAroundCursor(double fx, double fy);
    
    void paintEvent(QPaintEvent* ev);
    void resizeEvent(QResizeEvent* ev);
    
    void mousePressEvent(QMouseEvent* ev);
    void mouseReleaseEvent(QMouseEvent* ev);
    void mouseMoveEvent(QMouseEvent* ev);
    
    void wheelEvent(QWheelEvent* ev);
    
    bool fDragging;
    QPoint fCursorPos;
    
    void generateGrid();
    
    int getTileXId(int pos);
    int getTileYId(int pos);
    
    void flushTiles();
    void weedTiles();
    QPixmap renderTile(int xoff, int yoff);
    QPixmap getTile(int x, int y);
    float func(float x);
//    void updateDisplay();
//    void redrawDisplay();
//    void drawRegion(QPainter& painter, int t1, int t2);
//    float t2x(int t);
//    float u2y(float u, float u_per_div, float offset);
    
    std::map<uint32_t, QPixmap> fTiles;
    
    static const double cZoomFudge;
    static const int cTileWidth;
    int fTileHeight;
    
    VChannel fVChannels[N_VCHANNELS];
    
    VPTransform fTr;
};

#endif
