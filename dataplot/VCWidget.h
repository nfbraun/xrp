#ifndef VC_VCWIDGET_H
#define VC_VCWIDGET_H

#include <QWidget>
#include <QPaintEvent>
#include <QTimer>
#include <QPixmap>
#include <map>
#include <limits>
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
    
    QSize minimumSizeHint() const;
    QSize sizeHint() const;
    
    inline const VChannel* vchannel(int ch) const
      { if(ch < 0 || ch >= N_VCHANNELS) return 0; return &fVChannels[ch]; }
    inline VChannel* vchannel(int ch)
      { if(ch < 0 || ch >= N_VCHANNELS) return 0; return &fVChannels[ch]; }
    
    static const int N_VCHANNELS = 8;
    
    void chParamsChanged();
    
    void setBorders(int l, int r, int t, int b);
    
    bool xLocked()     { return fXLocked; }
    void setXLocked(bool locked)  { fXLocked = locked; }
    bool xDoZoomAroundMarker() { return fXDoZoomAroundMarker; }
    void setXDoZoomAroundMarker(bool zoom) { fXDoZoomAroundMarker = zoom; }
    bool showLegend()  { return fShowLegend; }
    void setShowLegend(bool show) { fShowLegend = show; update(); }
    
    double markerPos() { return fMarkerPos; }
    void setMarkerPos(double pos) { fMarkerPos = pos; }
    double markerUserPos()
        { return fTr.xScrToUsr(fTr.xBase() + fTr.fVPWidth * fMarkerPos); }
    void setMarkerUserPos(double pos);
    void setMarkerUserPos_Lazy(double pos);  // does not require tile flush
    
    void setXVisibleRegion(double xv);
    void setYVisibleRegion(double xv);
    
  public slots:
    void scaleToFit();
    void scaleXToFit();
    void scaleYToFit();
    void addNewData(double xmin, double xmax);
  
  signals:
    void xPosChanged(double xpos);
    
  protected:
    void zoomAround(double x, double y, double fx, double fy);
    
    void paintEvent(QPaintEvent* ev);
    void resizeEvent(QResizeEvent* ev);
    
    void drawMarker(QPainter& painter);
    void drawLegend(QPainter& painter);
    
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
    void renderCurves(QPainter& painter, int xoff, int yoff,
        double xmin = -std::numeric_limits<double>::infinity(),
        double xmax = std::numeric_limits<double>::infinity());
    
    std::map<uint32_t, QPixmap> fTiles;
    
    static const double cZoomFudge;
    static const int cTileWidth;
    int fTileHeight;
    
    VChannel fVChannels[N_VCHANNELS];
    
    VPTransform fTr;
    
    bool fXLocked;
    bool fXDoZoomAroundMarker;
    bool fShowLegend;
    double fMarkerPos;
};

#endif
