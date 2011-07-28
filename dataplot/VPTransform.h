#ifndef VC_VPTRANSFORM_H
#define VC_VPTRANSFORM_H

#include <cmath>

class VPTransform
{
  public:
    VPTransform(int yoverlap = 0);
  
    inline double xTileToUsr(double x) const
        { return x / xZoom() - fXUOffset; }
    inline double xUsrToTile(double x) const
        { return (x + fXUOffset) * xZoom(); }
    inline double xTileToScr(double x) const
        { return x + fXTileOffset; }
    inline double xScrToTile(double x) const
        { return x - fXTileOffset; }
    inline double xUsrToScr(double x) const
        { return xTileToScr(xUsrToTile(x)); }
    inline double xScrToUsr(double x) const
        { return xTileToUsr(xScrToTile(x)); }
    
    inline double yTileToUsr(double y) const
        { return y / yZoom() + fYUOffset; }
    inline double yUsrToTile(double y) const
        { return (y - fYUOffset) * yZoom(); }
    inline double yTileToScr(double y) const
        { return -y + fYTileOffset; }
    inline double yScrToTile(double y) const
        { return -y + fYTileOffset; }
    inline double yUsrToScr(double y) const
        { return yTileToScr(yUsrToTile(y)); }
    inline double yScrToUsr(double y) const
        { return yTileToUsr(yScrToTile(y)); }
    
    void setSize(int width, int height);
    void zoomAround(double x, double y, double fx, double fy);
    
    void setBorders(int l, int r, int t, int b);
    
    inline void shiftTileOffset(int dx, int dy)
        { fXTileOffset += dx; fYTileOffset += dy; }
    
    void setXUserOffset(double xo);
    void setXUserOffset_Lazy(double xo);
    void setYUserOffset(double yo);
    
    inline int xBase() const { return fLeftBorder; }
    inline int yBase() const { return fVPHeight + fTopBorder; }
    
    inline double xVisibleRegion() const { return fXVisibleRegion; }
    inline double xZoom() const { return fVPWidth / fXVisibleRegion; }
    inline double yVisibleRegion() const { return fYVisibleRegion; }
    inline double yZoom() const { return fVPHeight / fYVisibleRegion; }
    void setXVisibleRegion(double xv);
    void setYVisibleRegion(double yv);
    
    inline double getXOffsetDelta(double x, double f) const
        { return (1.0 - 1.0/f) * (double)(x - xBase()) / xZoom(); }
    inline double getYOffsetDelta(double y, double f) const
        { return (1.0 - 1.0/f) * (double)(yBase() - y) / yZoom(); }
    
    static inline int round(double x) { return (int) ceil(x - 0.5); }
    
  public:
    // Viewport dimensions
    int fVPWidth, fVPHeight;
    
    // Borders
    int fLeftBorder, fRightBorder, fTopBorder, fBottomBorder;
    
    int fXTileOffset, fYTileOffset;
    double fXUOffset, fYUOffset;
    double fXUOffset_error;
    
    double fXVisibleRegion, fYVisibleRegion;
    
    int fYOverlap;
};

#endif
