#include "VPTransform.h"
#include <iostream>

VPTransform::VPTransform(int yoverlap)
    : fVPWidth(1), fVPHeight(1),
      fLeftBorder(0), fRightBorder(0), fTopBorder(0), fBottomBorder(0),
      fXTileOffset(fLeftBorder), fYTileOffset(fTopBorder+fVPHeight+yoverlap),
      fXUOffset(0.), fYUOffset(-yoverlap/1.0), fXUOffset_error(0.),
      fXVisibleRegion(1.), fYVisibleRegion(1.),
      fYOverlap(yoverlap)
{ }

void VPTransform::setXUserOffset(double xo)
{
    fXUOffset = xo;
    fXUOffset_error = 0.;
    fXTileOffset = fLeftBorder;
}

// Approximate the effect of setting x user offset to xo by modifying
//  xTileOffset, not requiring tile flush
void VPTransform::setXUserOffset_Lazy(double xo)
{
    int xo_int = std::ceil((xo - fXUOffset) * xZoom());
    fXUOffset_error = (xo - fXUOffset) - (double)xo_int/xZoom();
    fXTileOffset = fLeftBorder + xo_int;
}

void VPTransform::setYUserOffset(double yo)
{
    fYUOffset = yo - fYOverlap / yZoom();
    fYTileOffset = fTopBorder + fVPHeight + fYOverlap;
}

void VPTransform::setXVisibleRegion(double xv)
{
    // Add what is left over from lazy offset setting
    fXUOffset += fXUOffset_error;
    fXUOffset_error = 0.;
    
    // Convert offset to user units
    fXUOffset += (fXTileOffset-fLeftBorder) / xZoom();
    fXTileOffset = fLeftBorder;
    
    fXVisibleRegion = xv;
}

void VPTransform::setYVisibleRegion(double yv)
{
    // Convert offset to user units
    fYUOffset += (fYTileOffset-fTopBorder-fVPHeight) / yZoom();
    fYTileOffset = fTopBorder + fVPHeight + fYOverlap;
    
    fYVisibleRegion = yv;
    
    fYUOffset -= fYOverlap / yZoom();
}

void VPTransform::setBorders(int l, int r, int t, int b)
{
    // Add what is left over from lazy offset setting
    fXUOffset += fXUOffset_error;
    fXUOffset_error = 0.;
    
    // Convert offset to user units
    fXUOffset += (fXTileOffset-fLeftBorder) / xZoom();
    fYUOffset += (fYTileOffset-fTopBorder-fVPHeight) / yZoom();
    
    fVPWidth += fLeftBorder - l + fRightBorder - r;
    fVPHeight += fTopBorder - t + fBottomBorder - b;
    
    fLeftBorder = l;
    fRightBorder = r;
    fTopBorder = t;
    fBottomBorder = b;
    
    fXTileOffset = fLeftBorder;
    fYTileOffset = fTopBorder + fVPHeight + fYOverlap;
    fYUOffset -= fYOverlap / yZoom();
}

void VPTransform::setSize(int width, int height)
{
    // Add what is left over from lazy offset setting
    fXUOffset += fXUOffset_error;
    fXUOffset_error = 0.;
    
    // Convert offset to user units
    fXUOffset += (fXTileOffset-fLeftBorder) / xZoom();
    fYUOffset += (fYTileOffset-fTopBorder-fVPHeight) / yZoom();
    
    fVPWidth = width - fLeftBorder - fRightBorder;
    fVPHeight = height - fTopBorder - fBottomBorder;
    
    fXTileOffset = fLeftBorder;
    fYTileOffset = fTopBorder + fVPHeight + fYOverlap;
    fYUOffset -= fYOverlap / yZoom();
}

void VPTransform::zoomAround(double x, double y, double fx, double fy)
{
    // Add what is left over from lazy offset setting
    fXUOffset += fXUOffset_error;
    fXUOffset_error = 0.;
    
    // Convert offset to user units
    fXUOffset += (fXTileOffset-fLeftBorder) / xZoom();
    fYUOffset += (fYTileOffset-fTopBorder-fVPHeight) / yZoom();
    fXTileOffset = fLeftBorder;
    fYTileOffset = fTopBorder + fVPHeight + fYOverlap;
    
    // Adjust offset such that the user coordinates at the cursor position
    // stay constant
    fXUOffset -= getXOffsetDelta(x, fx);
    fYUOffset += getYOffsetDelta(y, fy);
    
    fXVisibleRegion /= fx;
    fYVisibleRegion /= fy;
    
    fYUOffset -= fYOverlap / yZoom();
}
