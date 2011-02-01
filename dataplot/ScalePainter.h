/*
  A note about the coordinate system:
  fXBase is the point that corresponds to energy 0 (if fOffset = 0)
  fYBase is the point that corresponds to zero counts
  fOffset is an x shift, in pixels

  fXZoom is in pixels per energy
  fYZoom is in pixels per count
*/

#ifndef VC_SCALEPAINTER_H
#define VC_SCALEPAINTER_H

#include "VPTransform.h"
#include <QPainter>

class ScalePainter {
  public:
    static void drawXScale(QPainter& painter, const VPTransform& tr);
    static void drawYScale(QPainter& painter, const VPTransform& tr);
    static void drawGrid(QPainter& painter, const VPTransform& tr, QRect region,
        const QPen& majorPen, const QPen& minorPen);
    
  protected:
    static void getTicDistance(double tic, double& major_tic, double& minor_tic, int& n);
};

#endif
