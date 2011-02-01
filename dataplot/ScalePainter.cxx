#include "ScalePainter.h"
#include <QString>
#include <iostream>

void ScalePainter::getTicDistance(double tic, double& major_tic, double& minor_tic, int& n)
{
    double exp;
    
    // limit tic distance to a sensible value
    if(tic < 0.001)
        tic = 0.001;
    
    // Write tic in the form tic * exp, where exp = 10^n with n \in N
    // and 1 < tic <= 10
    exp = 1.0;
    n = 0;
    while(tic <= 1.0) {
        tic *= 10;
        exp *= 0.1;
        n--;
    }
    
    while(tic > 10.0) {
        tic *= 0.1;
        exp *= 10;
        n++;
    }
    
    if(tic > 5.0) {
        major_tic = 10.0 * exp;
        minor_tic = 5.0 * exp;
        n++;
    } else if(tic > 2.0) {
        major_tic = 5.0 * exp;
        minor_tic = 1.0 * exp;
    } else { // if(tic > 1.0)
        major_tic = 2.0 * exp;
        minor_tic = 1.0 * exp;
    }
}

void ScalePainter::drawXScale(QPainter& painter, const VPTransform& tr)
{
    int x;
    int y = tr.yBase() + 2;
    int i, i2;
    double major_tic, minor_tic;
    int n;
    
    getTicDistance((double) 50.0 / tr.xZoom(), major_tic, minor_tic, n);
    
    // Draw the minor tics
    i = (int) ceil(tr.xScrToUsr(tr.xBase()) / minor_tic);
    i2 = (int) floor(tr.xScrToUsr(tr.xBase() + tr.fVPWidth) / minor_tic);
    
    for(; i<=i2; ++i) {
        x = tr.round(tr.xUsrToScr((double) i * minor_tic));
        painter.drawLine(x, y+1, x, y+5);
    }
    
    // Draw the major tics
    i = (int) ceil(tr.xScrToUsr(tr.xBase()) / major_tic);
    i2 = (int) floor(tr.xScrToUsr(tr.xBase() + tr.fVPWidth) / major_tic);
    
    for(; i<=i2; ++i) {
        x = tr.round(tr.xUsrToScr((double) i * major_tic));
        painter.drawLine(x, y+1, x, y+9);
        
        QString str = QString("%1").arg((double) major_tic * i, 0, 'f',
            (n < 0) ? -n : 0);
        painter.drawText(x-50, y+12, 100, 100, Qt::AlignHCenter | Qt::AlignTop, str);
    }
}

void ScalePainter::drawYScale(QPainter& painter, const VPTransform& tr)
{
    int x = tr.xBase() - 2;
    int y;
    int i, i2;
    double major_tic, minor_tic;
    int n;
    
    getTicDistance((double) 50.0 / tr.yZoom(), major_tic, minor_tic, n);
    
    // Draw the minor tics
    i = (int) ceil(tr.yScrToUsr(tr.yBase()) / minor_tic);
    i2 = (int) floor(tr.yScrToUsr(tr.yBase() - tr.fVPHeight) / minor_tic);
    
    for(; i<=i2; ++i) {
        y = tr.round(tr.yUsrToScr((double) i * minor_tic));
        painter.drawLine(x-5, y, x, y);
    }
    
    // Draw the major tics
    i = (int) ceil(tr.yScrToUsr(tr.yBase()) / major_tic);
    i2 = (int) floor(tr.yScrToUsr(tr.yBase() - tr.fVPHeight) / major_tic);
    
    for(; i<=i2; ++i) {
        y = tr.round(tr.yUsrToScr((double) i * major_tic));
        painter.drawLine(x-9, y, x, y);
        
        QString str = QString("%1").arg((double) major_tic * i, 0, 'g', 4);
        painter.drawText(x-12-100, y-50, 100, 100, Qt::AlignRight | Qt::AlignVCenter, str);
    }
}

void ScalePainter::drawGrid(QPainter& painter, const VPTransform& tr, QRect region,
    const QPen& majorPen, const QPen& minorPen)
{
    int x, y;
    int i, i2;
    double x_major_tic, x_minor_tic;
    double y_major_tic, y_minor_tic;
    int n;
    
    getTicDistance((double) 50.0 / tr.xZoom(), x_major_tic, x_minor_tic, n);
    getTicDistance((double) 50.0 / tr.yZoom(), y_major_tic, y_minor_tic, n);
    
    // Draw the minor X grid lines
    i = (int) ceil(tr.xTileToUsr(region.x()) / x_minor_tic);
    i2 = (int) floor(tr.xTileToUsr(region.x() + region.width()) / x_minor_tic);
    
    painter.setPen(minorPen);
    for(; i<=i2; ++i) {
        x = tr.round(tr.xUsrToTile((double) i * x_minor_tic)) - region.x();
        painter.drawLine(x, 0, x, region.height());
    }
    
    // Draw the minor Y grid lines
    i = (int) ceil(tr.yTileToUsr(-region.y()-region.height()) / y_minor_tic);
    i2 = (int) floor(tr.yTileToUsr(-region.y()) / y_minor_tic);
    
    for(; i<=i2; ++i) {
        y = -region.y() - tr.round(tr.yUsrToTile((double) i * y_minor_tic));
        painter.drawLine(0, y, region.width(), y);
    }
    
    // Draw the major X grid lines
    i = (int) ceil(tr.xTileToUsr(region.x()) / x_major_tic);
    i2 = (int) floor(tr.xTileToUsr(region.x() + region.width()) / x_major_tic);
    
    painter.setPen(majorPen);
    for(; i<=i2; ++i) {
        x = tr.round(tr.xUsrToTile((double) i * x_major_tic)) - region.x();
        painter.drawLine(x, 0, x, region.height());
    }
    
    // Draw the major Y grid lines
    i = (int) ceil(tr.yTileToUsr(-region.y()-region.height()) / y_major_tic);
    i2 = (int) floor(tr.yTileToUsr(-region.y()) / y_major_tic);
    
    for(; i<=i2; ++i) {
        y = -region.y() - tr.round(tr.yUsrToTile((double) i * y_major_tic));
        painter.drawLine(0, y, region.width(), y);
    }
}

