#include "VCWidget.h"
#include "ScalePainter.h"
#include "CSVFile.h"
#include <iostream>
#include <sstream>
#include <cmath>
#include <algorithm>
#include <QPainter>

//#define DEBUG_TILE_CACHE
//#define DEBUG_MARK_TILES

const int VCWidget::cTileWidth = 128;
const double VCWidget::cZoomFudge = 0.1;

VCWidget::VCWidget(QWidget* parent)
    : QWidget(parent),
      fDragging(false),
      fTr(cTileWidth/2),
      fXLocked(false),
      fXDoZoomAroundMarker(false),
      fShowLegend(false),
      fMarkerPos(-1.)
{
    fVChannels[0] = VChannel(Qt::yellow);
    fVChannels[1] = VChannel(Qt::cyan);
    fVChannels[2] = VChannel(Qt::green);
    fVChannels[3] = VChannel(Qt::magenta);
    
    fTr.setBorders(50, 10, 10, 30);
    
    setMouseTracking(true);
}

void VCWidget::scaleToFit()
{
    scaleXToFit();
    scaleYToFit();
}

void VCWidget::scaleXToFit()
{
    double minX = std::numeric_limits<double>::infinity();
    double maxX = -std::numeric_limits<double>::infinity();
    
    for(int vch=0; vch < N_VCHANNELS; vch++) {
        if(vchannel(vch)->enabled()) {
            Channel ch(vchannel(vch)->channel());
            minX = std::min(minX, ch.data().minX());
            maxX = std::max(maxX, ch.data().maxX());
        }
    }
    
    if(minX != std::numeric_limits<double>::infinity()) {
        fTr.setXVisibleRegion(std::max((1. + cZoomFudge)*(maxX - minX), 1e-5));
        fTr.setXUserOffset(minX + cZoomFudge/2.*(maxX - minX));
    }
    
    flushTiles();
    update();
}

void VCWidget::scaleYToFit()
{
    double minY = std::numeric_limits<double>::infinity();
    double maxY = -std::numeric_limits<double>::infinity();
    
    for(int vch=0; vch < N_VCHANNELS; vch++) {
        if(vchannel(vch)->enabled()) {
            Channel ch(vchannel(vch)->channel());
            double gain = ch.gain();
            double offset = ch.offset();
            minY = std::min(minY, ch.data().getMinY()*gain + offset);
            maxY = std::max(maxY, ch.data().getMaxY()*gain + offset);
        }
    }
    
    if(minY != std::numeric_limits<double>::infinity()) {
        fTr.setYVisibleRegion(std::max((1. + cZoomFudge)*(maxY - minY), 1e-5));
        fTr.setYUserOffset(minY - cZoomFudge/2.*(maxY - minY));
    }
    
    flushTiles();
    update();
}

void VCWidget::setXVisibleRegion(double xv)
{
    fTr.setXVisibleRegion(xv);
    flushTiles();
    update();
}

void VCWidget::setYVisibleRegion(double yv)
{
    fTr.setYVisibleRegion(yv);
    flushTiles();
    update();
}

void VCWidget::chParamsChanged()
{
    flushTiles();
    update();
}

void VCWidget::setBorders(int l, int r, int t, int b)
{
    fTr.setBorders(l, r, t, b);
    flushTiles();
    update();
}

void VCWidget::setMarkerUserPos(double pos)
{
    fTr.setXUserOffset(-pos + fMarkerPos*fTr.xVisibleRegion());
    flushTiles();
    update();
}

void VCWidget::setMarkerUserPos_Lazy(double pos)
{
    fTr.setXUserOffset_Lazy(-pos + fMarkerPos*fTr.xVisibleRegion());
    update();
}

void VCWidget::resizeEvent(QResizeEvent* ev)
{
    fTr.setSize(ev->size().width(), ev->size().height());
    fTileHeight = fTr.fVPHeight + cTileWidth;
    
    flushTiles();
}

void VCWidget::mousePressEvent(QMouseEvent* ev)
{
    if(ev->button() == Qt::LeftButton)
        fDragging = true;
    ev->accept();
}

void VCWidget::mouseReleaseEvent(QMouseEvent* ev)
{
    if(ev->button() == Qt::LeftButton)
        fDragging = false;
    ev->accept();
}

void VCWidget::wheelEvent(QWheelEvent* ev)
{
    if(ev->orientation() == Qt::Vertical) {
        if(ev->modifiers() & Qt::ShiftModifier) {
            zoomAround(fCursorPos.x(), fCursorPos.y(),
                       1., exp((double) ev->delta() / 240. * M_LN2));
        } else {
            if(xLocked() || xDoZoomAroundMarker()) {
                zoomAround(fTr.xBase() + fTr.fVPWidth * fMarkerPos, 0,
                           exp((double) ev->delta() / 240. * M_LN2), 1.);
            } else {
                zoomAround(fCursorPos.x(), fCursorPos.y(),
                           exp((double) ev->delta() / 240. * M_LN2), 1.);
                emit xPosChanged(markerUserPos());
            }
        }
        
        ev->accept();
    }
}

void VCWidget::mouseMoveEvent(QMouseEvent* ev)
{
    int dX = (int) fCursorPos.x() - ev->x();
    int dY = (int) fCursorPos.y() - ev->y();
    
    fCursorPos = ev->pos();
    
    ev->accept();
    
    if(fDragging) {
        if(xLocked()) {
            fTr.shiftTileOffset(0, -dY);
        } else {
            fTr.shiftTileOffset(-dX, -dY);
            emit xPosChanged(markerUserPos());
        }
        update();
    }
}

void VCWidget::zoomAround(double x, double y, double fx, double fy)
{
    fTr.zoomAround(x, y, fx, fy);
    
    flushTiles();
    update();
}

void VCWidget::flushTiles()
{
    fTiles.clear();
}

void VCWidget::weedTiles()
{
    int16_t x, y;
    int xpos, ypos;
    std::map<uint32_t,QPixmap>::iterator iter, elem;
    
    #ifdef DEBUG_TILE_CACHE
    std::cout << "Weeding..." << std::endl;
    #endif
    
    iter = fTiles.begin();
    while(iter != fTiles.end()) {
        elem = iter;
        iter++;
        x = elem->first & 0xFFFF;
        y = elem->first >> 16;
        xpos = (int) x*cTileWidth+fTr.fXTileOffset;
        ypos = (int) y*fTileHeight+fTr.fYTileOffset;
        
        if(xpos < (-2 * cTileWidth) ||
           xpos > (fTr.fVPWidth + cTileWidth) ||
           ypos < (fTr.fTopBorder - fTileHeight - cTileWidth/2) ||
           ypos > (fTr.fTopBorder + fTr.fVPHeight + cTileWidth/2) )
        {
            #ifdef DEBUG_TILE_CACHE
            std::cout << "Deleting Tile " << x << " " << y << " " << xpos << " " << ypos << std::endl;
            #endif
            fTiles.erase(elem);
        }
    }
    
    #ifdef DEBUG_TILE_CACHE
    std::cout << fTiles.size() << " tiles remaining" << std::endl;
    #endif
}

QPixmap VCWidget::getTile(int x, int y)
{
    uint32_t id = (y << 16) | (x & 0xFFFF);
    std::map<uint32_t, QPixmap>::iterator iter;
    
    iter = fTiles.find(id);
    if(iter == fTiles.end()) {
        QPixmap tile = renderTile(x, y);
        
        #ifdef DEBUG_TILE_CACHE
        std::cout << "Rendering Tile " << x << " " << y << std::endl;
        #endif
        
        fTiles.insert(std::make_pair(id, tile));
        return tile;
    } else {
        return iter->second;
    }
}

QPixmap VCWidget::renderTile(int xoff, int yoff)
{
    QPixmap pixmap(cTileWidth, fTileHeight);
    pixmap.fill(Qt::black);
    QPainter painter(&pixmap);
    
    painter.setRenderHint(QPainter::Antialiasing, false);
    ScalePainter::drawGrid(painter, fTr, \
        QRect(xoff*cTileWidth, yoff*fTileHeight, cTileWidth, fTileHeight),
        QColor(50, 50, 50), QColor(20, 20, 20));
    
    renderCurves(painter, xoff, yoff);
    
    #ifdef DEBUG_MARK_TILES
    painter.setPen(QPen(Qt::white, 1));
    painter.drawRect(0, 0, cTileWidth, fTileHeight);
    painter.drawText(1, 1, 100, 100, Qt::AlignLeft | Qt::AlignTop,
        QString("(%1 %2)").arg(xoff).arg(yoff));
    #endif
    
    return pixmap;
}

void VCWidget::renderCurves(QPainter& painter, int xoff, int yoff, double xmin, double xmax)
{
    painter.setRenderHint(QPainter::Antialiasing, true);
    
    for(int vch=0; vch < N_VCHANNELS; vch++) {
        if(vchannel(vch)->enabled()) {
            Channel ch(vchannel(vch)->channel());
            double gain = ch.gain();
            double offset = ch.offset();
            Data::const_iterator_t d =
                ch.data().getFirst(std::max(fTr.xTileToUsr(xoff*cTileWidth-1),
                                            xmin));
            Data::const_iterator_t last =
                ch.data().getLast(std::min(fTr.xTileToUsr((xoff+1)*cTileWidth),
                                           xmax));
            
            QPen pen(vchannel(vch)->color(), 0);
            pen.setCapStyle(Qt::FlatCap);
            painter.setPen(pen);
            
            QPolygonF polyline;
            for(; d != last; d++) {
                float x = fTr.xUsrToTile(d->first) - cTileWidth*xoff;
                float y = -fTr.yUsrToTile(d->second*gain + offset) - fTileHeight*yoff;
                polyline.append(QPointF(x+.5, y+.5));
                //std::cout << d->first << "," << d->second << " ";
            }
            //std::cout << std::endl;
            painter.drawPolyline(polyline);
            //if(polyline.size() < 2)
            //    std::cout << "*****    " << xmin << " " << xmax << std::endl;
        }
    }
}

void VCWidget::addNewData(double xmin, double xmax)
{
    std::map<uint32_t,QPixmap>::iterator iter, elem;
    
    iter = fTiles.begin();
    while(iter != fTiles.end()) {
        elem = iter;
        iter++;
        int16_t x = elem->first & 0xFFFF;
        int16_t y = elem->first >> 16;
        if(fTr.xTileToUsr((x+1)*cTileWidth) >= xmin &&
           fTr.xTileToUsr(x*cTileWidth) <= xmax) {
            QPainter painter(&elem->second);
            renderCurves(painter, x, y, xmin, xmax);
        }
    }
    update();
}

int VCWidget::getTileXId(int pos)
{
    return pos < 0 ? (pos / cTileWidth) - 1 : pos / cTileWidth;
}

int VCWidget::getTileYId(int pos)
{
    return pos < 0 ? (pos / fTileHeight) - 1 : pos / fTileHeight;
}

void VCWidget::drawMarker(QPainter& painter)
{
    if(markerPos() >= 0. && markerPos() <= 1.) {
        int xp = std::ceil(fTr.xBase() + fTr.fVPWidth * fMarkerPos - 0.5);
        painter.setPen(QPen(Qt::white));
        painter.drawLine(xp, fTr.yBase() - fTr.fVPHeight, xp, fTr.yBase() - 1);
    }
}

void VCWidget::drawLegend(QPainter& painter)
{
    // Parameters
    const int INFOBOX_BORDER = 3;
    const int INFOBOX_DIST = 10;
    
    QFontMetrics fm = painter.fontMetrics();
    int text_width = 0;
    for(int vch=0; vch<N_VCHANNELS; vch++) {
        text_width = std::max(text_width,
            fm.width(vchannel(vch)->channel().name().c_str()));
    }
    
    int text_ascent = fm.ascent();
    int text_height = fm.height();
    int x = width() - fTr.fRightBorder - text_width - INFOBOX_DIST;
    int y = fTr.fTopBorder + INFOBOX_DIST;
    painter.fillRect(x-INFOBOX_BORDER, y-INFOBOX_BORDER,
                     text_width+2*INFOBOX_BORDER,
                     N_VCHANNELS*text_height + 2*INFOBOX_BORDER,
                     QColor(0, 0, 0, 200));
    for(int vch=0; vch<N_VCHANNELS; vch++) {
        if(vchannel(vch)->enabled()) {
            painter.setPen(vchannel(vch)->color());
            painter.drawText(x, y+text_ascent+vch*text_height,
                vchannel(vch)->channel().name().c_str());
        }
    }
}

void VCWidget::paintEvent(QPaintEvent*)
{
    // std::cout << "paintEvent()" << std::endl;
    
    QPainter painter(this);
    
    int x, y;
    int x1, y1, x2, y2;
    QPixmap tile;
    unsigned int NTiles;
    int src_x, src_y, p_width, p_height, dest_x, dest_y;
    
    x1 = getTileXId(fTr.fLeftBorder - fTr.fXTileOffset);
    x2 = getTileXId(fTr.fLeftBorder + fTr.fVPWidth - fTr.fXTileOffset - 1);
    y1 = getTileYId(fTr.fTopBorder - fTr.fYTileOffset);
    y2 = getTileYId(fTr.fTopBorder + fTr.fVPHeight - fTr.fYTileOffset - 1);
    
    for(x=x1; x<=x2; x++) {
        for(y=y1; y<=y2; y++) {
            tile = getTile(x, y);
          
            // Calculate parameters for tile copy operation
            src_x = 0; src_y = 0;
            p_width = cTileWidth; p_height = fTileHeight;
            dest_x = x*cTileWidth+fTr.fXTileOffset;
            dest_y = y*fTileHeight+fTr.fYTileOffset;
          
            // Perform clipping
            if(dest_x + p_width > fTr.fLeftBorder + fTr.fVPWidth)
                p_width = fTr.fLeftBorder + fTr.fVPWidth - dest_x;
            
            if(dest_y + p_height > fTr.fTopBorder + fTr.fVPHeight)
                p_height = fTr.fTopBorder + fTr.fVPHeight - dest_y;

            if(dest_x < fTr.fLeftBorder) {
                src_x += fTr.fLeftBorder - dest_x;
                p_width -= fTr.fLeftBorder - dest_x;
                dest_x = fTr.fLeftBorder;
            }
          
            if(dest_y < fTr.fTopBorder) {
                src_y += fTr.fTopBorder - dest_y;
                p_height -= fTr.fTopBorder - dest_y;
                dest_y = fTr.fTopBorder;
            }
            
            painter.drawPixmap(QPoint(dest_x, dest_y), tile, QRect(src_x, src_y, p_width, p_height));
        }
    }
    
    ScalePainter::drawXScale(painter, fTr);
    ScalePainter::drawYScale(painter, fTr);
    
    drawMarker(painter);
    if(showLegend())
        drawLegend(painter);
    
    NTiles = 2*(fTr.fVPWidth/cTileWidth + 4);
    if(fTiles.size() > NTiles)
        weedTiles();
}

QSize VCWidget::minimumSizeHint() const
{
    return QSize(100, 100);
}

QSize VCWidget::sizeHint() const
{
    return QSize(640, 480);
}
