#include "VCWidget.h"
#include "ScalePainter.h"
#include "CSVFile.h"
#include <iostream>
#include <sstream>
#include <cmath>
#include <QPainter>

//#define DEBUG_TILE_CACHE
//#define DEBUG_MARK_TILES

const int VCWidget::cTileWidth = 128;
const double VCWidget::cZoomFudge = 0.1;

VCWidget::VCWidget(QWidget* parent)
    : QWidget(parent),
      fDragging(false),
      fTr(cTileWidth/2)
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
            Channel* ch = vchannel(vch)->channel();
            minX = std::min(minX, ch->data().minX());
            maxX = std::max(maxX, ch->data().maxX());
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
            Channel* ch = vchannel(vch)->channel();
            double gain = ch->gain();
            double offset = ch->offset();
            minY = std::min(minY, ch->data().getMinY()*gain + offset);
            maxY = std::max(maxY, ch->data().getMaxY()*gain + offset);
        }
    }
    
    if(minY != std::numeric_limits<double>::infinity()) {
        fTr.setYVisibleRegion(std::max((1. + cZoomFudge)*(maxY - minY), 1e-5));
        fTr.setYUserOffset(minY - cZoomFudge/2.*(maxY - minY));
    }
    
    flushTiles();
    update();
}

void VCWidget::chParamsChanged()
{
    flushTiles();
    update();
}

void VCWidget::resizeEvent(QResizeEvent* ev)
{
/*     fGridPixmap = QPixmap(ev->size());
    fDisplayPixmap = QPixmap(ev->size());
    generateGrid();
    redrawDisplay(); */
    
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
        if(ev->modifiers() & Qt::ShiftModifier)
            zoomAroundCursor(1., exp((double) ev->delta() / 240. * M_LN2));
        else
            zoomAroundCursor(exp((double) ev->delta() / 240. * M_LN2), 1.);
        
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
        fTr.shiftTileOffset(-dX, -dY);
        update();
    }
}

void VCWidget::zoomAroundCursor(double fx, double fy)
{
    fTr.zoomAround(fCursorPos.x(), fCursorPos.y(), fx, fy);
    
    flushTiles();
    update();
}

/* float VCWidget::func(float x)
{
    return sin(x) * exp(-(x/20)*(x/20));
} */

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
    
    painter.setRenderHint(QPainter::Antialiasing, true);
    
    for(int vch=0; vch < N_VCHANNELS; vch++) {
        if(vchannel(vch)->enabled()) {
            Channel* ch = vchannel(vch)->channel();
            double gain = ch->gain();
            double offset = ch->offset();
            Data::const_iterator_t d =
                ch->data().getFirst(fTr.xTileToUsr(xoff*cTileWidth-1));
            Data::const_iterator_t last =
                ch->data().getLast(fTr.xTileToUsr((xoff+1)*cTileWidth));
            
            painter.setPen(QPen(vchannel(vch)->color(), 1));
            
            QPolygonF polyline;
            for(; d != last; d++) {
                float x = fTr.xUsrToTile(d->first) - cTileWidth*xoff;
                float y = -fTr.yUsrToTile(d->second*gain + offset) - fTileHeight*yoff;
                polyline.append(QPointF(x+.5, y+.5));
            }
            painter.drawPolyline(polyline);
        }
    }
    
    #ifdef DEBUG_MARK_TILES
    painter.setPen(QPen(Qt::white, 1));
    painter.drawRect(0, 0, cTileWidth, fTileHeight);
    painter.drawText(1, 1, 100, 100, Qt::AlignLeft | Qt::AlignTop,
        QString("(%1 %2)").arg(xoff).arg(yoff));
    #endif
    
    return pixmap;
}

int VCWidget::getTileXId(int pos)
{
    return pos < 0 ? (pos / cTileWidth) - 1 : pos / cTileWidth;
}

int VCWidget::getTileYId(int pos)
{
    return pos < 0 ? (pos / fTileHeight) - 1 : pos / fTileHeight;
}

/* void ScopeWidget::redrawDisplay()
{
    int head = fReader.head();
    
    QPainter painter(&fDisplayPixmap);
    painter.drawPixmap(QPoint(0, 0), fGridPixmap);
    
    if(head < 0) return;
    
    painter.setRenderHint(QPainter::Antialiasing, true);
    
    int t1 = (head/(ticsPerScreen()))*ticsPerScreen();
    int t2 = head;
    int t3 = head - 9*ticsPerDiv();
    int t4 = t1 - 1;
    
    if(t3 < 0) {
        drawRegion(painter, t1, t2);
    } else if((t3 % ticsPerScreen()) < (t2 % ticsPerScreen())) {
        drawRegion(painter, t3, t2);
    } else {
        drawRegion(painter, t1, t2);
        drawRegion(painter, t3, t4);
    }
    
    fLastHead = head;
} */

void VCWidget::paintEvent(QPaintEvent*)
{
    // std::cout << "paintEvent()" << std::endl;
    
    QPainter painter(this);
//    updateDisplay();
//     painter.drawPixmap(QPoint(0, 0), fTestPixmap);
    
    int x, y;
    int x1, y1, x2, y2;
    // bool cv = fCursorVisible;
    QPixmap tile;
    unsigned int NTiles;
    int src_x, src_y, p_width, p_height, dest_x, dest_y;
    
    x1 = getTileXId(fTr.fLeftBorder - fTr.fXTileOffset);
    x2 = getTileXId(fTr.fLeftBorder + fTr.fVPWidth - fTr.fXTileOffset - 1);
    y1 = getTileYId(fTr.fTopBorder - fTr.fYTileOffset);
    y2 = getTileYId(fTr.fTopBorder + fTr.fVPHeight - fTr.fYTileOffset - 1);
    
    // if(cv) DrawCursor();
    
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
    
    // if(cv) DrawCursor();
    
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
