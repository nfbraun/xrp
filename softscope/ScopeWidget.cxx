#include "ScopeWidget.h"
#include <iostream>
#include <cmath>
#include <QPainter>

const int ScopeWidget::UPDATE_INTERVAL = 50;

ScopeWidget::ScopeWidget(const char* sourcename, QWidget* parent, int tperdiv)
    : QWidget(parent),
      fRunning(true),
      fTicsPerDiv(tperdiv),
      fReader(sourcename, ticsPerScreen(), N_CHANNELS)
{
    fChannels[0] = Channel(Qt::yellow);
    fChannels[1] = Channel(Qt::cyan);
    fChannels[2] = Channel(Qt::green);
    fChannels[3] = Channel(Qt::magenta);

    fTimer = new QTimer(this);
    connect(fTimer, SIGNAL(timeout()), this, SLOT(timestep()));
    fTimer->start(UPDATE_INTERVAL);
    
    fLastHead = 0;
}

void ScopeWidget::setRunning(bool running)
{
    if(fRunning == running) return;
    if(fRunning) {
        fTimer->stop();
    } else {
        fReader.readAll();
        fReader.reset();
        fLastHead = 0;
        redrawDisplay();
        update();
        fTimer->start(UPDATE_INTERVAL);
    }
    fRunning = running;
}

void ScopeWidget::setTicsPerDiv(int tperdiv)
{
    fTicsPerDiv = tperdiv;
    fReader.reset(ticsPerScreen());
    fLastHead = 0;
    redrawDisplay();
    update();
}

void ScopeWidget::chParamsChanged()
{
    redrawDisplay();
    update();
}

void ScopeWidget::resizeEvent(QResizeEvent* ev)
{
    fGridPixmap = QPixmap(ev->size());
    fDisplayPixmap = QPixmap(ev->size());
    generateGrid();
    redrawDisplay();
}

void ScopeWidget::generateGrid()
{
    QPainter painter(&fGridPixmap);
    fGridPixmap.fill(Qt::black);
    int w = fGridPixmap.width(), h= fGridPixmap.height();
    
    painter.setPen(QPen(Qt::gray, 1, Qt::DashLine));
    for(int x=1; x<10; ++x) {
        int xp = x*(w-3)/10 + 1;
        painter.drawLine(xp, 2, xp, h-3);
    }
    for(int y=1; y<10; ++y) {
        int yp = y*(h-3)/10 + 1;
        painter.drawLine(2, yp, w-3, yp);
    }
    
    painter.setPen(QPen(Qt::white, 1));
    painter.drawRect(2, 2, w - 5, h - 5);
    painter.drawLine(2, (h-1)/2, w-3, (h-1)/2);
    painter.drawLine((w-1)/2, 2, (w-1)/2, h-3);
}

void ScopeWidget::drawRegion(QPainter& painter, int t1, int t2)
{
    for(int ch=0; ch<N_CHANNELS; ch++) {
        if(fChannels[ch].enabled()) {
            float x, y;
            float u_per_div = fChannels[ch].uPerDiv();
            float offset = fChannels[ch].offset();
            float lastx = t2x(t1);
            float lasty = u2y(fReader.at(t1, ch), u_per_div, offset);
            
            painter.setPen(QPen(fChannels[ch].color(), 1));
            
            for(int t=t1+1; t<=t2; t++) {
                x = t2x(t);
                y = u2y(fReader.at(t, ch), u_per_div, offset);
                painter.drawLine(lastx, lasty, x, y);
                lastx = x;
                lasty = y;
            }
        }
    }
}

void ScopeWidget::redrawDisplay()
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
}

float ScopeWidget::u2y(float u, float u_per_div, float offset)
{
    int h = size().height();
    return -(u+offset) * ((float) (h-5)/(10*u_per_div)) + (h-1)/2.;
}

float ScopeWidget::t2x(int t)
{
    int w = size().width();
    t %= ticsPerScreen();
    return ((float)t * ((float) (w-5)/ticsPerScreen())) + 2;
}

void ScopeWidget::updateDisplay()
{
    int head = fReader.head();
    
    if(fReader.checkAndClearReset())
        redrawDisplay();
    else if(head <= fLastHead)
        return;
    else if((head - fLastHead) >= ticsPerScreen())
        redrawDisplay();
    
    QPainter painter(&fDisplayPixmap);
    int w = size().width();
    int h = size().height();
    
    // Clean up
    int x1 = (int) floor(t2x(fLastHead + ticsPerDiv())) - 1;
    int x2 = (int) ceil(t2x(head + ticsPerDiv()));
    
    if(x2 >= x1) {
        painter.drawPixmap(QPoint(x1, 0), fGridPixmap, QRect(x1, 0, x2-x1+1, h));
    } else {
        painter.drawPixmap(QPoint(x1, 0), fGridPixmap, QRect(x1, 0, w-3-x1+1, h));
        painter.drawPixmap(QPoint(1, 0), fGridPixmap, QRect(1, 0, x2-1+1, h));
    }
    
    // Draw trace
    painter.setRenderHint(QPainter::Antialiasing, true);
    
    if((head % ticsPerScreen()) >= (fLastHead % ticsPerScreen())) {
        drawRegion(painter, fLastHead, head);
    } else {
        int t1 = (head/(ticsPerScreen()))*ticsPerScreen();
        drawRegion(painter, t1, head);
        drawRegion(painter, fLastHead, t1-1);
    }
    
    fLastHead = head;
}

void ScopeWidget::timestep()
{
    fReader.readAll();
    update();
}

void ScopeWidget::paintEvent(QPaintEvent*)
{
    QPainter painter(this);
    updateDisplay();
    painter.drawPixmap(QPoint(0, 0), fDisplayPixmap);
}

QSize ScopeWidget::minimumSizeHint() const
{
    return QSize(100, 100);
}

QSize ScopeWidget::sizeHint() const
{
    return QSize(640, 480);
}
