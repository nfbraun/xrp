#include "SimulationWidget.h"
#include <QMenu>
#include <QMenuBar>
#include <QSlider>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QSplitter>
#include <QLabel>
#include <QTextCodec>
#include <QCheckBox>

#include "dataplot/Data.h"

const char SimulationWidget::START[] = "▶";
const char SimulationWidget::PAUSE[] = "❚❚";

SimulationWidget::SimulationWidget(SimRunner* sim)
    : QMainWindow()
{
    fSimRunner = sim;

    QTextCodec::setCodecForTr(QTextCodec::codecForName("UTF-8"));

    fGLWidget = new GLWidget(sim, this, true);
    fTimeSlide = new QSlider(Qt::Horizontal);
    fTimeSlide->setRange(0, sim->GetDefaultEndTime());
    
    QSplitter *l = new QSplitter;
    l->setOrientation(Qt::Vertical);
    
    QHBoxLayout *l0 = new QHBoxLayout;
    
    QVBoxLayout *l1 = new QVBoxLayout;
    l0->addLayout(l1);
    l0->setStretch(0, 1);
    l1->addWidget(fGLWidget);
    
    QHBoxLayout *l2 = new QHBoxLayout;
    l1->addLayout(l2);
    
    fStartPauseButton = new QPushButton(fGLWidget->isPaused() ? tr(START) : tr(PAUSE),
                                        this);
    fStartPauseButton->setFixedWidth(30);
    connect(fStartPauseButton, SIGNAL(clicked()), this, SLOT(toggleSimulationRunning()));
    
    l2->addWidget(fStartPauseButton);
    l2->addWidget(fTimeSlide);
    
    fCamCtrlWidget = new CamCtrlWidget(this, fGLWidget);
    l0->addWidget(fCamCtrlWidget);
    
    QWidget* w = new QWidget;
    w->setLayout(l0);
    l->addWidget(w);
    
    fDataView = new DataViewWidget(sim, this);
    l->addWidget(fDataView);
    setCentralWidget(l);
    // fDataView->hide();
    
    // circular connection: fTimeSlide -> fGLWidget -> fDataView -> fTimeSlide
    connect(fTimeSlide, SIGNAL(valueChanged(int)), fGLWidget, SLOT(setTime(int)));
    connect(fGLWidget, SIGNAL(timeChanged(int)), fDataView, SLOT(setTime(int)));
    connect(fDataView, SIGNAL(timeChanged(int)), fTimeSlide, SLOT(setValue(int)));
    
    
    /** Build the menu **/
    /*** FILE ***/
    QMenu* fileMenu = menuBar()->addMenu(tr("&File"));
    QAction* quitAct = new QAction("&Quit", this);
    quitAct->setShortcut(QString("Ctrl+Q"));
    connect(quitAct, SIGNAL(triggered()), this, SLOT(close()));
    fileMenu->addAction(quitAct);
    
    /*** VIEW ***/
    QMenu* viewMenu = menuBar()->addMenu(tr("&View"));
    
    QAction* camCtrlAct = new QAction("Show &camera controls", this);
    camCtrlAct->setCheckable(true);
    camCtrlAct->setChecked(true);
    camCtrlAct->setShortcut(QString("F9"));
    connect(camCtrlAct, SIGNAL(toggled(bool)), fCamCtrlWidget,
        SLOT(setVisible(bool)));
    viewMenu->addAction(camCtrlAct);
    
    QAction* dataViewAct = new QAction("Show &data view", this);
    dataViewAct->setCheckable(true);
    dataViewAct->setChecked(true);
    dataViewAct->setShortcut(QString("F10"));
    connect(dataViewAct, SIGNAL(toggled(bool)), fDataView,
        SLOT(setVisible(bool)));
    viewMenu->addAction(dataViewAct);
    
    QAction* dataConfigAct = new QAction("C&onfigure data view...", this);
    dataConfigAct->setShortcut(QString("Ctrl+C"));
    connect(dataConfigAct, SIGNAL(triggered()), fDataView,
        SLOT(showChannelDialog()));
    viewMenu->addAction(dataConfigAct);
    
    viewMenu->addSeparator();
    
    QActionGroup* drawModeGroup = new QActionGroup(this);
    QSignalMapper* drawModeMapper = new QSignalMapper(this);
    
    for(int i=0; i<sim->GetNDrawModes(); i++) {
        QString title = QString("%1: %2").arg(i+1).arg(sim->GetDrawModeName(i));
        QAction* drawModeAct = new QAction(title, this);
        drawModeAct->setCheckable(true);
        if(i < 9)
            drawModeAct->setShortcut(QString("Ctrl+%1").arg(i+1));
        drawModeGroup->addAction(drawModeAct);
        viewMenu->addAction(drawModeAct);
        if(i == 0)
            drawModeAct->setChecked(true);
        
        drawModeMapper->setMapping(drawModeAct, i);
        connect(drawModeAct, SIGNAL(triggered()), drawModeMapper, SLOT(map()));
    }
    
    connect(drawModeMapper, SIGNAL(mapped(int)), this, SLOT(setDrawMode(int)));
    
    connect(this, SIGNAL(simHasNewData()), fDataView, SLOT(addNewData()));
    
    setWindowTitle(sim->GetTitle());
    
    fCamCtrlWidget->updateCamInfo();
    
    if(sim->GetDescriptor() >= 0) {
        fSockNotifier = new QSocketNotifier(sim->GetDescriptor(),
                                        QSocketNotifier::Read);
        fSockNotifier->setParent(this);
        connect(fSockNotifier, SIGNAL(activated(int)), this,
            SLOT(simDataReady()));
    } else {
        emit simHasNewData();
    }
}

void SimulationWidget::setDrawMode(int mode)
{
    fGLWidget->setDrawMode(mode);
}

void SimulationWidget::simDataReady()
{
    fSockNotifier->setEnabled(false);
    fSimRunner->ReadData();
    if(!fSimRunner->finished())
        fSockNotifier->setEnabled(true);
    emit simHasNewData();
}

void SimulationWidget::keyPressEvent(QKeyEvent* ev)
{
    if(ev->key() == Qt::Key_Space) {
        toggleSimulationRunning();
    } else {
        ev->ignore();
    }
}

void SimulationWidget::toggleSimulationRunning()
{
    if(fGLWidget->isPaused()) {
        fStartPauseButton->setText(tr(PAUSE));
        fDataView->setXLocked(true);
        fGLWidget->start();
    } else {
        fStartPauseButton->setText(tr(START));
        fDataView->setXLocked(false);
        fGLWidget->pause();
    }
}
