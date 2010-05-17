#include "SimulationViewer.h"
#include <QSlider>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QLabel>
#include <QTextCodec>
#include <QCheckBox>
#include <QDoubleValidator>

const char SimulationViewer::START[] = "▶";
const char SimulationViewer::PAUSE[] = "❚❚";

SimulationViewer::SimulationViewer(Simulation* sim)
{
    QTextCodec::setCodecForTr(QTextCodec::codecForName("UTF-8"));

    fGLWidget = new GLWidget(sim);
    fTimeSlide = new QSlider(Qt::Horizontal);
    fTimeSlide->setRange(0, sim->GetDefaultEndTime());
    
    connect(fTimeSlide, SIGNAL(valueChanged(int)), fGLWidget, SLOT(setTime(int)));
    connect(fGLWidget, SIGNAL(timeChanged(int)), fTimeSlide, SLOT(setValue(int)));
    connect(fGLWidget, SIGNAL(camChanged()), this, SLOT(updateCamInfo()));
    
    QHBoxLayout *l0 = new QHBoxLayout;
    
    QVBoxLayout *l1 = new QVBoxLayout;
    l0->addLayout(l1);
    l0->setStretch(0, 1);
    l1->addWidget(fGLWidget);
    
    QHBoxLayout *l2 = new QHBoxLayout;
    l1->addLayout(l2);
    
    fStartPauseButton = new QPushButton(tr(PAUSE), this);
    fStartPauseButton->setFixedWidth(30);
    connect(fStartPauseButton, SIGNAL(clicked()), this, SLOT(toggleSimulationRunning()));
    
    l2->addWidget(fStartPauseButton);
    l2->addWidget(fTimeSlide);
    
    QGridLayout* l3 = new QGridLayout;
    l0->addLayout(l3);
    
    QFrame* f;
    l3->addWidget(new QLabel(tr("Camera")), 0, 0, 1, 2);
    f = new QFrame;
    f->setFrameStyle(QFrame::HLine | QFrame::Sunken);
    l3->addWidget(f, 1, 0, 1, 2);
    
    feZ = makeInput(l3, tr("Distance"), 2);
    connect(feZ, SIGNAL(editingFinished()), this, SLOT(setCamDist()));
    
    l3->addWidget(new QCheckBox(tr("Lock to object")), 3, 0, 1, 2);
    l3->addWidget(new QCheckBox(tr("Lock Z axis")), 4, 0, 1, 2);
    
    fHomeButton = new QPushButton(tr("Home"));
    l3->addWidget(fHomeButton, 5, 0, 1, 2);
    connect(fHomeButton, SIGNAL(clicked()), fGLWidget, SLOT(resetCamPos()));
    
    l3->addWidget(new QLabel(tr("Rotation center")), 6, 0, 1, 2);
    f = new QFrame;
    f->setFrameStyle(QFrame::HLine | QFrame::Sunken);
    l3->addWidget(f, 7, 0, 1, 2);
    
    fecx = makeInput(l3, tr("X"), 8);
    fecy = makeInput(l3, tr("Y"), 9);
    fecz = makeInput(l3, tr("Z"), 10);
    
    l3->setRowStretch(11, 1);
    
    setLayout(l0);
    
    setWindowTitle(sim->GetTitle());
    
    updateCamInfo();
    
    fGLWidget->pause();
}

QLineEdit* SimulationViewer::makeInput(QGridLayout* l, const QString& text, int row)
{
    l->addWidget(new QLabel(text), row, 0);
    QLineEdit* e = new QLineEdit;
    e->setAlignment(Qt::AlignRight);
    e->setValidator(new QDoubleValidator(e));
    l->addWidget(e, row, 1);
    return e;
}

void SimulationViewer::updateCamInfo()
{
    feZ->setText(QString::number(fGLWidget->getCamDist(), 'f', 1));
    fecx->setText(QString::number(fGLWidget->getCenter().x(), 'f', 1));
    fecy->setText(QString::number(fGLWidget->getCenter().y(), 'f', 1));
    fecz->setText(QString::number(fGLWidget->getCenter().z(), 'f', 1));
}

void SimulationViewer::setCamDist()
{
    fGLWidget->setCamDist(feZ->text().toDouble());
}

void SimulationViewer::keyPressEvent(QKeyEvent* ev)
{
    if(ev->key() == Qt::Key_Space) {
        toggleSimulationRunning();
    } else {
        ev->ignore();
    }
}

void SimulationViewer::toggleSimulationRunning()
{
    if(fGLWidget->isPaused()) {
        fStartPauseButton->setText(tr(PAUSE));
        fGLWidget->start();
    } else {
        fStartPauseButton->setText(tr(START));
        fGLWidget->pause();
    }
}
