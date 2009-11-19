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
    
    l3->addWidget(new QLabel(tr("Camera")), 0, 0, 1, 2);
    QFrame* f = new QFrame;
    f->setFrameStyle(QFrame::HLine | QFrame::Sunken);
    l3->addWidget(f, 1, 0, 1, 2);
    
    feX = makeInput(l3, tr("X"), 2);
    connect(feX, SIGNAL(editingFinished()), this, SLOT(setCamPos()));
    feY = makeInput(l3, tr("Y"), 3);
    connect(feY, SIGNAL(editingFinished()), this, SLOT(setCamPos()));
    feZ = makeInput(l3, tr("Z"), 4);
    connect(feZ, SIGNAL(editingFinished()), this, SLOT(setCamPos()));
    feP = makeInput(l3, tr("φ"), 5);
    connect(feP, SIGNAL(editingFinished()), this, SLOT(setCamPhi()));
    feT = makeInput(l3, tr("θ"), 6);
    connect(feT, SIGNAL(editingFinished()), this, SLOT(setCamTheta()));
    
    // l3->addWidget(new QCheckBox(tr("Lock to object")), 7, 1);
    
    fHomeButton = new QPushButton(tr("Home"));
    l3->addWidget(fHomeButton, 7, 1);
    connect(fHomeButton, SIGNAL(clicked()), fGLWidget, SLOT(resetCamPos()));
    
    l3->setRowStretch(8, 1);
    
    setLayout(l0);
    
    setWindowTitle(sim->GetTitle());
    
    updateCamInfo();
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
    feX->setText(QString::number(fGLWidget->getCamPos().x(), 'f', 1));
    feY->setText(QString::number(fGLWidget->getCamPos().y(), 'f', 1));
    feZ->setText(QString::number(fGLWidget->getCamPos().z(), 'f', 1));
    feP->setText(QString::number(fGLWidget->getCamPhi(), 'f', 1));
    feT->setText(QString::number(fGLWidget->getCamTheta(), 'f', 1));
}

void SimulationViewer::setCamPos()
{
    fGLWidget->setCamPos(feX->text().toDouble(),
                         feY->text().toDouble(),
                         feZ->text().toDouble());
}

void SimulationViewer::setCamPhi()
{
    fGLWidget->setCamPhi(feP->text().toDouble());
    // Update with possibly normalized angle
    feP->setText(QString::number(fGLWidget->getCamPhi(), 'f', 1));
}

void SimulationViewer::setCamTheta()
{
    fGLWidget->setCamTheta(feT->text().toDouble());
    // Update with possibly normalized angle
    feT->setText(QString::number(fGLWidget->getCamTheta(), 'f', 1));
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
