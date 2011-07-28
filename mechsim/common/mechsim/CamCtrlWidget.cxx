#include "CamCtrlWidget.h"
#include <QLabel>
#include <QGridLayout>
#include <QCheckBox>
#include <QDoubleValidator>

CamCtrlWidget::CamCtrlWidget(QWidget* parent, GLWidget* glwidget)
    : QWidget(parent), fGLWidget(glwidget)
{
    QGridLayout* l3 = new QGridLayout;
    
    QFrame* f;
    l3->addWidget(new QLabel(tr("Camera")), 0, 0, 1, 2);
    f = new QFrame;
    f->setFrameStyle(QFrame::HLine | QFrame::Sunken);
    l3->addWidget(f, 1, 0, 1, 2);
    
    feX = makeInput(l3, tr("X"), 2, SLOT(setCamPos()));
    feY = makeInput(l3, tr("Y"), 3, SLOT(setCamPos()));
    feZ = makeInput(l3, tr("Z"), 4, SLOT(setCamPos()));
    
    feDist = makeInput(l3, tr("Dist"), 5, SLOT(setCamDist()));
    
    feTheta = makeInput(l3, tr("Theta"), 6, SLOT(setCamTheta()));
    fePhi = makeInput(l3, tr("Phi"), 7, SLOT(setCamPhi()));
    feRoll = makeInput(l3, tr("Roll"), 8, SLOT(setCamRoll()));
    
    QCheckBox* c;
    c = new QCheckBox(tr("Track object"));
    c->setCheckState(Qt::Checked);
    l3->addWidget(c, 9, 0, 1, 2);
    connect(c, SIGNAL(stateChanged(int)), this, SLOT(setTrackObject(int)));
    
    c = new QCheckBox(tr("Enable roll"));
    c->setCheckState(Qt::Unchecked);
    feRoll->setEnabled(false);
    l3->addWidget(c, 10, 0, 1, 2);
    connect(c, SIGNAL(stateChanged(int)), this, SLOT(setEnableRoll(int)));
    
    fHomeButton = new QPushButton(tr("Home"));
    l3->addWidget(fHomeButton, 11, 0, 1, 2);
    connect(fHomeButton, SIGNAL(clicked()), fGLWidget, SLOT(resetCamPos()));
    
    l3->addWidget(new QLabel(tr("Rotation center offset")), 12, 0, 1, 2);
    f = new QFrame;
    f->setFrameStyle(QFrame::HLine | QFrame::Sunken);
    l3->addWidget(f, 13, 0, 1, 2);
    
    fecx = makeInput(l3, tr("X"), 14, SLOT(setCenterOffset()));
    fecy = makeInput(l3, tr("Y"), 15, SLOT(setCenterOffset()));
    fecz = makeInput(l3, tr("Z"), 16, SLOT(setCenterOffset()));
    
    l3->setRowStretch(11, 1);
    
    connect(fGLWidget, SIGNAL(camChanged()), this, SLOT(updateCamInfo()));
    
    setLayout(l3);
}

QLineEdit* CamCtrlWidget::makeInput(QGridLayout* l, const QString& text, int row, const char* slot)
{
    l->addWidget(new QLabel(text), row, 0);
    QLineEdit* e = new QLineEdit;
    e->setAlignment(Qt::AlignRight);
    e->setValidator(new QDoubleValidator(e));
    l->addWidget(e, row, 1);
    if(slot)
        connect(e, SIGNAL(editingFinished()), this, slot);
    return e;
}

void CamCtrlWidget::updateCamInfo()
{
    feX->setText(QString::number(fGLWidget->getCamPos().x(), 'f', 1));
    feY->setText(QString::number(fGLWidget->getCamPos().y(), 'f', 1));
    feZ->setText(QString::number(fGLWidget->getCamPos().z(), 'f', 1));
    feDist->setText(QString::number(fGLWidget->getCamDist(), 'f', 1));
    feTheta->setText(QString::number(fGLWidget->getCamThetaDeg(), 'f', 1));
    fePhi->setText(QString::number(fGLWidget->getCamPhiDeg(), 'f', 1));
    feRoll->setText(QString::number(fGLWidget->getCamRollDeg(), 'f', 1));
    fecx->setText(QString::number(fGLWidget->getCenterOffset().x(), 'f', 1));
    fecy->setText(QString::number(fGLWidget->getCenterOffset().y(), 'f', 1));
    fecz->setText(QString::number(fGLWidget->getCenterOffset().z(), 'f', 1));
}

void CamCtrlWidget::setCamPos()
{
    fGLWidget->setCamPos(Vector3(feX->text().toDouble(),
                                 feY->text().toDouble(),
                                 feZ->text().toDouble()));
    updateCamInfo();
}

void CamCtrlWidget::setCamDist()
{
    fGLWidget->setCamDist(feDist->text().toDouble());
}

void CamCtrlWidget::setCamTheta()
{
    fGLWidget->setCamThetaDeg(feTheta->text().toDouble());
    updateCamInfo();
}

void CamCtrlWidget::setCamPhi()
{
    fGLWidget->setCamPhiDeg(fePhi->text().toDouble());
    updateCamInfo();
}

void CamCtrlWidget::setCamRoll()
{
    fGLWidget->setCamRollDeg(feRoll->text().toDouble());
    updateCamInfo();
}

void CamCtrlWidget::setCenterOffset()
{
    fGLWidget->setCenterOffset(Vector3(fecx->text().toDouble(),
                                       fecy->text().toDouble(),
                                       fecz->text().toDouble()));
}

void CamCtrlWidget::setTrackObject(int state)
{
    fGLWidget->setTrackObject(state == Qt::Checked);
}

void CamCtrlWidget::setEnableRoll(int state)
{
    if(state == Qt::Checked) {
        fGLWidget->setEnableRoll(true);
        feRoll->setEnabled(true);
    } else {
        fGLWidget->setEnableRoll(false);
        feRoll->setEnabled(false);
    }
}
