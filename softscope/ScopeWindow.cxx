#include "ScopeWindow.h"
#include <iostream>
#include <QString>
#include <QVBoxLayout>
#include <QLabel>

const int ScopeWindow::U_PER_DIV_CHOICES[] = 
    { 1, 2, 5, 10, 20, 50, 100, 200, 500, 1000, 2000, 5000, 10000, 20000, 50000 };
const int ScopeWindow::U_PER_DIV_DEFAULT = 12; // index!
const int ScopeWindow::U_PER_DIV_WIDTH = 5;   // max width in characters
const int ScopeWindow::T_PER_DIV_CHOICES[] = 
    { 1, 2, 5, 10, 20, 50, 100, 200, 500, 1000 };
const int ScopeWindow::T_PER_DIV_DEFAULT = 6; // index!
const int ScopeWindow::T_PER_DIV_WIDTH = 4;   // max width in characters

ScopeWindow::ScopeWindow()
    : QWidget()
{
    fScopeWidget = new ScopeWidget(this, T_PER_DIV_CHOICES[T_PER_DIV_DEFAULT]);
    
    QVBoxLayout *l0 = new QVBoxLayout;
    QGridLayout *l1 = new QGridLayout;
    l0->addLayout(l1);
    l0->addWidget(fScopeWidget);
    
    QSignalMapper* enableMapper = new QSignalMapper(this);
    QSignalMapper* scaleMapper = new QSignalMapper(this);
    QSignalMapper* offsetMapper = new QSignalMapper(this);
    for(int ch=0; ch < ScopeWidget::N_CHANNELS; ch++) {
        makeChannelCfg(l1, ch, enableMapper, scaleMapper, offsetMapper);
    }
    connect(enableMapper, SIGNAL(mapped(int)), this, SLOT(channelEnableChanged(int)));
    connect(scaleMapper, SIGNAL(mapped(int)), this, SLOT(channelScaleChanged(int)));
    connect(offsetMapper, SIGNAL(mapped(int)), this, SLOT(channelOffsetChanged(int)));
    
    l1->setColumnMinimumWidth(fScopeWidget->N_CHANNELS, 40);
    makeTimeCfg(l1, fScopeWidget->N_CHANNELS + 1);
    connect(fRunPauseButton, SIGNAL(clicked()), this, SLOT(runPauseButton()));
    connect(fTimeScaleWidget, SIGNAL(currentIndexChanged(int)), this, SLOT(timeScaleChanged()));
    
    setLayout(l0);
    setWindowTitle("SoftScope");
}

void ScopeWindow::runPauseButton()
{
    if(fScopeWidget->running()) {
        fScopeWidget->setRunning(false);
        fRunPauseButton->setText("Run");
        fTimeScaleWidget->setEnabled(false);
    } else {
        fScopeWidget->setRunning(true);
        fRunPauseButton->setText("Pause");
        fTimeScaleWidget->setEnabled(true);
    }
}

void ScopeWindow::timeScaleChanged()
{
    int scale_idx = fTimeScaleWidget->currentIndex();
    int scale = T_PER_DIV_CHOICES[scale_idx];
    
    fScopeWidget->setTicsPerDiv(scale);
}

void ScopeWindow::channelEnableChanged(int ch)
{
    bool enabled = fEnableWidgets[ch]->isChecked();
    
    fScopeWidget->channel(ch)->setEnabled(enabled);
    fScopeWidget->chParamsChanged();
    
    fScaleWidgets[ch]->setEnabled(enabled);
    fOffsetWidgets[ch]->setEnabled(enabled);
}

void ScopeWindow::channelScaleChanged(int ch)
{
    int scale_idx = fScaleWidgets[ch]->currentIndex();
    int scale = U_PER_DIV_CHOICES[scale_idx];
    
    fOffsetWidgets[ch]->setSingleStep(scale);
    fScopeWidget->channel(ch)->setUPerDiv(scale);
    fScopeWidget->chParamsChanged();
    
    std::cout << "scale(" << ch << ") = " << scale << std::endl;
}

void ScopeWindow::channelOffsetChanged(int ch)
{
    int offset = fOffsetWidgets[ch]->value();
    
    fScopeWidget->channel(ch)->setOffset(offset);
    fScopeWidget->chParamsChanged();
    
    std::cout << "offset(" << ch << ") = " << offset << std::endl;
}

void ScopeWindow::makeChannelCfg(QGridLayout* l, int ch, QSignalMapper* enableMapper, QSignalMapper *scaleMapper, QSignalMapper* offsetMapper)
{
    QWidget* bar = new QWidget();
    QPalette p;
    p.setColor(QPalette::Background, fScopeWidget->channel(ch)->color());
    bar->setAutoFillBackground(true);
    bar->setPalette(p);
    bar->setFixedHeight(8);
    
    QCheckBox* enable = new QCheckBox("Enable");
    enable->setCheckState(Qt::Checked);
    enableMapper->setMapping(enable, ch);
    connect(enable, SIGNAL(stateChanged(int)), enableMapper, SLOT(map()));
    fEnableWidgets[ch] = enable;
    
    QComboBox* scale = new QComboBox();
    for(unsigned int i=0; i<(sizeof(U_PER_DIV_CHOICES)/sizeof(U_PER_DIV_CHOICES[0])); i++) {
        QString str = QString("%1 /div").arg(U_PER_DIV_CHOICES[i], U_PER_DIV_WIDTH);
        scale->addItem(str);
    }
    scale->setCurrentIndex(U_PER_DIV_DEFAULT);
    scaleMapper->setMapping(scale, ch);
    connect(scale, SIGNAL(currentIndexChanged(int)), scaleMapper, SLOT(map()));
    fScaleWidgets[ch] = scale;
    
    fScopeWidget->channel(ch)->setUPerDiv(U_PER_DIV_CHOICES[U_PER_DIV_DEFAULT]);
    fScopeWidget->channel(ch)->setOffset(0.);
    
    QSpinBox* offset = new QSpinBox();
    offset->setRange(-1000000, 1000000);
    offset->setSingleStep(U_PER_DIV_CHOICES[U_PER_DIV_DEFAULT]);
    offsetMapper->setMapping(offset, ch);
    connect(offset, SIGNAL(valueChanged(int)), offsetMapper, SLOT(map()));
    fOffsetWidgets[ch] = offset;
    
    l->addWidget(bar, 0, ch);
    l->addWidget(enable, 1, ch);
    l->addWidget(scale, 2, ch);
    l->addWidget(offset, 3, ch);
}

void ScopeWindow::makeTimeCfg(QGridLayout* l, int col)
{
    QWidget* bar = new QWidget();
    QPalette p;
    p.setColor(QPalette::Background, Qt::white);
    bar->setAutoFillBackground(true);
    bar->setPalette(p);
    bar->setFixedHeight(8);
    
    fRunPauseButton = new QPushButton("Pause");
    
    fTimeScaleWidget = new QComboBox();
    for(unsigned int i=0; i<(sizeof(T_PER_DIV_CHOICES)/sizeof(T_PER_DIV_CHOICES[0])); i++) {
        QString str = QString("%1 /div").arg(T_PER_DIV_CHOICES[i], T_PER_DIV_WIDTH);
        fTimeScaleWidget->addItem(str);
    }
    fTimeScaleWidget->setCurrentIndex(T_PER_DIV_DEFAULT);
    
    l->addWidget(bar, 0, col);
    l->addWidget(fRunPauseButton, 1, col);
    l->addWidget(fTimeScaleWidget, 2, col);
}
