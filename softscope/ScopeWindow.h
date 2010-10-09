#ifndef __SCOPEWINDOW_H__
#define __SCOPEWINDOW_H__

#include <QWidget>
#include <QSignalMapper>
#include <QPushButton>
#include <QCheckBox>
#include <QGridLayout>
#include <QComboBox>
#include <QSpinBox>
#include "ScopeWidget.h"

class ScopeWindow : public QWidget
{
  Q_OBJECT
  
  public:
    ScopeWindow(const char* sourcename);
  
  public slots:
    void channelEnableChanged(int ch);
    void channelScaleChanged(int ch);
    void channelOffsetChanged(int ch);
    void runPauseButton();
    void timeScaleChanged();
  
  private:
    void makeChannelCfg(QGridLayout* l, int ch, QSignalMapper* enableMapper, QSignalMapper *scaleMapper, QSignalMapper* offsetMapper);
    void makeTimeCfg(QGridLayout* l, int col);
    
    static const int U_PER_DIV_CHOICES[];
    static const int U_PER_DIV_DEFAULT, U_PER_DIV_WIDTH;
    static const int T_PER_DIV_CHOICES[];
    static const int T_PER_DIV_DEFAULT, T_PER_DIV_WIDTH;
    
    ScopeWidget *fScopeWidget;
    QCheckBox* fEnableWidgets[ScopeWidget::N_CHANNELS];
    QComboBox* fScaleWidgets[ScopeWidget::N_CHANNELS];
    QSpinBox* fOffsetWidgets[ScopeWidget::N_CHANNELS];
    QPushButton* fRunPauseButton;
    QComboBox* fTimeScaleWidget;
};

#endif
