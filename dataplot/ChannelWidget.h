#ifndef VC_CHANNELWIDGET_H
#define VC_CHANNELWIDGET_H

#include <QWidget>
#include <QSignalMapper>
#include <QPushButton>
#include <QCheckBox>
#include <QGridLayout>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QAction>

#include "VCWidget.h"
#include "File.h"

class ChannelWidget : public QWidget
{
  Q_OBJECT
  
  public:
    ChannelWidget(VCWidget* vcWidget, QWidget* parent=0);
    
    QAction* toggleGOAction();
    void addFile(const File& file);
    void removeFile(const File& file);
    void removeAllFiles();
    void reloadFile(const File& file, const std::vector<Channel*>& oldChannels);
  
  public slots:
    void channelEnableChanged(int ch);
    void channelChannelChanged(int ch);
    void channelScaleChanged(int ch);
    void channelOffsetChanged(int ch);
    void showGOChanged(bool show);
  
  private:
    void makeChannelCfg(QGridLayout* l, int ch, VCWidget* vcWidget,
        QSignalMapper* enableMapper, QSignalMapper* channelMapper,
        QSignalMapper *scaleMapper, QSignalMapper* offsetMapper);
    
    QCheckBox* fEnableWidgets[VCWidget::N_VCHANNELS];
    QComboBox* fChannelWidgets[VCWidget::N_VCHANNELS];
    QComboBox* fGainWidgets[VCWidget::N_VCHANNELS];
    QDoubleSpinBox* fOffsetWidgets[VCWidget::N_VCHANNELS];
    
    VCWidget* fVCWidget;
};

#endif
