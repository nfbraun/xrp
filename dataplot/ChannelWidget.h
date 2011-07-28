#ifndef VC_CHANNELWIDGET_H
#define VC_CHANNELWIDGET_H

#include <QHash>
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

class ChannelComboBox : public QComboBox
{
  public:
    ChannelComboBox(QWidget* parent=0) : QComboBox(parent) {}
    const Channel itemData(int index) const;
    int findData(const Channel& data) const;
    void addItem(const QString& text, const Channel& userData = Channel());
    void insertItem(int index, const QString& text,
        const Channel& userData = Channel());
    void removeItem(int index);
    void clear();
    
  private:
    QHash<Channel::id_t, Channel> fHash;
};

class ChannelWidget : public QWidget
{
  Q_OBJECT
  
  public:
    ChannelWidget(VCWidget* vcWidget, QWidget* parent=0);
    
    QAction* toggleGOAction();
    void addChannel(const Channel& ch);
    void addFile(const File& file);
    void removeFile(const File& file);
    void removeAllFiles();
    void updateChannelNames();
    void reloadFile(const File& file, const std::vector<Channel>& oldChannels);
  
  public slots:
    void channelEnableChanged(int ch);
    void channelChannelChanged(int ch);
    void channelScaleChanged(int ch);
    void channelOffsetChanged(int ch);
    void showGOChanged(bool show);
  
  private:
    QString getChannelName(const Channel& ch);
    void updateChannelName(const Channel& ch);
    void makeChannelCfg(QGridLayout* l, int ch, VCWidget* vcWidget,
        QSignalMapper* enableMapper, QSignalMapper* channelMapper,
        QSignalMapper *scaleMapper, QSignalMapper* offsetMapper);
    
    QCheckBox* fEnableWidgets[VCWidget::N_VCHANNELS];
    ChannelComboBox* fChannelWidgets[VCWidget::N_VCHANNELS];
    QComboBox* fGainWidgets[VCWidget::N_VCHANNELS];
    QDoubleSpinBox* fOffsetWidgets[VCWidget::N_VCHANNELS];
    
    VCWidget* fVCWidget;
};

#endif
