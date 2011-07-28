#include "ChannelWidget.h"
#include "Channel.h"
#include <iostream>
#include <algorithm>
#include <limits>
#include <QString>
#include <QVBoxLayout>
#include <QLabel>
#include <QVariant>

const Channel ChannelComboBox::itemData(int index) const
{
    QVariant var = QComboBox::itemData(index);
    if(var.isValid())
        return fHash.value(var.value<Channel::id_t>());
    else
        return Channel();
}

int ChannelComboBox::findData(const Channel& data) const
{
    return QComboBox::findData(QVariant::fromValue<Channel::id_t>(data.id()));
}

void ChannelComboBox::addItem(const QString& text, const Channel& userData)
{
    fHash.insert(userData.id(), userData);
    QComboBox::addItem(text, QVariant::fromValue<Channel::id_t>(userData.id()));
}

void ChannelComboBox::insertItem(int index, const QString& text,
    const Channel& userData)
{
    fHash.insert(userData.id(), userData);
    QComboBox::insertItem(index, text, QVariant::fromValue<Channel::id_t>(userData.id()));
}

void ChannelComboBox::removeItem(int index)
{
    QVariant var = QComboBox::itemData(index);
    if(var.isValid())
        fHash.remove(var.value<Channel::id_t>());
    QComboBox::removeItem(index);
}

void ChannelComboBox::clear()
{
    fHash.clear();
    QComboBox::clear();
}

ChannelWidget::ChannelWidget(VCWidget* vcWidget, QWidget* parent)
    : QWidget(parent), fVCWidget(vcWidget)
{
    QGridLayout* l1 = new QGridLayout;
    
    QSignalMapper* enableMapper = new QSignalMapper(this);
    QSignalMapper* channelMapper = new QSignalMapper(this);
    QSignalMapper* scaleMapper = new QSignalMapper(this);
    QSignalMapper* offsetMapper = new QSignalMapper(this);
    for(int ch=0; ch < VCWidget::N_VCHANNELS; ch++) {
        makeChannelCfg(l1, ch, vcWidget, enableMapper, channelMapper, scaleMapper, offsetMapper);
    }
    connect(enableMapper, SIGNAL(mapped(int)), this, SLOT(channelEnableChanged(int)));
    connect(channelMapper, SIGNAL(mapped(int)), this, SLOT(channelChannelChanged(int)));
    connect(scaleMapper, SIGNAL(mapped(int)), this, SLOT(channelScaleChanged(int)));
    connect(offsetMapper, SIGNAL(mapped(int)), this, SLOT(channelOffsetChanged(int)));
    
    setLayout(l1);
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
}

QAction* ChannelWidget::toggleGOAction()
{
    QAction* act = new QAction(tr("Adjust gain/offset"), this);
    act->setCheckable(true);
    act->setChecked(false);
    connect(act, SIGNAL(toggled(bool)), this, SLOT(showGOChanged(bool)));
    return act;
}

void ChannelWidget::addChannel(const Channel& ch)
{
    for(int vch=0; vch < VCWidget::N_VCHANNELS; vch++) {
        std::string name = ch.name();
        fChannelWidgets[vch]->addItem(name.c_str(), ch);
    }
    
    // Find an ``empty'' vchannel and use it for channel ch
    int vch = 0;
    for(; vch < VCWidget::N_VCHANNELS; vch++) {
        if(fChannelWidgets[vch]->currentIndex() == 0) break;
    }
    if(vch < VCWidget::N_VCHANNELS) {
        int index = fChannelWidgets[vch]->findData(ch);
        fChannelWidgets[vch]->setCurrentIndex(index);
        fEnableWidgets[vch]->setChecked(true);
    }
}

void ChannelWidget::addFile(const File& file)
{
    File::ch_iterator_t tch = file.channels().begin();
    
    for(int vch=0; vch < VCWidget::N_VCHANNELS; vch++) {
        for(File::ch_iterator_t ch = file.channels().begin();
            ch != file.channels().end(); ch++)
        {
            std::string name = ch->name();
            fChannelWidgets[vch]->addItem(name.c_str(), *ch);
        }
        
        bool setDefault = (fChannelWidgets[vch]->currentIndex() == 0);
        if(setDefault && tch != file.channels().end()) {
            int index = fChannelWidgets[vch]->findData(*tch);
            fChannelWidgets[vch]->setCurrentIndex(index);
            fEnableWidgets[vch]->setChecked(true);
            tch++;
        }
    }
}

void ChannelWidget::removeFile(const File& file)
{
    for(File::ch_iterator_t ch = file.channels().begin();
        ch != file.channels().end(); ch++)
    {
        for(int vch=0; vch < VCWidget::N_VCHANNELS; vch++) {
            int index = fChannelWidgets[vch]->findData(*ch);
            fChannelWidgets[vch]->removeItem(index);
        }
    }
}

void ChannelWidget::removeAllFiles()
{
    for(int vch=0; vch < VCWidget::N_VCHANNELS; vch++) {
        fChannelWidgets[vch]->clear();
        fChannelWidgets[vch]->addItem(tr("<None>"));
    }
}

QString ChannelWidget::getChannelName(const Channel& ch)
{
    bool scaled = (ch.gainIdx() != Channel::CH_GAIN_DEFAULT ||
                   std::abs(ch.offset()) > 1e-10);
    QString name = ch.name().c_str();
    
    if(scaled)
        return (name + " (scaled)");
    else
        return name;
}

void ChannelWidget::updateChannelNames()
{
    for(int vch=0; vch < VCWidget::N_VCHANNELS; vch++) {
        int nch = fChannelWidgets[vch]->count();
        for(int index=0; index < nch; index++) {
            Channel ch(fChannelWidgets[vch]->itemData(index));
            if(ch)
                fChannelWidgets[vch]->setItemText(index, getChannelName(ch));
        }
    }
}

void ChannelWidget::reloadFile(const File& file, const std::vector<Channel>& oldChannels)
{
    int insertIndex = std::numeric_limits<int>::max()/2;
    int oldIndex[VCWidget::N_VCHANNELS];
    
    for(int vch=0; vch < VCWidget::N_VCHANNELS; vch++) {
        oldIndex[vch] = -1;
        for(std::vector<Channel>::const_iterator ch = oldChannels.begin();
            ch != oldChannels.end(); ch++) {
            int index = fChannelWidgets[vch]->findData(*ch);
            if(index == fChannelWidgets[vch]->currentIndex())
                oldIndex[vch] = index;
        }
        
        for(std::vector<Channel>::const_iterator ch = oldChannels.begin();
            ch != oldChannels.end(); ch++) {
            int index = fChannelWidgets[vch]->findData(*ch);
            fChannelWidgets[vch]->removeItem(index);
            insertIndex = std::min(insertIndex, index);
        }
    }
    
    File::ch_iterator_t tch = file.channels().begin();
    int new_nch = file.channels().size();
    
    for(int vch=0; vch < VCWidget::N_VCHANNELS; vch++) {
        int index = insertIndex;
        for(File::ch_iterator_t ch = file.channels().begin();
            ch != file.channels().end(); ch++)
        {
            std::string name = ch->name();
            fChannelWidgets[vch]->insertItem(index++, name.c_str(), *ch);
        }
        if(oldIndex[vch] >= 0) {
            if((oldIndex[vch] - insertIndex) >= new_nch) {
                fChannelWidgets[vch]->setCurrentIndex(0);
            } else {
                fChannelWidgets[vch]->setCurrentIndex(oldIndex[vch]);
            }
        }
    }
}

/* void ChannelWidget::loadFile(const char* fname)
{
    int old_nch = fVCWidget->nch();
    
    if(!fVCWidget->loadFile(fname)) return;
    
    for(int ch=0; ch<VCWidget::N_VCHANNELS; ch++) {
        for(int n = old_nch; n<fVCWidget->nch(); n++) {
            fChannelWidgets[ch]->addItem(fVCWidget->channel(n)->name().c_str());
        }
    }
    
    int n_end = std::min(fVCWidget->nch(), VCWidget::N_VCHANNELS);
    for(int n = old_nch; n < n_end; n++) {
        fChannelWidgets[n]->setCurrentIndex(n);
        fVCWidget->vchannel(n)->setChannel(n);
        fEnableWidgets[n]->setCheckState(Qt::Checked);
    }
} */

void ChannelWidget::showGOChanged(bool show)
{
    for(int ch=0; ch<VCWidget::N_VCHANNELS; ch++) {
        fGainWidgets[ch]->setVisible(show);
        fOffsetWidgets[ch]->setVisible(show);
    }
}

void ChannelWidget::updateChannelName(const Channel& ch)
{
    for(int vch=0; vch < VCWidget::N_VCHANNELS; vch++) {
        int index = fChannelWidgets[vch]->findData(ch);
        if(index >= 0)
            fChannelWidgets[vch]->setItemText(index, getChannelName(ch));
    }
}

void ChannelWidget::channelEnableChanged(int ch)
{
    bool enabled = fEnableWidgets[ch]->isChecked();
    
    fVCWidget->vchannel(ch)->setEnabled(enabled);
    fVCWidget->chParamsChanged();
    
    int idx = fChannelWidgets[ch]->currentIndex();
    Channel channel(fChannelWidgets[ch]->itemData(idx));
    
    fChannelWidgets[ch]->setEnabled(enabled);
    fGainWidgets[ch]->setEnabled(enabled && (bool)channel);
    fOffsetWidgets[ch]->setEnabled(enabled && (bool)channel);
}

void ChannelWidget::channelChannelChanged(int ch)
{
    int idx = fChannelWidgets[ch]->currentIndex();
    Channel channel(fChannelWidgets[ch]->itemData(idx));
    fVCWidget->vchannel(ch)->setChannel(channel);
    fVCWidget->chParamsChanged();
    
    if(channel) {
        fOffsetWidgets[ch]->setValue(channel.offset());
        fGainWidgets[ch]->setCurrentIndex(channel.gainIdx());
    } else {
        fOffsetWidgets[ch]->setValue(0);
        fGainWidgets[ch]->setCurrentIndex(Channel::CH_GAIN_DEFAULT);
    }
    fGainWidgets[ch]->setEnabled((bool)channel);
    fOffsetWidgets[ch]->setEnabled((bool)channel);
}

void ChannelWidget::channelScaleChanged(int vch)
{
    int gain_idx = fGainWidgets[vch]->currentIndex();
    
    if(!fVCWidget->vchannel(vch)->channel())
        return;
    
    fVCWidget->vchannel(vch)->channel().setGainIdx(gain_idx);
    updateChannelName(fVCWidget->vchannel(vch)->channel());
    fVCWidget->chParamsChanged();
    
    // Read back all gain values
    for(int n=0; n < VCWidget::N_VCHANNELS; n++) {
        int idx = fChannelWidgets[n]->currentIndex();
        Channel channel(fChannelWidgets[n]->itemData(idx));
        
        if(channel) {
            fGainWidgets[n]->setCurrentIndex(channel.gainIdx());
        } else {
            fGainWidgets[n]->setCurrentIndex(Channel::CH_GAIN_DEFAULT);
        }
    }
}

void ChannelWidget::channelOffsetChanged(int vch)
{
    double offset = fOffsetWidgets[vch]->value();
    
    if(fVCWidget->vchannel(vch)->channel()) {
        fVCWidget->vchannel(vch)->channel().setOffset(offset);
        updateChannelName(fVCWidget->vchannel(vch)->channel());
        fVCWidget->chParamsChanged();
    }
    
    // Read back all offset values
    for(int n=0; n < VCWidget::N_VCHANNELS; n++) {
        int idx = fChannelWidgets[n]->currentIndex();
        Channel channel(fChannelWidgets[n]->itemData(idx));
        
        if(channel) {
            fOffsetWidgets[n]->setValue(channel.offset());
        } else {
            fOffsetWidgets[n]->setValue(0);
        }
    }
}

void ChannelWidget::makeChannelCfg(QGridLayout* l, int ch, VCWidget* vcWidget,
    QSignalMapper* enableMapper, QSignalMapper* channelMapper,
    QSignalMapper* scaleMapper, QSignalMapper* offsetMapper)
{
    QWidget* bar = new QWidget();
    QPalette p;
    p.setColor(QPalette::Background, vcWidget->vchannel(ch)->color());
    bar->setAutoFillBackground(true);
    bar->setPalette(p);
    bar->setFixedHeight(8);
    
    QCheckBox* enable = new QCheckBox("Enable");
    enable->setCheckState(Qt::Unchecked);
    enableMapper->setMapping(enable, ch);
    connect(enable, SIGNAL(stateChanged(int)), enableMapper, SLOT(map()));
    fEnableWidgets[ch] = enable;
    
    ChannelComboBox* channel = new ChannelComboBox();
    channel->addItem(tr("<None>"));
    channel->setEnabled(false);
    channelMapper->setMapping(channel, ch);
    connect(channel, SIGNAL(currentIndexChanged(int)), channelMapper, SLOT(map()));
    fChannelWidgets[ch] = channel;
    
    QComboBox* scale = new QComboBox();
    for(unsigned int i=0; i<Channel::N_CH_GAIN_CHOICES; i++) {
        QString str = QString("x%1").arg(Channel::CH_GAIN_CHOICES[i]);
        scale->addItem(str);
    }
    scale->setEnabled(false);
    scale->setCurrentIndex(Channel::CH_GAIN_DEFAULT);
    scaleMapper->setMapping(scale, ch);
    connect(scale, SIGNAL(currentIndexChanged(int)), scaleMapper, SLOT(map()));
    fGainWidgets[ch] = scale;
    
    QDoubleSpinBox* offset = new QDoubleSpinBox();
    offset->setEnabled(false);
    offset->setRange(-1000000., 1000000.);
    offset->setDecimals(1);
    offset->setSingleStep(0.1);
    offsetMapper->setMapping(offset, ch);
    connect(offset, SIGNAL(valueChanged(double)), offsetMapper, SLOT(map()));
    fOffsetWidgets[ch] = offset;
    
    scale->hide();
    offset->hide();
    
    l->addWidget(bar, 0, ch);
    l->addWidget(enable, 1, ch);
    l->addWidget(channel, 2, ch);
    l->addWidget(scale, 3, ch);
    l->addWidget(offset, 4, ch);
}

