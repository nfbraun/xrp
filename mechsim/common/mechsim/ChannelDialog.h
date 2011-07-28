#ifndef MSIM_CHANNELDIALOG_H
#define MSIM_CHANNELDIALOG_H

#include <QDialog>
#include "dataplot/VCWidget.h"
#include "dataplot/ChannelWidget.h"

class ChannelDialog : public QDialog
{
    public:
        ChannelDialog(VCWidget* vc, QWidget* parent = 0);
        void addChannel(const Channel& ch)
            { fChannelWidget->addChannel(ch); }
    
    private:
        ChannelWidget* fChannelWidget;
};

#endif
