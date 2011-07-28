#include "ChannelDialog.h"
#include "DataViewWidget.h"

ChannelDialog::ChannelDialog(VCWidget* vc, QWidget* parent)
    : QDialog(parent)
{
    setWindowTitle("Configure data channels");
    QHBoxLayout* l = new QHBoxLayout;
    fChannelWidget = new ChannelWidget(vc, this);
    fChannelWidget->showGOChanged(true);
    l->addWidget(fChannelWidget);
    setLayout(l);
}
