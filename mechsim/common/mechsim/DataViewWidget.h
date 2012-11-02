#ifndef MSIM_DATAVIEWWIDGET_H
#define MSIM_DATAVIEWWIDGET_H

#include <vector>
#include "ChannelDialog.h"
#include "dataplot/VCWidget.h"
#include "dataplot/Channel.h"
#include "SimRunner.h"

class DataViewWidget : public VCWidget
{
  Q_OBJECT
  
  public:
    DataViewWidget(SimRunner* sim, QWidget* parent = 0);
  
  public slots:
    void addNewData();
    void showChannelDialog();
    void setTime(int t);
    void setTime_d(double t_d);
    
  signals:
    void timeChanged(int t);
  
  private:
    SimRunner* fSimRunner;
    ChannelDialog *fChannelDialog;
    int fT;  // in units of SimRunner::GetTimestep()
    int fCurEndT;
    std::vector<Channel> fChannels;
};

#endif
