#ifndef MSIM_DATAVIEWWIDGET_H
#define MSIM_DATAVIEWWIDGET_H

#include <vector>
#include "ChannelDialog.h"
#include "dataplot/VCWidget.h"
#include "dataplot/Channel.h"
#include "Simulation.h"

class DataViewWidget : public VCWidget
{
  Q_OBJECT
  
  public:
    DataViewWidget(Simulation* sim, QWidget* parent = 0);
  
  public slots:
    void addNewData();
    void showChannelDialog();
    void setTime(int t);
    void setTime_d(double t_d);
    
  signals:
    void timeChanged(int t);
  
  private:
    Simulation* fSimulation;
    ChannelDialog *fChannelDialog;
    int fT;  // in units of Simulation::TIMESTEPs
    int fCurEndT;
    std::vector<Channel> fChannels;
};

#endif
