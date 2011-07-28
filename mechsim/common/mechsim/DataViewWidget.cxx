#include "DataViewWidget.h"
#include <iostream>

DataViewWidget::DataViewWidget(Simulation* sim, QWidget* parent)
    : VCWidget(parent), fSimulation(sim), fT(0), fCurEndT(-1)
{
    int nch = sim->GetNDataCh();
    
    fChannelDialog = new ChannelDialog(this, parent);
    
    for(int ch=0; ch<nch; ch++) {
        Channel channel(true);
        channel.setName(sim->GetChName(ch));
        fChannels.push_back(channel);
        fChannelDialog->addChannel(channel);
    }
    
    //addNewData();
    
    setBorders(40, 5, 5, 30);
    setShowLegend(true);
    setXVisibleRegion(10.);
    setYVisibleRegion(15.);
    setMarkerPos(0.5);
    setMarkerUserPos(0.0);
    chParamsChanged();
    
    setXDoZoomAroundMarker(true);
    
    connect(this, SIGNAL(xPosChanged(double)), this, SLOT(setTime_d(double)));
}

void DataViewWidget::addNewData()
{
    int t, tend = fSimulation->GetDefaultEndTime();
    double tstep = fSimulation->GetTimestep();
    int nch = fSimulation->GetNDataCh();
    
    for(t=fCurEndT+1; t<=tend; t++) {
        const SimulationState* state = fSimulation->GetState(t);
        if(state == 0) break;
        for(int ch=0; ch<nch; ch++) {
            fChannels[ch].data().data()[(double)t * tstep] =
                state->GetData(ch);
        }
    }
    VCWidget::addNewData(((double)fCurEndT + 0.5)*tstep,
                         ((double)t - 0.5)*tstep);
    fCurEndT = t-1;
}

void DataViewWidget::showChannelDialog()
{
    fChannelDialog->show();
    fChannelDialog->raise();
    fChannelDialog->activateWindow();
}

void DataViewWidget::setTime(int t)
{
    if(t == fT) return;
    fT = t;
    setMarkerUserPos_Lazy(t * fSimulation->GetTimestep());
    emit timeChanged(t);
}

void DataViewWidget::setTime_d(double t_d)
{
    fT = (int) ceil(t_d / fSimulation->GetTimestep() - .5);
    emit timeChanged(fT);
}
