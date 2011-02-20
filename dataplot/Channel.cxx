#include "Channel.h"

const double _Channel::CH_GAIN_CHOICES[] = 
    { 0.001, 0.002, 0.005, 0.01, 0.02, 0.05, 0.1, 0.2, 0.5,
      1., 2., 5., 10., 20., 50., 100., 200., 500., 1000. };
const int _Channel::CH_GAIN_DEFAULT = 9; // index!
const unsigned int _Channel::N_CH_GAIN_CHOICES =
    (sizeof(Channel::CH_GAIN_CHOICES)/sizeof(Channel::CH_GAIN_CHOICES[0]));

Channel::Channel(bool notnull)
{
    fObj = new ChannelObj(notnull);
    fObj->fRefcount++;
}

Channel::Channel(const Channel& src)
{
    fObj = src.fObj;
    fObj->fRefcount++;
}

Channel::Channel(const Data& data, const std::string& name)
{
    fObj = new ChannelObj(data, name);
    fObj->fRefcount++;
}

Channel& Channel::operator=(const Channel& src)
{
    if(&src == this) return (*this);
    
    fObj->fRefcount--;
    if(fObj->fRefcount == 0)
        delete fObj;
    
    fObj = src.fObj;
    fObj->fRefcount++;
    
    return (*this);
}

Channel::~Channel()
{
    fObj->fRefcount--;
    if(fObj->fRefcount==0)
        delete fObj;
}

