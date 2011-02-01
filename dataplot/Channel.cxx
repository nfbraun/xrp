#include "Channel.h"

const double Channel::CH_GAIN_CHOICES[] = 
    { 0.001, 0.002, 0.005, 0.01, 0.02, 0.05, 0.1, 0.2, 0.5,
      1., 2., 5., 10., 20., 50., 100., 200., 500., 1000. };
const int Channel::CH_GAIN_DEFAULT = 9; // index!
const unsigned int Channel::N_CH_GAIN_CHOICES =
    (sizeof(Channel::CH_GAIN_CHOICES)/sizeof(Channel::CH_GAIN_CHOICES[0]));
