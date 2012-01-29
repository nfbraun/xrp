#include "Playback.h"
#include "GLHelper.h"
#include <cmath>
#include <iostream>
#include <stdexcept>

const int Playback::STEP_PER_SEC = 16;
const char Playback::TITLE[] = "Acrobot (playback)";

void AcroState::Draw(int) const
{
    const double L1 = .5;
    const double L2 = .6;
    const double R_T = 0.03;
    const double R_S = 0.05;
    
    Eigen::Vector3d B1_pos = L1*Eigen::Vector3d(sin(fPhi1), 0., cos(fPhi1));
    Eigen::Vector3d B2_pos;
    B2_pos = B1_pos + L2*Eigen::Vector3d(sin(fPhi1+fPhi2), 0., cos(fPhi1+fPhi2));

    GL::drawSphere(R_S, B1_pos);
    GL::drawSphere(R_S, B2_pos);
    
    GL::drawTube(R_T, Eigen::Vector3d::Zero(), B1_pos);
    GL::drawTube(R_T, B1_pos, B2_pos);
}

Playback::Playback()
    : fCurStep(0), fCurTIdx(0),
      fDataFile("acrobot.dat")
{
    if(fDataFile.fail())
        throw std::runtime_error("Failed to open data: acrobot.dat");
}

double Playback::GetInterpValue(int value_id, double t, int tidx)
{
    if(tidx < 1) return fDataFile.data().at(0).at(value_id);
    if(tidx >= fDataFile.data().size())
        return fDataFile.data().at(fDataFile.data().size()-1).at(value_id);
    
    double t0 = fDataFile.data().at(tidx-1).at(0);
    double t1 = fDataFile.data().at(tidx).at(0);
    double x0 = fDataFile.data().at(tidx-1).at(value_id);
    double x1 = fDataFile.data().at(tidx).at(value_id);
    
    return x0 + (x1-x0)/(t1-t0) * (t - t0);
}

void Playback::Advance()
{
    fCurStep++;
    
    double t = (double) fCurStep / STEP_PER_SEC;
    while((fCurTIdx < fDataFile.data().size())
        && fDataFile.data().at(fCurTIdx).at(0) < t) fCurTIdx++;
}

AcroState Playback::GetCurrentState()
{
    AcroState state;
    
    state.fT = (double) fCurStep / STEP_PER_SEC;
    
    state.fPhi1   = GetInterpValue(1, state.fT, fCurTIdx);
    state.fPhi2   = GetInterpValue(2, state.fT, fCurTIdx);
    state.fOmega1 = GetInterpValue(3, state.fT, fCurTIdx);
    state.fOmega2 = GetInterpValue(4, state.fT, fCurTIdx);
    state.fU      = GetInterpValue(5, state.fT, fCurTIdx);
    
    return state;
}
