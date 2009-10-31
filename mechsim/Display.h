#ifndef __DISPLAY_H__
#define __DISPLAY_H__

#include "Simulation.h"
#include <string>

namespace Display {

void DrawText(const std::string& text);
void DisplayCallback();
void DrawStatusText(const SimulationState& state);
void ReshapeCallback(int w, int h);
void IdleCallback();
void InitGL();
void Init();
void Run();

} // end namespace Display

#endif
