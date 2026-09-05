#pragma once

#include "configuration.h"

#if defined(MOONHUT_TANK) && HAS_SCREEN

#include <OLEDDisplay.h>
#include <OLEDDisplayUi.h>

namespace graphics
{
namespace TankRenderer
{
/// Frame 0: what a human at the tank wants - fill percentage and water depth. Says
/// UNCALIBRATED rather than inventing a number when no tank height has been set.
void drawLevelFrame(OLEDDisplay *display, OLEDDisplayUiState *state, int16_t x, int16_t y);

/// Frame 1: distance, sample spread, echo count and uptime - the numbers you want when
/// deciding whether to trust the reading, and what the bench range test needs.
void drawTankFrame(OLEDDisplay *display, OLEDDisplayUiState *state, int16_t x, int16_t y);
} // namespace TankRenderer
} // namespace graphics

#endif
