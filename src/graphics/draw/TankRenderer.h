#pragma once

#include "configuration.h"

#if defined(MOONHUT_TANK) && HAS_SCREEN

#include <OLEDDisplay.h>
#include <OLEDDisplayUi.h>

namespace graphics
{
namespace TankRenderer
{
/// Distance, sample spread and session min/max - the numbers the bench range test needs.
void drawTankFrame(OLEDDisplay *display, OLEDDisplayUiState *state, int16_t x, int16_t y);
} // namespace TankRenderer
} // namespace graphics

#endif
