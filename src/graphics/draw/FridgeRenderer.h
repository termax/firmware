#pragma once

#include "configuration.h"

#if defined(MOONHUT_FRIDGE) && HAS_SCREEN

#include <OLEDDisplay.h>
#include <OLEDDisplayUi.h>

namespace graphics
{
namespace FridgeRenderer
{
/// MoonHut: fullscreen probe-temperature frame. Shows the primary probe as a
/// large numeral with any further probes listed underneath, so a fridge reading
/// is legible across a room.
void drawFridgeFrame(OLEDDisplay *display, OLEDDisplayUiState *state, int16_t x, int16_t y);
} // namespace FridgeRenderer
} // namespace graphics

#endif
