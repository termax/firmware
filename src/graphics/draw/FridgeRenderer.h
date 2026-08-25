#pragma once

#include "configuration.h"

#if defined(MOONHUT_FRIDGE) && HAS_SCREEN

#include <OLEDDisplay.h>
#include <OLEDDisplayUi.h>

namespace graphics
{
namespace FridgeRenderer
{
/// Every probe at once: the frame you read from across the room.
void drawFridgeFrame(OLEDDisplay *display, OLEDDisplayUiState *state, int16_t x, int16_t y);

/// One probe, large, with its alarm band. Frame N+1 is probe N.
void drawProbeFrame(OLEDDisplay *display, OLEDDisplayUiState *state, int16_t x, int16_t y, uint8_t probeIdx);

/// A plain frame callback bound to probe `idx`. The UI library takes a bare function
/// pointer with no user data, so each probe needs its own thunk; this hands out the
/// pre-built one. Returns nullptr past MOONHUT_FRIDGE_MAX_PROBES.
typedef void (*ProbeFrameCallback)(OLEDDisplay *, OLEDDisplayUiState *, int16_t, int16_t);
ProbeFrameCallback probeFrameFor(uint8_t idx);
} // namespace FridgeRenderer
} // namespace graphics

#endif
