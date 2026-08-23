#include "FridgeRenderer.h"

#if defined(MOONHUT_FRIDGE) && HAS_SCREEN

#include "graphics/ScreenFonts.h"
#include "graphics/SharedUIDisplay.h"
#include "modules/MoonFridgeModule.h"

namespace graphics
{
namespace FridgeRenderer
{

void drawFridgeFrame(OLEDDisplay *display, OLEDDisplayUiState *state, int16_t x, int16_t y)
{
    display->setTextAlignment(TEXT_ALIGN_LEFT);
    display->setFont(FONT_SMALL);

    const bool alarming = moonFridgeModule && moonFridgeModule->alarmActive();
    graphics::drawCommonHeader(display, x, y, alarming ? "FRIDGE !" : "FRIDGE");

    const int16_t headerH = FONT_HEIGHT_SMALL;
    int16_t cursorY = y + headerH;

    if (!moonFridgeModule || moonFridgeModule->probeCount() == 0) {
        display->setFont(FONT_MEDIUM);
        display->setTextAlignment(TEXT_ALIGN_CENTER);
        display->drawString(x + display->getWidth() / 2, cursorY + 8, "no probe");
        display->setFont(FONT_SMALL);
        display->drawString(x + display->getWidth() / 2, cursorY + 8 + FONT_HEIGHT_MEDIUM, "check 4.7k + data wire");
        display->setTextAlignment(TEXT_ALIGN_LEFT);
        return;
    }

    // --- Primary probe: big, centred, the thing you read from across the room ---
    float c = 0.0f;
    display->setTextAlignment(TEXT_ALIGN_CENTER);
    const int16_t midX = x + display->getWidth() / 2;

    if (moonFridgeModule->getTempC(0, c)) {
        char big[16];
        snprintf(big, sizeof(big), "%.1f°C", c);
        display->setFont(FONT_LARGE);
        display->drawString(midX, cursorY + 2, big);
    } else {
        display->setFont(FONT_LARGE);
        display->drawString(midX, cursorY + 2, "--.-°C");
    }
    cursorY += FONT_HEIGHT_LARGE + 4;

    // --- Any further probes, one compact line each ---
    display->setFont(FONT_SMALL);
    const uint8_t n = moonFridgeModule->probeCount();
    if (n > 1) {
        String line;
        for (uint8_t i = 1; i < n; i++) {
            float t = 0.0f;
            if (i > 1)
                line += "  ";
            line += MoonFridgeModule::probeLabel(i);
            line += ": ";
            line += moonFridgeModule->getTempC(i, t) ? String(t, 1) + "°C" : String("--");
        }
        display->drawString(midX, cursorY, line);
        cursorY += FONT_HEIGHT_SMALL;
    }

    if (alarming)
        display->drawString(midX, cursorY, "*** TOO WARM ***");
    else if (moonFridgeModule->probeFault())
        display->drawString(midX, cursorY, "probe not answering");

    display->setTextAlignment(TEXT_ALIGN_LEFT);
}

} // namespace FridgeRenderer
} // namespace graphics

#endif
