#include "FridgeRenderer.h"

#if defined(MOONHUT_FRIDGE) && HAS_SCREEN

#include "graphics/ScreenFonts.h"
#include "graphics/SharedUIDisplay.h"
#include "modules/MoonFridgeModule.h"

namespace graphics
{
namespace FridgeRenderer
{

// Trim a label until it fits the column, so a long name can never bleed into its
// neighbour on a 4-probe panel where each column is only ~74 px wide.
static String fitWidth(OLEDDisplay *display, const String &text, int16_t maxW)
{
    String out = text;
    while (out.length() > 1 && display->getStringWidth(out) > maxW)
        out.remove(out.length() - 1);
    return out;
}

void drawFridgeFrame(OLEDDisplay *display, OLEDDisplayUiState *state, int16_t x, int16_t y)
{
    display->setTextAlignment(TEXT_ALIGN_LEFT);
    display->setFont(FONT_SMALL);

    const bool alarming = moonFridgeModule && moonFridgeModule->alarmActive();
    graphics::drawCommonHeader(display, x, y, alarming ? "FRIDGE !" : "FRIDGE");

    const int16_t headerH = FONT_HEIGHT_SMALL;
    const int16_t width = display->getWidth();
    int16_t cursorY = y + headerH;

    if (!moonFridgeModule || moonFridgeModule->probeCount() == 0) {
        display->setFont(FONT_MEDIUM);
        display->setTextAlignment(TEXT_ALIGN_CENTER);
        display->drawString(x + width / 2, cursorY + 8, "no probe");
        display->setFont(FONT_SMALL);
        display->drawString(x + width / 2, cursorY + 8 + FONT_HEIGHT_MEDIUM, "check 4.7k + data wire");
        display->setTextAlignment(TEXT_ALIGN_LEFT);
        return;
    }

    // Every probe is drawn the same size, in its own column. There is deliberately no
    // "primary" probe any more: which one matters is the operator's call, not the bus
    // enumeration order, and the fridge and the freezer are equally worth reading.
    const uint8_t n = moonFridgeModule->probeCount();
    const int16_t colW = width / n;
    const uint8_t *bigFont = (n <= 2) ? FONT_LARGE : FONT_MEDIUM;
    const int16_t bigH = (n <= 2) ? FONT_HEIGHT_LARGE : FONT_HEIGHT_MEDIUM;

    display->setTextAlignment(TEXT_ALIGN_CENTER);
    const int16_t nameY = cursorY + 2;
    const int16_t tempY = nameY + FONT_HEIGHT_SMALL;

    for (uint8_t i = 0; i < n; i++) {
        const int16_t midX = x + (colW * i) + (colW / 2);

        display->setFont(FONT_SMALL);
        display->drawString(midX, nameY, fitWidth(display, String(moonFridgeModule->probeName(i)), colW - 4));

        float t = 0.0f;
        String value = moonFridgeModule->getTempC(i, t) ? String(t, 1) + "°C" : String("--.-°C");
        display->setFont(bigFont);
        display->drawString(midX, tempY, fitWidth(display, value, colW - 2));

        // Hairline between columns, not after the last one.
        if (i + 1 < n)
            display->drawVerticalLine(x + colW * (i + 1), nameY, FONT_HEIGHT_SMALL + bigH);
    }

    cursorY = tempY + bigH + 2;

    display->setFont(FONT_SMALL);
    if (alarming)
        display->drawString(x + width / 2, cursorY, "*** TOO WARM ***");
    else if (moonFridgeModule->probeFault())
        display->drawString(x + width / 2, cursorY, "probe not answering");

    display->setTextAlignment(TEXT_ALIGN_LEFT);
}

} // namespace FridgeRenderer
} // namespace graphics

#endif
