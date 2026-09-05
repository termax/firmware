#include "TankRenderer.h"

#if defined(MOONHUT_TANK) && HAS_SCREEN

#include "graphics/ScreenFonts.h"
#include "graphics/SharedUIDisplay.h"
#include "modules/MoonTankModule.h"

namespace graphics
{
namespace TankRenderer
{

void drawLevelFrame(OLEDDisplay *display, OLEDDisplayUiState *state, int16_t x, int16_t y)
{
    display->setTextAlignment(TEXT_ALIGN_LEFT);
    display->setFont(FONT_SMALL);
    graphics::drawCommonHeader(display, x, y, "LEVEL");

    const int16_t width = display->getWidth();
    const int16_t midX = x + width / 2;
    int16_t cursorY = y + FONT_HEIGHT_SMALL + 2;

    display->setTextAlignment(TEXT_ALIGN_CENTER);

    if (!moonTankModule) {
        display->setFont(FONT_MEDIUM);
        display->drawString(midX, cursorY + 8, "no sensor");
        display->setTextAlignment(TEXT_ALIGN_LEFT);
        return;
    }

    if (!moonTankModule->isCalibrated()) {
        // Deliberately NOT a percentage of a guessed height. An invented number on a
        // panel is worse than an honest gap: somebody will believe it.
        display->setFont(FONT_MEDIUM);
        display->drawString(midX, cursorY, "UNCALIBRATED");
        cursorY += FONT_HEIGHT_MEDIUM + 2;
        display->setFont(FONT_SMALL);
        display->drawString(midX, cursorY, "set tank:height=<m>");
        cursorY += FONT_HEIGHT_SMALL;
        const float m = moonTankModule->distanceM();
        char line[40];
        if (isnan(m))
            snprintf(line, sizeof(line), "no echo");
        else
            snprintf(line, sizeof(line), "distance %.3f m", (double)m);
        display->drawString(midX, cursorY, line);
        display->setTextAlignment(TEXT_ALIGN_LEFT);
        return;
    }

    const float pct = moonTankModule->levelPct();
    const float lvl = moonTankModule->levelM();

    display->setFont(FONT_LARGE);
    char big[16];
    if (isnan(pct))
        snprintf(big, sizeof(big), "-- %%");
    else
        snprintf(big, sizeof(big), "%.0f %%", (double)pct);
    display->drawString(midX, cursorY, big);
    cursorY += FONT_HEIGHT_LARGE + 2;

    display->setFont(FONT_SMALL);
    char line[48];
    if (isnan(lvl))
        snprintf(line, sizeof(line), "no echo returned");
    else
        snprintf(line, sizeof(line), "%.2f m of %.2f m", (double)lvl,
                 (double)(moonTankModule->tankHeight() - moonTankModule->deadTopM()));
    display->drawString(midX, cursorY, line);
    cursorY += FONT_HEIGHT_SMALL;

    // The rate, when the fit has a long enough window. + is filling, - is draining.
    const float r = moonTankModule->levelRateMphPublic();
    if (!isnan(r)) {
        snprintf(line, sizeof(line), "%+.2f m/h", (double)r);
        display->drawString(midX, cursorY, line);
    }

    display->setTextAlignment(TEXT_ALIGN_LEFT);
}

void drawTankFrame(OLEDDisplay *display, OLEDDisplayUiState *state, int16_t x, int16_t y)
{
    display->setTextAlignment(TEXT_ALIGN_LEFT);
    display->setFont(FONT_SMALL);
    graphics::drawCommonHeader(display, x, y, "TANK");

    const int16_t width = display->getWidth();
    const int16_t midX = x + width / 2;
    int16_t cursorY = y + FONT_HEIGHT_SMALL + 2;

    if (!moonTankModule) {
        display->setFont(FONT_MEDIUM);
        display->setTextAlignment(TEXT_ALIGN_CENTER);
        display->drawString(midX, cursorY + 8, "no sensor");
        display->setTextAlignment(TEXT_ALIGN_LEFT);
        return;
    }

    const float m = moonTankModule->distanceM();

    display->setTextAlignment(TEXT_ALIGN_CENTER);
    display->setFont(FONT_LARGE);
    if (isnan(m)) {
        display->drawString(midX, cursorY, "-- . -- m");
    } else {
        char big[16];
        snprintf(big, sizeof(big), "%.3f m", m);
        display->drawString(midX, cursorY, big);
    }
    cursorY += FONT_HEIGHT_LARGE + 2;

    // Spread is the trust indicator: a tight cluster is a real surface, a wide one
    // means the sensor picked a different target on every ping.
    display->setFont(FONT_SMALL);
    char line[48];
    if (isnan(m)) {
        snprintf(line, sizeof(line), "no echo returned");
    } else {
        snprintf(line, sizeof(line), "spread %.0f mm   %u echoes", moonTankModule->spreadM() * 1000.0f,
                 moonTankModule->validSamples());
    }
    display->drawString(midX, cursorY, line);
    cursorY += FONT_HEIGHT_SMALL;

    if (!isnan(moonTankModule->minM())) {
        snprintf(line, sizeof(line), "session %.3f - %.3f m", moonTankModule->minM(), moonTankModule->maxM());
        display->drawString(midX, cursorY, line);
        cursorY += FONT_HEIGHT_SMALL;
    }
    // Uptime on the panel for the same reason it is in every mesh report: it separates a
    // node that rebooted from one that stalled, without a serial cable.
    snprintf(line, sizeof(line), "up %lus", (unsigned long)(millis() / 1000));
    display->drawString(midX, cursorY, line);

    display->setTextAlignment(TEXT_ALIGN_LEFT);
}

} // namespace TankRenderer
} // namespace graphics

#endif
