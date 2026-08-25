#include "FridgeRenderer.h"

#if defined(MOONHUT_FRIDGE) && HAS_SCREEN

#include "graphics/ScreenFonts.h"
#include "graphics/SharedUIDisplay.h"
#include "modules/MoonFridgeModule.h"

namespace graphics
{
namespace FridgeRenderer
{

// Trim a label until it fits, so a long name can never bleed into its neighbour on a
// crowded overview.
static String fitWidth(OLEDDisplay *display, const String &text, int16_t maxW)
{
    String out = text;
    while (out.length() > 1 && display->getStringWidth(out) > maxW)
        out.remove(out.length() - 1);
    return out;
}

// A probe's reading, or the reason there isn't one. "--.-" is deliberately the same
// width as a temperature so the layout does not jump when a probe drops out.
static String readingOf(uint8_t i)
{
    float t = 0.0f;
    if (moonFridgeModule->getTempC(i, t))
        return String(t, 1) + "°C";
    return moonFridgeModule->probePresent(i) ? String("--.-°C") : String("gone");
}

static void drawNoProbe(OLEDDisplay *display, int16_t x, int16_t y, int16_t cursorY, int16_t width)
{
    display->setFont(FONT_MEDIUM);
    display->setTextAlignment(TEXT_ALIGN_CENTER);
    display->drawString(x + width / 2, cursorY + 8, "no probe");
    display->setFont(FONT_SMALL);
    display->drawString(x + width / 2, cursorY + 8 + FONT_HEIGHT_MEDIUM, "check 4.7k + data wire");
    display->setTextAlignment(TEXT_ALIGN_LEFT);
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
        drawNoProbe(display, x, y, cursorY, width);
        return;
    }

    // Every probe is drawn the same size. There is deliberately no "primary" probe:
    // which one matters is the operator's call, not the bus enumeration order.
    const uint8_t n = moonFridgeModule->probeCount();
    display->setTextAlignment(TEXT_ALIGN_CENTER);

    if (n <= 4) {
        // Columns. 296 px will not hold a fifth.
        const int16_t colW = width / n;
        const uint8_t *bigFont = (n <= 2) ? FONT_LARGE : FONT_MEDIUM;
        const int16_t bigH = (n <= 2) ? FONT_HEIGHT_LARGE : FONT_HEIGHT_MEDIUM;
        const int16_t nameY = cursorY + 2;
        const int16_t tempY = nameY + FONT_HEIGHT_SMALL;

        for (uint8_t i = 0; i < n; i++) {
            const int16_t midX = x + (colW * i) + (colW / 2);

            display->setFont(FONT_SMALL);
            String label = String(moonFridgeModule->probeName(i));
            if (moonFridgeModule->probeAlarming(i))
                label = "!" + label;
            display->drawString(midX, nameY, fitWidth(display, label, colW - 4));

            display->setFont(bigFont);
            display->drawString(midX, tempY, fitWidth(display, readingOf(i), colW - 2));

            if (i + 1 < n)
                display->drawVerticalLine(x + colW * (i + 1), nameY, FONT_HEIGHT_SMALL + bigH);
        }
        cursorY = tempY + bigH + 2;
    } else {
        // Past four probes a column each is unreadable, so switch to a two-per-row
        // list: name left, temperature right, which stays legible down to ten.
        display->setTextAlignment(TEXT_ALIGN_LEFT);
        display->setFont(FONT_SMALL);
        const int16_t colW = width / 2;
        const int16_t rowH = FONT_HEIGHT_SMALL + 1;

        for (uint8_t i = 0; i < n; i++) {
            const int16_t col = i % 2;
            const int16_t row = i / 2;
            const int16_t left = x + col * colW;
            const int16_t rowY = cursorY + row * rowH;

            String label = String(moonFridgeModule->probeName(i));
            if (moonFridgeModule->probeAlarming(i))
                label = "!" + label;
            display->drawString(left + 2, rowY, fitWidth(display, label, colW - 52));

            display->setTextAlignment(TEXT_ALIGN_RIGHT);
            display->drawString(left + colW - 4, rowY, readingOf(i));
            display->setTextAlignment(TEXT_ALIGN_LEFT);
        }
        cursorY += ((n + 1) / 2) * rowH;
        display->setTextAlignment(TEXT_ALIGN_CENTER);
    }

    display->setFont(FONT_SMALL);
    if (alarming)
        display->drawString(x + width / 2, cursorY, "*** OUT OF RANGE ***");
    else if (moonFridgeModule->probeFault())
        display->drawString(x + width / 2, cursorY, "a probe is not answering");

    display->setTextAlignment(TEXT_ALIGN_LEFT);
}

void drawProbeFrame(OLEDDisplay *display, OLEDDisplayUiState *state, int16_t x, int16_t y, uint8_t probeIdx)
{
    display->setTextAlignment(TEXT_ALIGN_LEFT);
    display->setFont(FONT_SMALL);

    const int16_t width = display->getWidth();
    const int16_t midX = x + width / 2;

    if (!moonFridgeModule || probeIdx >= moonFridgeModule->probeCount()) {
        graphics::drawCommonHeader(display, x, y, "PROBE");
        drawNoProbe(display, x, y, y + FONT_HEIGHT_SMALL, width);
        return;
    }

    const bool alarming = moonFridgeModule->probeAlarming(probeIdx);

    // The header carries the probe's own name, so a detail frame photographed or
    // glanced at is self-identifying - the whole point of naming them.
    char title[24];
    snprintf(title, sizeof(title), "%s%s", moonFridgeModule->probeName(probeIdx), alarming ? " !" : "");
    graphics::drawCommonHeader(display, x, y, title);

    int16_t cursorY = y + FONT_HEIGHT_SMALL + 2;

    display->setTextAlignment(TEXT_ALIGN_CENTER);
    display->setFont(FONT_LARGE);
    display->drawString(midX, cursorY, readingOf(probeIdx));
    cursorY += FONT_HEIGHT_LARGE + 2;

    display->setFont(FONT_SMALL);
    float hiC = 0.0f, loC = 0.0f;
    if (moonFridgeModule->probeBand(probeIdx, hiC, loC)) {
        char band[40];
        // A low limit left at its "off" default is noise on the panel - only show a
        // limit that someone has actually set.
        if (loC <= -39.0f)
            snprintf(band, sizeof(band), "alarm above %.1f°C", (double)hiC);
        else
            snprintf(band, sizeof(band), "band %.1f to %.1f°C", (double)loC, (double)hiC);
        display->drawString(midX, cursorY, band);
        cursorY += FONT_HEIGHT_SMALL;
    }

    if (!moonFridgeModule->probePresent(probeIdx))
        display->drawString(midX, cursorY, "not on the bus");
    else if (alarming)
        display->drawString(midX, cursorY, "*** OUT OF RANGE ***");

    display->setTextAlignment(TEXT_ALIGN_LEFT);
}

// The UI library takes a bare function pointer per frame with no user-data argument,
// so a per-probe frame needs a distinct function per probe. A template thunk gives us
// exactly MOONHUT_FRIDGE_MAX_PROBES of them with no duplication.
template <uint8_t N> static void probeThunk(OLEDDisplay *d, OLEDDisplayUiState *s, int16_t x, int16_t y)
{
    drawProbeFrame(d, s, x, y, N);
}

static const ProbeFrameCallback probeFrames[] = {
    probeThunk<0>,  probeThunk<1>,  probeThunk<2>,  probeThunk<3>, probeThunk<4>,  probeThunk<5>,
    probeThunk<6>,  probeThunk<7>,  probeThunk<8>,  probeThunk<9>, probeThunk<10>, probeThunk<11>,
    probeThunk<12>, probeThunk<13>, probeThunk<14>, probeThunk<15>,
};

ProbeFrameCallback probeFrameFor(uint8_t idx)
{
    if (idx >= (sizeof(probeFrames) / sizeof(probeFrames[0])) || idx >= MOONHUT_FRIDGE_MAX_PROBES)
        return nullptr;
    return probeFrames[idx];
}

} // namespace FridgeRenderer
} // namespace graphics

#endif
