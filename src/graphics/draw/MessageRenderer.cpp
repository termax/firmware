#include "configuration.h"
#if HAS_SCREEN
#include "MessageRenderer.h"

// Core includes
#include "MessageStore.h"
#include "NodeDB.h"
#include "UIRenderer.h"
#include "gps/RTC.h"
#include "graphics/EmoteRenderer.h"
#ifdef MOONHUT_SIGN
#include "graphics/qrcodegen.h" // vendored Nayuki QR generator (MIT) for the qr: sign mode
#endif
#include "graphics/Screen.h"
#include "graphics/ScreenFonts.h"
#include "graphics/SharedUIDisplay.h"
#include "graphics/TimeFormatters.h"
#include "graphics/emotes.h"
#ifdef MOONHUT_SIGN
#include "PowerFSM.h"                        // page flips poke the FSM to hold the screen awake while cycling
#include "PowerStatus.h"                     // battery percentage in the sign's bottom row
#include "graphics/EInkDynamicDisplay.h"     // EINK_ADD_FRAMEFLAG: page flips demand an immediate fast refresh
#include "graphics/fonts/EinkDisplayFonts.h" // global scope: Monospaced_plain_30 for the MoonHut sign
#endif
#include "main.h"
#include "meshUtils.h"
#include <string>
#include <vector>

// External declarations
extern bool hasUnreadMessage;
extern graphics::Screen *screen;

using graphics::Emote;
using graphics::emotes;
using graphics::numEmotes;

namespace graphics
{
namespace MessageRenderer
{

static std::vector<std::string> cachedLines;
static std::vector<int> cachedHeights;
static bool manualScrolling = false;

// Scroll state (file scope so we can reset on new message)
float scrollY = 0.0f;
uint32_t lastTime = 0;
uint32_t scrollStartDelay = 0;
uint32_t pauseStart = 0;
bool waitingToReset = false;
bool scrollStarted = false;
static bool didReset = false;
static constexpr int MESSAGE_BLOCK_GAP = 6;

void scrollUp()
{
    manualScrolling = true;
    scrollY -= 12;
    if (scrollY < 0)
        scrollY = 0;
}

void scrollDown()
{
    manualScrolling = true;

    int totalHeight = 0;
    for (int h : cachedHeights)
        totalHeight += h;

    int visibleHeight = screen->getHeight() - (FONT_HEIGHT_SMALL * 2);
    int maxScroll = totalHeight - visibleHeight;
    if (maxScroll < 0)
        maxScroll = 0;

    scrollY += 12;
    if (scrollY > maxScroll)
        scrollY = maxScroll;
}

void drawStringWithEmotes(OLEDDisplay *display, int x, int y, const std::string &line, const Emote *emotes, int emoteCount)
{
    graphics::EmoteRenderer::drawStringWithEmotes(display, x, y, line, FONT_HEIGHT_SMALL, emotes, emoteCount);
}

// Reset scroll state when new messages arrive
void resetScrollState()
{
    scrollY = 0.0f;
    scrollStarted = false;
    waitingToReset = false;
    scrollStartDelay = millis();
    lastTime = millis();
    manualScrolling = false;
    didReset = false;
}

void nudgeScroll(int8_t direction)
{
    if (direction == 0)
        return;

    if (cachedHeights.empty()) {
        scrollY = 0.0f;
        return;
    }

    OLEDDisplay *display = (screen != nullptr) ? screen->getDisplayDevice() : nullptr;
    const int displayHeight = display ? display->getHeight() : 64;
    const int navHeight = FONT_HEIGHT_SMALL;
    const int usableHeight = std::max(0, displayHeight - navHeight);

    int totalHeight = 0;
    for (int h : cachedHeights)
        totalHeight += h;

    if (totalHeight <= usableHeight) {
        scrollY = 0.0f;
        return;
    }

    const int scrollStop = std::max(0, totalHeight - usableHeight + cachedHeights.back());
    const int step = std::max(FONT_HEIGHT_SMALL, usableHeight / 3);

    float newScroll = scrollY + static_cast<float>(direction) * static_cast<float>(step);
    if (newScroll < 0.0f)
        newScroll = 0.0f;
    if (newScroll > scrollStop)
        newScroll = static_cast<float>(scrollStop);

    if (newScroll != scrollY) {
        scrollY = newScroll;
        waitingToReset = false;
        scrollStarted = false;
        scrollStartDelay = millis();
        lastTime = millis();
    }
}

// Fully free cached message data from heap
void clearMessageCache()
{
    std::vector<std::string>().swap(cachedLines);
    std::vector<int>().swap(cachedHeights);

    // Reset scroll so we rebuild cleanly next time we enter the screen
    resetScrollState();
}

// Current thread state
static ThreadMode currentMode = ThreadMode::ALL;
static int currentChannel = -1;
static uint32_t currentPeer = 0;

// Registry of seen threads for manual toggle
static std::vector<int> seenChannels;
static std::vector<uint32_t> seenPeers;

// Public helper so menus / store can clear stale registries
void clearThreadRegistries()
{
    seenChannels.clear();
    seenPeers.clear();
}

// Setter so other code can switch threads
void setThreadMode(ThreadMode mode, int channel /* = -1 */, uint32_t peer /* = 0 */)
{
    currentMode = mode;
    currentChannel = channel;
    currentPeer = peer;
    didReset = false; // force reset when mode changes

    // Track channels we’ve seen
    if (mode == ThreadMode::CHANNEL && channel >= 0) {
        if (std::find(seenChannels.begin(), seenChannels.end(), channel) == seenChannels.end()) {
            seenChannels.push_back(channel);
        }
    }

    // Track DMs we’ve seen
    if (mode == ThreadMode::DIRECT && peer != 0) {
        if (std::find(seenPeers.begin(), seenPeers.end(), peer) == seenPeers.end()) {
            seenPeers.push_back(peer);
        }
    }
}

ThreadMode getThreadMode()
{
    return currentMode;
}

int getThreadChannel()
{
    return currentChannel;
}

uint32_t getThreadPeer()
{
    return currentPeer;
}

// Accessors for menuHandler
const std::vector<int> &getSeenChannels()
{
    return seenChannels;
}
const std::vector<uint32_t> &getSeenPeers()
{
    return seenPeers;
}

static int centerYForRow(int y, int size)
{
    int midY = y + (FONT_HEIGHT_SMALL / 2);
    return midY - (size / 2);
}

// Helpers for drawing status marks (thickened strokes)
static void drawCheckMark(OLEDDisplay *display, int x, int y, int size)
{
    int topY = centerYForRow(y, size);
    display->setColor(WHITE);
    display->drawLine(x, topY + size / 2, x + size / 3, topY + size);
    display->drawLine(x, topY + size / 2 + 1, x + size / 3, topY + size + 1);
    display->drawLine(x + size / 3, topY + size, x + size, topY);
    display->drawLine(x + size / 3, topY + size + 1, x + size, topY + 1);
}

static void drawXMark(OLEDDisplay *display, int x, int y, int size = 8)
{
    int topY = centerYForRow(y, size);
    display->setColor(WHITE);
    display->drawLine(x, topY, x + size, topY + size);
    display->drawLine(x, topY + 1, x + size, topY + size + 1);
    display->drawLine(x + size, topY, x, topY + size);
    display->drawLine(x + size, topY + 1, x, topY + size + 1);
}

static void drawRelayMark(OLEDDisplay *display, int x, int y, int size = 8)
{
    int r = size / 2;
    int centerY = centerYForRow(y, size) + r;
    int centerX = x + r;
    display->setColor(WHITE);
    display->drawCircle(centerX, centerY, r);
    display->drawLine(centerX, centerY - 2, centerX, centerY);
    display->setPixel(centerX, centerY + 2);
    display->drawLine(centerX - 1, centerY - 4, centerX + 1, centerY - 4);
}

static inline int getRenderedLineWidth(OLEDDisplay *display, const std::string &line, const Emote *emotes, int emoteCount)
{
    return graphics::EmoteRenderer::analyzeLine(display, line, 0, emotes, emoteCount).width;
}

struct MessageBlock {
    size_t start;
    size_t end;
    bool mine;
};

static int getDrawnLinePixelBottom(int lineTopY, const std::string &line, bool isHeaderLine)
{
    if (isHeaderLine) {
        return lineTopY + (FONT_HEIGHT_SMALL - 1);
    }

    const int tallest = graphics::EmoteRenderer::analyzeLine(nullptr, line, FONT_HEIGHT_SMALL, emotes, numEmotes).tallestHeight;

    const int lineHeight = std::max(FONT_HEIGHT_SMALL, tallest);
    const int iconTop = lineTopY + (lineHeight - tallest) / 2;

    return iconTop + tallest - 1;
}

static std::vector<MessageBlock> buildMessageBlocks(const std::vector<bool> &isHeaderVec, const std::vector<bool> &isMineVec)
{
    std::vector<MessageBlock> blocks;
    if (isHeaderVec.empty())
        return blocks;

    size_t start = 0;
    bool mine = isMineVec[0];

    for (size_t i = 1; i < isHeaderVec.size(); ++i) {
        if (isHeaderVec[i]) {
            MessageBlock b;
            b.start = start;
            b.end = i - 1;
            b.mine = mine;
            blocks.push_back(b);

            start = i;
            mine = isMineVec[i];
        }
    }

    MessageBlock last;
    last.start = start;
    last.end = isHeaderVec.size() - 1;
    last.mine = mine;
    blocks.push_back(last);

    return blocks;
}

static void drawMessageScrollbar(OLEDDisplay *display, int visibleHeight, int totalHeight, int scrollOffset, int startY)
{
    if (totalHeight <= visibleHeight)
        return; // no scrollbar needed

    int scrollbarX = display->getWidth() - 2;
    int scrollbarHeight = visibleHeight;
    int thumbHeight = std::max(6, (scrollbarHeight * visibleHeight) / totalHeight);
    int maxScroll = std::max(1, totalHeight - visibleHeight);
    int thumbY = startY + (scrollbarHeight - thumbHeight) * scrollOffset / maxScroll;

    for (int i = 0; i < thumbHeight; i++) {
        display->setPixel(scrollbarX, thumbY + i);
    }
}

#ifdef MOONHUT_SIGN
// ---- MoonHut sign: latest MoonPaper message, rendered fullscreen ----
// Layout policy (agreed 2026-07-04): user line breaks are never removed. Each user line
// gets its own font — the largest that lets the whole block fit; when the block is too
// tall, the tallest (tie: longest) lines shrink first, so short lines stay big. If the
// block still overflows at the floor font, the message is split into pages and cycled
// MOON_PAGE_CYCLES times, then rests on page 1 with an ellipsis until the next message.
#define MOON_MIN_FONT_IDX 3 // floor font index into MOON_FONTS (3 = ArialMT_Plain_10); set 2 for a 16pt floor
#define MOON_PAGE_MS 8000   // ms per page while cycling
#define MOON_PAGE_CYCLES 3  // full passes through all pages before resting truncated

#ifdef OLED_RU
// Cyrillic-capable ladder: the _RU faces cover Latin + CP-1251 Cyrillic (the UTF-8
// mapping lives in customFontTableLookup). The 30pt mono is Latin-only, so lines
// containing non-ASCII start one step down (see moonLayout).
#define MOON_FONT_L ArialMT_Plain_24_RU
#define MOON_FONT_M ArialMT_Plain_16_RU
#define MOON_FONT_S ArialMT_Plain_10_RU
#else
#define MOON_FONT_L ArialMT_Plain_24
#define MOON_FONT_M ArialMT_Plain_16
#define MOON_FONT_S ArialMT_Plain_10
#endif
static const uint8_t *MOON_FONTS[] = {Monospaced_plain_30, MOON_FONT_L, MOON_FONT_M, MOON_FONT_S};
static char s_moonMsg[200] = "";
static char s_moonAttr[64] = "";
static bool s_moonHas = false;
static bool s_moonDirty = false;
static bool s_moonNewPersistent = false; // fresh MoonPaper message → restart paging cycles
static char s_moonFlashMsg[200] = "";    // transient overlay (DMs / other channels), never replaces s_moonMsg
static char s_moonFlashAttr[64] = "";
static uint32_t s_moonFlashUntil = 0;    // millis deadline; expired = show persistent message again
static bool s_moonLayoutIsFlash = false; // which text the row cache currently holds
static std::vector<std::string> s_moonRows;      // final wrapped rows, in draw order
static std::vector<uint8_t> s_moonRowFontIdx;    // font index per row
static std::vector<int> s_moonRowH;              // pixel height per row (font or emote, whichever is taller)
static std::vector<int> s_moonPageFirstRow;      // first row index of each page
static uint32_t s_moonPagingStart = 0;
static int s_moonLastPage = -1;
static int s_moonManualPage = -1; // >=0: user flips pages with the PRG button; auto-cycling stops

static inline bool moonFlashActive()
{
    return s_moonFlashMsg[0] && (int32_t)(millis() - s_moonFlashUntil) < 0;
}

bool moonSignNextPage()
{
    if (moonFlashActive()) { // PRG dismisses a flash early
        s_moonFlashUntil = 0;
        return true;
    }
    const int pages = (int)s_moonPageFirstRow.size();
    if (!s_moonHas || pages <= 1)
        return false;
    const int cur = (s_moonLastPage >= 0) ? s_moonLastPage : 0;
    s_moonManualPage = (cur + 1) % pages;
    return true;
}

void setMoonSignMessage(const char *msg, const char *attribution)
{
    if (!msg)
        return;
    strncpy(s_moonMsg, msg, sizeof(s_moonMsg) - 1);
    s_moonMsg[sizeof(s_moonMsg) - 1] = '\0';
    strncpy(s_moonAttr, attribution ? attribution : "", sizeof(s_moonAttr) - 1);
    s_moonAttr[sizeof(s_moonAttr) - 1] = '\0';
    s_moonHas = true;
    s_moonDirty = true; // layout is recomputed on next render (needs the display for text metrics)
    s_moonNewPersistent = true;
    s_moonFlashUntil = 0; // a new sign message outranks any flash in progress
}

void setMoonFlashMessage(const char *msg, const char *attribution, uint32_t durationMs)
{
    if (!msg || !msg[0])
        return;
    strncpy(s_moonFlashMsg, msg, sizeof(s_moonFlashMsg) - 1);
    s_moonFlashMsg[sizeof(s_moonFlashMsg) - 1] = '\0';
    strncpy(s_moonFlashAttr, attribution ? attribution : "", sizeof(s_moonFlashAttr) - 1);
    s_moonFlashAttr[sizeof(s_moonFlashAttr) - 1] = '\0';
    s_moonFlashUntil = millis() + durationMs;
    s_moonDirty = true;
}

// Tiny crescent-moon accent. Ink is drawn in WHITE (renders dark on e-ink); BLACK carves the crescent.
static void drawMoonAccent(OLEDDisplay *display, int cx, int cy, int r)
{
    display->setColor(WHITE);
    display->fillCircle(cx, cy, r);
    display->setColor(BLACK);
    display->fillCircle(cx + (r * 3) / 5, cy - (r * 2) / 5, r);
    display->setColor(WHITE);
}

static inline int moonFontH(uint8_t fontIdx)
{
    return pgm_read_byte(MOON_FONTS[fontIdx] + 1) + 1;
}

// Wrap one user line at the given font into rows: break on spaces; a single word wider
// than the line is force-broken at UTF-8 character boundaries. Empty lines produce one
// empty row (preserving the user's visual break).
static void moonWrapLine(OLEDDisplay *display, const std::string &text, int maxWidth, uint8_t fontIdx,
                         std::vector<std::string> &rowsOut)
{
    display->setFont(MOON_FONTS[fontIdx]);
    if (text.empty()) {
        rowsOut.push_back("");
        return;
    }
    // Width measurement is emote-aware: emoji tokens count as their bitmap width.
    auto lineW = [&](const std::string &s) { return EmoteRenderer::measureStringWithEmotes(display, s); };
    std::string cur;
    size_t i = 0;
    while (i < text.size()) {
        std::string word;
        while (i < text.size() && text[i] != ' ')
            word += text[i++];
        if (i < text.size())
            i++; // consume the space
        std::string cand = cur.empty() ? word : cur + " " + word;
        if (lineW(cand) <= maxWidth) {
            cur = cand;
            continue;
        }
        if (!cur.empty()) {
            rowsOut.push_back(cur);
            cur.clear();
        }
        // Word alone is too wide: split at UTF-8 boundaries
        while (lineW(word) > maxWidth) {
            size_t cut = word.size();
            while (cut > 1) {
                cut--;
                while (cut > 1 && (word[cut] & 0xC0) == 0x80)
                    cut--; // don't split inside a UTF-8 sequence
                if (lineW(word.substr(0, cut)) <= maxWidth)
                    break;
            }
            rowsOut.push_back(word.substr(0, cut));
            word = word.substr(cut);
        }
        cur = word;
    }
    if (!cur.empty())
        rowsOut.push_back(cur);
}

// True if the string contains non-ASCII text OUTSIDE of emote tokens (e.g. Cyrillic) —
// such lines can't use the Latin-only 30pt font. Emoji render as bitmaps at any font.
static bool moonNeedsLocalizedFont(const std::string &s)
{
    size_t i = 0;
    while (i < s.size()) {
        size_t matchLen = 0;
        if (EmoteRenderer::findEmoteAt(s, i, matchLen) && matchLen > 0) {
            i += matchLen;
            continue;
        }
        if ((unsigned char)s[i] >= 0x80)
            return true;
        i++;
    }
    return false;
}

// Compute the full layout for the current message: per-user-line fonts, wrapped rows, pages.
static void moonLayout(OLEDDisplay *display, int maxWidth, int areaH, const char *sourceText)
{
    // Split into user lines, preserving every \n
    std::vector<std::string> lines;
    {
        std::string cur;
        for (const char *p = sourceText; *p; p++) {
            if (*p == '\n') {
                lines.push_back(cur);
                cur.clear();
            } else {
                cur += *p;
            }
        }
        lines.push_back(cur);
    }
    const int n = (int)lines.size();

    // Start every non-empty line at the largest font; empty lines cost the floor font's height
    std::vector<uint8_t> fidx(n);
    std::vector<int> hgt(n);
    std::vector<std::string> scratch;
    auto lineHeight = [&](int i) {
        if (lines[i].empty())
            return moonFontH(MOON_MIN_FONT_IDX);
        scratch.clear();
        moonWrapLine(display, lines[i], maxWidth, fidx[i], scratch);
        return (int)scratch.size() * moonFontH(fidx[i]);
    };
    int total = 0;
    for (int i = 0; i < n; i++) {
        if (lines[i].empty()) {
            fidx[i] = MOON_MIN_FONT_IDX;
        } else {
#ifdef OLED_RU
            fidx[i] = moonNeedsLocalizedFont(lines[i]) ? 1 : 0; // 30pt mono has no Cyrillic glyphs
#else
            fidx[i] = 0;
#endif
        }
        hgt[i] = lineHeight(i);
        total += hgt[i];
    }

    // Shrink the tallest (tie: longest) shrinkable line until the block fits or all hit the floor
    while (total > areaH) {
        int best = -1;
        for (int i = 0; i < n; i++) {
            if (lines[i].empty() || fidx[i] >= MOON_MIN_FONT_IDX)
                continue;
            if (best < 0 || hgt[i] > hgt[best] || (hgt[i] == hgt[best] && lines[i].size() > lines[best].size()))
                best = i;
        }
        if (best < 0)
            break; // everything at the floor — paging will handle the overflow
        fidx[best]++;
        total -= hgt[best];
        hgt[best] = lineHeight(best);
        total += hgt[best];
    }

    // Build the final row list; row height accounts for emote bitmaps taller than the font
    s_moonRows.clear();
    s_moonRowFontIdx.clear();
    s_moonRowH.clear();
    for (int i = 0; i < n; i++) {
        moonWrapLine(display, lines[i], maxWidth, fidx[i], s_moonRows);
        while (s_moonRowFontIdx.size() < s_moonRows.size()) {
            size_t r = s_moonRowFontIdx.size();
            s_moonRowFontIdx.push_back(fidx[i]);
            display->setFont(MOON_FONTS[fidx[i]]);
            int fh = moonFontH(fidx[i]);
            EmoteRenderer::LineMetrics m = EmoteRenderer::analyzeLine(display, s_moonRows[r], fh - 1);
            s_moonRowH.push_back(m.hasEmote ? std::max(fh, m.tallestHeight + 1) : fh);
        }
    }

    // Pack rows into pages by height
    s_moonPageFirstRow.clear();
    int h = 0;
    for (size_t r = 0; r < s_moonRows.size(); r++) {
        int rh = s_moonRowH[r];
        if (s_moonPageFirstRow.empty() || h + rh > areaH) {
            s_moonPageFirstRow.push_back((int)r);
            h = 0;
        }
        h += rh;
    }
    if (s_moonPageFirstRow.empty())
        s_moonPageFirstRow.push_back(0);
}

// Bottom status row shared by the sign and the idle screensaver:
// battery % left, device name centered (attribution right is drawn by the caller).
static void drawMoonBottomRow(OLEDDisplay *display, int W, int H, int tinyH)
{
    if (powerStatus && powerStatus->getHasBattery()) {
        char batt[8];
        snprintf(batt, sizeof(batt), "%u%%", powerStatus->getBatteryChargePercent());
        display->setFont(MOON_FONT_S);
        display->setTextAlignment(TEXT_ALIGN_LEFT);
        display->drawString(2, H - tinyH, batt);
    }
    display->setFont(MOON_FONT_S);
    display->setTextAlignment(TEXT_ALIGN_CENTER);
    display->drawString(W / 2, H - tinyH, owner.short_name); // short name (4 chars) keeps the row uncrowded
}

// Render "qr:" payload (<data>|<caption>) as a QR code sized to the message area.
// Dark modules are drawn in WHITE (= ink on this e-ink); quiet zone is untouched paper.
// Version capped at 6 (41x41): at 2px/module it still fits the 122px screen and scans
// at close range; longer payloads render an error instead of an unscannable code.
static void drawMoonQrContent(OLEDDisplay *display, const char *payload, int W, int H, const char *attr)
{
    const int tinyH = 13; // for the too-long error placement only
    char data[200];
    char caption[80] = "";
    strncpy(data, payload, sizeof(data) - 1);
    data[sizeof(data) - 1] = '\0';
    char *sep = strchr(data, '|');
    if (sep) {
        strncpy(caption, sep + 1, sizeof(caption) - 1);
        caption[sizeof(caption) - 1] = '\0';
        *sep = '\0';
    }

    uint8_t qr[qrcodegen_BUFFER_LEN_FOR_VERSION(6)];
    uint8_t tmp[qrcodegen_BUFFER_LEN_FOR_VERSION(6)];
    // ECC LOW (with boost) keeps the version — and thus module count — minimal, so each
    // module gets the most pixels; on pristine high-contrast e-ink, damage tolerance is
    // the least valuable thing to spend pixels on and camera-readability the most.
    bool ok = data[0] && qrcodegen_encodeText(data, tmp, qr, qrcodegen_Ecc_LOW, 1, 6, qrcodegen_Mask_AUTO, true);
    if (!ok) {
        display->setFont(MOON_FONT_M);
        display->setTextAlignment(TEXT_ALIGN_CENTER);
        display->drawString(W / 2, (H - tinyH) / 2 - 10, "QR: data too long");
        return;
    }

    // Use the FULL display height: the white paper around the code is the quiet zone.
    // QR full-height on the left; caption + service info (sender/time, battery, name)
    // stack in the leftover right-hand column, so no vertical pixel is taken from the code.
    const int size = qrcodegen_getSize(qr);
    int scale = H / size;
    if (scale < 2)
        scale = 2;
    const int qrPix = size * scale;
    const int x0 = 4;
    int y0 = (H - qrPix) / 2;
    if (y0 < 0)
        y0 = 0;

    display->setColor(WHITE);
    for (int my = 0; my < size; my++)
        for (int mx = 0; mx < size; mx++)
            if (qrcodegen_getModule(qr, mx, my))
                display->fillRect(x0 + mx * scale, y0 + my * scale, scale, scale);

    const int colX = x0 + qrPix + 8;
    const int colW = W - colX - 2;
    const int rowH = moonFontH(MOON_MIN_FONT_IDX);
    display->setFont(MOON_FONTS[MOON_MIN_FONT_IDX]);
    display->setTextAlignment(TEXT_ALIGN_LEFT);

    // Caption at the top of the column
    int colY = 2;
    if (caption[0] && colW > 20) {
        std::vector<std::string> rows;
        moonWrapLine(display, caption, colW, MOON_MIN_FONT_IDX, rows);
        for (const auto &r : rows) {
            if (colY + rowH > H - 2 * rowH) // leave room for the service rows
                break;
            display->drawString(colX, colY, r.c_str());
            colY += rowH;
        }
    }

    // Service rows pinned to the column bottom: attribution, then battery + name
    if (colW > 20) {
        int svcY = H - rowH - 1;
        char status[24];
        if (powerStatus && powerStatus->getHasBattery())
            snprintf(status, sizeof(status), "%u%%  %s", powerStatus->getBatteryChargePercent(), owner.short_name);
        else
            snprintf(status, sizeof(status), "%s", owner.short_name);
        display->drawString(colX, svcY, status);
        if (attr && attr[0]) {
            svcY -= rowH;
            display->drawString(colX, svcY, attr);
        }
    }
}

static void drawMoonSignFrame(OLEDDisplay *display, int16_t x, int16_t y)
{
    hasUnreadMessage = false;
    const int W = SCREEN_WIDTH;
    const int H = SCREEN_HEIGHT;
    const int tinyH = 13; // ArialMT_Plain_10 line height

    display->clear();
    display->setColor(WHITE);

    // Idle state (fresh boot, no message yet, and nothing flashing): sleeping-moon screensaver
    if (!moonFlashActive() && (!s_moonHas || s_moonMsg[0] == '\0')) {
        // Night sky
        static const uint8_t stars[][2] = {{12, 10},  {30, 22},  {60, 10}, {90, 8},   {115, 14}, {140, 6}, {170, 10},
                                           {200, 6},  {228, 12}, {240, 30}, {18, 78},  {238, 68}, {120, 20}};
        for (size_t i = 0; i < sizeof(stars) / sizeof(stars[0]); i++)
            display->setPixel(stars[i][0], stars[i][1]);
        // A few 4-point sparkles
        static const uint8_t sparks[][2] = {{25, 40}, {215, 18}, {185, 70}};
        for (size_t i = 0; i < sizeof(sparks) / sizeof(sparks[0]); i++) {
            const int sx = sparks[i][0], sy = sparks[i][1];
            display->drawLine(sx - 2, sy, sx + 2, sy);
            display->drawLine(sx, sy - 2, sx, sy + 2);
        }

        // Big sleeping crescent moon with a closed eye and a soft smile
        const int cx = 55, cy = 52, r = 18;
        drawMoonAccent(display, cx, cy, r);
        display->setColor(BLACK);
        display->drawLine(cx - 11, cy + 1, cx - 5, cy + 1); // closed eye ‿
        display->setPixel(cx - 12, cy);
        display->setPixel(cx - 4, cy);
        display->drawLine(cx - 9, cy + 8, cx - 5, cy + 8); // smile ‿
        display->setPixel(cx - 10, cy + 7);
        display->setPixel(cx - 4, cy + 7);
        display->setColor(WHITE);

        // z's drifting up from the moon
        display->setFont(ArialMT_Plain_10);
        display->setTextAlignment(TEXT_ALIGN_LEFT);
        display->drawString(cx + 22, cy - 24, "z");
        display->drawString(cx + 30, cy - 31, "z");
        display->drawString(cx + 39, cy - 38, "z");

        display->setFont(ArialMT_Plain_24);
        display->setTextAlignment(TEXT_ALIGN_CENTER);
        display->drawString(165, 34, "MoonHut");
        display->setFont(ArialMT_Plain_10);
        display->drawString(165, 64, "waiting for messages...");

        drawMoonBottomRow(display, W, H, tinyH);
        return;
    }

    const int msgWidth = W - 4;
    const int msgTop = 1;
    const int msgBottom = H - tinyH - 1; // bottom status row is always drawn
    const int msgAreaH = msgBottom - msgTop;

    // Decide which text owns the screen: an active flash (DM / other-channel message)
    // temporarily overlays the persistent MoonPaper message, then it comes back.
    const bool flashActive = moonFlashActive();

    // QR mode (POC for the dynamic-QR project): a message of the form "qr:<data>|<caption>"
    // renders as a scannable QR code instead of text. Works for both the persistent sign
    // message and flashes; paging never applies.
    {
        const char *activeMsg = flashActive ? s_moonFlashMsg : s_moonMsg;
        if (strncmp(activeMsg, "qr:", 3) == 0) {
            // Dedicated layout: QR owns the full height on the left; caption and the
            // service info (attribution, battery, name) live in the right-hand column.
            drawMoonQrContent(display, activeMsg + 3, W, H, flashActive ? s_moonFlashAttr : s_moonAttr);
            return;
        }
    }

    // (Re)layout when a new message arrived OR the flash/persistent source changed
    if (s_moonDirty || flashActive != s_moonLayoutIsFlash) {
        moonLayout(display, msgWidth, msgAreaH, flashActive ? s_moonFlashMsg : s_moonMsg);
        s_moonLayoutIsFlash = flashActive;
        s_moonDirty = false;
        s_moonLastPage = -1;
        EINK_ADD_FRAMEFLAG(display, DEMAND_FAST); // repaint promptly on any source switch
        if (!flashActive) {
            const int pgs = (int)s_moonPageFirstRow.size();
            if (s_moonNewPersistent) {
                // Fresh MoonPaper message: full paging cycles from the start
                s_moonNewPersistent = false;
                s_moonPagingStart = millis();
                s_moonManualPage = -1;
            } else {
                // Returning from a flash: settle directly on the resting view (page 1)
                s_moonPagingStart = millis() - (uint32_t)pgs * MOON_PAGE_CYCLES * MOON_PAGE_MS;
            }
        }
    }

    // Paging state: cycle all pages MOON_PAGE_CYCLES times, then rest on page 1.
    // A PRG press (moonSignNextPage) overrides with manual flipping until the next message.
    // A flash never pages: it shows its first page for its few seconds, then expires.
    const int pages = (int)s_moonPageFirstRow.size();
    int page = 0;
    if (!flashActive && pages > 1) {
        bool autoCycling = false;
        if (s_moonManualPage >= 0) {
            page = s_moonManualPage;
        } else {
            uint32_t seq = (millis() - s_moonPagingStart) / MOON_PAGE_MS;
            if (seq < (uint32_t)pages * MOON_PAGE_CYCLES) {
                page = (int)(seq % pages);
                autoCycling = true;
            } // else: resting on page 1 until the next message (or a PRG press)
        }
        if (page != s_moonLastPage) {
            s_moonLastPage = page;
            // While auto-cycling, each flip counts as user activity (same event the keyboard
            // uses) so the screen stays awake — EVENT_RECEIVED_MSG does NOT reset the screen
            // timer. Manual flips come from a real button press which already woke the FSM.
            if (autoCycling)
                powerFSM.trigger(EVENT_PRESS);
            // Make the e-ink show the new page NOW rather than on the throttled
            // background-refresh cadence.
            EINK_ADD_FRAMEFLAG(display, DEMAND_FAST);
        }
    }

    // Rows of the current page
    const int rFirst = s_moonPageFirstRow[page];
    const int rEnd = (page + 1 < pages) ? s_moonPageFirstRow[page + 1] : (int)s_moonRows.size();
    int blockH = 0;
    for (int r = rFirst; r < rEnd; r++)
        blockH += s_moonRowH[r];

    // Single page: vertically centered as before. Multi-page: top-aligned for stable reading.
    int rowY = (pages == 1) ? msgTop + (msgAreaH - blockH) / 2 : msgTop;
    if (rowY < msgTop)
        rowY = msgTop;
    display->setTextAlignment(TEXT_ALIGN_LEFT); // emote-aware drawing is left-anchored; we center manually
    for (int r = rFirst; r < rEnd; r++) {
        const uint8_t f = s_moonRowFontIdx[r];
        display->setFont(MOON_FONTS[f]);
        if (!s_moonRows[r].empty()) {
            int w = EmoteRenderer::measureStringWithEmotes(display, s_moonRows[r]);
            int x0 = (W - w) / 2;
            if (x0 < 2)
                x0 = 2;
            EmoteRenderer::drawStringWithEmotes(display, x0, rowY, s_moonRows[r], moonFontH(f) - 1, emotes, numEmotes);
        }
        rowY += s_moonRowH[r];
    }

    // Small moon accent, tucked in the top-left corner so it barely intrudes on the message.
    drawMoonAccent(display, 9, 9, 7);

    // Tiny attribution (sender + time) bottom-right; a flash shows its own (prefixed) one.
    const char *attr = flashActive ? s_moonFlashAttr : s_moonAttr;
    if (attr[0]) {
        display->setFont(MOON_FONT_S);
        display->setTextAlignment(TEXT_ALIGN_RIGHT);
        display->drawString(W - 2, H - tinyH, attr);
    }

    // Battery % left + device name centered, same row.
    drawMoonBottomRow(display, W, H, tinyH);

    // Page indicator to the LEFT of the centered device name (right side belongs to the
    // attribution; battery corner belongs to the sleep-moon overlay). Hidden during a flash.
    if (!flashActive && pages > 1) {
        char pg[12];
        snprintf(pg, sizeof(pg), "%d/%d", page + 1, pages);
        display->setFont(MOON_FONT_S);
        int nameW = display->getStringWidth(owner.short_name);
        display->setTextAlignment(TEXT_ALIGN_RIGHT);
        display->drawString(W / 2 - nameW / 2 - 8, H - tinyH, pg);
    }
}
#endif // MOONHUT_SIGN

void drawTextMessageFrame(OLEDDisplay *display, OLEDDisplayUiState *state, int16_t x, int16_t y)
{
#ifdef MOONHUT_SIGN
    // MoonHut: the text-message frame IS the sign — render the latest MoonPaper message fullscreen.
    drawMoonSignFrame(display, x, y);
    return;
#endif
    // Ensure any boot-relative timestamps are upgraded if RTC is valid
    messageStore.upgradeBootRelativeTimestamps();

    if (!didReset) {
        resetScrollState();
        didReset = true;
    }


    // Clear the unread message indicator when viewing the message
    hasUnreadMessage = false;

    // Filter messages based on thread mode
    std::deque<StoredMessage> filtered;
    for (const auto &m : messageStore.getLiveMessages()) {
        bool include = false;
        switch (currentMode) {
        case ThreadMode::ALL:
            include = true;
            break;
        case ThreadMode::CHANNEL:
            if (m.type == MessageType::BROADCAST && (int)m.channelIndex == currentChannel)
                include = true;
            break;
        case ThreadMode::DIRECT:
            if (m.dest != NODENUM_BROADCAST && (m.sender == currentPeer || m.dest == currentPeer))
                include = true;
            break;
        }
        if (include)
            filtered.push_back(m);
    }

    display->clear();
    display->setTextAlignment(TEXT_ALIGN_LEFT);
    display->setFont(FONT_SMALL);
    const int navHeight = FONT_HEIGHT_SMALL;
    const int scrollBottom = SCREEN_HEIGHT - navHeight;
    const int usableHeight = scrollBottom;
    constexpr int LEFT_MARGIN = 2;
    constexpr int RIGHT_MARGIN = 2;
    constexpr int SCROLLBAR_WIDTH = 3;
    constexpr int BUBBLE_PAD_X = 3;
    constexpr int BUBBLE_PAD_Y = 4;
    constexpr int BUBBLE_RADIUS = 4;
    constexpr int BUBBLE_MIN_W = 24;
    constexpr int BUBBLE_TEXT_INDENT = 2;

    // Check if bubbles are enabled
    const bool showBubbles = config.display.enable_message_bubbles;
    const int textIndent = showBubbles ? (BUBBLE_PAD_X + BUBBLE_TEXT_INDENT) : LEFT_MARGIN;

    // Derived widths
    const int leftTextWidth = SCREEN_WIDTH - LEFT_MARGIN - RIGHT_MARGIN - (showBubbles ? (BUBBLE_PAD_X * 2) : 0);
    const int rightTextWidth = SCREEN_WIDTH - LEFT_MARGIN - RIGHT_MARGIN - SCROLLBAR_WIDTH;

    // Title string depending on mode
    char titleStr[48];
    snprintf(titleStr, sizeof(titleStr), "Messages");
    switch (currentMode) {
    case ThreadMode::ALL:
        snprintf(titleStr, sizeof(titleStr), "Messages");
        break;
    case ThreadMode::CHANNEL: {
        const char *cname = channels.getName(currentChannel);
        if (cname && cname[0]) {
            snprintf(titleStr, sizeof(titleStr), "#%s", cname);
        } else {
            snprintf(titleStr, sizeof(titleStr), "Ch%d", currentChannel);
        }
        break;
    }
    case ThreadMode::DIRECT: {
        meshtastic_NodeInfoLite *node = nodeDB->getMeshNode(currentPeer);
        if (node && node->has_user && node->user.short_name[0]) {
            snprintf(titleStr, sizeof(titleStr), "@%s", node->user.short_name);
        } else {
            snprintf(titleStr, sizeof(titleStr), "@%08x", currentPeer);
        }
        break;
    }
    }

    if (filtered.empty()) {
        // If current conversation is empty go back to ALL view
        if (currentMode != ThreadMode::ALL) {
            setThreadMode(ThreadMode::ALL);
            resetScrollState();
            return; // Next draw will rerun in ALL mode
        }

        // Still in ALL mode and no messages at all → show placeholder
        graphics::drawCommonHeader(display, x, y, titleStr);
        didReset = false;
        const char *messageString = "No messages";
        int center_text = (SCREEN_WIDTH / 2) - (display->getStringWidth(messageString) / 2);
        display->drawString(center_text, getTextPositions(display)[2], messageString);
        graphics::drawCommonFooter(display, x, y);
        return;
    }

    // Build lines for filtered messages (newest first)
    std::vector<std::string> allLines;
    std::vector<bool> isMine;   // track alignment
    std::vector<bool> isHeader; // track header lines
    std::vector<AckStatus> ackForLine;
    // Hard limit on total cached lines to prevent unbounded growth from a single long message.
    // Reserve to the actual cache cap up front, because a single message can expand to many more
    // wrapped display lines than a small per-message estimate would predict. For a display
    // rendering only ~5-30 lines at a time, caching more than this limit wastes heap. Stop
    // appending once we reach MAX_CACHED_LINES to prevent a single message from blowing out the
    // heap.
    constexpr size_t MAX_CACHED_LINES = 100U; // ~5-6KB for std::string overhead on 32-bit (if each ~50-60 bytes avg)
    allLines.reserve(MAX_CACHED_LINES);
    isMine.reserve(MAX_CACHED_LINES);
    isHeader.reserve(MAX_CACHED_LINES);
    ackForLine.reserve(MAX_CACHED_LINES);

    for (auto it = filtered.rbegin(); it != filtered.rend(); ++it) {
        const auto &m = *it;

        // Channel / destination labeling
        char chanType[32] = "";
        if (currentMode == ThreadMode::ALL) {
            if (m.dest == NODENUM_BROADCAST) {
                const char *name = channels.getName(m.channelIndex);
                if (currentResolution == ScreenResolution::Low || currentResolution == ScreenResolution::UltraLow) {
                    if (strcmp(name, "ShortTurbo") == 0)
                        name = "ShortT";
                    else if (strcmp(name, "ShortSlow") == 0)
                        name = "ShortS";
                    else if (strcmp(name, "ShortFast") == 0)
                        name = "ShortF";
                    else if (strcmp(name, "MediumSlow") == 0)
                        name = "MedS";
                    else if (strcmp(name, "MediumFast") == 0)
                        name = "MedF";
                    else if (strcmp(name, "LongSlow") == 0)
                        name = "LongS";
                    else if (strcmp(name, "LongFast") == 0)
                        name = "LongF";
                    else if (strcmp(name, "LongTurbo") == 0)
                        name = "LongT";
                    else if (strcmp(name, "LongMod") == 0)
                        name = "LongM";
                }
                snprintf(chanType, sizeof(chanType), "#%s", name);
            } else {
                snprintf(chanType, sizeof(chanType), "(DM)");
            }
        }

        // Calculate how long ago
        uint32_t nowSecs = getValidTime(RTCQuality::RTCQualityDevice, true);
        uint32_t seconds = 0;
        bool invalidTime = true;

        if (m.timestamp > 0 && nowSecs > 0) {
            if (nowSecs >= m.timestamp) {
                seconds = nowSecs - m.timestamp;
                invalidTime = (seconds > 315360000); // >10 years
            } else {
                uint32_t ahead = m.timestamp - nowSecs;
                if (ahead <= 600) { // allow small skew
                    seconds = 0;
                    invalidTime = false;
                }
            }
        } else if (m.timestamp > 0 && nowSecs == 0) {
            // RTC not valid: only trust boot-relative if same boot
            uint32_t bootNow = millis() / 1000;
            if (m.isBootRelative && m.timestamp <= bootNow) {
                seconds = bootNow - m.timestamp;
                invalidTime = false;
            } else {
                invalidTime = true; // old persisted boot-relative, ignore until healed
            }
        }

        char timeBuf[16];
        if (invalidTime) {
            snprintf(timeBuf, sizeof(timeBuf), "???");
        } else if (seconds < 60) {
            snprintf(timeBuf, sizeof(timeBuf), "%us", seconds);
        } else if (seconds < 3600) {
            snprintf(timeBuf, sizeof(timeBuf), "%um", seconds / 60);
        } else if (seconds < 86400) {
            snprintf(timeBuf, sizeof(timeBuf), "%uh", seconds / 3600);
        } else {
            snprintf(timeBuf, sizeof(timeBuf), "%ud", seconds / 86400);
        }

        // Build header line for this message
        meshtastic_NodeInfoLite *node = nodeDB->getMeshNode(m.sender);
        meshtastic_NodeInfoLite *node_recipient = nodeDB->getMeshNode(m.dest);

        char senderName[64] = "";
        if (node && node->has_user) {
            if (node->user.long_name[0]) {
                strncpy(senderName, node->user.long_name, sizeof(senderName) - 1);
            } else if (node->user.short_name[0]) {
                strncpy(senderName, node->user.short_name, sizeof(senderName) - 1);
            }
            senderName[sizeof(senderName) - 1] = '\0';
        }
        if (!senderName[0]) {
            snprintf(senderName, sizeof(senderName), "(%08x)", m.sender);
        }

        // If this is *our own* message, override senderName to who the recipient was
        bool mine = (m.sender == nodeDB->getNodeNum());
        if (mine && node_recipient && node_recipient->has_user) {
            if (node_recipient->user.long_name[0]) {
                strncpy(senderName, node_recipient->user.long_name, sizeof(senderName) - 1);
                senderName[sizeof(senderName) - 1] = '\0';
            } else if (node_recipient->user.short_name[0]) {
                strncpy(senderName, node_recipient->user.short_name, sizeof(senderName) - 1);
                senderName[sizeof(senderName) - 1] = '\0';
            }
        }
        // If recipient info is missing/empty, prefer a recipient identifier for outbound messages.
        if (mine && (!node_recipient || !node_recipient->has_user ||
                     (!node_recipient->user.long_name[0] && !node_recipient->user.short_name[0]))) {
            snprintf(senderName, sizeof(senderName), "(%08x)", m.dest);
        }

        // Shrink Sender name if needed
        int availWidth = (mine ? rightTextWidth : leftTextWidth) - display->getStringWidth(timeBuf) -
                         display->getStringWidth(chanType) - graphics::UIRenderer::measureStringWithEmotes(display, "   @...");
        if (availWidth < 0)
            availWidth = 0;
        char truncatedSender[64];
        graphics::UIRenderer::truncateStringWithEmotes(display, senderName, truncatedSender, sizeof(truncatedSender), availWidth);

        // Final header line
        char headerStr[128];
        if (mine) {
            if (currentMode == ThreadMode::ALL) {
                if (strcmp(chanType, "(DM)") == 0) {
                    snprintf(headerStr, sizeof(headerStr), "%s to %s", timeBuf, truncatedSender);
                } else {
                    snprintf(headerStr, sizeof(headerStr), "%s to %s", timeBuf, chanType);
                }
            } else {
                snprintf(headerStr, sizeof(headerStr), "%s", timeBuf);
            }
        } else {
            snprintf(headerStr, sizeof(headerStr), chanType[0] ? "%s @%s %s" : "%s @%s", timeBuf, truncatedSender, chanType);
        }

        // Push header line
        allLines.push_back(headerStr);
        isMine.push_back(mine);
        isHeader.push_back(true);
        ackForLine.push_back(m.ackStatus);

        const char *msgText = MessageStore::getText(m);

        int wrapWidth = mine ? rightTextWidth : leftTextWidth;
        std::vector<std::string> wrapped = generateLines(display, "", msgText, wrapWidth);
        // Per-message wrap-line limit: even if wrapping produces many lines, cap them to prevent
        // a single long message from consuming most or all of the cache.
        constexpr size_t MAX_WRAPPED_LINES_PER_MSG = 20U;
        size_t wrappedCount = 0;
        for (auto &ln : wrapped) {
            if (allLines.size() >= MAX_CACHED_LINES || wrappedCount >= MAX_WRAPPED_LINES_PER_MSG)
                break; // Cache limit or per-message limit reached; stop adding lines from this message
            allLines.emplace_back(std::move(ln));
            isMine.push_back(mine);
            isHeader.push_back(false);
            ackForLine.push_back(AckStatus::NONE);
            ++wrappedCount;
        }
    }

    // Cache lines and heights
    cachedLines.swap(allLines);
    cachedHeights = calculateLineHeights(cachedLines, emotes, isHeader);

    std::vector<MessageBlock> blocks = buildMessageBlocks(isHeader, isMine);

    // Scrolling logic (unchanged)
    int totalHeight = 0;
    for (size_t i = 0; i < cachedHeights.size(); ++i)
        totalHeight += cachedHeights[i];
    int usableScrollHeight = usableHeight;
    int scrollStop = std::max(0, totalHeight - usableScrollHeight + cachedHeights.back());

#ifndef USE_EINK
    uint32_t now = millis();
    float delta = (now - lastTime) / 400.0f;
    lastTime = now;
    const float scrollSpeed = 2.0f;

    if (scrollStartDelay == 0)
        scrollStartDelay = now;
    if (!scrollStarted && now - scrollStartDelay > 2000)
        scrollStarted = true;

    if (!manualScrolling && totalHeight > usableScrollHeight) {
        if (scrollStarted) {
            if (!waitingToReset) {
                scrollY += delta * scrollSpeed;
                if (scrollY >= scrollStop) {
                    scrollY = scrollStop;
                    waitingToReset = true;
                    pauseStart = lastTime;
                }
            } else if (lastTime - pauseStart > 3000) {
                scrollY = 0;
                waitingToReset = false;
                scrollStarted = false;
                scrollStartDelay = lastTime;
            }
        }
    } else if (!manualScrolling) {
        scrollY = 0;
    }
#else
    // E-Ink: disable autoscroll
    scrollY = 0.0f;
    waitingToReset = false;
    scrollStarted = false;
    lastTime = millis();
#endif

    int finalScroll = (int)scrollY;
    int yOffset = -finalScroll + getTextPositions(display)[1];
    const int contentTop = getTextPositions(display)[1];
    const int contentBottom = scrollBottom; // already excludes nav line
    const int rightEdge = SCREEN_WIDTH - SCROLLBAR_WIDTH - RIGHT_MARGIN;
    const int bubbleGapY = std::max(1, MESSAGE_BLOCK_GAP / 2);

    std::vector<int> lineTop;
    lineTop.resize(cachedLines.size());
    {
        int acc = 0;
        for (size_t i = 0; i < cachedLines.size(); ++i) {
            lineTop[i] = yOffset + acc;
            acc += cachedHeights[i];
        }
    }

    // Draw bubbles (only if enabled)
    if (showBubbles) {
        for (size_t bi = 0; bi < blocks.size(); ++bi) {
            const auto &b = blocks[bi];
            if (b.start >= cachedLines.size() || b.end >= cachedLines.size() || b.start > b.end)
                continue;

            int visualTop = lineTop[b.start];

            int topY;
            if (isHeader[b.start]) {
                // Header start
                constexpr int BUBBLE_PAD_TOP_HEADER = 1; // try 1 or 2
                topY = visualTop - BUBBLE_PAD_TOP_HEADER;
            } else {
                // Body start
                const bool thisLineHasEmote =
                    graphics::EmoteRenderer::analyzeLine(nullptr, cachedLines[b.start].c_str(), 0, emotes, numEmotes).hasEmote;
                if (thisLineHasEmote) {
                    constexpr int EMOTE_PADDING_ABOVE = 4;
                    visualTop -= EMOTE_PADDING_ABOVE;
                }
                topY = visualTop - BUBBLE_PAD_Y;
            }
            int visualBottom = getDrawnLinePixelBottom(lineTop[b.end], cachedLines[b.end], isHeader[b.end]);
            int bottomY = visualBottom + BUBBLE_PAD_Y;

            if (bi + 1 < blocks.size()) {
                int nextHeaderIndex = (int)blocks[bi + 1].start;
                int nextTop = lineTop[nextHeaderIndex];
                int maxBottom = nextTop - 1 - bubbleGapY;
                if (bottomY > maxBottom)
                    bottomY = maxBottom;
            }

            if (bottomY <= topY + 2)
                continue;

            if (bottomY < contentTop || topY > contentBottom - 1)
                continue;

            int maxLineW = 0;

            for (size_t i = b.start; i <= b.end; ++i) {
                int w = 0;
                if (isHeader[i]) {
                    w = graphics::UIRenderer::measureStringWithEmotes(display, cachedLines[i].c_str());
                    if (b.mine)
                        w += 12; // room for ACK/NACK/relay mark
                } else {
                    w = getRenderedLineWidth(display, cachedLines[i], emotes, numEmotes);
                }
                if (w > maxLineW)
                    maxLineW = w;
            }

            int bubbleW = std::max(BUBBLE_MIN_W, maxLineW + (textIndent * 2));
            int bubbleH = (bottomY - topY) + 1;
            int bubbleX = 0;
            if (b.mine) {
                bubbleX = rightEdge - bubbleW;
            } else {
                bubbleX = x;
            }
            if (bubbleX < x)
                bubbleX = x;
            if (bubbleX + bubbleW > rightEdge)
                bubbleW = std::max(1, rightEdge - bubbleX);

            // Draw rounded rectangle bubble
            if (bubbleW > BUBBLE_RADIUS * 2 && bubbleH > BUBBLE_RADIUS * 2) {
                const int r = BUBBLE_RADIUS;
                const int bx = bubbleX;
                const int by = topY;
                const int bw = bubbleW;
                const int bh = bubbleH;

                // Draw the 4 corner arcs using drawCircleQuads
                display->drawCircleQuads(bx + r, by + r, r, 0x2);                   // Top-left
                display->drawCircleQuads(bx + bw - r - 1, by + r, r, 0x1);          // Top-right
                display->drawCircleQuads(bx + r, by + bh - r - 1, r, 0x4);          // Bottom-left
                display->drawCircleQuads(bx + bw - r - 1, by + bh - r - 1, r, 0x8); // Bottom-right

                // Draw the 4 edges between corners
                display->drawHorizontalLine(bx + r, by, bw - 2 * r);          // Top edge
                display->drawHorizontalLine(bx + r, by + bh - 1, bw - 2 * r); // Bottom edge
                display->drawVerticalLine(bx, by + r, bh - 2 * r);            // Left edge
                display->drawVerticalLine(bx + bw - 1, by + r, bh - 2 * r);   // Right edge
            } else if (bubbleW > 1 && bubbleH > 1) {
                // Fallback to simple rectangle for very small bubbles
                display->drawRect(bubbleX, topY, bubbleW, bubbleH);
            }
        }
    } // end if (showBubbles)

    // Render visible lines
    int lineY = yOffset;
    for (size_t i = 0; i < cachedLines.size(); ++i) {

        if (lineY > -cachedHeights[i] && lineY < scrollBottom) {
            if (isHeader[i]) {

                int w = graphics::UIRenderer::measureStringWithEmotes(display, cachedLines[i].c_str());
                int headerX;
                if (isMine[i]) {
                    // push header left to avoid overlap with scrollbar
                    headerX = (SCREEN_WIDTH - SCROLLBAR_WIDTH - RIGHT_MARGIN) - w - (showBubbles ? textIndent : 0);
                    if (headerX < LEFT_MARGIN)
                        headerX = LEFT_MARGIN;
                } else {
                    headerX = x + textIndent;
                }
                graphics::UIRenderer::drawStringWithEmotes(display, headerX, lineY, cachedLines[i].c_str(), FONT_HEIGHT_SMALL, 1,
                                                           false);

                // Draw underline just under header text
                int underlineY = lineY + FONT_HEIGHT_SMALL;

                int underlineW = w;
                int maxW = rightEdge - headerX;
                if (maxW < 0)
                    maxW = 0;
                if (underlineW > maxW)
                    underlineW = maxW;

                for (int px = 0; px < underlineW; ++px) {
                    display->setPixel(headerX + px, underlineY);
                }

                // Draw ACK/NACK mark for our own messages
                if (isMine[i]) {
                    int markX = headerX - 10;
                    int markY = lineY;
                    if (ackForLine[i] == AckStatus::ACKED) {
                        // Destination ACK
                        drawCheckMark(display, markX, markY, 8);
                    } else if (ackForLine[i] == AckStatus::NACKED || ackForLine[i] == AckStatus::TIMEOUT) {
                        // Failure or timeout
                        drawXMark(display, markX, markY, 8);
                    } else if (ackForLine[i] == AckStatus::RELAYED) {
                        // Relay ACK
                        drawRelayMark(display, markX, markY, 8);
                    }
                    // AckStatus::NONE → show nothing
                }

            } else {
                // Render message line
                if (isMine[i]) {
                    // Calculate actual rendered width including emotes
                    int renderedWidth = getRenderedLineWidth(display, cachedLines[i], emotes, numEmotes);
                    int rightX = (SCREEN_WIDTH - SCROLLBAR_WIDTH - RIGHT_MARGIN) - renderedWidth - (showBubbles ? textIndent : 0);
                    if (rightX < LEFT_MARGIN)
                        rightX = LEFT_MARGIN;

                    drawStringWithEmotes(display, rightX, lineY, cachedLines[i], emotes, numEmotes);
                } else {
                    drawStringWithEmotes(display, x + textIndent, lineY, cachedLines[i], emotes, numEmotes);
                }
            }
        }

        lineY += cachedHeights[i];
    }

    // Draw scrollbar
    drawMessageScrollbar(display, usableHeight, totalHeight, finalScroll, getTextPositions(display)[1]);
    graphics::drawCommonHeader(display, x, y, titleStr);
    graphics::drawCommonFooter(display, x, y);
}

std::vector<std::string> generateLines(OLEDDisplay *display, const char *headerStr, const char *messageBuf, int textWidth)
{
    std::vector<std::string> lines;

    // Only push headerStr if it's not empty (prevents extra blank line after headers)
    if (headerStr && headerStr[0] != '\0') {
        lines.push_back(std::string(headerStr));
    }

    std::string line, word;
    for (int i = 0; messageBuf[i]; ++i) {
        char ch = messageBuf[i];
        if ((unsigned char)messageBuf[i] == 0xE2 && (unsigned char)messageBuf[i + 1] == 0x80 &&
            (unsigned char)messageBuf[i + 2] == 0x99) {
            ch = '\''; // plain apostrophe
            i += 2;    // skip over the extra UTF-8 bytes
        }
        if (ch == '\n') {
            if (!word.empty())
                line += word;
            if (!line.empty())
                lines.push_back(line);
            line.clear();
            word.clear();
        } else if (ch == ' ') {
            line += word + ' ';
            word.clear();
        } else {
            word += ch;
            std::string test = line + word;
            uint16_t strWidth = graphics::UIRenderer::measureStringWithEmotes(display, test.c_str());
            if (strWidth > textWidth) {
                if (!line.empty())
                    lines.push_back(line);
                line = word;
                word.clear();
            }
        }
    }

    if (!word.empty())
        line += word;
    if (!line.empty())
        lines.push_back(line);

    return lines;
}
std::vector<int> calculateLineHeights(const std::vector<std::string> &lines, const Emote *emotes,
                                      const std::vector<bool> &isHeaderVec)
{
    // Tunables for layout control
    constexpr int HEADER_UNDERLINE_GAP = 0; // space between underline and first body line
    constexpr int HEADER_UNDERLINE_PIX = 1; // underline thickness (1px row drawn)
    constexpr int BODY_LINE_LEADING = -4;   // default vertical leading for normal body lines
    constexpr int EMOTE_PADDING_ABOVE = 4;  // space above emote line (added to line above)
    constexpr int EMOTE_PADDING_BELOW = 3;  // space below emote line (added to emote line)

    std::vector<int> rowHeights;
    rowHeights.reserve(lines.size());
    std::vector<graphics::EmoteRenderer::LineMetrics> lineMetrics;
    lineMetrics.reserve(lines.size());

    for (const auto &line : lines) {
        lineMetrics.push_back(graphics::EmoteRenderer::analyzeLine(nullptr, line, FONT_HEIGHT_SMALL, emotes, numEmotes));
    }

    for (size_t idx = 0; idx < lines.size(); ++idx) {
        const int baseHeight = FONT_HEIGHT_SMALL;
        int lineHeight = baseHeight;

        const int tallestEmote = lineMetrics[idx].tallestHeight;
        const bool hasEmote = lineMetrics[idx].hasEmote;
        const bool nextHasEmote = (idx + 1 < lines.size()) && lineMetrics[idx + 1].hasEmote;

        if (isHeaderVec[idx]) {
            // Header line spacing
            lineHeight = baseHeight + HEADER_UNDERLINE_PIX + HEADER_UNDERLINE_GAP;
        } else {
            // Base spacing for normal lines
            int desiredBody = baseHeight + BODY_LINE_LEADING;

            if (hasEmote) {
                // Emote line: add overshoot + bottom padding
                int overshoot = std::max(0, tallestEmote - baseHeight);
                lineHeight = desiredBody + overshoot + EMOTE_PADDING_BELOW;
            } else {
                // Regular line: no emote → standard spacing
                lineHeight = desiredBody;

                // If next line has an emote → add top padding *here*
                if (nextHasEmote) {
                    lineHeight += EMOTE_PADDING_ABOVE;
                }
            }

            // Add block gap if next is a header
            if (idx + 1 < lines.size() && isHeaderVec[idx + 1]) {
                lineHeight += MESSAGE_BLOCK_GAP;
            }
        }

        rowHeights.push_back(lineHeight);
    }

    return rowHeights;
}

void handleNewMessage(OLEDDisplay *display, const StoredMessage &sm, const meshtastic_MeshPacket &packet)
{
    if (packet.from != 0) {
        hasUnreadMessage = true;

        // Determine if message belongs to a muted channel
        bool isChannelMuted = false;
        if (sm.type == MessageType::BROADCAST) {
            const meshtastic_Channel channel = channels.getByIndex(packet.channel ? packet.channel : channels.getPrimaryIndex());
            if (channel.settings.has_module_settings && channel.settings.module_settings.is_muted)
                isChannelMuted = true;
        }

        // Banner logic
        const meshtastic_NodeInfoLite *node = nodeDB->getMeshNode(packet.from);
        char longName[64] = "?";
        if (node && node->has_user) {
            if (node->user.long_name[0]) {
                strncpy(longName, node->user.long_name, sizeof(longName) - 1);
                longName[sizeof(longName) - 1] = '\0';
            } else if (node->user.short_name[0]) {
                strncpy(longName, node->user.short_name, sizeof(longName) - 1);
                longName[sizeof(longName) - 1] = '\0';
            }
        }
        int availWidth = display->getWidth() - ((currentResolution == ScreenResolution::High) ? 40 : 20);
        if (availWidth < 0)
            availWidth = 0;
        char truncatedLongName[64];
        graphics::UIRenderer::truncateStringWithEmotes(display, longName, truncatedLongName, sizeof(truncatedLongName),
                                                       availWidth);
        const char *msgRaw = reinterpret_cast<const char *>(packet.decoded.payload.bytes);

        char banner[256];
        bool isAlert = false;

        // Check if alert detection is enabled via external notification module
        if (moduleConfig.external_notification.alert_bell || moduleConfig.external_notification.alert_bell_vibra ||
            moduleConfig.external_notification.alert_bell_buzzer) {
            for (size_t i = 0; i < packet.decoded.payload.size && i < 100; i++) {
                if (msgRaw[i] == '\x07') {
                    isAlert = true;
                    break;
                }
            }
        }

        if (isAlert) {
            if (truncatedLongName[0])
                snprintf(banner, sizeof(banner), "Alert Received from\n%s", truncatedLongName);
            else
                strcpy(banner, "Alert Received");
        } else {
            // Skip muted channels unless it's an alert
            if (isChannelMuted)
                return;

            if (truncatedLongName[0]) {
                if (currentResolution == ScreenResolution::UltraLow) {
                    strcpy(banner, "New Message");
                } else {
                    snprintf(banner, sizeof(banner), "New Message from\n%s", truncatedLongName);
                }
            } else
                strcpy(banner, "New Message");
        }

        // Append context (which channel or DM) so the banner shows where the message arrived
        {
            char contextBuf[64] = "";
            if (sm.type == MessageType::BROADCAST) {
                const char *cname = channels.getName(sm.channelIndex);
                if (cname && cname[0])
                    snprintf(contextBuf, sizeof(contextBuf), "in #%s", cname);
                else
                    snprintf(contextBuf, sizeof(contextBuf), "in Ch%d", sm.channelIndex);
            }

            if (contextBuf[0]) {
                size_t cur = strlen(banner);
                if (cur + 1 < sizeof(banner)) {
                    if (cur > 0 && banner[cur - 1] != '\n') {
                        banner[cur] = '\n';
                        banner[cur + 1] = '\0';
                        cur++;
                    }
                    strncat(banner, contextBuf, sizeof(banner) - cur - 1);
                }
            }
        }

        // Shorter banner if already in a conversation (Channel or Direct)
        bool inThread = (getThreadMode() != ThreadMode::ALL);

        if (shouldWakeOnReceivedMessage()) {
            screen->setOn(true);
        }

        screen->showSimpleBanner(banner, inThread ? 1000 : 3000);
    }

    // Always focus into the correct conversation thread when a message with real text arrives
    const char *msgText = MessageStore::getText(sm);
    if (msgText && msgText[0] != '\0') {
        setThreadFor(sm, packet);
    }

    // Reset scroll for a clean start
    resetScrollState();
}

void setThreadFor(const StoredMessage &sm, const meshtastic_MeshPacket &packet)
{
    if (packet.to == 0 || packet.to == NODENUM_BROADCAST) {
        setThreadMode(ThreadMode::CHANNEL, sm.channelIndex);
    } else {
        uint32_t localNode = nodeDB->getNodeNum();
        uint32_t peer = (sm.sender == localNode) ? packet.to : sm.sender;
        setThreadMode(ThreadMode::DIRECT, -1, peer);
    }
}

} // namespace MessageRenderer
} // namespace graphics
#endif
