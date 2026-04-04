#include "../ui_display.h"
#include "ui_display_internal.h"

#include "../hw/hw_objects.h"

namespace {
using namespace ui_display_internal;

struct StartupScreenLayout {
  int displayW;
  int displayH;
  int topBarH;
  int bottomBarH;
  int footerY;
  int contentY;
  int contentH;
  bool isLargeDisplay;
};

int footer_bar_height_px(int displayH) {
  return max(16, (displayH * 8) / 100);
}

void draw_horizontal_separator(int y, int width, uint16_t color) {
  tft.drawLine(0, y, width - 1, y, color);
}

String two_digits(int value) {
  if (value < 10) return String("0") + String(value);
  return String(value);
}

String rtc_timestamp_text() {
  const DateTime now = rtc.now();
  return two_digits(now.day()) + "/" + two_digits(now.month()) + "/" +
         two_digits(now.year() % 100) + " " + two_digits(now.hour()) + ":" +
         two_digits(now.minute());
}

StartupScreenLayout startup_screen_layout() {
  const int displayW = uiDisplayWidthPx();
  const int displayH = uiDisplayHeightPx();
  const int topBarH = (displayH * 10) / 100;
  const int bottomBarH = footer_bar_height_px(displayH);
  const int footerY = displayH - bottomBarH;
  return {displayW, displayH, topBarH, bottomBarH, footerY, topBarH, footerY - topBarH, displayW >= 400};
}

void draw_startup_borders(const StartupScreenLayout &layout) {
  uiDisplayDrawRect(0, 0, layout.displayW, layout.displayH, kUiBorder);
  draw_horizontal_separator(layout.topBarH, layout.displayW, kUiBorder);
  draw_horizontal_separator(layout.footerY, layout.displayW, kUiBorder);
}

void draw_startup_base(const StartupScreenLayout &layout, const char *title, bool rtcDetected) {
  const uint8_t barTextFont = 2;
  const uint8_t barTextSize = layout.isLargeDisplay ? 2 : 1;
  const int topBarTextY =
      ((layout.topBarH - uiDisplayFontHeight(barTextSize, barTextFont)) / 2) + (layout.isLargeDisplay ? 1 : 0);
  const int footerTextY =
      layout.footerY + ((layout.bottomBarH - uiDisplayFontHeight(kFooterTextSize, kFooterTextFont)) / 2);

  uiDisplayClear();
  uiDisplayFillRect(0, 0, layout.displayW, layout.topBarH, kUiAccent);
  uiDisplayFillRect(1, layout.contentY + 1, layout.displayW - 2, layout.contentH - 1, kUiModeAreaBg);
  uiDisplayFillRect(0, layout.footerY, layout.displayW, layout.bottomBarH, kUiAccent);
  draw_startup_borders(layout);

  uiDisplayPrintStyledAt(layout.displayW / 50, topBarTextY, title, kUiText, kUiAccent, barTextSize, barTextFont);
  uiDisplayPrintStyledAt(4, footerTextY, kFirmwareVersion, kUiText, kUiAccent, kFooterTextSize, kFooterTextFont);
  if (!rtcDetected) {
    return;
  }

  const String footerDateTime = rtc_timestamp_text();
  uiDisplayPrintStyledAt(layout.displayW - uiDisplayTextWidth(footerDateTime, kFooterTextSize, kFooterTextFont) - 4,
                         footerTextY,
                         footerDateTime,
                         kUiText,
                         kUiAccent,
                         kFooterTextSize,
                         kFooterTextFont);
}

String startup_status_text(bool detected) {
  return detected ? "Detected" : "Not detected";
}
}

void uiDisplayRenderStartupSplash(void) {
  const StartupScreenLayout layout = startup_screen_layout();
  const uint8_t titleFont = 2;
  const uint8_t titleSize = layout.isLargeDisplay ? 2 : 1;
  const uint8_t textFont = 2;
  const uint8_t textSize = layout.isLargeDisplay ? 2 : 1;
  const String line1 = "DC Electronic Load";
  const String line2 = "Initializing system...";
  const String line3 = "By Guy & Codex";
  const int centerY = layout.contentY + (layout.contentH / 2) - uiDisplayFontHeight(textSize, textFont);
  const int gap = layout.isLargeDisplay ? 20 : 12;
  const int line1Y = centerY - uiDisplayFontHeight(titleSize, titleFont) - gap;
  const int line2Y = centerY;
  const int line3Y = centerY + uiDisplayFontHeight(textSize, textFont) + gap;

  draw_startup_base(layout, "STARTUP", false);
  uiDisplayPrintStyledAt((layout.displayW - uiDisplayTextWidth(line1, titleSize, titleFont)) / 2,
                         line1Y,
                         line1,
                         kUiHighlight,
                         kUiModeAreaBg,
                         titleSize,
                         titleFont);
  uiDisplayPrintStyledAt((layout.displayW - uiDisplayTextWidth(line2, textSize, textFont)) / 2,
                         line2Y,
                         line2,
                         kUiText,
                         kUiModeAreaBg,
                         textSize,
                         textFont);
  uiDisplayPrintStyledAt((layout.displayW - uiDisplayTextWidth(line3, textSize, textFont)) / 2,
                         line3Y,
                         line3,
                         kUiText,
                         kUiModeAreaBg,
                         textSize,
                         textFont);
  draw_startup_borders(layout);
}

void uiDisplayRenderStartupHealthCheck(bool dacDetected, bool adsDetected, bool rtcDetected, bool sensorOk) {
  const StartupScreenLayout layout = startup_screen_layout();
  const uint8_t textFont = 2;
  const uint8_t textSize = layout.isLargeDisplay ? 2 : 1;
  const int lineHeight = uiDisplayFontHeight(textSize, textFont);
  const int verticalGap = layout.isLargeDisplay ? 16 : 10;
  const int totalBlockH = (lineHeight * 4) + (verticalGap * 3);
  const int startY = layout.contentY + max(8, (layout.contentH - totalBlockH) / 2);
  const String line1 = String("DAC: ") + startup_status_text(dacDetected);
  const String line2 = String("ADS: ") + startup_status_text(adsDetected);
  const String line3 = String("RTC: ") + startup_status_text(rtcDetected);
  const String line4 = String("TEMP: ") + (sensorOk ? "Detected" : "Fail");

  draw_startup_base(layout, "HEALTH CHECK", rtcDetected);
  uiDisplayPrintStyledAt((layout.displayW - uiDisplayTextWidth(line1, textSize, textFont)) / 2,
                         startY,
                         line1,
                         dacDetected ? kUiText : kUiHighlight,
                         kUiModeAreaBg,
                         textSize,
                         textFont);
  uiDisplayPrintStyledAt((layout.displayW - uiDisplayTextWidth(line2, textSize, textFont)) / 2,
                         startY + lineHeight + verticalGap,
                         line2,
                         adsDetected ? kUiText : kUiHighlight,
                         kUiModeAreaBg,
                         textSize,
                         textFont);
  uiDisplayPrintStyledAt((layout.displayW - uiDisplayTextWidth(line3, textSize, textFont)) / 2,
                         startY + ((lineHeight + verticalGap) * 2),
                         line3,
                         rtcDetected ? kUiText : kUiHighlight,
                         kUiModeAreaBg,
                         textSize,
                         textFont);
  uiDisplayPrintStyledAt((layout.displayW - uiDisplayTextWidth(line4, textSize, textFont)) / 2,
                         startY + ((lineHeight + verticalGap) * 3),
                         line4,
                         sensorOk ? kUiText : kUiHighlight,
                         kUiModeAreaBg,
                         textSize,
                         textFont);
  draw_startup_borders(layout);
}
