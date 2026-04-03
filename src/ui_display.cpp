#include "ui_display.h"

#include "hw/hw_objects.h"

namespace {
constexpr uint16_t TFT_TEXT_COLOR = TFT_CYAN;
constexpr uint16_t TFT_BG_COLOR = TFT_BLACK;

void restore_tft_text_style() {
  tft.setTextColor(TFT_TEXT_COLOR, TFT_BG_COLOR);
  tft.setTextFont(1);
  tft.setTextSize(1);
}
}  // namespace

int uiDisplayWidthPx() { return tft.width(); }

int uiDisplayHeightPx() { return tft.height(); }

int uiDisplayTextWidth(const char *text, uint8_t textSize, uint8_t textFont) {
  tft.setTextFont(textFont);
  tft.setTextSize(textSize);
  const int width = tft.textWidth(text);
  restore_tft_text_style();
  return width;
}

int uiDisplayTextWidth(const String &text, uint8_t textSize, uint8_t textFont) {
  return uiDisplayTextWidth(text.c_str(), textSize, textFont);
}

int uiDisplayFontHeight(uint8_t textSize, uint8_t textFont) {
  tft.setTextFont(textFont);
  tft.setTextSize(textSize);
  const int height = tft.fontHeight();
  restore_tft_text_style();
  return height;
}

void uiDisplayInit(void) {
  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BG_COLOR);
  restore_tft_text_style();
  tft.setTextWrap(false);
}

void uiDisplayClear(void) {
  tft.fillScreen(TFT_BG_COLOR);
  tft.setCursor(0, 0);
  restore_tft_text_style();
}

void uiDisplayFillRect(int x, int y, int w, int h, uint16_t color) {
  tft.fillRect(x, y, w, h, color);
}

void uiDisplayDrawRect(int x, int y, int w, int h, uint16_t color) {
  tft.drawRect(x, y, w, h, color);
}

void uiDisplayFillCircle(int x, int y, int r, uint16_t color) {
  tft.fillCircle(x, y, r, color);
}

void uiDisplayDrawCircle(int x, int y, int r, uint16_t color) {
  tft.drawCircle(x, y, r, color);
}

void uiDisplayPrintAt(int x, int y, const char *text, uint16_t fg, uint16_t bg, uint8_t textSize) {
  uiDisplayPrintStyledAt(x, y, text, fg, bg, textSize, 1);
}

void uiDisplayPrintStyledAt(int x, int y, const char *text, uint16_t fg, uint16_t bg, uint8_t textSize, uint8_t textFont) {
  tft.setTextColor(fg, bg);
  tft.setTextFont(textFont);
  tft.setTextSize(textSize);
  tft.setCursor(x, y);
  tft.print(text);
  restore_tft_text_style();
}

void uiDisplayPrintAt(int x, int y, const String &text, uint16_t fg, uint16_t bg, uint8_t textSize) {
  uiDisplayPrintStyledAt(x, y, text, fg, bg, textSize, 1);
}

void uiDisplayPrintStyledAt(int x, int y, const String &text, uint16_t fg, uint16_t bg, uint8_t textSize, uint8_t textFont) {
  tft.setTextColor(fg, bg);
  tft.setTextFont(textFont);
  tft.setTextSize(textSize);
  tft.setCursor(x, y);
  tft.print(text);
  restore_tft_text_style();
}
