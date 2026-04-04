#include "../ui_display.h"
#include "ui_display_internal.h"

#include "../app/app_ota.h"
#include "../hw/hw_objects.h"

namespace {
using namespace ui_display_internal;

bool g_fwUpdateLayoutDrawn = false;
int g_fwUpdateLastDisplayW = 0;
int g_fwUpdateLastDisplayH = 0;
String g_fwUpdateLastStatus;
String g_fwUpdateLastDetail;
String g_fwUpdateLastHint;
}

void uiDisplayInvalidateFwUpdateLayout(void) {
  g_fwUpdateLayoutDrawn = false;
  g_fwUpdateLastDisplayW = 0;
  g_fwUpdateLastDisplayH = 0;
  g_fwUpdateLastStatus = "";
  g_fwUpdateLastDetail = "";
  g_fwUpdateLastHint = "";
}

void uiDisplayRenderFwUpdateScreen(const char *statusLine, const char *detailLine, const char *hintLine) {
  const int displayW = uiDisplayWidthPx();
  const int displayH = uiDisplayHeightPx();
  const bool isLargeDisplay = displayW >= 400;
  const uint8_t titleSize = isLargeDisplay ? 2 : 1;
  const uint8_t textSize = isLargeDisplay ? 2 : 1;
  const uint8_t font = 2;
  const int outerPad = isLargeDisplay ? 10 : 6;
  const int panelX = outerPad;
  const int panelY = outerPad;
  const int panelW = displayW - (outerPad * 2);
  const int panelH = displayH - (outerPad * 2);
  const int lineHeight = uiDisplayFontHeight(textSize, font);
  const int titleHeight = uiDisplayFontHeight(titleSize, font);
  const int gap = isLargeDisplay ? 12 : 8;
  const String status = statusLine ? String(statusLine) : "";
  const String detail = detailLine ? String(detailLine) : "";
  const String hint = hintLine ? String(hintLine) : "";
  const bool uploading = app_ota_is_uploading();
  const bool error = app_ota_has_error();
  const int totalTextHeight = titleHeight + (lineHeight * 3) + (gap * 3);
  const int titleY = panelY + max(isLargeDisplay ? 12 : 8, (panelH - totalTextHeight) / 2);
  const int startY = titleY + titleHeight + gap;
  const bool layoutChanged = !g_fwUpdateLayoutDrawn || g_fwUpdateLastDisplayW != displayW || g_fwUpdateLastDisplayH != displayH;

  if (layoutChanged) {
    uiDisplayClear();
    uiDisplayFillRect(panelX, panelY, panelW, panelH, kUiModeAreaBg);
    uiDisplayDrawRect(panelX, panelY, panelW, panelH, kUiBorder);
    const String title = uploading ? "OTA ACTIVE" : "FW UPDATE";
    uiDisplayPrintStyledAt((displayW - uiDisplayTextWidth(title, titleSize, font)) / 2,
                           titleY,
                           title,
                           uploading ? kUiAlertText : kUiHighlight,
                           kUiModeAreaBg,
                           titleSize,
                           font);
    g_fwUpdateLayoutDrawn = true;
    g_fwUpdateLastDisplayW = displayW;
    g_fwUpdateLastDisplayH = displayH;
    g_fwUpdateLastStatus = "";
    g_fwUpdateLastDetail = "";
    g_fwUpdateLastHint = "";
  }

  const int textBlockX = panelX + 10;
  const int textBlockW = panelW - 20;
  const int statusY = startY;
  const int detailY = startY + (lineHeight + gap);
  const int hintY = startY + ((lineHeight + gap) * 2);

  if (g_fwUpdateLastStatus != status) {
    uiDisplayFillRect(textBlockX, statusY - 3, textBlockW, lineHeight + 8, kUiModeAreaBg);
    if (uploading || error) {
      const int statusW = uiDisplayTextWidth(status, textSize, font);
      const int statusX = (displayW - statusW) / 2;
      const int statusH = uiDisplayFontHeight(textSize, font);
      const uint16_t statusBg = error ? kUiAlertBg : tft.color565(180, 0, 0);
      uiDisplayFillRect(statusX - 6, statusY - 2, statusW + 12, statusH + 4, statusBg);
      uiDisplayPrintStyledAt(statusX, statusY, status, kUiAlertText, statusBg, textSize, font);
    } else {
      uiDisplayPrintStyledAt((displayW - uiDisplayTextWidth(status, textSize, font)) / 2,
                             statusY,
                             status,
                             kUiText,
                             kUiModeAreaBg,
                             textSize,
                             font);
    }
    g_fwUpdateLastStatus = status;
  }

  if (g_fwUpdateLastDetail != detail) {
    uiDisplayFillRect(textBlockX, detailY - 3, textBlockW, lineHeight + 8, kUiModeAreaBg);
    uiDisplayPrintStyledAt((displayW - uiDisplayTextWidth(detail, textSize, font)) / 2,
                           detailY,
                           detail,
                           kUiText,
                           kUiModeAreaBg,
                           textSize,
                           font);
    g_fwUpdateLastDetail = detail;
  }

  if (g_fwUpdateLastHint != hint) {
    uiDisplayFillRect(textBlockX, hintY - 3, textBlockW, lineHeight + 8, kUiModeAreaBg);
    const uint16_t hintBg = (uploading || error) ? kUiHighlight : kUiModeAreaBg;
    const uint16_t hintFg = (uploading || error) ? kUiDark : kUiText;
    uiDisplayPrintStyledAt((displayW - uiDisplayTextWidth(hint, textSize, font)) / 2,
                           hintY,
                           hint,
                           hintFg,
                           hintBg,
                           textSize,
                           font);
    g_fwUpdateLastHint = hint;
  }
}
