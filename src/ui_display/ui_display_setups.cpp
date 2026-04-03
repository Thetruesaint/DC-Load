#include "../ui_display.h"
#include "ui_display_internal.h"

#include "../ui/ui_view_state.h"

using namespace ui_display_internal;

namespace ui_display_internal {
ManagedZoneLayout setup_managed_zone_layout() {
  return shared_managed_zone_layout();
}

ContentTextPanelLayout setup_content_text_panel_layout(const ManagedZoneLayout &layout) {
  return shared_content_text_panel_layout(layout);
}

void setup_draw_screen_base(const UiViewState &state, const char *modeLabel) {
  shared_setup_draw_screen_base(state, modeLabel);
}

void setup_draw_content_panel_title(const ManagedZoneLayout &layout, const ContentTextPanelLayout &panel, const String &title) {
  shared_draw_content_panel_title(layout, panel, title);
}

int setup_centered_content_text_x(const ManagedZoneLayout &layout, const String &text, uint8_t textSize, uint8_t textFont) {
  return shared_centered_content_text_x(layout, text, textSize, textFont);
}

void setup_draw_input_zone(const String &prefix, const String &valueText) {
  shared_draw_input_zone(prefix, valueText);
}

void setup_draw_zone_borders(const ManagedZoneLayout &layout) {
  shared_draw_zone_borders(layout);
}

void setup_draw_content_stage_line(const ContentTextPanelLayout &panel, const String &text, int x, int y, bool selected) {
  shared_draw_content_stage_line(panel, text, x, y, selected);
}

void setup_clear_blue_content_zone(const ManagedZoneLayout &layout) {
  shared_clear_blue_content_zone(layout);
}

String setup_format_transient_current(float currentA) {
  return String(currentA, 3) + "A";
}
}  // namespace ui_display_internal

void uiDisplayRenderBatterySetupTask(const UiViewState &state) {
  const ManagedZoneLayout layout = setup_managed_zone_layout();
  const ContentTextPanelLayout panel = setup_content_text_panel_layout(layout);
  const String title = "BATTERY SETUP";
  const int gap = layout.isLargeDisplay ? 34 : 18;
  const int leftColW = max(uiDisplayTextWidth("1 Stor Li-Po", panel.textSize, panel.textFont),
                           max(uiDisplayTextWidth("3 Disc Li-Po", panel.textSize, panel.textFont),
                               uiDisplayTextWidth("5 Custom Cutoff", panel.textSize, panel.textFont)));
  const int rightColW = max(uiDisplayTextWidth("2 Stor Li-Ion", panel.textSize, panel.textFont),
                            uiDisplayTextWidth("4 Disc Li-Ion", panel.textSize, panel.textFont));
  const int blockW = leftColW + gap + rightColW;
  const int leftX = (layout.displayW - blockW) / 2;
  const int rightX = leftX + leftColW + gap;

  setup_draw_screen_base(state, "BC");
  setup_draw_content_panel_title(layout, panel, title);
  uiDisplayPrintStyledAt(leftX, panel.row1Y, "1 Stor Li-Po", kUiText, kUiModeAreaBg, panel.textSize, panel.textFont);
  uiDisplayPrintStyledAt(rightX, panel.row1Y, "2 Stor Li-Ion", kUiText, kUiModeAreaBg, panel.textSize, panel.textFont);
  uiDisplayPrintStyledAt(leftX, panel.row2Y, "3 Disc Li-Po", kUiText, kUiModeAreaBg, panel.textSize, panel.textFont);
  uiDisplayPrintStyledAt(rightX, panel.row2Y, "4 Disc Li-Ion", kUiText, kUiModeAreaBg, panel.textSize, panel.textFont);
  uiDisplayPrintStyledAt(leftX, panel.row3Y, "5 Custom Cutoff", kUiText, kUiModeAreaBg, panel.textSize, panel.textFont);
  setup_draw_input_zone("SEL> ", "1..5");
  setup_draw_zone_borders(layout);
}

void uiDisplayRenderBatterySetupCustom(const UiViewState &state) {
  const ManagedZoneLayout layout = setup_managed_zone_layout();
  const ContentTextPanelLayout panel = setup_content_text_panel_layout(layout);
  const String title = "CUSTOM CUTOFF";
  const String line1 = String("Battery: ") + String(state.batteryType);
  const String line2 = "Enter cutoff voltage";
  const String line3 = "Range: 0.1v to 25.0v";

  setup_draw_screen_base(state, "BC");
  setup_draw_content_panel_title(layout, panel, title);
  uiDisplayPrintStyledAt(setup_centered_content_text_x(layout, line1, panel.textSize, panel.textFont), panel.row1Y, line1, kUiText, kUiModeAreaBg, panel.textSize, panel.textFont);
  uiDisplayPrintStyledAt(setup_centered_content_text_x(layout, line2, panel.textSize, panel.textFont), panel.row2Y, line2, kUiText, kUiModeAreaBg, panel.textSize, panel.textFont);
  uiDisplayPrintStyledAt(setup_centered_content_text_x(layout, line3, panel.textSize, panel.textFont), panel.row3Y, line3, kUiText, kUiModeAreaBg, panel.textSize, panel.textFont);
  uiDisplayUpdateBatterySetupCustomValue(state);
  setup_draw_zone_borders(layout);
}

void uiDisplayRenderBatterySetupCells(const UiViewState &state) {
  const ManagedZoneLayout layout = setup_managed_zone_layout();
  const ContentTextPanelLayout panel = setup_content_text_panel_layout(layout);
  const String title = "CELL COUNT";
  const String line1 = String("Battery: ") + String(state.batteryType);
  const String line2 = "Enter cells in series";
  const String line3 = "Range: 1 to 6";

  setup_draw_screen_base(state, "BC");
  setup_draw_content_panel_title(layout, panel, title);
  uiDisplayPrintStyledAt(setup_centered_content_text_x(layout, line1, panel.textSize, panel.textFont), panel.row1Y, line1, kUiText, kUiModeAreaBg, panel.textSize, panel.textFont);
  uiDisplayPrintStyledAt(setup_centered_content_text_x(layout, line2, panel.textSize, panel.textFont), panel.row2Y, line2, kUiText, kUiModeAreaBg, panel.textSize, panel.textFont);
  uiDisplayPrintStyledAt(setup_centered_content_text_x(layout, line3, panel.textSize, panel.textFont), panel.row3Y, line3, kUiText, kUiModeAreaBg, panel.textSize, panel.textFont);
  uiDisplayUpdateBatterySetupCellsValue(state);
  setup_draw_zone_borders(layout);
}

void uiDisplayUpdateBatterySetupCustomValue(const UiViewState &state) {
  const String setValue = (state.batteryInputText[0] != '\0') ? String(state.batteryInputText) + "v" : "";
  setup_draw_input_zone("SET> ", setValue);
}

void uiDisplayUpdateBatterySetupCellsValue(const UiViewState &state) {
  const String setValue = (state.batteryInputText[0] != '\0') ? String(state.batteryInputText) + "S" : "";
  setup_draw_input_zone("SEL> ", setValue);
}

void uiDisplayRenderTransientContSetup(const UiViewState &state) {
  const ManagedZoneLayout layout = setup_managed_zone_layout();
  const ContentTextPanelLayout panel = setup_content_text_panel_layout(layout);
  const String title = "TRANSIENT CONT";
  const String line1 = String("I1(A): ") + ((state.transientSetupStage > 0) ? setup_format_transient_current(state.transientLowCurrentA) : "--");
  const String line2 = String("I2(A): ") + ((state.transientSetupStage > 1) ? setup_format_transient_current(state.transientHighCurrentA) : "--");
  const String line3 = String("dt(ms): ") + ((state.transientSetupStage > 2) ? String(static_cast<unsigned long>(state.transientPeriodMs)) : "--");

  setup_draw_screen_base(state, "TC");
  setup_draw_content_panel_title(layout, panel, title);
  setup_draw_content_stage_line(panel, line1, setup_centered_content_text_x(layout, line1, panel.textSize, panel.textFont), panel.row1Y, state.transientSetupStage == 0);
  setup_draw_content_stage_line(panel, line2, setup_centered_content_text_x(layout, line2, panel.textSize, panel.textFont), panel.row2Y, state.transientSetupStage == 1);
  setup_draw_content_stage_line(panel, line3, setup_centered_content_text_x(layout, line3, panel.textSize, panel.textFont), panel.row3Y, state.transientSetupStage >= 2);
  uiDisplayUpdateTransientContSetupValue(state);
  setup_draw_zone_borders(layout);
}

void uiDisplayUpdateTransientContSetupContent(const UiViewState &state) {
  const ManagedZoneLayout layout = setup_managed_zone_layout();
  const ContentTextPanelLayout panel = setup_content_text_panel_layout(layout);
  const String title = "TRANSIENT CONT";
  const String line1 = String("I1(A): ") + ((state.transientSetupStage > 0) ? setup_format_transient_current(state.transientLowCurrentA) : "--");
  const String line2 = String("I2(A): ") + ((state.transientSetupStage > 1) ? setup_format_transient_current(state.transientHighCurrentA) : "--");
  const String line3 = String("dt(ms): ") + ((state.transientSetupStage > 2) ? String(static_cast<unsigned long>(state.transientPeriodMs)) : "--");

  setup_clear_blue_content_zone(layout);
  setup_draw_content_panel_title(layout, panel, title);
  setup_draw_content_stage_line(panel, line1, setup_centered_content_text_x(layout, line1, panel.textSize, panel.textFont), panel.row1Y, state.transientSetupStage == 0);
  setup_draw_content_stage_line(panel, line2, setup_centered_content_text_x(layout, line2, panel.textSize, panel.textFont), panel.row2Y, state.transientSetupStage == 1);
  setup_draw_content_stage_line(panel, line3, setup_centered_content_text_x(layout, line3, panel.textSize, panel.textFont), panel.row3Y, state.transientSetupStage >= 2);
}

void uiDisplayUpdateTransientContSetupValue(const UiViewState &state) {
  String suffix = "";
  if (state.transientSetupStage < 2) {
    suffix = " A";
  } else {
    suffix = " ms";
  }

  const String setValue = (state.transientInputText[0] != '\0') ? String(state.transientInputText) + suffix : "";
  setup_draw_input_zone("SET> ", setValue);
}

void uiDisplayRenderTransientListSetup(const UiViewState &state) {
  setup_draw_screen_base(state, "TL");
  uiDisplayUpdateTransientListSetupContent(state);
  uiDisplayUpdateTransientListSetupValue(state);
  setup_draw_zone_borders(setup_managed_zone_layout());
}

void uiDisplayUpdateTransientListSetupContent(const UiViewState &state) {
  const ManagedZoneLayout layout = setup_managed_zone_layout();
  const ContentTextPanelLayout panel = setup_content_text_panel_layout(layout);
  const String title = "TRANSIENT LIST";

  setup_clear_blue_content_zone(layout);
  setup_draw_content_panel_title(layout, panel, title);

  if (state.transientListSetupStage == 0) {
    const String line1 = "How many steps?";
    const String line2Prefix = "Allowed range: ";
    const String line2Value = "2 to 10";
    const String line2 = line2Prefix + line2Value;
    const int line1X = setup_centered_content_text_x(layout, line1, panel.textSize, panel.textFont);
    const int line2X = setup_centered_content_text_x(layout, line2, panel.textSize, panel.textFont);
    const int line2ValueX = line2X + uiDisplayTextWidth(line2Prefix, panel.textSize, panel.textFont);
    const int highlightPadX = layout.isLargeDisplay ? 10 : 6;
    const int highlightPadY = layout.isLargeDisplay ? 4 : 2;
    uiDisplayPrintStyledAt(line1X, panel.row1Y, line1, kUiText, kUiModeAreaBg, panel.textSize, panel.textFont);
    uiDisplayPrintStyledAt(line2X, panel.row2Y, line2, kUiText, kUiModeAreaBg, panel.textSize, panel.textFont);
    uiDisplayFillRect(line2ValueX - highlightPadX,
                      panel.row2Y - highlightPadY,
                      uiDisplayTextWidth(line2Value, panel.textSize, panel.textFont) + (highlightPadX * 2),
                      panel.textH + (highlightPadY * 2),
                      kUiHighlight);
    uiDisplayPrintStyledAt(line2ValueX, panel.row2Y, line2Value, kUiDark, kUiHighlight, panel.textSize, panel.textFont);
  } else {
    const int visibleStep = state.transientListDraftStepIndex + 1;
    const int totalSteps = max(2, static_cast<int>(state.transientListDraftStepCount));
    const String line1 = "Step " + String(visibleStep) + " of " + String(totalSteps);
    const String line2 = String("I(A): ") +
                         ((state.transientListDraftField > 0) ? setup_format_transient_current(state.transientListCurrentA) : "--");
    const String line3 = String("dt(ms): ") +
                         ((state.transientListCurrentPeriodMs > 0.0f)
                              ? String(static_cast<unsigned long>(state.transientListCurrentPeriodMs))
                              : "--");
    const int line1X = setup_centered_content_text_x(layout, line1, panel.textSize, panel.textFont);
    const int line2X = setup_centered_content_text_x(layout, line2, panel.textSize, panel.textFont);
    const int line3X = setup_centered_content_text_x(layout, line3, panel.textSize, panel.textFont);
    uiDisplayPrintStyledAt(line1X, panel.row1Y, line1, kUiHighlight, kUiModeAreaBg, panel.textSize, panel.textFont);
    setup_draw_content_stage_line(panel, line2, line2X, panel.row2Y, state.transientListDraftField == 0);
    setup_draw_content_stage_line(panel, line3, line3X, panel.row3Y, state.transientListDraftField != 0);
  }
}

void uiDisplayUpdateTransientListSetupValue(const UiViewState &state) {
  String suffix = "";
  if (state.transientListSetupStage == 0) {
    suffix = "";
  } else if (state.transientListDraftField == 0) {
    suffix = " A";
  } else {
    suffix = " ms";
  }

  const String setValue = (state.transientListInputText[0] != '\0') ? String(state.transientListInputText) + suffix : "";
  const String prefix = (state.transientListSetupStage == 0) ? "STEPS> " : "SET> ";
  setup_draw_input_zone(prefix, setValue);
}
