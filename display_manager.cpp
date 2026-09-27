// ============================================================
// display_manager.cpp — ILI9341 240x320 portrait, 4 pages, touch nav
// ============================================================
#include "display_manager.h"
#include <TFT_eSPI.h>
#include <XPT2046_Touchscreen.h>
#include <SPI.h>

// TFT_eSPI reads pins from the library's User_Setup.h (see User_Setup.h
// shipped alongside this sketch — copy it into the TFT_eSPI library
// folder). Touch uses its own SPIClass instance on the same VSPI bus.
static TFT_eSPI tft = TFT_eSPI();
static SPIClass s_touchSPI(VSPI);
static XPT2046_Touchscreen ts(PIN_TOUCH_CS, PIN_TOUCH_IRQ);

// Raw ADC calibration for the touch panel (typical XPT2046 range).
#define TOUCH_RAW_MIN_X 300
#define TOUCH_RAW_MAX_X 3800
#define TOUCH_RAW_MIN_Y 300
#define TOUCH_RAW_MAX_Y 3800

#define COLOR_BG      TFT_BLACK
#define COLOR_CARD    TFT_NAVY
#define COLOR_TEXT    TFT_WHITE
#define COLOR_OK      TFT_GREEN
#define COLOR_WARN    TFT_YELLOW
#define COLOR_DANGER  TFT_RED
#define COLOR_TAB_ON  0x4ECA
#define COLOR_TAB_OFF 0x2124

static LcdPage s_page = PAGE_STATUS;
static LcdPage s_lastDrawnPage = PAGE_COUNT; // force full redraw first frame
static LcdCommandResult s_pending;

struct Rect { int16_t x, y, w, h; };
static Rect TAB_RECTS[4] = {
  {0, 290, 60, 30}, {60, 290, 60, 30}, {120, 290, 60, 30}, {180, 290, 60, 30}
};
static const char *TAB_LABELS[4] = {"STATUS", "PID", "EFFIC", "TEST"};

// Page 1 action buttons
static Rect BTN_START = {5, 236, 74, 40};
static Rect BTN_STOP  = {83, 236, 74, 40};
static Rect BTN_TUNE  = {161, 236, 74, 40};

// Page 2 stepper buttons (setpoint, kp/ki/kd nudge, t_shutdown, target vol)
static Rect BTN_SP_MINUS  = {180, 40, 28, 28};
static Rect BTN_SP_PLUS   = {212, 40, 28, 28};
static Rect BTN_TS_MINUS  = {180, 190, 28, 28};
static Rect BTN_TS_PLUS   = {212, 190, 28, 28};
static Rect BTN_VOL_MINUS = {180, 220, 28, 28};
static Rect BTN_VOL_PLUS  = {212, 220, 28, 28};

// Page 4 test-mode buttons
static Rect BTN_FAN_MINUS  = {20, 80, 40, 40};
static Rect BTN_FAN_PLUS   = {180, 80, 40, 40};
static Rect BTN_OIL_ON     = {10, 150, 100, 40};
static Rect BTN_OIL_OFF    = {120, 150, 100, 40};
static Rect BTN_WATER_ON   = {10, 210, 100, 40};
static Rect BTN_WATER_OFF  = {120, 210, 100, 40};

static uint8_t s_testFanPct = 0;
static LcdData s_lastData; // cached snapshot for touch handlers that need current values

static bool touchInRect(int16_t x, int16_t y, const Rect &r) {
  return x >= r.x && x <= r.x + r.w && y >= r.y && y <= r.y + r.h;
}

static void drawButton(const Rect &r, const char *label, uint16_t color) {
  tft.fillRoundRect(r.x, r.y, r.w, r.h, 6, color);
  tft.drawRoundRect(r.x, r.y, r.w, r.h, 6, TFT_WHITE);
  tft.setTextColor(TFT_WHITE, color);
  tft.setTextDatum(MC_DATUM);
  tft.drawString(label, r.x + r.w / 2, r.y + r.h / 2, 2);
  tft.setTextDatum(TL_DATUM);
}

static void drawTabs() {
  for (int i = 0; i < 4; i++) {
    uint16_t c = (i == s_page) ? COLOR_TAB_ON : COLOR_TAB_OFF;
    tft.fillRect(TAB_RECTS[i].x, TAB_RECTS[i].y, TAB_RECTS[i].w, TAB_RECTS[i].h, c);
    tft.drawRect(TAB_RECTS[i].x, TAB_RECTS[i].y, TAB_RECTS[i].w, TAB_RECTS[i].h, TFT_WHITE);
    tft.setTextColor(TFT_WHITE, c);
    tft.setTextDatum(MC_DATUM);
    tft.drawString(TAB_LABELS[i], TAB_RECTS[i].x + TAB_RECTS[i].w / 2,
                    TAB_RECTS[i].y + TAB_RECTS[i].h / 2, 1);
  }
  tft.setTextDatum(TL_DATUM);
}

static void drawHeader(const char *title) {
  tft.fillRect(0, 0, 240, 28, COLOR_CARD);
  tft.setTextColor(COLOR_TEXT, COLOR_CARD);
  tft.setTextDatum(MC_DATUM);
  tft.drawString(title, 120, 14, 2);
  tft.setTextDatum(TL_DATUM);
}

void display_init() {
  tft.init();
  tft.setRotation(0); // portrait 240x320
  tft.fillScreen(COLOR_BG);

  s_touchSPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI, PIN_TOUCH_CS);
  ts.begin(s_touchSPI);
  ts.setRotation(0);

  s_lastDrawnPage = PAGE_COUNT;
  Serial.println(F("[display] ILI9341 + XPT2046 initialized"));
}

// ------------------------------------------------------------
// PAGE 1 — STATUS
// ------------------------------------------------------------
static float s_lastSuhu = -1000, s_lastSp = -1000, s_lastEff = -1000, s_lastVol = -1000;
static uint8_t s_lastFanPct = 255;
static SystemState s_lastState = (SystemState)255;
static unsigned long s_lastDurationS = 0xFFFFFFFF;
static float s_lastIse = -1;
static uint32_t s_lastRunCount = 0xFFFFFFFF;

static const char *stateName(SystemState s) {
  switch (s) {
    case STATE_IDLE: return "IDLE";
    case STATE_AUTOTUNE: return "AUTOTUNE";
    case STATE_RUNNING: return "RUNNING";
    case STATE_SHUTDOWN: return "SHUTDOWN";
    case STATE_LEARNING: return "LEARNING";
  }
  return "?";
}

static void drawPage1Static() {
  drawHeader("PYROLYSIS CONTROLLER");
  tft.fillRect(0, 28, 240, 262, COLOR_BG);

  tft.fillRoundRect(5, 33, 230, 65, 6, COLOR_CARD);
  tft.fillRoundRect(5, 102, 230, 65, 6, COLOR_CARD);
  tft.fillRoundRect(5, 171, 230, 62, 6, COLOR_CARD);

  drawButton(BTN_START, "START", COLOR_OK);
  drawButton(BTN_STOP, "STOP", COLOR_DANGER);
  drawButton(BTN_TUNE, "TUNE", COLOR_WARN);

  s_lastSuhu = s_lastSp = s_lastEff = s_lastVol = -1000;
  s_lastFanPct = 255;
  s_lastState = (SystemState)255;
  s_lastDurationS = 0xFFFFFFFF;
  s_lastIse = -1;
  s_lastRunCount = 0xFFFFFFFF;
}

static void drawValueField(int16_t x, int16_t y, int16_t w, const char *text, uint16_t color) {
  tft.fillRect(x, y, w, 18, COLOR_CARD);
  tft.setTextColor(color, COLOR_CARD);
  tft.drawString(text, x, y, 2);
}

static void updatePage1(const LcdData &d) {
  char buf[40];

  if (fabs(d.suhu - s_lastSuhu) > 0.05f) {
    uint16_t c = (d.suhu >= TEMP_EMERGENCY - 50) ? COLOR_DANGER :
                 (d.suhu > d.setpoint + 20) ? COLOR_WARN : COLOR_OK;
    snprintf(buf, sizeof(buf), "SUHU  : %.1f C", d.suhu);
    drawValueField(12, 40, 220, buf, c);
    s_lastSuhu = d.suhu;
  }
  if (fabs(d.setpoint - s_lastSp) > 0.05f) {
    snprintf(buf, sizeof(buf), "TARGET: %.1f C", d.setpoint);
    drawValueField(12, 60, 220, buf, COLOR_TEXT);
    s_lastSp = d.setpoint;
  }
  if (d.state != s_lastState) {
    uint16_t c = (d.state == STATE_RUNNING) ? COLOR_OK :
                 (d.state == STATE_SHUTDOWN) ? COLOR_DANGER : COLOR_WARN;
    snprintf(buf, sizeof(buf), "STATUS: %s", stateName(d.state));
    drawValueField(12, 80, 220, buf, c);
    s_lastState = d.state;
  }

  if (d.fan_pct != s_lastFanPct) {
    snprintf(buf, sizeof(buf), "Fan PWM : %u%%", d.fan_pct);
    drawValueField(12, 109, 220, buf, COLOR_TEXT);
    s_lastFanPct = d.fan_pct;
  }
  if (fabs(d.volume_ml - s_lastVol) > 0.05f) {
    snprintf(buf, sizeof(buf), "Volume  : %.1f mL", d.volume_ml);
    drawValueField(12, 129, 220, buf, COLOR_TEXT);
    s_lastVol = d.volume_ml;
  }
  if (fabs(d.efficiency_ml_h - s_lastEff) > 0.5f) {
    snprintf(buf, sizeof(buf), "Efisien.: %.0f mL/jam", d.efficiency_ml_h);
    drawValueField(12, 149, 220, buf, COLOR_TEXT);
    s_lastEff = d.efficiency_ml_h;
  }

  unsigned long durS = d.duration_ms / 1000;
  if (durS != s_lastDurationS) {
    unsigned long h = durS / 3600, m = (durS % 3600) / 60, s = durS % 60;
    snprintf(buf, sizeof(buf), "Durasi  : %02lu:%02lu:%02lu", h, m, s);
    drawValueField(12, 178, 220, buf, COLOR_TEXT);
    s_lastDurationS = durS;
  }
  if (fabs(d.ise - s_lastIse) > 0.5f) {
    snprintf(buf, sizeof(buf), "ISE     : %.1f", d.ise);
    drawValueField(12, 198, 220, buf, COLOR_TEXT);
    s_lastIse = d.ise;
  }
  if (d.run_count != s_lastRunCount) {
    snprintf(buf, sizeof(buf), "Run #%lu", (unsigned long)d.run_count);
    drawValueField(12, 218, 220, buf, COLOR_TEXT);
    s_lastRunCount = d.run_count;
  }
}

// ------------------------------------------------------------
// PAGE 2 — PID SETTINGS
// ------------------------------------------------------------
static void drawPage2Static() {
  drawHeader("PID SETTINGS");
  tft.fillRect(0, 28, 240, 262, COLOR_BG);

  drawButton(BTN_SP_MINUS, "-", COLOR_TAB_OFF);
  drawButton(BTN_SP_PLUS, "+", COLOR_TAB_OFF);
  drawButton(BTN_TS_MINUS, "-", COLOR_TAB_OFF);
  drawButton(BTN_TS_PLUS, "+", COLOR_TAB_OFF);
  drawButton(BTN_VOL_MINUS, "-", COLOR_TAB_OFF);
  drawButton(BTN_VOL_PLUS, "+", COLOR_TAB_OFF);
}

static void updatePage2(const LcdData &d) {
  char buf[48];
  tft.setTextColor(COLOR_TEXT, COLOR_BG);

  tft.fillRect(10, 35, 165, 60, COLOR_BG);
  snprintf(buf, sizeof(buf), "Setpoint: %.1f C", d.setpoint);
  tft.drawString(buf, 10, 40, 2);

  int runsForSp = -1;
  for (uint8_t i = 0; i < d.eff_count; i++) {
    if (fabs(d.eff_setpoint[i] - d.setpoint) < 0.01f) { runsForSp = d.eff_runs[i]; break; }
  }
  snprintf(buf, sizeof(buf), "(%d runs tercatat)", runsForSp < 0 ? 0 : runsForSp);
  tft.drawString(buf, 10, 62, 2);

  tft.fillRect(10, 90, 220, 90, COLOR_BG);
  snprintf(buf, sizeof(buf), "Kp: %.3f", d.kp); tft.drawString(buf, 10, 90, 2);
  snprintf(buf, sizeof(buf), "Ki: %.4f", d.ki); tft.drawString(buf, 10, 112, 2);
  snprintf(buf, sizeof(buf), "Kd: %.3f", d.kd); tft.drawString(buf, 10, 134, 2);

  tft.fillRect(10, 165, 165, 30, COLOR_BG);
  snprintf(buf, sizeof(buf), "T shutdown: %.1f C", d.t_shutdown);
  tft.drawString(buf, 10, 168, 2);

  tft.fillRect(10, 195, 165, 30, COLOR_BG);
  snprintf(buf, sizeof(buf), "Target vol: %.0f mL", d.target_volume_ml);
  tft.drawString(buf, 10, 198, 2);

  tft.fillRect(10, 250, 220, 30, COLOR_BG);
  snprintf(buf, sizeof(buf), "w1=%.2f w2=%.2f w3=%.2f", d.w1, d.w2, d.w3);
  tft.drawString(buf, 10, 254, 2);
}

// ------------------------------------------------------------
// PAGE 3 — EFFICIENCY MAP
// ------------------------------------------------------------
static void drawPage3Static() {
  drawHeader("EFFICIENCY MAP");
  tft.fillRect(0, 28, 240, 262, COLOR_BG);
  tft.setTextColor(COLOR_TEXT, COLOR_BG);
  tft.drawString("SP", 10, 34, 2);
  tft.drawString("Runs", 60, 34, 2);
  tft.drawString("Eff mL/h", 110, 34, 2);
  tft.drawString("Score", 180, 34, 2);
  tft.drawFastHLine(5, 52, 230, COLOR_TEXT);
}

static void updatePage3(const LcdData &d) {
  tft.fillRect(0, 55, 240, 220, COLOR_BG);
  int y = 58;
  for (uint8_t i = 0; i < d.eff_count && i < 8; i++) {
    bool rec = fabs(d.eff_setpoint[i] - d.recommended_setpoint) < 0.01f;
    uint16_t rowColor = rec ? COLOR_OK : COLOR_TEXT;
    char b1[12], b2[8], b3[12], b4[12];
    snprintf(b1, sizeof(b1), "%.0f", d.eff_setpoint[i]);
    snprintf(b2, sizeof(b2), "%u", d.eff_runs[i]);
    snprintf(b3, sizeof(b3), "%.0f", d.eff_efficiency[i]);
    snprintf(b4, sizeof(b4), "%.0f", d.eff_score[i]);
    tft.setTextColor(rowColor, COLOR_BG);
    tft.drawString(b1, 10, y, 2);
    tft.drawString(b2, 60, y, 2);
    tft.drawString(b3, 110, y, 2);
    tft.drawString(b4, 180, y, 2);
    y += 22;
  }
}

// ------------------------------------------------------------
// PAGE 4 — TEST MODE
// ------------------------------------------------------------
static void drawPage4Static() {
  drawHeader("TEST MODE");
  tft.fillRect(0, 28, 240, 262, COLOR_BG);

  drawButton(BTN_FAN_MINUS, "-", COLOR_TAB_OFF);
  drawButton(BTN_FAN_PLUS, "+", COLOR_TAB_OFF);
  drawButton(BTN_OIL_ON, "OIL ON", COLOR_OK);
  drawButton(BTN_OIL_OFF, "OIL OFF", COLOR_DANGER);
  drawButton(BTN_WATER_ON, "WATER ON", COLOR_OK);
  drawButton(BTN_WATER_OFF, "WATER OFF", COLOR_DANGER);
}

static void updatePage4(const LcdData &d) {
  char buf[40];
  tft.fillRect(60, 90, 120, 24, COLOR_BG);
  snprintf(buf, sizeof(buf), "Fan: %u%%", s_testFanPct);
  tft.setTextColor(COLOR_TEXT, COLOR_BG);
  tft.setTextDatum(MC_DATUM);
  tft.drawString(buf, 120, 100, 4);
  tft.setTextDatum(TL_DATUM);

  tft.fillRect(10, 260, 220, 20, COLOR_BG);
  snprintf(buf, sizeof(buf), "Suhu: %.1f C | Oil:%s Water:%s",
           d.suhu, d.oil_pump ? "ON" : "OFF", d.water_pump ? "ON" : "OFF");
  tft.drawString(buf, 10, 260, 1);
}

// ------------------------------------------------------------
// Public API
// ------------------------------------------------------------
void display_update(const LcdData &d) {
  s_lastData = d;

  if (s_page != s_lastDrawnPage) {
    switch (s_page) {
      case PAGE_STATUS: drawPage1Static(); break;
      case PAGE_PID_SETTINGS: drawPage2Static(); break;
      case PAGE_EFFICIENCY: drawPage3Static(); break;
      case PAGE_TEST_MODE: drawPage4Static(); break;
      default: break;
    }
    drawTabs();
    s_lastDrawnPage = s_page;
  }

  switch (s_page) {
    case PAGE_STATUS: updatePage1(d); break;
    case PAGE_PID_SETTINGS: updatePage2(d); break;
    case PAGE_EFFICIENCY: updatePage3(d); break;
    case PAGE_TEST_MODE: updatePage4(d); break;
    default: break;
  }

  if (!d.sd_ok) {
    tft.fillRect(0, 0, 10, 10, COLOR_DANGER);
  }

  if (d.suhu >= TEMP_EMERGENCY) {
    tft.fillRect(0, 0, 240, 28, COLOR_DANGER);
    tft.setTextColor(TFT_WHITE, COLOR_DANGER);
    tft.setTextDatum(MC_DATUM);
    tft.drawString("EMERGENCY", 120, 14, 4);
    tft.setTextDatum(TL_DATUM);
  }
}

static unsigned long s_lastTouchMs = 0;
#define TOUCH_DEBOUNCE_MS 200

void display_poll_touch() {
  if (millis() - s_lastTouchMs < TOUCH_DEBOUNCE_MS) return;
  if (!ts.tirqTouched() || !ts.touched()) return;
  s_lastTouchMs = millis();

  TS_Point p = ts.getPoint();
  int16_t x = map(p.x, TOUCH_RAW_MIN_X, TOUCH_RAW_MAX_X, 0, 240);
  int16_t y = map(p.y, TOUCH_RAW_MIN_Y, TOUCH_RAW_MAX_Y, 0, 320);
  x = constrain(x, 0, 240);
  y = constrain(y, 0, 320);

  // Tab bar navigation (always active)
  for (int i = 0; i < 4; i++) {
    if (touchInRect(x, y, TAB_RECTS[i])) {
      s_page = (LcdPage)i;
      return;
    }
  }

  switch (s_page) {
    case PAGE_STATUS:
      if (touchInRect(x, y, BTN_START)) s_pending = {LCD_CMD_START, 0};
      else if (touchInRect(x, y, BTN_STOP)) s_pending = {LCD_CMD_STOP, 0};
      else if (touchInRect(x, y, BTN_TUNE)) s_pending = {LCD_CMD_AUTOTUNE, 0};
      break;

    case PAGE_PID_SETTINGS:
      if (touchInRect(x, y, BTN_SP_MINUS)) {
        s_pending = {LCD_CMD_SET_SETPOINT, s_lastData.setpoint - 10.0f};
      } else if (touchInRect(x, y, BTN_SP_PLUS)) {
        s_pending = {LCD_CMD_SET_SETPOINT, s_lastData.setpoint + 10.0f};
      } else if (touchInRect(x, y, BTN_TS_MINUS)) {
        s_pending = {LCD_CMD_SET_TSHUTDOWN, s_lastData.t_shutdown - 5.0f};
      } else if (touchInRect(x, y, BTN_TS_PLUS)) {
        s_pending = {LCD_CMD_SET_TSHUTDOWN, s_lastData.t_shutdown + 5.0f};
      } else if (touchInRect(x, y, BTN_VOL_MINUS)) {
        s_pending = {LCD_CMD_SET_TARGET_VOL, s_lastData.target_volume_ml - 100.0f};
      } else if (touchInRect(x, y, BTN_VOL_PLUS)) {
        s_pending = {LCD_CMD_SET_TARGET_VOL, s_lastData.target_volume_ml + 100.0f};
      }
      break;

    case PAGE_TEST_MODE:
      if (touchInRect(x, y, BTN_FAN_MINUS)) {
        if (s_testFanPct >= 5) s_testFanPct -= 5;
        s_pending = {LCD_CMD_TEST_FAN, (float)map(s_testFanPct, 0, 100, 0, 255)};
      } else if (touchInRect(x, y, BTN_FAN_PLUS)) {
        if (s_testFanPct <= 95) s_testFanPct += 5;
        s_pending = {LCD_CMD_TEST_FAN, (float)map(s_testFanPct, 0, 100, 0, 255)};
      } else if (touchInRect(x, y, BTN_OIL_ON)) {
        s_pending = {LCD_CMD_TEST_OIL_ON, 0};
      } else if (touchInRect(x, y, BTN_OIL_OFF)) {
        s_pending = {LCD_CMD_TEST_OIL_OFF, 0};
      } else if (touchInRect(x, y, BTN_WATER_ON)) {
        s_pending = {LCD_CMD_TEST_WATER_ON, 0};
      } else if (touchInRect(x, y, BTN_WATER_OFF)) {
        s_pending = {LCD_CMD_TEST_WATER_OFF, 0};
      }
      break;

    default:
      break;
  }
}

LcdCommandResult display_get_pending_command() {
  LcdCommandResult r = s_pending;
  s_pending = LcdCommandResult();
  return r;
}
