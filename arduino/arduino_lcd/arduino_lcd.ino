#include <MCUFRIEND_kbv.h>
#include <Adafruit_GFX.h>
#include <SoftwareSerial.h>
#include <ArduinoJson.h>

// A4 = RX (terima dari ESP32 GPIO17/TX2)
// A5 = TX (kirim ke ESP32 GPIO16/RX2)
// Pakai A4/A5 karena D2/D3 dipakai LCD shield data bus
SoftwareSerial espSerial(A5, A4);

MCUFRIEND_kbv tft;

#define COLOR_BLACK   0x0000
#define COLOR_WHITE   0xFFFF
#define COLOR_NAVY    0x000F
#define COLOR_RED     0xF800
#define COLOR_GREEN   0x07E0
#define COLOR_BLUE    0x001F
#define COLOR_YELLOW  0xFFE0
#define COLOR_GREY    0x8410

#define VOLUME_PER_TIP_ML 1.42
#define LINE_BUF_SIZE 200

static char lineBuf[LINE_BUF_SIZE];
static uint8_t lineIdx = 0;

static float  prevTemp     = -9999.0;
static float  prevSetpoint = -9999.0;
static float  prevVolume   = -9999.0;
static int    prevTips     = -1;
static int    prevFanPct   = -1;
static int    prevOil      = -1;
static int    prevWater    = -1;
static String prevStatus   = "";

#define FAN_BAR_X  80
#define FAN_BAR_Y  142
#define FAN_BAR_W  150
#define FAN_BAR_H  16

void drawHeader() {
  tft.fillRect(0, 0, 320, 35, COLOR_NAVY);
  tft.setTextColor(COLOR_WHITE);
  tft.setTextSize(2);
  tft.setCursor(10, 10);
  tft.print("CONTROL PYROLYSIS ILUL");
}

void drawStaticLabels() {
  tft.setTextColor(COLOR_GREY);
  tft.setTextSize(2);

  tft.setCursor(10, 45);  tft.print("SUHU     :");
  tft.setCursor(10, 75);  tft.print("SETPOINT :");
  tft.setCursor(10, 105); tft.print("VOLUME   :");

  tft.drawFastHLine(0, 130, 320, COLOR_GREY);

  tft.setCursor(10, 140);  tft.print("FAN :");
  tft.setCursor(10, 170);  tft.print("OIL  :");
  tft.setCursor(160, 170); tft.print("WATER:");
  tft.setCursor(10, 200);  tft.print("STATUS:");

  tft.drawRect(FAN_BAR_X, FAN_BAR_Y, FAN_BAR_W, FAN_BAR_H, COLOR_GREY);
}

void drawDefaultValues() {
  tft.setTextColor(COLOR_WHITE);
  tft.setTextSize(2);
  tft.setCursor(140, 45);  tft.print("--");
  tft.setCursor(140, 75);  tft.print("--");
  tft.setCursor(140, 105); tft.print("--");
  tft.setCursor(240, 140); tft.print("--");
}

void updateTemp(float t, float sp, bool spChanged) {
  bool tempChanged = (fabs(t - prevTemp) > 0.05);
  if (!tempChanged && !spChanged) return;

  tft.fillRect(140, 45, 170, 18, COLOR_BLACK);
  uint16_t color = COLOR_WHITE;
  if (t > sp)      color = COLOR_RED;
  else if (t < sp) color = COLOR_GREEN;

  tft.setTextColor(color);
  tft.setTextSize(2);
  tft.setCursor(140, 45);
  tft.print(t, 1);
  tft.print(" C");
  prevTemp = t;
}

void updateSetpoint(float sp) {
  if (fabs(sp - prevSetpoint) <= 0.05) return;
  tft.fillRect(140, 75, 170, 18, COLOR_BLACK);
  tft.setTextColor(COLOR_WHITE);
  tft.setTextSize(2);
  tft.setCursor(140, 75);
  tft.print(sp, 1);
  tft.print(" C");
  prevSetpoint = sp;
}

void updateVolume(float v) {
  int tips = (int)round(v / VOLUME_PER_TIP_ML);
  if (fabs(v - prevVolume) <= 0.05 && tips == prevTips) return;

  tft.fillRect(140, 105, 170, 28, COLOR_BLACK);
  tft.setTextColor(COLOR_WHITE);
  tft.setTextSize(2);
  tft.setCursor(140, 105);
  tft.print(v, 1);
  tft.print(" mL");

  tft.setTextSize(1);
  tft.setCursor(140, 122);
  tft.print("(");
  tft.print(tips);
  tft.print(" tips)");

  prevVolume = v;
  prevTips   = tips;
}

void updateFan(int pct) {
  if (pct == prevFanPct) return;

  tft.fillRect(FAN_BAR_X + 1, FAN_BAR_Y + 1,
               FAN_BAR_W - 2, FAN_BAR_H - 2, COLOR_BLACK);
  int barLen = map(pct, 0, 100, 0, FAN_BAR_W - 2);
  if (barLen > 0) {
    tft.fillRect(FAN_BAR_X + 1, FAN_BAR_Y + 1,
                 barLen, FAN_BAR_H - 2, COLOR_GREEN);
  }

  tft.fillRect(240, 140, 70, 18, COLOR_BLACK);
  tft.setTextColor(COLOR_WHITE);
  tft.setTextSize(2);
  tft.setCursor(240, 140);
  tft.print(pct);
  tft.print("%");

  prevFanPct = pct;
}

void drawBadge(int x, int y, int w, int h,
               bool on, const char *onText, const char *offText) {
  uint16_t bg = on ? COLOR_GREEN : COLOR_RED;
  uint16_t fg = on ? COLOR_BLACK : COLOR_WHITE;
  tft.fillRect(x, y, w, h, bg);
  tft.setTextColor(fg);
  tft.setTextSize(2);
  tft.setCursor(x + 6, y + 4);
  tft.print(on ? onText : offText);
}

void updateOil(bool oil) {
  int v = oil ? 1 : 0;
  if (v == prevOil) return;
  drawBadge(70, 165, 80, 24, oil, " ON ", " OFF");
  prevOil = v;
}

void updateWater(bool water) {
  int v = water ? 1 : 0;
  if (v == prevWater) return;
  drawBadge(230, 165, 80, 24, water, " ON ", " OFF");
  prevWater = v;
}

void updateStatus(const String &s) {
  if (s == prevStatus) return;

  uint16_t bg, fg;
  if (s == "RUN")       { bg = COLOR_GREEN;  fg = COLOR_BLACK; }
  else if (s == "IDLE") { bg = COLOR_BLUE;   fg = COLOR_WHITE; }
  else                  { bg = COLOR_YELLOW; fg = COLOR_BLACK; }

  tft.fillRect(100, 195, 200, 26, COLOR_BLACK);
  tft.fillRect(100, 195, 150, 26, bg);
  tft.setTextColor(fg);
  tft.setTextSize(2);
  tft.setCursor(108, 202);
  tft.print(s);
  prevStatus = s;
}

void processLine(char *line) {
  size_t len = strlen(line);
  if (len < 2) return;

  if (line[0] != '{' || line[len - 1] != '}') {
    Serial.print(F("[warn] buang: "));
    Serial.println(line);
    return;
  }

  StaticJsonDocument<200> doc;
  DeserializationError err = deserializeJson(doc, line);
  if (err) {
    Serial.print(F("[warn] JSON error: "));
    Serial.println(err.c_str());
    return;
  }

  float      t   = doc["t"]     | 0.0;
  float      sp  = doc["sp"]    | 0.0;
  int        pw  = doc["pw"]    | 0;
  float      v   = doc["v"]     | 0.0;
  int        oil = doc["oil"]   | 0;
  int        wat = doc["water"] | 0;
  const char *s  = doc["s"]     | "IDLE";

  Serial.print(F("[ok] t="));  Serial.print(t);
  Serial.print(F(" sp="));     Serial.print(sp);
  Serial.print(F(" pw="));     Serial.print(pw);
  Serial.print(F(" oil="));    Serial.print(oil);
  Serial.print(F(" water="));  Serial.println(wat);

  bool spChanged = (fabs(sp - prevSetpoint) > 0.05);
  updateTemp(t, sp, spChanged);
  updateSetpoint(sp);
  updateVolume(v);
  updateFan(pw);
  updateOil(oil != 0);
  updateWater(wat != 0);
  updateStatus(String(s));
}

void setup() {
  Serial.begin(9600);
  delay(200);
  Serial.println(F("[arduino] booting..."));

  espSerial.begin(9600);
  Serial.println(F("[arduino] SoftwareSerial 9600 siap di A4/A5"));

  uint16_t id = tft.readID();
  Serial.print(F("[arduino] LCD ID: 0x"));
  Serial.println(id, HEX);

  if (id == 0xD3D3 || id == 0x0000 || id == 0xFFFF) {
    id = 0x9481;
    Serial.println(F("[arduino] fallback 0x9481"));
  }

  tft.begin(id);
  tft.setRotation(1);
  tft.fillScreen(COLOR_BLACK);

  drawHeader();
  drawStaticLabels();
  drawDefaultValues();

  Serial.println(F("[arduino] siap, tunggu data ESP32 di A4..."));
}

void loop() {
  while (espSerial.available()) {
    char c = (char)espSerial.read();

    if (c == '\n') {
      lineBuf[lineIdx] = '\0';
      if (lineIdx > 0) processLine(lineBuf);
      lineIdx = 0;
    } else if (c != '\r') {
      if (lineIdx < LINE_BUF_SIZE - 1) lineBuf[lineIdx++] = c;
      else lineIdx = 0;
    }
  }
}