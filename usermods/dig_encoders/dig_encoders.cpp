// Dig-Uno two-encoder usermod for WLED (written for WLED 16.0.x)
//
// Two rotary encoders with push buttons, read through an Adafruit MCP23017 port expander (I2C).
//   Encoder 1 -> front segment (segment 0): turn = brightness (rear follows), short push = next favorite effect,
//                double click = next solid colour, long push = front on/off
//   Encoder 2 -> turn = white mix (0% = colour only ... 100% = warm white only, both segments),
//                short push = rear brightness steps 0% / 25% / 50% / 100% of the front, long push = rear on/off
//   The rear segment (segment 1) always copies the front's effect, speed, intensity and colour.
//
// Adafruit MCP23017 breakout wiring (port A, labelled A0..A7 on the board):
//   enc1 A pin -> A0, B pin -> A1, switch -> A2;  enc2 A pin -> A3, B pin -> A4, switch -> A5.
// I2C: board SDA <- Dig-Uno Q4 (GPIO32), board SCL <- Dig-Uno Q1 (GPIO15); VIN <- 3v3, GND <- GND.
// All inputs use the chip's internal pull-ups; the encoder common pin and the other switch pin go to GND.
//
// All changes to WLED are sent as small JSON commands (the same ones the web page uses), so this
// file depends on as little of WLED's internal code as possible.

#include "wled.h"
#include <Wire.h>
#include <vector>

// ---------------------------------------------------------------------------------------------
// Fast background reader (runs in its own task so quick knob turns are not missed)
// ---------------------------------------------------------------------------------------------
static volatile int16_t  g_delta[2]  = {0, 0};      // clicks turned since last read (+ / -)
static volatile uint8_t  g_event[2]  = {0, 0};      // 1 = short push, 2 = double click, 3 = long push
static volatile uint8_t  g_addr      = 0x20;
static volatile uint8_t  g_steps     = 4;           // quadrature transitions per click
static volatile uint8_t  g_invert[2] = {0, 0};
static volatile uint16_t g_longMs    = 800;
static volatile uint16_t g_dblMs     = 350;
static TaskHandle_t      g_task      = nullptr;

static bool mcpWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(g_addr);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

static bool mcpReadA(uint8_t &out) {
  Wire.beginTransmission(g_addr);
  Wire.write((uint8_t)0x12);                         // GPIOA register
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)g_addr, 1) != 1) return false;
  out = Wire.read();
  return true;
}

struct Btn {
  bool     stable = false;   // debounced "is pressed"
  bool     raw    = false;
  uint32_t tRaw   = 0;
  uint8_t  st     = 0;       // 0 idle, 1 down, 2 waiting for 2nd click, 3 2nd click held, 4 long held
  uint32_t t      = 0;
};

static void btnUpdate(Btn &b, uint8_t idx, bool pressed, uint32_t now, bool useDouble) {
  if (pressed != b.raw) { b.raw = pressed; b.tRaw = now; }
  bool edge = false;
  if (b.raw != b.stable && (now - b.tRaw) >= 20) { b.stable = b.raw; edge = true; }   // 20 ms debounce
  if (edge) {
    if (b.stable) {                                   // pressed
      if (b.st == 2) { g_event[idx] = 2; b.st = 3; }  // second click inside the window
      else           { b.st = 1; b.t = now; }
    } else {                                          // released
      if (b.st == 1) {
        if (useDouble) { b.st = 2; b.t = now; }       // wait to see if a second click follows
        else           { g_event[idx] = 1; b.st = 0; }
      } else b.st = 0;
    }
  } else {
    if (b.st == 1 && b.stable && (now - b.t) >= g_longMs) { g_event[idx] = 3; b.st = 4; }
    else if (b.st == 2 && (now - b.t) >= g_dblMs)          { g_event[idx] = 1; b.st = 0; }
  }
}

static void encTask(void *) {
  // index = (previous 2 bits << 2) | current 2 bits ; +1 / -1 for a valid step, 0 otherwise
  static const int8_t QTAB[16] = {0, 1, -1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1, 1, 0};
  uint8_t prev[2] = {3, 3};
  int8_t  acc[2]  = {0, 0};
  Btn     btn[2];
  for (;;) {
    uint8_t v;
    if (mcpReadA(v)) {
      uint32_t now = millis();
      for (uint8_t i = 0; i < 2; i++) {
        uint8_t sh  = i * 3;
        uint8_t cur = (v >> sh) & 0x03;
        int8_t  d   = QTAB[(prev[i] << 2) | cur];
        prev[i] = cur;
        if (g_invert[i]) d = -d;
        acc[i] += d;
        if (acc[i] >= (int8_t)g_steps)       { g_delta[i] = g_delta[i] + 1; acc[i] = 0; }
        else if (acc[i] <= -(int8_t)g_steps) { g_delta[i] = g_delta[i] - 1; acc[i] = 0; }
        bool pressed = !((v >> (sh + 2)) & 1);        // switch to ground = pressed
        btnUpdate(btn[i], i, pressed, now, i == 0);   // only encoder 1 uses double click
      }
    }
    vTaskDelay(1);
  }
}

// ---------------------------------------------------------------------------------------------
// The usermod itself
// ---------------------------------------------------------------------------------------------
class DigEncodersUsermod : public Usermod {
 private:
  static const char _name[];

  // ---- settings (editable on the WLED "Usermods" settings page) ----
  bool   enabled        = true;
  int    pinSda         = 32;   // Dig-Uno pin labelled Q4 (GPIO32)
  int    pinScl         = 15;   // Dig-Uno pin labelled Q1 (GPIO15)
  int    mcpAddr        = 0x20;
  int    stepsPerClick  = 4;    // 4 for most encoders; use 2 for 30-detent/15-pulse versions
  bool   invertEnc1     = false;
  bool   invertEnc2     = false;
  int    briStep        = 8;    // brightness change per click (encoder 1)
  int    mixStep        = 5;    // white-mix change per click, in percent (encoder 2)
  int    rearPercent    = 50;   // rear brightness as a percentage of the front brightness (0/25/50/100, remembered)
  int    longPressMs    = 800;
  int    doubleClickMs  = 350;
  bool   forceSegments  = true; // rebuild the two segments at every boot (needs the 2D matrix set up)
  String favEffects     = "Colorloop,Rainbow,Palette,Aurora,Fire 2012,Plasma,Noise2D,Soap,Lissajous,Matrix,Game Of Life,Waving Cell,Plasma Ball,Distortion Waves,Blobs,Drift,Drift Rose,Julia,Squared Swirl,Ghost Rider,Hiphotic,Polar Lights,Sun Radiation,Washing Machine";
  String favPalettes    = "6,7,8,9,10,11,12,13,14,22,35,36,44,45,46,50";
  String favColors      = "FF0000,FF4000,FF9000,FFE000,40FF00,00FF60,00FFFF,0060FF,0000FF,8000FF,FF00FF,FF0060";

  // ---- remembered state (saved automatically a few seconds after any change) ----
  bool sFrontOn = true, sRearOn = true;
  int  sFrontBri = 200, sMode = -1, sPal = 6, sMix = 0, sColor = -1;

  // ---- runtime ----
  bool     started = false, wireStarted = false, mcpOk = false, favDirty = true, dirty = false;
  int      mixPct = 0;            // 0..100, used by handleOverlayDraw()
  uint32_t lastTick = 0, lastInitTry = 0, lastChange = 0, lastCheck = 0, snapshot = 0;
  std::vector<uint8_t> fxIds;
  std::vector<uint8_t> palIds;
  std::vector<uint32_t> colIds;
  String   missingNames;

  Segment &frontSeg() { return strip.getSegment(0); }
  Segment &rearSeg() { return strip.getSegment(1); }
  bool haveRear() { return strip.getSegmentsNum() >= 2; }

  static String modeName(uint8_t id) {
    String s;
    const char *d = strip.getModeData(id);
    if (!d) return s;
    char c;
    while ((c = pgm_read_byte(d++)) && c != '@') s += c;
    return s;
  }

  // Send a small JSON command to WLED, exactly like the web page / JSON API does.
  void sendJson(const String &json) {
    DynamicJsonDocument doc(1536);
    if (deserializeJson(doc, json) != DeserializationError::Ok) return;
    JsonObject root = doc.as<JsonObject>();
    deserializeState(root, CALL_MODE_BUTTON);
    lastChange = millis();
  }

  void segCmd(uint8_t id, const String &fields) {
    sendJson(String("{\"seg\":[{\"id\":") + String((int)id) + "," + fields + "}]}");
  }

  int rearBriFor(int frontBri) {
    return constrain((frontBri * constrain(rearPercent, 0, 100)) / 100, 0, 255);
  }

  void resolveFavorites() {
    fxIds.clear(); palIds.clear(); missingNames = "";
    // effects, by name
    int from = 0;
    String list = favEffects + ",";
    while (true) {
      int to = list.indexOf(',', from);
      if (to < 0) break;
      String name = list.substring(from, to);
      from = to + 1;
      name.trim();
      if (!name.length()) continue;
      bool found = false;
      for (uint8_t i = 0; i < strip.getModeCount(); i++) {
        if (name.equalsIgnoreCase(modeName(i))) { fxIds.push_back(i); found = true; break; }
      }
      if (!found) { if (missingNames.length()) missingNames += ", "; missingNames += name; }
    }
    if (fxIds.empty()) {                              // nothing matched: fall back to every effect
      for (uint8_t i = 1; i < strip.getModeCount(); i++) fxIds.push_back(i);
    }
    // palettes, by number
    from = 0;
    list = favPalettes + ",";
    while (true) {
      int to = list.indexOf(',', from);
      if (to < 0) break;
      String tok = list.substring(from, to);
      from = to + 1;
      tok.trim();
      if (!tok.length()) continue;
      int n = tok.toInt();
      if (n >= 0 && n < 255) palIds.push_back((uint8_t)n);
    }
    if (palIds.empty()) palIds.push_back(6);
    // solid colours, as 6-digit hex (RRGGBB)
    colIds.clear();
    from = 0;
    list = favColors + ",";
    while (true) {
      int to = list.indexOf(',', from);
      if (to < 0) break;
      String tok = list.substring(from, to);
      from = to + 1;
      tok.trim();
      if (tok.startsWith("#")) tok = tok.substring(1);
      if (tok.length() != 6) continue;
      colIds.push_back((uint32_t)strtoul(tok.c_str(), nullptr, 16));
    }
    if (colIds.empty()) colIds.push_back(0xFF0000);
    favDirty = false;
  }

  void pushSettingsToTask() {
    g_addr = (uint8_t)mcpAddr;
    g_steps = (uint8_t)constrain(stepsPerClick, 1, 8);
    g_invert[0] = invertEnc1;
    g_invert[1] = invertEnc2;
    g_longMs = (uint16_t)constrain(longPressMs, 300, 3000);
    g_dblMs = (uint16_t)constrain(doubleClickMs, 150, 800);
  }

  void tryInit() {
    lastInitTry = millis();
    pushSettingsToTask();
    if (!wireStarted) { Wire.begin(pinSda, pinScl, 400000); wireStarted = true; }
    mcpOk = mcpWrite(0x00, 0xFF) && mcpWrite(0x0C, 0xFF);   // port A: all inputs, pull-ups on
    if (mcpOk && !g_task) xTaskCreatePinnedToCore(encTask, "digenc", 3072, nullptr, 2, &g_task, 0);
  }

  void captureState() {
    Segment &f = frontSeg();
    sFrontOn = f.on; sFrontBri = f.opacity; sMode = f.mode; sPal = f.palette;
    if (haveRear()) { Segment &r = rearSeg(); sRearOn = r.on; }
    sMix = mixPct;
  }

  uint32_t makeSnapshot() {
    Segment &f = frontSeg();
    uint32_t h = 17;
    h = h * 31 + f.on; h = h * 31 + f.opacity; h = h * 31 + f.mode; h = h * 31 + f.palette;
    if (haveRear()) { Segment &r = rearSeg(); h = h * 31 + r.on; }
    h = h * 31 + (uint32_t)mixPct;
    h = h * 31 + (uint32_t)(sColor + 1);
    h = h * 31 + (uint32_t)rearPercent;
    return h;
  }

  // ,"col":[[r,g,b,0]] for the remembered solid colour (empty if none chosen yet)
  String colJson() {
    if (sColor < 0 || sColor >= (int)colIds.size()) return "";
    uint32_t c = colIds[sColor];
    return String(",\"col\":[[") + String((int)((c >> 16) & 255)) + "," + String((int)((c >> 8) & 255)) + "," + String((int)(c & 255)) + ",0]]";
  }

  void startup() {
    if (favDirty) resolveFavorites();
    mixPct = constrain(sMix, 0, 100);
    if (sColor < 0 || sColor >= (int)colIds.size()) sColor = 0;
    sPal = 2;                                          // palette "Color 1": every effect uses the chosen solid colour
    uint8_t m = (sMode >= 0 && sMode < (int)strip.getModeCount()) ? (uint8_t)sMode : fxIds[0];
    int fb = constrain(sFrontBri, 2, 255);
    bool geometry = forceSegments && strip.isMatrix;
    // front = 5 wide x 10 tall (left part), rear = 2 wide x 19 tall (right part) of the 7 x 19 map
    String j = "{\"on\":true,\"bri\":255,\"seg\":[{\"id\":0,";
    if (geometry) j += "\"start\":0,\"stop\":5,\"startY\":0,\"stopY\":10,";
    j += String("\"on\":") + (sFrontOn ? "true" : "false") + ",\"bri\":" + String(fb) +
         ",\"fx\":" + String((int)m) + ",\"pal\":" + String(sPal) + colJson() + "},{\"id\":1,";
    if (geometry) j += "\"start\":5,\"stop\":7,\"startY\":0,\"stopY\":19,";
    j += String("\"on\":") + (sRearOn ? "true" : "false") + ",\"bri\":" + String(rearBriFor(fb)) +
         ",\"fx\":" + String((int)m) + ",\"pal\":" + String(sPal) + colJson() + "}]}";
    sendJson(j);
    snapshot = makeSnapshot();
  }

  void changeFrontBrightness(int clicks) {
    Segment &s = frontSeg();
    if (!s.on) return;
    int b = constrain((int)s.opacity + clicks * briStep, 2, 255);
    segCmd(0, String("\"bri\":") + String(b));
  }

  void changeMix(int clicks) {
    mixPct = constrain(mixPct + clicks * constrain(mixStep, 1, 50), 0, 100);
    lastChange = millis();
  }

  void toggleOn(uint8_t id) {
    Segment &s = strip.getSegment(id);
    segCmd(id, String("\"on\":") + (s.on ? "false" : "true"));
  }

  void nextEffect() {
    if (fxIds.empty()) return;
    Segment &f = frontSeg();
    size_t idx = 0;
    bool found = false;
    for (size_t i = 0; i < fxIds.size(); i++) if (fxIds[i] == f.mode) { idx = i; found = true; break; }
    size_t next = found ? (idx + 1) % fxIds.size() : 0;
    segCmd(0, String("\"fx\":") + String((int)fxIds[next]));
  }

  void nextPalette() {
    if (palIds.empty()) return;
    Segment &f = frontSeg();
    size_t idx = 0;
    bool found = false;
    for (size_t i = 0; i < palIds.size(); i++) if (palIds[i] == f.palette) { idx = i; found = true; break; }
    size_t next = found ? (idx + 1) % palIds.size() : 0;
    String p = String((int)palIds[next]);
    String j = "{\"seg\":[{\"id\":0,\"pal\":" + p + "}";
    if (haveRear()) j += ",{\"id\":1,\"pal\":" + p + "}";   // palette is always shared
    j += "]}";
    sendJson(j);
  }

  // quick colour change: next solid colour on front and rear; palette set to "Color 1" (id 2)
  void stepRear() {
    static const int lv[4] = {0, 25, 50, 100};
    int next = 0;
    for (int i = 0; i < 4; i++) if (lv[i] > rearPercent) { next = lv[i]; break; }
    rearPercent = next;
    lastChange = millis();
  }

  void nextColor() {
    if (colIds.empty()) return;
    sColor = (sColor + 1) % (int)colIds.size();
    String c = colJson() + ",\"pal\":2";
    String j = "{\"seg\":[{\"id\":0" + c + "}";
    if (haveRear()) j += ",{\"id\":1" + c + "}";
    j += "]}";
    sendJson(j);
  }

  // runs about 60 times per second: the rear always copies the front
  void syncRear() {
    if (!haveRear()) return;
    Segment &f = frontSeg();
    Segment &r = rearSeg();
    String fields;
    auto add = [&](const String &s) { if (fields.length()) fields += ","; fields += s; };
    if (r.colors[0] != f.colors[0]) {
      uint32_t c = f.colors[0];
      add(String("\"col\":[[") + String((int)((c >> 16) & 255)) + "," + String((int)((c >> 8) & 255)) + "," + String((int)(c & 255)) + ",0]]");
    }
    if (r.palette != f.palette) add(String("\"pal\":") + String((int)f.palette));
    if (r.mode != f.mode) add(String("\"fx\":") + String((int)f.mode));
    if (r.speed != f.speed) add(String("\"sx\":") + String((int)f.speed));
    if (r.intensity != f.intensity) add(String("\"ix\":") + String((int)f.intensity));
    int target = rearBriFor((int)f.opacity);
    if ((int)r.opacity != target) add(String("\"bri\":") + String(target));
    if (fields.length()) segCmd(1, fields);
  }

 public:
  void setup() override {
    // real start-up work is done a moment after boot in loop(), once the saved settings have been read
  }

  void loop() override {
    if (!enabled) return;
    uint32_t now = millis();
    if (!started) {
      if (now < 2500) return;
      tryInit();
      startup();
      started = true;
      return;
    }
    if (!g_task && now - lastInitTry > 5000) tryInit();   // retry if the expander was not found
    if (now - lastTick < 15) return;
    lastTick = now;
    if (favDirty) resolveFavorites();

    // ---- encoder 1: front brightness / effect / palette / on-off ----
    int16_t d0 = g_delta[0];
    if (d0) { g_delta[0] = g_delta[0] - d0; changeFrontBrightness(d0); }
    uint8_t e0 = g_event[0];
    if (e0) {
      g_event[0] = 0;
      if (e0 == 1) nextEffect();
      else if (e0 == 2) nextColor();
      else if (e0 == 3) toggleOn(0);
    }
    // ---- encoder 2: white mix / quick palette change / rear on-off ----
    int16_t d1 = g_delta[1];
    if (d1) { g_delta[1] = g_delta[1] - d1; changeMix(d1); }
    uint8_t e1 = g_event[1];
    if (e1) {
      g_event[1] = 0;
      if (e1 == 1) stepRear();
      else if (e1 == 3 && haveRear()) toggleOn(1);
    }

    syncRear();

    // ---- remember state: save a few seconds after the last change (also catches changes from the web page) ----
    if (now - lastCheck > 500) {
      lastCheck = now;
      uint32_t s = makeSnapshot();
      if (s != snapshot) { snapshot = s; dirty = true; lastChange = now; }
      if (dirty && now - lastChange > 5000) {
        captureState();
        configNeedsWrite = true;                       // asks WLED to save its settings (includes this usermod's saved state)
        dirty = false;
      }
    }
  }

  // Called just before every frame is sent to the LEDs: blends colour towards the warm-white LED.
  void handleOverlayDraw() override {
    if (!enabled || !started || mixPct <= 0) return;
    const int m = mixPct;
    const unsigned total = strip.getLengthTotal();
    for (unsigned i = 0; i < total; i++) {
      uint32_t c = strip.getPixelColor(i);
      if (!c) continue;
      int r = R(c), g = G(c), b = B(c), w = W(c);
      int v = max(r, max(g, b));                      // how bright the colour part is
      int keep = 100 - m;
      r = (r * keep) / 100;
      g = (g * keep) / 100;
      b = (b * keep) / 100;
      w = min(255, w + (v * m) / 100);
      strip.setPixelColor(i, RGBW32(r, g, b, w));
    }
  }

  void addToJsonInfo(JsonObject &root) override {
    JsonObject user = root["u"];
    if (user.isNull()) user = root.createNestedObject("u");
    JsonArray a = user.createNestedArray("Encoders");
    a.add(mcpOk ? "MCP23017 found" : "MCP23017 NOT found");
    JsonArray b = user.createNestedArray("Favorite effects");
    b.add((int)fxIds.size());
    JsonArray m = user.createNestedArray("White mix %");
    m.add(mixPct);
    if (missingNames.length()) {
      JsonArray c = user.createNestedArray("Effect names not found");
      c.add(missingNames);
    }
    if (!strip.isMatrix) {
      JsonArray w = user.createNestedArray("2D matrix");
      w.add("NOT set up (see guide)");
    }
  }

  void addToConfig(JsonObject &root) override {
    if (started) captureState();
    JsonObject top = root.createNestedObject(FPSTR(_name));
    top["enabled"] = enabled;
    top["sdaPin"] = pinSda;
    top["sclPin"] = pinScl;
    top["mcpAddress"] = mcpAddr;
    top["stepsPerClick"] = stepsPerClick;
    top["reverseEncoder1"] = invertEnc1;
    top["reverseEncoder2"] = invertEnc2;
    top["brightnessStep"] = briStep;
    top["whiteMixStepPercent"] = mixStep;
    top["rearBrightnessPercent"] = rearPercent;
    top["longPressMs"] = longPressMs;
    top["doubleClickMs"] = doubleClickMs;
    top["rebuildSegmentsAtBoot"] = forceSegments;
    top["favoriteEffects"] = favEffects;
    top["favoritePalettes"] = favPalettes;
    top["favoriteColors"] = favColors;
    top["savedColorIndex"] = sColor;
    top["savedFrontOn"] = sFrontOn;
    top["savedFrontBrightness"] = sFrontBri;
    top["savedRearOn"] = sRearOn;
    top["savedEffect"] = sMode;
    top["savedPalette"] = sPal;
    top["savedWhiteMixPercent"] = sMix;
  }

  bool readFromConfig(JsonObject &root) override {
    JsonObject top = root[FPSTR(_name)];
    if (top.isNull()) return false;
    enabled        = top["enabled"] | enabled;
    if (!started) {                                   // pins/address only change after a reboot
      pinSda  = top["sdaPin"] | pinSda;
      pinScl  = top["sclPin"] | pinScl;
      mcpAddr = top["mcpAddress"] | mcpAddr;
    }
    stepsPerClick  = top["stepsPerClick"] | stepsPerClick;
    invertEnc1     = top["reverseEncoder1"] | invertEnc1;
    invertEnc2     = top["reverseEncoder2"] | invertEnc2;
    briStep        = top["brightnessStep"] | briStep;
    mixStep        = top["whiteMixStepPercent"] | mixStep;
    rearPercent    = top["rearBrightnessPercent"] | rearPercent;
    longPressMs    = top["longPressMs"] | longPressMs;
    doubleClickMs  = top["doubleClickMs"] | doubleClickMs;
    forceSegments  = top["rebuildSegmentsAtBoot"] | forceSegments;
    const char *fe = top["favoriteEffects"] | "";
    if (fe[0]) favEffects = fe;
    const char *fp = top["favoritePalettes"] | "";
    if (fp[0]) favPalettes = fp;
    const char *fc = top["favoriteColors"] | "";
    if (fc[0]) favColors = fc;
    if (!started) {                                   // saved state is only applied at boot
      sFrontOn  = top["savedFrontOn"] | sFrontOn;
      sFrontBri = top["savedFrontBrightness"] | sFrontBri;
      sRearOn   = top["savedRearOn"] | sRearOn;
      sMode     = top["savedEffect"] | sMode;
      sPal      = top["savedPalette"] | sPal;
      sMix      = top["savedWhiteMixPercent"] | sMix;
      sColor    = top["savedColorIndex"] | sColor;
    }
    favDirty = true;
    pushSettingsToTask();
    return !top["savedWhiteMixPercent"].isNull();     // false the first time, so WLED writes the new settings
  }
};

const char DigEncodersUsermod::_name[] PROGMEM = "DigEncoders";

static DigEncodersUsermod digEncodersUsermod;
REGISTER_USERMOD(digEncodersUsermod);
