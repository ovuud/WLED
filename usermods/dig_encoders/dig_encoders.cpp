// Dig-Uno two-encoder usermod for WLED (written for WLED 16.0.x)
//
// Two rotary encoders with push buttons, read through an Adafruit MCP23017 port expander (I2C).
//   Encoder 1 -> front segment (segment 0): turn = brightness (rear follows), short push = next favorite effect,
//                long push = front on/off,
//                push and turn = move the colour around the hue wheel in 3 degree steps
//   Encoder 2 -> turn = white mix (0% = colour only ... 100% = warm white only, both segments),
//                short push = next solid colour, long push = rear on/off
//   Some effects use a second colour that is a fixed number of degrees further round the colour wheel than the
//   chosen colour (setting effectHueShift, default Wipe=15). Fixed colour pairs are also possible (effectColors).
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
static volatile int16_t  g_hue       = 0;           // clicks turned on encoder 1 while its button was held down
static volatile int16_t  g_speed     = 0;           // clicks turned on encoder 2 while its button was held down
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
  bool     turned = false;   // knob was turned while the button was held (so the push is not a click)
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
      if (b.turned) { b.turned = false; b.st = 0; }     // push-and-turn: no click event on release
      else if (b.st == 1) {
        if (useDouble) { b.st = 2; b.t = now; }       // wait to see if a second click follows
        else           { g_event[idx] = 1; b.st = 0; }
      } else b.st = 0;
    }
  } else {
    if (b.st == 1 && b.stable && !b.turned && (now - b.t) >= g_longMs) { g_event[idx] = 3; b.st = 4; }
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
        int8_t click = 0;
        if (acc[i] >= (int8_t)g_steps)       { click = 1;  acc[i] = 0; }
        else if (acc[i] <= -(int8_t)g_steps) { click = -1; acc[i] = 0; }
        if (click) {
          if (i == 0 && btn[0].stable) { g_hue = g_hue + click; btn[0].turned = true; }   // push and turn = hue wheel
          else if (i == 1 && btn[1].stable) { g_speed = g_speed + click; btn[1].turned = true; }   // push and turn on encoder 2 = effect speed
          else g_delta[i] = g_delta[i] + click;
        }
        bool pressed = !((v >> (sh + 2)) & 1);        // switch to ground = pressed
        btnUpdate(btn[i], i, pressed, now, i == 0);   // encoder 1 has a double click (colour relationship); encoder 2 acts at once
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
  int    wideDeg        = 35;   // double click on encoder 1: the 'dramatic' colour relationship, in degrees (replaces the normal angle of every shifted effect)
  int    speedStep      = 8;    // effect-speed change per click (encoder 2, push and turn), out of 255
  int    whiteLevel     = 100;  // how strong the white LED is at 100% mix, relative to the colour it replaces (lower if the white looks brighter than the colour)
  int    rearPercent    = 70;   // rear brightness as a percentage of the front brightness (editable in the settings)
  int    longPressMs    = 800;
  int    doubleClickMs  = 350;
  bool   forceSegments  = true; // rebuild the two segments at every boot (needs the 2D matrix set up)
  String favEffects     = "Wipe,Colorloop,Rainbow,Palette,Aurora,Fire 2012,Plasma,Noise2D,Soap,Lissajous,Matrix,Game Of Life,Waving Cell,Plasma Ball,Distortion Waves,Blobs,Drift,Drift Rose,Julia,Squared Swirl,Ghost Rider,Hiphotic,Polar Lights,Sun Radiation,Washing Machine";
  String favPalettes    = "6,7,8,9,10,11,12,13,14,22,35,36,44,45,46,50";
  String effectColors   = "";                      // optional fixed pairs: Name=RRGGBB+RRGGBB, separate entries with commas
  String effectHueShift = "Wipe=15";               // Name=degrees: colour 2 = the chosen colour turned this many degrees clockwise on the wheel
  String stockColorEffects = "Colorwaves,PS Fire,PS Waterfall";   // effects that keep WLED's own colours: their stock palette, no chosen colour, no hue shift
  String effectPalette  = "Railway=3";             // Name=paletteNumber: effects that take their shades from the palette; 3 = "Colors 1&2" (colour 1 to colour 2)
  String favColors      = "FF0000,FF4000,FF9000,FFE000,40FF00,00FF60,00FFFF,0060FF,0000FF,8000FF,FF00FF,FF0060";

  // ---- remembered state (saved automatically a few seconds after any change) ----
  bool sFrontOn = true, sRearOn = true;
  int  sFrontBri = 200, sMode = -1, sPal = 6, sMix = 0, sColor = -1, sHue = -1, sSpeed = -1;
  bool sWide = false;                              // true while the dramatic colour relationship is on   // sSpeed: effect speed 0..255, -1 = not chosen yet; sHue: 0..359 when the colour came from the hue wheel, else -1

  // ---- runtime ----
  int      heldBri = -1;           // front brightness chosen while the front is OFF (-1 = none); applied when the front is switched on
  bool     rearUserOn = true;      // the rear's own on/off, set only by encoder 2 long push (WLED turns a segment ON whenever it is sent a brightness, so this is enforced explicitly)
  bool     started = false, wireStarted = false, mcpOk = false, favDirty = true, dirty = false;
  int      mixPct = 0;            // 0..100, used by handleOverlayDraw()
  uint32_t lastTick = 0, lastInitTry = 0, lastChange = 0, lastCheck = 0, snapshot = 0;
  std::vector<uint8_t> fxIds;
  std::vector<uint8_t> palIds;
  std::vector<uint32_t> colIds;
  struct FxPair { uint8_t fx; uint32_t c1, c2; };
  std::vector<FxPair> fxPairs;
  struct FxShift { uint8_t fx; int deg; };
  std::vector<FxShift> fxShifts;
  std::vector<uint8_t> stockIds;
  struct FxPal { uint8_t fx; uint8_t pal; };
  std::vector<FxPal> fxPals;
  String   missingNames;

  Segment &frontSeg() { return strip.getSegment(0); }
  Segment &rearSeg() { return strip.getSegment(1); }
  bool haveRear() { return strip.getSegmentsNum() >= 2; }
  bool haveRear2() { return strip.getSegmentsNum() >= 3; }
  // the two rear strips are segments 1 (left of the front, LEDs 1-19) and 2 (right of the front, LEDs 20-38)
  // c starts with a comma:  ,{"id":1<c>},{"id":2<c>}
  String rearSegs(const String &c) {
    String j;
    if (haveRear())  j += String(",{\"id\":1") + c + "}";
    if (haveRear2()) j += String(",{\"id\":2") + c + "}";
    return j;
  }

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
    return constrain((frontBri * constrain(rearPercent, 0, 100)) / 100, 1, 255);   // never 0: WLED treats a segment brightness of 0 as "off"
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
    // effects with their own colour pair, written as  Name=RRGGBB+RRGGBB
    fxPairs.clear();
    from = 0;
    list = effectColors + ",";
    while (true) {
      int to = list.indexOf(',', from);
      if (to < 0) break;
      String tok = list.substring(from, to);
      from = to + 1;
      int eq = tok.indexOf('=');
      int plus = tok.indexOf('+');
      if (eq < 1 || plus < eq) continue;
      String name = tok.substring(0, eq); name.trim();
      String a = tok.substring(eq + 1, plus); a.trim();
      String b = tok.substring(plus + 1); b.trim();
      if (a.startsWith("#")) a = a.substring(1);
      if (b.startsWith("#")) b = b.substring(1);
      if (a.length() != 6 || b.length() != 6) continue;
      for (uint8_t i = 0; i < strip.getModeCount(); i++) {
        if (name.equalsIgnoreCase(modeName(i))) {
          fxPairs.push_back({i, (uint32_t)strtoul(a.c_str(), nullptr, 16), (uint32_t)strtoul(b.c_str(), nullptr, 16)});
          break;
        }
      }
    }
    // effects whose second colour is the chosen colour turned by some degrees, written as  Name=degrees
    fxShifts.clear();
    from = 0;
    list = effectHueShift + ",";
    while (true) {
      int to = list.indexOf(',', from);
      if (to < 0) break;
      String tok = list.substring(from, to);
      from = to + 1;
      int eq = tok.indexOf('=');
      if (eq < 1) continue;
      String name = tok.substring(0, eq); name.trim();
      int deg = tok.substring(eq + 1).toInt();
      for (uint8_t i = 0; i < strip.getModeCount(); i++) {
        if (name.equalsIgnoreCase(modeName(i))) { fxShifts.push_back({i, deg}); break; }
      }
    }
    // effects that keep WLED's own colours, by name
    stockIds.clear();
    from = 0;
    list = stockColorEffects + ",";
    while (true) {
      int to = list.indexOf(',', from);
      if (to < 0) break;
      String name = list.substring(from, to);
      from = to + 1;
      name.trim();
      if (!name.length()) continue;
      for (uint8_t i = 0; i < strip.getModeCount(); i++) {
        if (name.equalsIgnoreCase(modeName(i))) { stockIds.push_back(i); break; }
      }
    }
    // effects that need a particular palette, written as  Name=number
    fxPals.clear();
    from = 0;
    list = effectPalette + ",";
    while (true) {
      int to = list.indexOf(',', from);
      if (to < 0) break;
      String tok = list.substring(from, to);
      from = to + 1;
      int eq = tok.indexOf('=');
      if (eq < 1) continue;
      String name = tok.substring(0, eq); name.trim();
      int pn = tok.substring(eq + 1).toInt();
      for (uint8_t i = 0; i < strip.getModeCount(); i++) {
        if (name.equalsIgnoreCase(modeName(i))) { fxPals.push_back({i, (uint8_t)constrain(pn, 0, 254)}); break; }
      }
    }
    favDirty = false;
  }

  bool isStock(uint8_t fx) {
    for (auto id : stockIds) if (id == fx) return true;
    return false;
  }

  // the palette an effect is meant to start with (the "pal=" in its definition); 0 = WLED's default if it names none
  int stockPalette(uint8_t fx) {
    const char *d = strip.getModeData(fx);
    if (!d) return 0;
    String t;
    char c;
    while ((c = pgm_read_byte(d++))) t += c;
    int i = t.indexOf("pal=");
    return (i < 0) ? 0 : t.substring(i + 4).toInt();
  }

  // palette to use for effect fx: its own stock palette, or "Color 1" (id 2) so the chosen solid colour is what shows
  int palForFx(uint8_t fx) {
    if (isStock(fx)) return stockPalette(fx);
    for (auto &p : fxPals) if (p.fx == fx) return p.pal;   // e.g. Railway needs "Colors 1&2" to show both colours
    return 2;
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

  // the front's brightness as the user sees it: while the front is off, the level chosen with encoder 1 in the meantime
  int frontBriNow() {
    Segment &f = frontSeg();
    return (!f.on && heldBri >= 0) ? heldBri : (int)f.opacity;
  }

  void captureState() {
    Segment &f = frontSeg();
    sFrontOn = f.on; sFrontBri = frontBriNow(); sMode = f.mode; sPal = f.palette;
    sRearOn = rearUserOn;
    sMix = mixPct;
    sSpeed = f.speed;
  }

  uint32_t makeSnapshot() {
    Segment &f = frontSeg();
    uint32_t h = 17;
    h = h * 31 + f.on; h = h * 31 + frontBriNow(); h = h * 31 + (uint32_t)(heldBri + 1); h = h * 31 + f.mode; h = h * 31 + f.palette;
    h = h * 31 + rearUserOn;
    h = h * 31 + (uint32_t)mixPct;
    h = h * 31 + (uint32_t)(sColor + 1);
    h = h * 31 + (uint32_t)(sHue + 1);
    h = h * 31 + f.speed;
    h = h * 31 + (sWide ? 1 : 0);
    h = h * 31 + (uint32_t)rearPercent;
    return h;
  }

  static uint32_t hueToRgb(int h) {
    h = ((h % 360) + 360) % 360;
    int f = (h % 60) * 255 / 60;
    int r, g, b;
    switch (h / 60) {
      case 0:  r = 255;     g = f;       b = 0;       break;
      case 1:  r = 255 - f; g = 255;     b = 0;       break;
      case 2:  r = 0;       g = 255;     b = f;       break;
      case 3:  r = 0;       g = 255 - f; b = 255;     break;
      case 4:  r = f;       g = 0;       b = 255;     break;
      default: r = 255;     g = 0;       b = 255 - f; break;
    }
    return ((uint32_t)r << 16) | ((uint32_t)g << 8) | (uint32_t)b;
  }

  static int rgbToHue(uint32_t c) {
    int r = (c >> 16) & 255, g = (c >> 8) & 255, b = c & 255;
    int mx = max(r, max(g, b)), mn = min(r, min(g, b));
    if (mx == mn) return 0;
    float d = (float)(mx - mn), h;
    if (mx == r)      h = fmodf((g - b) / d, 6.0f);
    else if (mx == g) h = (b - r) / d + 2.0f;
    else              h = (r - g) / d + 4.0f;
    int deg = (int)(h * 60.0f + 0.5f);
    return ((deg % 360) + 360) % 360;
  }

  // push and turn on encoder 1: move around the hue wheel, 3 degrees per click, front and rear together
  void changeHue(int clicks) {
    if (sHue < 0) {
      uint32_t base = (sColor >= 0 && sColor < (int)colIds.size()) ? colIds[sColor] : 0xFF0000;
      sHue = rgbToHue(base);
    }
    sHue = (((sHue + clicks * 3) % 360) + 360) % 360;
    if (isStock(frontSeg().mode)) return;              // stock-colour effect: the choice is remembered for the next effect, nothing shown now
    String c = colJsonFor(frontSeg().mode) + ",\"pal\":" + String(palForFx(frontSeg().mode));
    String j = "{\"seg\":[{\"id\":0" + c + "}";
    j += rearSegs(c);
    j += "]}";
    sendJson(j);
  }

  static String rgbArr(uint32_t c) {
    return String("[") + String((int)((c >> 16) & 255)) + "," + String((int)((c >> 8) & 255)) + "," + String((int)(c & 255)) + ",0]";
  }

  bool pairFor(uint8_t fx, uint32_t &c1, uint32_t &c2) {
    for (auto &p : fxPairs) if (p.fx == fx) { c1 = p.c1; c2 = p.c2; return true; }
    return false;
  }

  // the remembered solid colour (what a double click / push-and-turn last chose)
  uint32_t solidColor() {
    if (sHue >= 0) return hueToRgb(sHue);
    if (sColor >= 0 && sColor < (int)colIds.size()) return colIds[sColor];
    return 0xFF0000;
  }

  // ,"col":[...] for effect fx: its own pair if it has one, otherwise the solid colour with colour 2 black
  String colJsonFor(uint8_t fx) {
    if (isStock(fx)) return "";                        // stock-colour effects keep whatever colours WLED has
    uint32_t c1, c2;
    bool shifted = false;
    for (auto &p : fxShifts) if (p.fx == fx) {
      c1 = solidColor();
      c2 = hueToRgb(rgbToHue(c1) + (sWide ? wideDeg : p.deg));              // fully saturated colour on the outer rim of the wheel
      shifted = true;
      break;
    }
    if (!shifted && !pairFor(fx, c1, c2)) { c1 = solidColor(); c2 = 0; }
    return String(",\"col\":[") + rgbArr(c1) + "," + rgbArr(c2) + "]";
  }

  // ,"col":[[r,g,b,0]] for the remembered solid colour (empty if none chosen yet)
  String colJson() {
    uint32_t c;
    if (sHue >= 0) c = hueToRgb(sHue);
    else if (sColor >= 0 && sColor < (int)colIds.size()) c = colIds[sColor];
    else return "";
    return String(",\"col\":[[") + String((int)((c >> 16) & 255)) + "," + String((int)((c >> 8) & 255)) + "," + String((int)(c & 255)) + ",0]]";
  }

  void startup() {
    if (favDirty) resolveFavorites();
    mixPct = constrain(sMix, 0, 100);
    rearUserOn = sRearOn;
    if (sColor < 0 || sColor >= (int)colIds.size()) sColor = 0;
    uint8_t m = (sMode >= 0 && sMode < (int)strip.getModeCount()) ? (uint8_t)sMode : fxIds[0];
    sPal = palForFx(m);                                // palette "Color 1" (the chosen solid colour), or the effect's own palette for stock-colour effects
    int fb = constrain(sFrontBri, 2, 255);
    bool geometry = forceSegments && strip.isMatrix;
    // Two layouts are understood, chosen by the width of the LED map:
    //   9 wide x 19 tall: columns 0-1 = rear strip 1 (LED in column 0), columns 2-6 = front (lit LEDs on every second row, level with the
    //                     strip LEDs), columns 7-8 = rear strip 2 (LED in column 8). A segment needs 2+ columns for WLED to treat it as 2D.
    //   7 wide x 19 tall: columns 0-4 = front (5 wide x 10 tall), columns 5-6 = both rear strips side by side as one segment.
    const bool wide = Segment::maxWidth >= 9;
    String spd = (sSpeed >= 0) ? String(",\"sx\":") + String(constrain(sSpeed, 0, 255)) : String("");
    String j = "{\"on\":true,\"bri\":255,\"seg\":[{\"id\":0,";
    if (geometry) j += wide ? "\"start\":2,\"stop\":7,\"startY\":0,\"stopY\":19," : "\"start\":0,\"stop\":5,\"startY\":0,\"stopY\":10,";
    j += String("\"on\":") + (sFrontOn ? "true" : "false") + ",\"bri\":" + String(fb) +
         ",\"fx\":" + String((int)m) + ",\"pal\":" + String(sPal) + colJsonFor(m) + spd + "}";
    String rest = String("\"on\":") + ((rearUserOn && rearPercent > 0) ? "true" : "false") + ",\"bri\":" + String(rearBriFor(fb)) +
         ",\"fx\":" + String((int)m) + ",\"pal\":" + String(sPal) + colJsonFor(m) + spd;
    j += ",{\"id\":1,";
    if (geometry) j += wide ? "\"start\":0,\"stop\":2,\"startY\":0,\"stopY\":19," : "\"start\":5,\"stop\":7,\"startY\":0,\"stopY\":19,";
    j += rest + "}";
    if (wide) {
      j += ",{\"id\":2,";
      if (geometry) j += "\"start\":7,\"stop\":9,\"startY\":0,\"stopY\":19,";
      j += rest + "}";
    }
    j += "]}";
    sendJson(j);
    snapshot = makeSnapshot();
  }

  void changeFrontBrightness(int clicks) {
    Segment &s = frontSeg();
    int b = constrain(frontBriNow() + clicks * briStep, 2, 255);
    if (!s.on) {                                       // front is off: remember the level, send nothing (a brightness would switch it on)
      heldBri = b;
      lastChange = millis();
      return;
    }
    segCmd(0, String("\"bri\":") + String(b));
  }

  // double click on encoder 1: switch the second colour between the normal angle (e.g. 15 degrees) and the dramatic one (35)
  void toggleWide() {
    sWide = !sWide;
    lastChange = millis();
    if (isStock(frontSeg().mode)) return;              // stock-colour effect: nothing to show now
    String c = colJsonFor(frontSeg().mode) + ",\"pal\":" + String(palForFx(frontSeg().mode));
    String j = "{\"seg\":[{\"id\":0" + c + "}";
    j += rearSegs(c);
    j += "]}";
    sendJson(j);
  }

  // push and turn on encoder 2: effect speed for front and rear (the rear copies it from the front in syncRear)
  void changeSpeed(int clicks) {
    int sp = constrain((int)frontSeg().speed + clicks * constrain(speedStep, 1, 64), 0, 255);
    // Wipe-type effects work out their position as (clock % cycle length), so a new speed (new cycle length) would make them jump
    // back to some other point of the sweep. To keep the sweep where it is, the shared effect clock is shifted by the amount
    // that puts the new cycle at the same point as the old one. (All segments run the same effect, so shifting the clock is safe.)
    {
      String nm = modeName(frontSeg().mode);
      if (nm == "Wipe" || nm == "Sweep" || nm == "Wipe Random" || nm == "Sweep Random") {
        uint32_t oldC = 750 + (uint32_t)(255 - frontSeg().speed) * 150;
        uint32_t newC = 750 + (uint32_t)(255 - sp) * 150;
        uint32_t now  = (uint32_t)(millis() + strip.timebase);
        uint64_t keep = ((uint64_t)(now % oldC) * newC) / oldC;          // where in the new cycle we want to be
        int64_t  shift = (int64_t)keep - (int64_t)(now % newC);           // how far the clock must move
        strip.timebase = strip.timebase + (unsigned long)(int32_t)shift;
      }
    }
    // front and rear get the new speed in ONE command (like the web page's slider does). Sending it to the front first and letting
    // the rear catch up a moment later made the two run at different speeds for an instant, which showed up as flicker.
    String c = String(",\"sx\":") + String(sp);
    String j = String("{\"seg\":[{\"id\":0") + c + "}" + rearSegs(c) + "]}";
    sendJson(j);
  }

  void changeMix(int clicks) {
    mixPct = constrain(mixPct + clicks * constrain(mixStep, 1, 50), 0, 100);
    lastChange = millis();
  }

  void toggleOn(uint8_t id) {
    if (id == 1) {                                     // rear: remembered by the firmware, enforced in syncRear()
      rearUserOn = !rearUserOn;
      if (rearUserOn && rearPercent == 0) rearPercent = 25;   // switching on at 0% would show nothing
      lastChange = millis();
      return;
    }
    Segment &s = strip.getSegment(id);
    if (id == 0 && !s.on && heldBri >= 0) {            // switching the front on: apply the level chosen while it was off
      segCmd(0, String("\"bri\":") + String(heldBri) + ",\"on\":true");
      heldBri = -1;
      return;
    }
    heldBri = -1;
    segCmd(id, String("\"on\":") + (s.on ? "false" : "true"));
  }

  void nextEffect() {
    if (fxIds.empty()) return;
    Segment &f = frontSeg();
    size_t idx = 0;
    bool found = false;
    for (size_t i = 0; i < fxIds.size(); i++) if (fxIds[i] == f.mode) { idx = i; found = true; break; }
    size_t next = found ? (idx + 1) % fxIds.size() : 0;
    uint8_t fx = fxIds[next];
    String c = String("\"fx\":") + String((int)fx) + ",\"pal\":" + String(palForFx(fx)) + colJsonFor(fx);
    String j = "{\"seg\":[{\"id\":0," + c + "}";
    j += rearSegs(String(",") + c);
    j += "]}";
    sendJson(j);
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
    j += rearSegs(String(",\"pal\":") + p);   // palette is always shared
    j += "]}";
    sendJson(j);
  }

  // quick colour change: next solid colour on front and rear; palette set to "Color 1" (id 2)
  void nextColor() {
    if (colIds.empty()) return;
    sColor = (sColor + 1) % (int)colIds.size();
    sHue = -1;                                          // back to the colour list
    if (isStock(frontSeg().mode)) return;              // stock-colour effect: the choice is remembered for the next effect, nothing shown now
    String c = colJsonFor(frontSeg().mode) + ",\"pal\":" + String(palForFx(frontSeg().mode));
    String j = "{\"seg\":[{\"id\":0" + c + "}";
    j += rearSegs(c);
    j += "]}";
    sendJson(j);
  }

  // runs about 60 times per second: both rear strips always copy the front
  void syncRear() {
    if (!haveRear()) return;
    Segment &f = frontSeg();
    String all;
    for (uint8_t id = 1; id <= (haveRear2() ? 2 : 1); id++) {
      Segment &r = strip.getSegment(id);
      String fields;
      auto add = [&](const String &s) { if (fields.length()) fields += ","; fields += s; };
      bool d0 = r.colors[0] != f.colors[0], d1 = r.colors[1] != f.colors[1];
      if (d0 || d1) {                                  // one "col" entry; an empty [] leaves that colour unchanged
        add(String("\"col\":[") + (d0 ? rgbArr(f.colors[0]) : String("[]")) + "," + (d1 ? rgbArr(f.colors[1]) : String("[]")) + "]");
      }
      if (r.palette != f.palette) add(String("\"pal\":") + String((int)f.palette));
      if (r.mode != f.mode) add(String("\"fx\":") + String((int)f.mode));
      if (r.speed != f.speed) add(String("\"sx\":") + String((int)f.speed));
      if (r.intensity != f.intensity) add(String("\"ix\":") + String((int)f.intensity));
      // WLED switches a segment ON for an instant whenever it receives a brightness, so while the rear is meant to be off
      // it is never sent one. When it is switched back on, brightness and "on" go out together.
      bool wantOn = rearUserOn && rearPercent > 0;
      if (!wantOn) {
        if (r.on) add("\"on\":false");
      } else {
        int target = rearBriFor(frontBriNow());
        if ((int)r.opacity != target) { add(String("\"bri\":") + String(target)); add("\"on\":true"); }
        else if (!r.on) add("\"on\":true");
      }
      if (fields.length()) { if (all.length()) all += ","; all += String("{\"id\":") + String((int)id) + "," + fields + "}"; }
    }
    if (all.length()) sendJson(String("{\"seg\":[") + all + "]}");
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
    int16_t h0 = g_hue;
    if (h0) { g_hue = g_hue - h0; changeHue(h0); }
    int16_t d0 = g_delta[0];
    if (d0) { g_delta[0] = g_delta[0] - d0; changeFrontBrightness(d0); }
    uint8_t e0 = g_event[0];
    if (e0) {
      g_event[0] = 0;
      if (e0 == 1) nextEffect();
      else if (e0 == 2) toggleWide();
      else if (e0 == 3) toggleOn(0);
    }
    // ---- encoder 2: white mix / quick palette change / rear on-off ----
    int16_t s1 = g_speed;
    if (s1) { g_speed = g_speed - s1; changeSpeed(s1); }
    int16_t d1 = g_delta[1];
    if (d1) { g_delta[1] = g_delta[1] - d1; changeMix(d1); }
    uint8_t e1 = g_event[1];
    if (e1) {
      g_event[1] = 0;
      if (e1 == 1) nextColor();
      else if (e1 == 3 && haveRear()) toggleOn(1);
    }

    syncRear();

    // ---- remember state: save a few seconds after the last change (also catches changes from the web page) ----
    if (now - lastCheck > 500) {
      lastCheck = now;
      if (Bus::getGlobalAWMode() != RGBW_MODE_MANUAL_ONLY) Bus::setGlobalAWMode(RGBW_MODE_MANUAL_ONLY);   // WLED must not compute its own white, or the white-mix knob is overridden
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
    // WLED applies gamma correction after this point, so a channel at half value only gives off ~22% of its light.
    // To keep the total light output constant across the knob's range, colour and white are mixed in "light" terms
    // (colour share = 1 - t, white share = t) and then converted back to channel values with 1/gamma.
    const float t = mixPct / 100.0f;
    const float gam = (gammaCorrectCol && gammaCorrectVal > 0.5f) ? gammaCorrectVal : 1.0f;
    const float inv = 1.0f / gam;
    const float colorFactor = powf(1.0f - t, inv);                                     // 1.0 -> 0.0
    const float whiteFactor = powf(t * constrain(whiteLevel, 10, 200) / 100.0f, inv);  // 0.0 -> white strength
    const unsigned total = strip.getLengthTotal();
    for (unsigned i = 0; i < total; i++) {
      uint32_t c = strip.getPixelColor(i);
      if (!c) continue;
      int r = R(c), g = G(c), b = B(c), w = W(c);
      int v = max(r, max(g, b));                      // how bright the colour part is
      r = (int)(r * colorFactor + 0.5f);
      g = (int)(g * colorFactor + 0.5f);
      b = (int)(b * colorFactor + 0.5f);
      w = min(255, w + (int)(v * whiteFactor + 0.5f));
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
    top["speedStep"] = speedStep;
    top["dramaticShiftDegrees"] = wideDeg;
    top["rearBrightnessPercent"] = rearPercent;
    top["whiteLevelPercent"] = whiteLevel;
    top["longPressMs"] = longPressMs;
    top["doubleClickMs"] = doubleClickMs;
    top["rebuildSegmentsAtBoot"] = forceSegments;
    top["favoriteEffects"] = favEffects;
    top["favoritePalettes"] = favPalettes;
    top["favoriteColors"] = favColors;
    top["effectColors"] = effectColors;
    top["effectHueShift"] = effectHueShift;
    top["stockColorEffects"] = stockColorEffects;
    top["effectPalette"] = effectPalette;
    top["savedColorIndex"] = sColor;
    top["savedHue"] = sHue;
    top["savedFrontOn"] = sFrontOn;
    top["savedFrontBrightness"] = sFrontBri;
    top["savedRearOn"] = sRearOn;
    top["savedEffect"] = sMode;
    top["savedPalette"] = sPal;
    top["savedWhiteMixPercent"] = sMix;
    top["savedSpeed"] = sSpeed;
    top["savedDramaticShift"] = sWide;
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
    speedStep      = top["speedStep"] | speedStep;
    wideDeg        = top["dramaticShiftDegrees"] | wideDeg;
    rearPercent    = top["rearBrightnessPercent"] | rearPercent;
    whiteLevel     = top["whiteLevelPercent"] | whiteLevel;
    longPressMs    = top["longPressMs"] | longPressMs;
    doubleClickMs  = top["doubleClickMs"] | doubleClickMs;
    forceSegments  = top["rebuildSegmentsAtBoot"] | forceSegments;
    const char *fe = top["favoriteEffects"] | "";
    if (fe[0]) favEffects = fe;
    const char *fp = top["favoritePalettes"] | "";
    if (fp[0]) favPalettes = fp;
    const char *fc = top["favoriteColors"] | "";
    if (fc[0]) favColors = fc;
    const char *ec = top["effectColors"] | "";
    if (ec[0]) effectColors = ec;
    const char *eh = top["effectHueShift"] | "";
    if (eh[0]) effectHueShift = eh;
    if (!top["effectPalette"].isNull()) { const char *ep = top["effectPalette"]; if (ep) effectPalette = ep; }
    if (!top["stockColorEffects"].isNull()) { const char *sc = top["stockColorEffects"]; if (sc) stockColorEffects = sc; }
    if (!started) {                                   // saved state is only applied at boot
      sFrontOn  = top["savedFrontOn"] | sFrontOn;
      sFrontBri = top["savedFrontBrightness"] | sFrontBri;
      sRearOn   = top["savedRearOn"] | sRearOn;
      sMode     = top["savedEffect"] | sMode;
      sPal      = top["savedPalette"] | sPal;
      sMix      = top["savedWhiteMixPercent"] | sMix;
      sColor    = top["savedColorIndex"] | sColor;
      sHue      = top["savedHue"] | sHue;
      sSpeed    = top["savedSpeed"] | sSpeed;
      sWide     = top["savedDramaticShift"] | sWide;
    }
    favDirty = true;
    pushSettingsToTask();
    return !top["savedWhiteMixPercent"].isNull();     // false the first time, so WLED writes the new settings
  }
};

const char DigEncodersUsermod::_name[] PROGMEM = "DigEncoders";

static DigEncodersUsermod digEncodersUsermod;
REGISTER_USERMOD(digEncodersUsermod);
