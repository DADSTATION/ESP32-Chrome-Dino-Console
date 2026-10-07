// ============================================================
// DAD STATION - Dino Runner (Chrome Dino style)
// ESP32-C3 SuperMini + SH1106 128x64 I2C OLED + TTP223 Touch
// + Active Buzzer
// ============================================================
// WIRING:
//   - SH1106 I2C SDA : GPIO 6
//   - SH1106 I2C SCL : GPIO 7   (I2C address 0x3C)
//   - TTP223 Touch   : GPIO 3   (active HIGH, plain INPUT)
//   - Active Buzzer  : GPIO 2   (active HIGH)
//
// OPERATION (game only, no clock mode, no physical button):
//   - Short tap on touch sensor (event on RELEASE, debounced 25 ms):
//       READY     -> start game
//       RUNNING   -> jump
//       GAME_OVER -> retry (after a short lockout)
//   - No long-press mode switching.
//
// TIMING:
//   - Everything is nonblocking (millis() based), no delay().
//   - Frame tick ~33 ms. Fixed-point physics preserved from the
//     previous sketch (jump velocity -2000, gravity 145/frame).
// ============================================================

#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>

// ---------------- Hardware & Pins ----------------

constexpr uint8_t OLED_SDA_PIN = 6;
constexpr uint8_t OLED_SCL_PIN = 7;
constexpr uint8_t OLED_ADDRESS_1 = 0x3C;
constexpr uint8_t OLED_ADDRESS_2 = 0x3D;

constexpr uint8_t TOUCH_PIN  = 3;  // TTP223, active HIGH
constexpr uint8_t BUZZER_PIN = 2;  // Active buzzer, active HIGH

constexpr int16_t SCREEN_WIDTH  = 128;
constexpr int16_t SCREEN_HEIGHT = 64;

Adafruit_SH1106G display(128, 64, &Wire, -1);

Preferences preferences;

bool displayAvailable = false;
uint8_t detectedOledAddress = 0;
uint32_t lastOledDiagnosticAt = 0;
constexpr uint32_t OLED_DIAGNOSTIC_INTERVAL_MS = 5000;

// ---------------- Timing Constants ----------------

constexpr uint32_t FRAME_INTERVAL_MS = 33;    // ~30 FPS
constexpr uint32_t DEBOUNCE_MS       = 25;    // touch debounce
constexpr uint32_t BOOT_SCREEN_MS    = 1300;  // boot splash duration
constexpr uint32_t RUN_ANIMATION_MS  = 150;   // dino leg animation
constexpr uint32_t SCORE_INTERVAL_MS = 100;   // score tick
constexpr uint32_t GAME_OVER_LOCK_MS = 450;   // retry lockout
constexpr uint32_t BEEP_SHORT_MS     = 40;    // jump/start/retry beep
constexpr uint32_t BEEP_GAMEOVER_MS  = 350;   // game-over beep

// ---------------- Game Physics & Layout ----------------

constexpr int16_t GROUND_Y = 56;
constexpr int16_t DINO_X = 10;
constexpr int16_t DINO_W = 32;
constexpr int16_t DINO_H = 32;
constexpr int16_t DINO_REST_Y = GROUND_Y - DINO_H;

constexpr int32_t PHYSICS_SCALE = 256;   // fixed point
constexpr int32_t JUMP_VELOCITY = -2000; // fixed-point units
constexpr int32_t GRAVITY_PER_FRAME = 145;

// ---------------- Game States ----------------

enum class GameState : uint8_t {
  BOOT,
  READY,
  RUNNING,
  GAME_OVER
};

GameState gameState = GameState::BOOT;

// ---------------- State Timing ----------------

uint32_t bootStartedAt   = 0;
uint32_t lastFrameAt     = 0;
uint32_t lastAnimationAt = 0;
uint32_t lastScoreAt     = 0;
uint32_t gameOverAt      = 0;

// ---------------- Touch Input (debounced, nonblocking) --------

bool touchRawPressed    = false;
bool touchStablePressed = false;
bool touchShortEvent    = false;   // short tap, event on release
uint32_t touchRawChangeAt = 0;

// ---------------- Buzzer (nonblocking service) ----------------

bool buzzerActive = false;
uint32_t buzzerStartAt = 0;
uint32_t buzzerDuration = 0;

// Nonblocking beep: turns buzzer ON, serviced (turned OFF) in loop.
void startBeep(uint32_t now, uint32_t durationMs) {
  digitalWrite(BUZZER_PIN, HIGH);
  buzzerActive = true;
  buzzerStartAt = now;
  buzzerDuration = durationMs;
}

// Turn the buzzer OFF when its beep time has elapsed (call each loop).
void serviceBuzzer(uint32_t now) {
  if (buzzerActive && (now - buzzerStartAt >= buzzerDuration)) {
    digitalWrite(BUZZER_PIN, LOW);
    buzzerActive = false;
  }
}

// ---------------- Player ----------------

int32_t playerYFixed = DINO_REST_Y * PHYSICS_SCALE;
int32_t playerVelocity = 0;
bool playerOnGround = true;
uint8_t runFrame = 0;

// ---------------- Score ----------------

uint32_t score = 0;
uint32_t highScore = 0;
bool newRecord = false;

// ---------------- Obstacles ----------------

enum class CactusType : uint8_t {
  SMALL,
  LEFT_ARM,
  RIGHT_ARM,
  TALL,
  DOUBLE
};

struct Cactus {
  int16_t x;
  CactusType type;
};

Cactus cactus = { SCREEN_WIDTH + 20, CactusType::SMALL };

uint16_t nextGapPixels = 48;
uint8_t speedFixed = 10;      // sub-pixel speed: pixels*4 per frame
int16_t obstacleSubpixel = 0;

// Clouds (parallax decoration)
struct Cloud {
  int16_t x;
  int16_t y;
  uint8_t speedDiv;           // higher = slower
};
Cloud clouds[3] = {
  { 100, 12, 6 },
  {  40, 22, 4 },
  { 150,  8, 5 }
};

// ============================================================
// Dinosaur Sprites - 32 x 32 pixels (MSB-first per row)
// ============================================================

const uint8_t PROGMEM DINO_SPRITE_STILL[32][4] = {
  {0x00, 0x00, 0x3F, 0xFC},
  {0x00, 0x00, 0xFF, 0xFE},
  {0x00, 0x00, 0xE7, 0xFE},
  {0x00, 0x00, 0xE7, 0xFE},
  {0x00, 0x00, 0xFF, 0xFE},
  {0x00, 0x00, 0xFF, 0xFE},
  {0x00, 0x00, 0xFF, 0xFE},
  {0x00, 0x00, 0xFF, 0xFE},
  {0x00, 0x00, 0xFF, 0x00},
  {0x00, 0x00, 0xFF, 0x00},
  {0x00, 0x00, 0xFF, 0xF0},
  {0x40, 0x03, 0xFC, 0x00},
  {0x40, 0x03, 0xFC, 0x00},
  {0x40, 0x0F, 0xFC, 0x00},
  {0x70, 0x3F, 0xFF, 0x80},
  {0x70, 0x3F, 0xFF, 0x80},
  {0x78, 0xFF, 0xFC, 0x80},
  {0x7F, 0xFF, 0xFC, 0x00},
  {0x7F, 0xFF, 0xFC, 0x00},
  {0x7F, 0xFF, 0xFC, 0x00},
  {0x3F, 0xFF, 0xFC, 0x00},
  {0x0F, 0xFF, 0xF8, 0x00},
  {0x0F, 0xFF, 0xF8, 0x00},
  {0x07, 0xFF, 0xE0, 0x00},
  {0x07, 0xFF, 0xE0, 0x00},
  {0x01, 0xFF, 0xC0, 0x00},
  {0x00, 0xFB, 0xC0, 0x00},
  {0x00, 0xFB, 0xC0, 0x00},
  {0x00, 0xE0, 0xC0, 0x00},
  {0x00, 0xC0, 0xC0, 0x00},
  {0x00, 0xE0, 0xE0, 0x00},
  {0x00, 0xE0, 0xE0, 0x00}
};

const uint8_t PROGMEM DINO_SPRITE_RUN1[32][4] = {
  {0x00, 0x00, 0x3F, 0xFC},
  {0x00, 0x00, 0xFF, 0xFE},
  {0x00, 0x00, 0xE7, 0xFE},
  {0x00, 0x00, 0xE7, 0xFE},
  {0x00, 0x00, 0xFF, 0xFE},
  {0x00, 0x00, 0xFF, 0xFE},
  {0x00, 0x00, 0xFF, 0xFE},
  {0x00, 0x00, 0xFF, 0xFE},
  {0x00, 0x00, 0xFF, 0x00},
  {0x00, 0x00, 0xFF, 0x00},
  {0x00, 0x00, 0xFF, 0xF0},
  {0x40, 0x03, 0xFC, 0x00},
  {0x40, 0x03, 0xFC, 0x00},
  {0x40, 0x0F, 0xFC, 0x00},
  {0x70, 0x3F, 0xFF, 0x80},
  {0x70, 0x3F, 0xFF, 0x80},
  {0x78, 0xFF, 0xFC, 0x80},
  {0x7F, 0xFF, 0xFC, 0x00},
  {0x7F, 0xFF, 0xFC, 0x00},
  {0x7F, 0xFF, 0xFC, 0x00},
  {0x3F, 0xFF, 0xFC, 0x00},
  {0x0F, 0xFF, 0xF8, 0x00},
  {0x0F, 0xFF, 0xF8, 0x00},
  {0x07, 0xFF, 0xE0, 0x00},
  {0x07, 0xFF, 0xE0, 0x00},
  {0x01, 0xFF, 0xC0, 0x00},
  {0x00, 0xFB, 0xC0, 0x00},
  {0x00, 0xFB, 0xC0, 0x00},
  {0x00, 0xE0, 0xC0, 0x00},
  {0x00, 0xC0, 0xC0, 0x00},
  {0x01, 0xE0, 0xE0, 0x00},
  {0x01, 0xE0, 0xE0, 0x00}
};

const uint8_t PROGMEM DINO_SPRITE_RUN2[32][4] = {
  {0x00, 0x00, 0x3F, 0xFC},
  {0x00, 0x00, 0xFF, 0xFE},
  {0x00, 0x00, 0xE7, 0xFE},
  {0x00, 0x00, 0xE7, 0xFE},
  {0x00, 0x00, 0xFF, 0xFE},
  {0x00, 0x00, 0xFF, 0xFE},
  {0x00, 0x00, 0xFF, 0xFE},
  {0x00, 0x00, 0xFF, 0xFE},
  {0x00, 0x00, 0xFF, 0x00},
  {0x00, 0x00, 0xFF, 0x00},
  {0x00, 0x00, 0xFF, 0xF0},
  {0x40, 0x03, 0xFC, 0x00},
  {0x40, 0x03, 0xFC, 0x00},
  {0x40, 0x0F, 0xFC, 0x00},
  {0x70, 0x3F, 0xFF, 0x80},
  {0x70, 0x3F, 0xFF, 0x80},
  {0x78, 0xFF, 0xFC, 0x80},
  {0x7F, 0xFF, 0xFC, 0x00},
  {0x7F, 0xFF, 0xFC, 0x00},
  {0x7F, 0xFF, 0xFC, 0x00},
  {0x3F, 0xFF, 0xFC, 0x00},
  {0x0F, 0xFF, 0xF8, 0x00},
  {0x0F, 0xFF, 0xF8, 0x00},
  {0x07, 0xFF, 0xE0, 0x00},
  {0x07, 0xFF, 0xE0, 0x00},
  {0x01, 0xFF, 0xC0, 0x00},
  {0x00, 0xFB, 0xC0, 0x00},
  {0x00, 0xFB, 0xC0, 0x00},
  {0x00, 0xE0, 0xC0, 0x00},
  {0x00, 0xC0, 0xC0, 0x00},
  {0x00, 0xE0, 0xE0, 0x40},
  {0x00, 0xE0, 0xE0, 0x40}
};

// ============================================================
// Obstacle Helpers
// ============================================================

// FIX: explicit forward declarations. Arduino's auto-prototype generator
// inserts prototypes for cactusWidth/cactusHeight near the top of the file,
// BEFORE the enum class CactusType is declared, so the generated prototypes
// fail with "'CactusType' does not name a type". Declaring them ourselves
// here (AFTER the enum) suppresses the auto-generated ones. Using uint8_t
// parameters additionally makes the sketches robust even if a stray
// auto-prototype is emitted.
int16_t cactusWidth(uint8_t type);
int16_t cactusHeight(uint8_t type);
int16_t cactusWidth(uint8_t type) {
  switch (static_cast<CactusType>(type)) {
    case CactusType::SMALL:     return 8;
    case CactusType::LEFT_ARM:  return 12;
    case CactusType::RIGHT_ARM: return 12;
    case CactusType::TALL:      return 10;
    case CactusType::DOUBLE:    return 18;
  }
  return 8;
}

int16_t cactusHeight(uint8_t type) {
  switch (static_cast<CactusType>(type)) {
    case CactusType::SMALL:     return 13;
    case CactusType::LEFT_ARM:  return 17;
    case CactusType::RIGHT_ARM: return 17;
    case CactusType::TALL:      return 20;
    case CactusType::DOUBLE:    return 15;
  }
  return 13;
}

// ============================================================
// Game Core
// ============================================================

void resetPlayer() {
  playerYFixed = DINO_REST_Y * PHYSICS_SCALE;
  playerVelocity = 0;
  playerOnGround = true;
  runFrame = 0;
}

void chooseNextObstacle() {
  cactus.type = static_cast<CactusType>(random(0, 5));
  const int16_t minGap = 44 + speedFixed / 4;
  const int16_t maxGap = 76 + speedFixed / 3;
  nextGapPixels = static_cast<uint16_t>(random(minGap, maxGap + 1));
  cactus.x = SCREEN_WIDTH + nextGapPixels;
  obstacleSubpixel = 0;
}

void startGame(uint32_t now) {
  score = 0;
  newRecord = false;
  speedFixed = 10;
  obstacleSubpixel = 0;

  resetPlayer();
  cactus.type = static_cast<CactusType>(random(0, 5));
  cactus.x = SCREEN_WIDTH + 22;

  // Restart clouds at varied positions
  clouds[0].x = 100; clouds[1].x = 40; clouds[2].x = 150;

  lastScoreAt = now;
  lastAnimationAt = now;
  gameState = GameState::RUNNING;

  startBeep(now, BEEP_SHORT_MS);   // start / retry beep
}

void jump(uint32_t now) {
  if (!playerOnGround) {
    return;
  }
  playerVelocity = JUMP_VELOCITY;
  playerOnGround = false;
  startBeep(now, BEEP_SHORT_MS);   // jump beep
}

void finishGame(uint32_t now) {
  gameState = GameState::GAME_OVER;
  gameOverAt = now;

  if (score > highScore) {
    highScore = score;
    newRecord = true;
    preferences.putUInt("high", highScore);   // namespace dad-dino
  }

  startBeep(now, BEEP_GAMEOVER_MS);  // distinct game-over beep
}

void handleGameAction(uint32_t now) {
  switch (gameState) {
    case GameState::BOOT:
      break;

    case GameState::READY:
      startGame(now);
      jump(now);
      break;

    case GameState::RUNNING:
      jump(now);
      break;

    case GameState::GAME_OVER:
      if (now - gameOverAt >= GAME_OVER_LOCK_MS) {
        startGame(now);
        jump(now);
      }
      break;
  }
}

// ============================================================
// Input Handling (debounced TTP223, short tap on release)
// ============================================================

void updateInputs(uint32_t now) {
  touchShortEvent = false;

  const bool tchRaw = (digitalRead(TOUCH_PIN) == HIGH);
  if (tchRaw != touchRawPressed) {
    touchRawPressed = tchRaw;
    touchRawChangeAt = now;
  }
  // Debounce: only accept stable state after DEBOUNCE_MS
  if (tchRaw != touchStablePressed && (now - touchRawChangeAt >= DEBOUNCE_MS)) {
    touchStablePressed = tchRaw;
    if (!touchStablePressed) {
      // Short tap event fires on RELEASE
      touchShortEvent = true;
    }
    // No long-press handling: every tap is treated the same.
  }
}

void processActions(uint32_t now) {
  if (touchShortEvent) {
    handleGameAction(now);
  }
}

// ============================================================
// Game Logic Updates
// ============================================================

void updatePlayer() {
  if (playerOnGround) {
    return;
  }

  playerVelocity += GRAVITY_PER_FRAME;
  playerYFixed += playerVelocity;

  const int32_t groundPosition = DINO_REST_Y * PHYSICS_SCALE;
  if (playerYFixed >= groundPosition) {
    playerYFixed = groundPosition;
    playerVelocity = 0;
    playerOnGround = true;
  }
}

void updateObstacle() {
  obstacleSubpixel += speedFixed;

  const int16_t wholePixels = obstacleSubpixel / 4;
  obstacleSubpixel %= 4;

  cactus.x -= wholePixels;

  if (cactus.x + cactusWidth(static_cast<uint8_t>(cactus.type)) < 0) {
    chooseNextObstacle();
  }
}

// Axis-aligned bounding-box collision test.
bool playerHitsCactus() {
  const int16_t playerY = playerYFixed / PHYSICS_SCALE;

  // Dino collision box (inset so grazes feel fair)
  const int16_t dinoLeft   = DINO_X + 8;
  const int16_t dinoRight  = DINO_X + DINO_W - 1;
  const int16_t dinoTop    = playerY + 2;
  const int16_t dinoBottom = playerY + DINO_H - 1;

  // Cactus collision box (slightly inset)
  const int16_t cactusLeft   = cactus.x + 1;
  const int16_t cactusRight  = cactus.x + cactusWidth(static_cast<uint8_t>(cactus.type)) - 1;
  const int16_t cactusTop    = GROUND_Y - cactusHeight(static_cast<uint8_t>(cactus.type)) + 2;
  const int16_t cactusBottom = GROUND_Y;

  return (
    dinoLeft < cactusRight &&
    dinoRight > cactusLeft &&
    dinoTop < cactusBottom &&
    dinoBottom > cactusTop
  );
}

void updateClouds() {
  for (uint8_t i = 0; i < 3; i++) {
    // Move each cloud once every speedDiv frames for parallax
    if ((lastFrameAt / FRAME_INTERVAL_MS) % clouds[i].speedDiv == 0) {
      clouds[i].x -= 1;
      if (clouds[i].x < -24) {
        clouds[i].x = SCREEN_WIDTH + random(8, 60);
        clouds[i].y = random(6, 30);
      }
    }
  }
}

void updateRunningGame(uint32_t now) {
  updatePlayer();
  updateObstacle();
  updateClouds();

  if (playerHitsCactus()) {
    finishGame(now);
    return;
  }

  if (now - lastAnimationAt >= RUN_ANIMATION_MS) {
    lastAnimationAt = now;
    if (playerOnGround) {
      runFrame ^= 1;
    }
  }

  if (now - lastScoreAt >= SCORE_INTERVAL_MS) {
    lastScoreAt = now;
    ++score;
    // Speed progression: 10 -> 22 over time
    speedFixed = static_cast<uint8_t>(min(22UL, 10UL + score / 40UL));
  }
}

// ============================================================
// Drawing & Rendering (monochrome)
// ============================================================

constexpr uint16_t COLOR_ON  = SH110X_WHITE;
constexpr uint16_t COLOR_OFF = SH110X_BLACK;

void drawCloud(int16_t x, int16_t y) {
  display.drawPixel(x + 1, y + 4, COLOR_ON);
  display.drawLine(x + 2, y + 4, x + 4, y + 4, COLOR_ON);
  display.drawPixel(x + 5, y + 3, COLOR_ON);
  display.drawLine(x + 6, y + 2, x + 8, y + 2, COLOR_ON);
  display.drawPixel(x + 9, y + 1, COLOR_ON);
  display.drawLine(x + 10, y, x + 13, y, COLOR_ON);
  display.drawPixel(x + 14, y + 1, COLOR_ON);
  display.drawLine(x + 15, y + 2, x + 17, y + 2, COLOR_ON);
  display.drawPixel(x + 18, y + 3, COLOR_ON);
  display.drawLine(x + 19, y + 4, x + 23, y + 4, COLOR_ON);
}

void drawGround(uint32_t now) {
  display.drawFastHLine(0, GROUND_Y, SCREEN_WIDTH, COLOR_ON);

  // Moving ground texture (nonblocking, driven by millis)
  const int16_t groundShift = (now / 85) % 24;

  for (int16_t x = -24; x < SCREEN_WIDTH + 24; x += 24) {
    const int16_t movingX = x - groundShift;
    display.drawFastHLine(movingX + 3, GROUND_Y + 3, 3, COLOR_ON);
    display.drawPixel(movingX + 15, GROUND_Y + 5, COLOR_ON);
  }
}

void drawDino() {
  const int16_t playerY = playerYFixed / PHYSICS_SCALE;

  const uint8_t (*sprite)[4];

  if (!playerOnGround) {
    sprite = DINO_SPRITE_STILL;
  } else {
    sprite = (runFrame == 0) ? DINO_SPRITE_RUN1 : DINO_SPRITE_RUN2;
  }

  for (uint8_t row = 0; row < DINO_H; row++) {
    for (uint8_t col = 0; col < DINO_W; col++) {
      const uint8_t byteValue = pgm_read_byte(&sprite[row][col / 8]);
      const uint8_t bitMask   = 0x80 >> (col % 8);

      if (byteValue & bitMask) {
        display.drawPixel(DINO_X + col, playerY + row, COLOR_ON);
      }
    }
  }
}

void drawCactus() {
  const int16_t x = cactus.x;
  const int16_t bottom = GROUND_Y;

  switch (cactus.type) {
    case CactusType::SMALL:      // small column with left arm
      display.fillRect(x + 3, bottom - 13, 4, 13, COLOR_ON);
      display.fillRect(x,     bottom - 8,  3, 3,  COLOR_ON);
      display.fillRect(x + 1, bottom - 8,  2, 6,  COLOR_ON);
      break;

    case CactusType::LEFT_ARM:   // tall column, arm on the left
      display.fillRect(x + 6, bottom - 17, 4, 17, COLOR_ON);
      display.fillRect(x + 1, bottom - 11, 3, 7,  COLOR_ON);
      display.fillRect(x + 3, bottom - 6,  4, 3,  COLOR_ON);
      break;

    case CactusType::RIGHT_ARM:  // tall column, arm on the right
      display.fillRect(x + 2, bottom - 17, 4, 17, COLOR_ON);
      display.fillRect(x + 8, bottom - 12, 3, 7,  COLOR_ON);
      display.fillRect(x + 5, bottom - 7,  4, 3,  COLOR_ON);
      break;

    case CactusType::TALL:       // big branching cactus
      display.fillRect(x + 3, bottom - 20, 4, 20, COLOR_ON);
      display.fillRect(x,     bottom - 13, 3, 20, COLOR_ON);
      display.fillRect(x + 7, bottom - 16, 3, 8,  COLOR_ON);
      display.fillRect(x + 1, bottom - 8,  8, 3,  COLOR_ON);
      break;

    case CactusType::DOUBLE:     // two columns
      display.fillRect(x + 2,  bottom - 12, 4, 12, COLOR_ON);
      display.fillRect(x + 11, bottom - 15, 4, 15, COLOR_ON);
      display.fillRect(x,      bottom - 7,  3, 3,  COLOR_ON);
      display.fillRect(x + 15, bottom - 9,  3, 3,  COLOR_ON);
      break;
  }
}

void drawScore() {
  char scoreText[24];
  snprintf(scoreText, sizeof(scoreText), "HI %05lu %05lu",
           static_cast<unsigned long>(highScore % 100000UL),
           static_cast<unsigned long>(score % 100000UL));

  display.setTextSize(1);
  display.setTextColor(COLOR_ON);
  display.setCursor(20, 2);
  display.print(scoreText);
}

void drawCenteredText(const char* text, int16_t y, uint16_t color, uint8_t textSize) {
  display.setTextSize(textSize);
  display.setTextColor(color);

  int16_t x1, y1;
  uint16_t width, height;
  display.getTextBounds(text, 0, y, &x1, &y1, &width, &height);

  const int16_t x = (SCREEN_WIDTH - width) / 2;
  display.setCursor(x, y);
  display.print(text);
}

void drawBootScreen() {
  drawCenteredText("DAD STATION", 20, COLOR_ON, 2);
  drawCenteredText("DINO RUNNER", 45, COLOR_ON, 1);
}

void drawClouds() {
  for (uint8_t i = 0; i < 3; i++) {
    drawCloud(clouds[i].x, clouds[i].y);
  }
}

void drawReadyScreen(uint32_t now) {
  drawScore();
  drawClouds();
  drawGround(now);
  drawDino();
  drawCenteredText("TAP TO START", 14, COLOR_ON, 1);
}

void drawRunningScreen(uint32_t now) {
  drawScore();
  drawClouds();
  drawGround(now);
  drawDino();
  drawCactus();
}

void drawGameOverScreen(uint32_t now) {
  drawRunningScreen(now);

  display.fillRect(15, 17, 99, 29, COLOR_ON);
  display.drawRect(15, 17, 99, 29, COLOR_OFF);

  drawCenteredText("GAME OVER", 20, COLOR_OFF, 1);

  if (newRecord) {
    drawCenteredText("NEW RECORD!", 33, COLOR_OFF, 1);
  } else {
    drawCenteredText("TAP TO RETRY", 33, COLOR_OFF, 1);
  }
}

void render(uint32_t now) {
  if (!displayAvailable) {
    return;  // Never call display drawing APIs if initialization failed.
  }
  display.clearDisplay();

  switch (gameState) {
    case GameState::BOOT:
      drawBootScreen();
      break;

    case GameState::READY:
      drawReadyScreen(now);
      break;

    case GameState::RUNNING:
      drawRunningScreen(now);
      break;

    case GameState::GAME_OVER:
      drawGameOverScreen(now);
      break;
  }

  display.display();
}

// ============================================================
// OLED probe and diagnostics
// ============================================================

bool probeI2CAddress(uint8_t address) {
  Wire.beginTransmission(address);
  return Wire.endTransmission() == 0;
}

void printNoOledDiagnosis(uint32_t now) {
  if (now - lastOledDiagnosticAt < OLED_DIAGNOSTIC_INTERVAL_MS) {
    return;
  }
  lastOledDiagnosticAt = now;
  Serial.println(F("DIAGNOSIS: no OLED initialized; game/touch/buzzer loop remains active."));
  Serial.println(F("Check OLED power, common GND, SDA=GPIO6, SCL=GPIO7, wiring and I2C address (0x3C/0x3D)."));
  Serial.println(F("If the module is SSD1306 rather than SH1106, use the matching driver/library."));
}

// ============================================================
// Arduino Setup & Loop
// ============================================================

void setup() {
  Serial.begin(115200);

  // Drive the active-high buzzer low as soon as possible after boot.
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  pinMode(TOUCH_PIN, INPUT);  // TTP223 active HIGH

  const uint32_t setupNow = millis();
  startBeep(setupNow, 70);  // brief nonblocking boot life-sign chirp

  Serial.println();
  Serial.println(F("DAD STATION Dino diagnostics starting"));
  Serial.println(F("I2C pins: SDA GPIO6, SCL GPIO7; probing 0x3C and 0x3D."));

  Wire.begin(OLED_SDA_PIN, OLED_SCL_PIN);
  Wire.setClock(100000);

  const bool responds3C = probeI2CAddress(OLED_ADDRESS_1);
  const bool responds3D = probeI2CAddress(OLED_ADDRESS_2);
  if (responds3C) Serial.println(F("I2C ACK detected at 0x3C."));
  if (responds3D) Serial.println(F("I2C ACK detected at 0x3D."));

  // Probe first to avoid needlessly initializing at an address with no device.
  // Prefer 0x3C. If it responds but begin fails, try 0x3D only if it also ACKed.
  bool begin3CFailed = false;
  if (responds3C) {
    if (display.begin(OLED_ADDRESS_1, true)) {
      displayAvailable = true;
      detectedOledAddress = OLED_ADDRESS_1;
    } else {
      begin3CFailed = true;
      Serial.println(F("OLED ACK at 0x3C, but display.begin(0x3C) failed."));
    }
  }

  if (!displayAvailable && responds3D) {
    if (display.begin(OLED_ADDRESS_2, true)) {
      displayAvailable = true;
      detectedOledAddress = OLED_ADDRESS_2;
    } else {
      Serial.println(F("OLED ACK at 0x3D, but display.begin(0x3D) failed."));
    }
  }

  if (displayAvailable) {
    Serial.print(F("SH1106 display initialized at I2C address 0x"));
    Serial.println(detectedOledAddress, HEX);
    if (detectedOledAddress == OLED_ADDRESS_2 && !responds3C) {
      Serial.println(F("Using 0x3D fallback address."));
    }
    display.clearDisplay();
    display.setRotation(0);
    display.display();
  } else {
    if (!responds3C && !responds3D) {
      Serial.println(F("NO OLED RESPONSE at 0x3C or 0x3D; check wiring/power/address."));
    } else if (begin3CFailed || responds3D) {
      Serial.println(F("I2C device responded, but SH1106 display initialization failed."));
    }
    Serial.println(F("A controller mismatch (SSD1306 vs SH1106) is also possible."));
    Serial.println(F("Continuing without OLED: touch, game state and buzzer remain active."));
  }

  // Load high score from namespace "dad-dino", key "high".
  preferences.begin("dad-dino", false);
  highScore = preferences.getUInt("high", 0);

  randomSeed(esp_random());
  bootStartedAt = millis();
  lastFrameAt = bootStartedAt;
  lastOledDiagnosticAt = bootStartedAt - OLED_DIAGNOSTIC_INTERVAL_MS;

  // Shows the existing boot screen only when display.begin succeeded.
  render(bootStartedAt);
}

void loop() {
  const uint32_t now = millis();

  // Nonblocking buzzer service (including the brief boot chirp).
  serviceBuzzer(now);

  // Nonblocking debounced touch input.
  updateInputs(now);

  // BOOT -> READY after splash duration, whether or not OLED is present.
  if (gameState == GameState::BOOT && (now - bootStartedAt >= BOOT_SCREEN_MS)) {
    gameState = GameState::READY;
  }

  processActions(now);

  if (!displayAvailable) {
    printNoOledDiagnosis(now);
  }

  // Frame tick: game simulation continues even without a display.
  if (now - lastFrameAt >= FRAME_INTERVAL_MS) {
    lastFrameAt = now;
    if (gameState == GameState::RUNNING) {
      updateRunningGame(now);
    }
    render(now);  // guarded internally when OLED initialization failed
  }
}
