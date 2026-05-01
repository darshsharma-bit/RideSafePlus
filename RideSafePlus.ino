// =============================================================
//  RideSafe+ — INDUSTRY-LEVEL STABLE VERSION
//  ESP32 + MPU6050 + Ultrasonic + OLED
//  Pothole and road anomaly detection via multi-sensor fusion.
// =============================================================

#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <math.h>

// ── OLED (hardware I2C) ───────────────────────────────────────
U8G2_SH1106_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

// ── I2C / SENSOR PINS ────────────────────────────────────────
#define I2C_SDA       4
#define I2C_SCL       5

#define MPU_ADDR      0x68
#define MPU_PWR_REG   0x6B
#define MPU_CONFIG    0x1A
#define MPU_SMPLRT    0x19
#define MPU_ACCEL_CFG 0x1C
#define MPU_ACCEL_REG 0x3B
#define LSB_PER_G     16384.0f

#define TRIG_PIN      12
#define ECHO_PIN      18
#define ULTRA_TIMEOUT  8000UL

// ── Timing ───────────────────────────────────────────────────
#define MPU_PERIOD_MS      20UL
#define ULTRA_PERIOD_MS    70UL
#define DISP_PERIOD_MS     80UL
#define SERIAL_PERIOD_MS  100UL

// ── Filtering / detection tuning ─────────────────────────────
#define EMA_FAST_ALPHA     0.45f
#define EMA_SLOW_ALPHA     0.08f
#define JERK_SMOOTH_ALPHA  0.30f

#define MIN_VALID_DIST     2.0f
#define MAX_VALID_DIST     400.0f

#define DIST_CHANGE_MIN    3.0f
#define DIST_CHANGE_MAX   20.0f
#define JERK_MIN_MARGIN    0.012f

#define CONFIRM_HITS       2
#define COOLDOWN_MS      1200UL

// ── Graph ────────────────────────────────────────────────────
#define GRAPH_W  128
#define GRAPH_H   28
#define GRAPH_Y0  62

// ── Distance smoothing ────────────────────────────────────────
#define DIST_HIST_N 5

// ── IMU state ────────────────────────────────────────────────
struct Imu {
  float emaFast = 0.0f;
  float emaSlow = 0.0f;
  float jerk    = 0.0f;
  float jerkSm  = 0.0f;
  float noise   = 0.01f;
  float grav    = 1.0f;
  bool  ok      = false;
};

struct Detection {
  bool          active     = false;
  float         severity   = 0.0f;
  float         confidence = 0.0f;
  uint32_t      count      = 0;
  unsigned long lastTime   = 0;
  const char*   label      = "LOW";
};

static Imu imu;
static Detection det;

static float dist = -1.0f;
static float prevDist = -1.0f;
static float prevPrevDist = -1.0f;

static float distHist[DIST_HIST_N];
static uint8_t distHistIdx = 0;
static uint8_t distHistCount = 0;

static int graph[GRAPH_W];
static int gIdx = 0;

static unsigned long lastMPU = 0;
static unsigned long lastUltra = 0;
static unsigned long lastDisp = 0;
static unsigned long lastSerial = 0;

static int confirmCount = 0;
static float lastJerkSm = 0.0f;

// =============================================================
// Small helpers
// =============================================================
static float clamp01(float x) {
  if (x < 0.0f) return 0.0f;
  if (x > 1.0f) return 1.0f;
  return x;
}

static float median3(float a, float b, float c) {
  if (a > b) {
    float t = a; a = b; b = t;
  }
  if (b > c) {
    float t = b; b = c; c = t;
  }
  if (a > b) {
    float t = a; a = b; b = t;
  }
  return b;
}

static float medianDistFromHistory() {
  if (distHistCount == 0) return -1.0f;

  float temp[DIST_HIST_N];
  uint8_t n = 0;

  for (uint8_t i = 0; i < distHistCount; i++) {
    float v = distHist[i];
    if (v >= MIN_VALID_DIST && v <= MAX_VALID_DIST) {
      temp[n++] = v;
    }
  }

  if (n == 0) return -1.0f;
  if (n == 1) return temp[0];

  for (uint8_t i = 1; i < n; i++) {
    float key = temp[i];
    int j = i - 1;
    while (j >= 0 && temp[j] > key) {
      temp[j + 1] = temp[j];
      j--;
    }
    temp[j + 1] = key;
  }

  if (n % 2) return temp[n / 2];
  return 0.5f * (temp[n / 2 - 1] + temp[n / 2]);
}

static void pushDistHistory(float v) {
  distHist[distHistIdx] = v;
  distHistIdx = (distHistIdx + 1) % DIST_HIST_N;
  if (distHistCount < DIST_HIST_N) distHistCount++;
}

// =============================================================
// MPU6050
// =============================================================
static bool mpuWriteReg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return (Wire.endTransmission(true) == 0);
}

static bool mpuInit() {
  if (!mpuWriteReg(MPU_PWR_REG, 0x00)) return false;
  delay(10);

  bool ok = true;
  ok &= mpuWriteReg(MPU_CONFIG, 0x03);
  ok &= mpuWriteReg(MPU_SMPLRT, 0x04);
  ok &= mpuWriteReg(MPU_ACCEL_CFG, 0x00);
  delay(10);
  return ok;
}

static bool mpuRead(int16_t &ax, int16_t &ay, int16_t &az) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_ACCEL_REG);
  if (Wire.endTransmission(false) != 0) return false;

  if (Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)6, (uint8_t)true) != 6) {
    return false;
  }

  ax = (int16_t)((Wire.read() << 8) | Wire.read());
  ay = (int16_t)((Wire.read() << 8) | Wire.read());
  az = (int16_t)((Wire.read() << 8) | Wire.read());
  return true;
}

static void calibrateImu() {
  const uint16_t N = 120;
  float sum = 0.0f;
  float sum2 = 0.0f;
  uint16_t good = 0;

  for (uint16_t i = 0; i < N; i++) {
    int16_t ax, ay, az;
    if (mpuRead(ax, ay, az)) {
      float mag = sqrtf((float)ax * ax + (float)ay * ay + (float)az * az) / LSB_PER_G;
      sum += mag;
      sum2 += mag * mag;
      good++;
    }
    delay(8);
  }

  if (good > 0) {
    float mean = sum / good;
    float var = (sum2 / good) - (mean * mean);
    if (var < 0.0f) var = 0.0f;

    imu.grav = mean;
    imu.noise = sqrtf(var);

    if (imu.noise < 0.004f) imu.noise = 0.004f;
    if (imu.noise > 0.060f) imu.noise = 0.060f;
  }
}

// =============================================================
// Ultrasonic
// =============================================================
static float readUltrasonicRaw() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(3);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long dur = pulseIn(ECHO_PIN, HIGH, ULTRA_TIMEOUT);
  if (dur <= 0) return -1.0f;

  float d = dur * 0.0343f * 0.5f;
  if (d < MIN_VALID_DIST || d > MAX_VALID_DIST) return -1.0f;
  return d;
}

static float ultraRead() {
  if (millis() - lastUltra < ULTRA_PERIOD_MS) return dist;
  lastUltra = millis();

  float a = readUltrasonicRaw();
  delayMicroseconds(200);
  float b = readUltrasonicRaw();
  delayMicroseconds(200);
  float c = readUltrasonicRaw();

  if (a < 0.0f) a = dist > 0.0f ? dist : 0.0f;
  if (b < 0.0f) b = dist > 0.0f ? dist : 0.0f;
  if (c < 0.0f) c = dist > 0.0f ? dist : 0.0f;

  float md = median3(a, b, c);

  if (md >= MIN_VALID_DIST && md <= MAX_VALID_DIST) {
    dist = md;
    pushDistHistory(dist);
  } else {
    dist = -1.0f;
  }

  return dist;
}

// =============================================================
// IMU update
// =============================================================
static void imuUpdate() {
  if (millis() - lastMPU < MPU_PERIOD_MS) return;
  lastMPU = millis();

  int16_t ax, ay, az;
  if (!mpuRead(ax, ay, az)) {
    imu.ok = false;
    mpuInit();
    return;
  }

  imu.ok = true;

  float mag = sqrtf((float)ax * ax + (float)ay * ay + (float)az * az) / LSB_PER_G;
  float vibration = fabsf(mag - imu.grav);

  imu.emaFast = EMA_FAST_ALPHA * vibration + (1.0f - EMA_FAST_ALPHA) * imu.emaFast;
  imu.emaSlow = EMA_SLOW_ALPHA * vibration + (1.0f - EMA_SLOW_ALPHA) * imu.emaSlow;

  float rawJerk = fabsf(imu.emaFast - imu.emaSlow);

  imu.jerkSm = JERK_SMOOTH_ALPHA * rawJerk + (1.0f - JERK_SMOOTH_ALPHA) * imu.jerkSm;
  imu.jerk = rawJerk;
}

// =============================================================
// Detection logic
// =============================================================
static void runDetection() {
  float d = ultraRead();

  float distRef = medianDistFromHistory();
  bool distValid = (d >= MIN_VALID_DIST && d <= MAX_VALID_DIST);

  float distDelta = 0.0f;
  bool distFeature = false;

  if (distValid && distRef > 0.0f) {
    distDelta = fabsf(d - distRef);
    distFeature = (distDelta >= DIST_CHANGE_MIN);
  }

  // Detect normal monotonic movement and reject it.
  bool monotonicTrend = false;
  bool reversal = false;

  if (distValid && prevDist > 0.0f && prevPrevDist > 0.0f) {
    const float eps = 0.8f;

    float s0 = prevDist - prevPrevDist;
    float s1 = d - prevDist;

    bool bothDown = (s0 < -eps && s1 < -eps);
    bool bothUp   = (s0 >  eps && s1 >  eps);

    monotonicTrend = (bothDown || bothUp);
    reversal = (s0 * s1 < -eps * eps);
  }

  // Motion feature using calibrated noise floor.
  float jerkScore = 0.0f;
  if (imu.ok) {
    float jerkAboveNoise = imu.jerkSm - fmaxf(imu.noise * 4.0f, JERK_MIN_MARGIN);
    jerkScore = clamp01(jerkAboveNoise / 0.08f);
  }

  float distScore = 0.0f;
  if (distFeature) {
    float x = (distDelta - DIST_CHANGE_MIN) / (DIST_CHANGE_MAX - DIST_CHANGE_MIN);
    distScore = clamp01(x);

    if (monotonicTrend) {
      distScore *= 0.15f;
    }

    if (reversal) {
      distScore = clamp01(distScore + 0.25f);
    }
  }

  // Impact score: sudden jerk acceleration matters too.
  float jerkRate = fabsf(imu.jerkSm - lastJerkSm);
  float impactScore = clamp01(jerkRate / 0.04f);
  lastJerkSm = imu.jerkSm;

  // Confidence from multiple signals.
  float confidence = 0.52f * jerkScore + 0.33f * distScore + 0.15f * impactScore;
  confidence = clamp01(confidence);

  // Final pothole pattern:
  // - not just monotonic approach/retreat
  // - must have distance anomaly
  // - must have jerk evidence
  // - reversal increases trust
  bool candidate =
      (imu.ok &&
       distFeature &&
       !monotonicTrend &&
       jerkScore > 0.35f &&
       confidence > 0.50f &&
       (reversal || distDelta > (DIST_CHANGE_MIN + 1.5f)));

  if (candidate) {
    confirmCount++;
  } else {
    confirmCount = 0;
  }

  bool confirmed = (confirmCount >= CONFIRM_HITS);

  if (confirmed && (millis() - det.lastTime > COOLDOWN_MS)) {
    det.active = true;
    det.confidence = confidence;
    det.severity = confidence * 10.0f;
    det.lastTime = millis();
    det.count++;

    if      (det.severity >= 7.0f) det.label = "HIGH";
    else if (det.severity >= 4.0f) det.label = "MED";
    else                            det.label = "LOW";
  }

  if (det.active && (millis() - det.lastTime > COOLDOWN_MS)) {
    det.active = false;
  }

  det.confidence = confidence;
  det.severity = confidence * 10.0f;

  // Update previous distances after decision
  if (distValid) {
    prevPrevDist = prevDist;
    prevDist = d;
  }
}

// =============================================================
// Display
// =============================================================
static void updateDisplay() {
  if (millis() - lastDisp < DISP_PERIOD_MS) return;
  lastDisp = millis();

  float liveSev = imu.jerkSm * 80.0f;
  liveSev = constrain(liveSev, 0.0f, 10.0f);

  int gVal = constrain((int)(liveSev / 10.0f * GRAPH_H), 0, GRAPH_H);
  graph[gIdx] = gVal;
  gIdx = (gIdx + 1) % GRAPH_W;

  char bufDist[20], bufSev[16], bufConf[16], bufCnt[16];

  if (dist < 0.0f) snprintf(bufDist, sizeof(bufDist), "D:---");
  else snprintf(bufDist, sizeof(bufDist), "D:%.0fcm", dist);

  snprintf(bufSev, sizeof(bufSev), "Sev:%.1f", liveSev);
  snprintf(bufConf, sizeof(bufConf), "C:%.0f%%", det.confidence * 100.0f);
  snprintf(bufCnt, sizeof(bufCnt), "#%lu", (unsigned long)det.count);

  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_6x10_tr);
    u8g2.drawStr(0, 9, "RideSafe+");
    u8g2.drawStr(72, 9, bufCnt);

    if (det.active) {
      u8g2.drawStr(0, 20, "POTHOLE!");
      u8g2.drawStr(56, 20, det.label);
    } else {
      u8g2.drawStr(0, 20, "Smooth");
    }

    u8g2.setFont(u8g2_font_5x7_tr);
    u8g2.drawStr(0, 30, bufDist);
    u8g2.drawStr(50, 30, bufSev);
    u8g2.drawStr(95, 30, bufConf);

    int barW = constrain((int)(liveSev * 6.0f), 0, 60);
    u8g2.drawFrame(0, 33, 62, 7);
    if (barW > 0) u8g2.drawBox(1, 34, barW, 5);

    u8g2.drawLine(0, GRAPH_Y0, 127, GRAPH_Y0);
    for (int i = 0; i < GRAPH_W - 1; i++) {
      int i1 = (gIdx + i) % GRAPH_W;
      int i2 = (gIdx + i + 1) % GRAPH_W;
      int y1 = GRAPH_Y0 - graph[i1];
      int y2 = GRAPH_Y0 - graph[i2];
      u8g2.drawLine(i, y1, i + 1, y2);
    }

    if (!imu.ok) {
      u8g2.drawStr(90, 42, "MPU?");
    }
    if (dist < 0.0f) {
      u8g2.drawStr(0, 42, "ULTRA?");
    }

  } while (u8g2.nextPage());
}

// =============================================================
// Serial log
// =============================================================
static void serialHeader() {
  Serial.println("ts_ms,dist_cm,distDelta,jerk,jerkSm,confidence,severity,pothole,count");
}

static void serialLog() {
  if (millis() - lastSerial < SERIAL_PERIOD_MS) return;
  lastSerial = millis();

  float distRef = medianDistFromHistory();
  float distDelta = 0.0f;
  if (dist > 0.0f && distRef > 0.0f) distDelta = fabsf(dist - distRef);

  Serial.print(millis()); Serial.print(',');
  Serial.print(dist, 1); Serial.print(',');
  Serial.print(distDelta, 2); Serial.print(',');
  Serial.print(imu.jerk, 4); Serial.print(',');
  Serial.print(imu.jerkSm, 4); Serial.print(',');
  Serial.print(det.confidence, 3); Serial.print(',');
  Serial.print(det.severity, 2); Serial.print(',');
  Serial.print(det.active ? 1 : 0); Serial.print(',');
  Serial.println(det.count);
}

// =============================================================
// Setup
// =============================================================
void setup() {
  Serial.begin(115200);
  delay(200);

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);
  Wire.setTimeout(50);

  u8g2.begin();

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);

  memset(graph, 0, sizeof(graph));
  memset(distHist, 0, sizeof(distHist));

  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_6x10_tr);
    u8g2.drawStr(10, 32, "RideSafe+ INIT");
  } while (u8g2.nextPage());

  imu.ok = mpuInit();
  if (!imu.ok) {
    Serial.println("[WARN] MPU6050 init failed.");
  }

  delay(300);
  calibrateImu();

  serialHeader();
  Serial.println("[INFO] Calibration done.");
  Serial.print("[INFO] Gravity baseline: "); Serial.println(imu.grav, 4);
  Serial.print("[INFO] Noise floor: "); Serial.println(imu.noise, 5);
}

// =============================================================
// Loop
// =============================================================
void loop() {
  imuUpdate();
  runDetection();
  updateDisplay();
  serialLog();
}
