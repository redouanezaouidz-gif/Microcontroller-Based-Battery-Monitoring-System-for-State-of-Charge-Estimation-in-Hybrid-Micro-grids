// ============================================================
//  Battery SOC Estimator — WT32-ETH01 (ESP32)
//  Sensors : INA3221 (voltage, I2C)  +  ACS712 (current, ADC)
//  Display : SSD1306 0.96" OLED (I2C)
//  Filter  : Standard Kalman Filter on current reading
//  Pins    : SDA=IO14  SCL=IO15  ADC=IO35
// ============================================================
//  Libraries needed (install via Arduino Library Manager):
//    - Adafruit_INA3221       (by Adafruit)
//    - Adafruit_SSD1306       (by Adafruit)
//    - Adafruit_GFX           (by Adafruit)
//    - Wire                   (built-in ESP32)
// ============================================================

#include <Wire.h>
#include <Adafruit_INA3221.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>

// ─── Pin definitions ────────────────────────────────────────
#define SDA_PIN       14        // IO14 → SDA
#define SCL_PIN       15        // IO15 → SCL
#define ACS712_PIN    35        // IO35 → ADC (voltage divider output)

// ─── OLED config ────────────────────────────────────────────
#define OLED_WIDTH    128
#define OLED_HEIGHT   64
#define OLED_ADDR     0x3C      // most common SSD1306 address
#define OLED_RESET    -1        // no reset pin used

// ─── INA3221 config ─────────────────────────────────────────
#define INA3221_ADDR  0x40      // default I2C address (A0=GND)
#define INA_CHANNEL   1         // CH1 used for battery voltage

// ─── ACS712 config ──────────────────────────────────────────
// ACS712-20A sensitivity = 100 mV/A
// Vcc of ACS712 = 5V  →  idle output = 2.5V  →  after divider ≈ 1.6V
// Divider ratio = R2/(R1+R2) = 12/(6.8+12) = 0.6383
// ADC reference = 3.3V, 12-bit = 4095 counts
#define ACS712_SENSITIVITY  0.100f   // V/A  (100 mV/A for 20A model)
#define DIVIDER_RATIO       0.6383f  // R2/(R1+R2)
#define ADC_VREF            3.3f
#define ADC_RESOLUTION      4095.0f
#define ACS712_IDLE_V       2.5f     // idle output voltage (no current)
#define ADC_SAMPLES         20       // averages per reading

// ─── Battery config (12V 7Ah Lead-Acid) ─────────────────────
#define BATTERY_CAPACITY_AH   7.0f   // Ah
#define BATTERY_FULL_V        12.7f  // fully charged open-circuit voltage
#define BATTERY_EMPTY_V       11.6f  // fully discharged (safe cutoff)

// ─── Kalman Filter state ─────────────────────────────────────
struct KalmanFilter {
  float Q;   // process noise covariance  (tune: higher = trust measurement more)
  float R;   // measurement noise covariance (tune: higher = smoother but slower)
  float P;   // estimation error covariance
  float K;   // Kalman gain
  float x;   // current state estimate (filtered value)
};

// ─── Global objects ──────────────────────────────────────────
Adafruit_INA3221 ina3221;
Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, OLED_RESET);
KalmanFilter kf;

// ─── Coulomb counting state ──────────────────────────────────
float  chargeRemaining_Ah = BATTERY_CAPACITY_AH;  // starts full
float  soc_percent        = 100.0f;
unsigned long lastTime_ms = 0;

// ============================================================
//  Kalman Filter — init
// ============================================================
void kalman_init(KalmanFilter &kf, float Q, float R, float initial_value) {
  kf.Q = Q;
  kf.R = R;
  kf.P = 1.0f;
  kf.x = initial_value;
  kf.K = 0.0f;
}

// ============================================================
//  Kalman Filter — update (one step)
// ============================================================
float kalman_update(KalmanFilter &kf, float measurement) {
  // Predict
  kf.P = kf.P + kf.Q;

  // Update (Kalman gain)
  kf.K = kf.P / (kf.P + kf.R);

  // Correct estimate
  kf.x = kf.x + kf.K * (measurement - kf.x);

  // Update error covariance
  kf.P = (1.0f - kf.K) * kf.P;

  return kf.x;
}

// ============================================================
//  Read ACS712 current (averaged ADC + Kalman filtered)
// ============================================================
float readCurrent_A() {
  long sum = 0;
  for (int i = 0; i < ADC_SAMPLES; i++) {
    sum += analogRead(ACS712_PIN);
    delayMicroseconds(200);
  }
  float adcAvg = (float)sum / ADC_SAMPLES;

  // Convert ADC count → voltage at IO35
  float v_io35 = (adcAvg / ADC_RESOLUTION) * ADC_VREF;

  // Undo voltage divider to get actual ACS712 output voltage
  float v_acs_out = v_io35 / DIVIDER_RATIO;

  // Convert ACS712 voltage → current
  // I = (Vout - Vidle) / sensitivity
  float current_A = (v_acs_out - ACS712_IDLE_V) / ACS712_SENSITIVITY;

  // Apply Kalman filter
  float filtered_A = kalman_update(kf, current_A);

  // Clamp very small noise around zero
  if (fabsf(filtered_A) < 0.05f) filtered_A = 0.0f;

  return filtered_A;
}

// ============================================================
//  Read battery voltage via INA3221 (Channel 1 bus voltage)
// ============================================================
float readVoltage_V() {
  float busV = ina3221.getBusVoltage_V(INA_CHANNEL);
  return busV;
}

// ============================================================
//  SOC from voltage (lookup table, lead-acid 12V)
//  Used only as initial reference / cross-check
// ============================================================
float soc_from_voltage(float v) {
  // Simple piecewise linear map for 12V lead-acid
  if (v >= BATTERY_FULL_V)  return 100.0f;
  if (v <= BATTERY_EMPTY_V) return 0.0f;
  return ((v - BATTERY_EMPTY_V) / (BATTERY_FULL_V - BATTERY_EMPTY_V)) * 100.0f;
}

// ============================================================
//  Update SOC via Coulomb Counting
// ============================================================
void update_soc_coulomb(float current_A, unsigned long now_ms) {
  float dt_h = (now_ms - lastTime_ms) / 3600000.0f;  // ms → hours
  lastTime_ms = now_ms;

  // Positive current = discharging (load draws from battery)
  chargeRemaining_Ah -= current_A * dt_h;

  // Clamp
  chargeRemaining_Ah = constrain(chargeRemaining_Ah, 0.0f, BATTERY_CAPACITY_AH);
  soc_percent        = (chargeRemaining_Ah / BATTERY_CAPACITY_AH) * 100.0f;
}

// ============================================================
//  OLED display update
// ============================================================
void updateDisplay(float voltage, float current, float soc) {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  // Title
  display.setTextSize(1);
  display.setCursor(10, 0);
  display.print(F("-- SOC MONITOR --"));

  // Voltage
  display.setTextSize(1);
  display.setCursor(0, 14);
  display.print(F("Voltage : "));
  display.setTextSize(2);
  display.setCursor(0, 22);
  display.print(voltage, 2);
  display.print(F(" V"));

  // Current
  display.setTextSize(1);
  display.setCursor(0, 40);
  display.print(F("Current : "));
  display.print(current, 2);
  display.print(F(" A"));

  // SOC bar
  display.setTextSize(1);
  display.setCursor(0, 52);
  display.print(F("SOC: "));
  display.print((int)soc);
  display.print(F("%  ["));
  int barLen = (int)(soc / 100.0f * 40);
  for (int i = 0; i < 40; i++) display.print(i < barLen ? '|' : ' ');
  display.print(F("]"));

  display.display();
}

// ============================================================
//  SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println(F("\n=== Battery SOC Monitor — WT32-ETH01 ==="));

  // Start I2C on custom pins IO14/IO15
  Wire.begin(SDA_PIN, SCL_PIN);

  // ── OLED init ──
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println(F("[ERROR] OLED not found! Check wiring."));
    while (true) delay(1000);
  }
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(20, 28);
  display.print(F("Initialising..."));
  display.display();
  Serial.println(F("[OK] OLED initialised"));

  // ── INA3221 init ──
  if (!ina3221.begin(INA3221_ADDR, &Wire)) {
    Serial.println(F("[ERROR] INA3221 not found! Check wiring."));
    while (true) delay(1000);
  }
  Serial.println(F("[OK] INA3221 initialised"));

  // ── ADC config ──
  analogReadResolution(12);          // 12-bit ADC (ESP32 default)
  analogSetAttenuation(ADC_11db);    // 0-3.3V input range
  Serial.println(F("[OK] ADC configured on IO35"));

  // ── Kalman filter init ──
  // Q=0.01 (low process noise), R=0.5 (moderate measurement noise)
  // Tune R higher for smoother output, lower for faster response
  kalman_init(kf, 0.01f, 0.5f, 0.0f);
  Serial.println(F("[OK] Kalman filter initialised"));

  // ── Seed SOC from voltage at startup ──
  float initV = readVoltage_V();
  soc_percent = soc_from_voltage(initV);
  chargeRemaining_Ah = (soc_percent / 100.0f) * BATTERY_CAPACITY_AH;
  lastTime_ms = millis();

  Serial.printf("[OK] Initial voltage: %.2f V  →  SOC seed: %.1f%%\n", initV, soc_percent);
  delay(1000);
}

// ============================================================
//  LOOP
// ============================================================
void loop() {
  unsigned long now = millis();

  // Read sensors
  float voltage = readVoltage_V();
  float current = readCurrent_A();   // Kalman filtered

  // Update coulomb counting
  update_soc_coulomb(current, now);

  // Print to Serial Monitor
  Serial.printf("V=%.3fV  I=%.3fA  SOC=%.1f%%  Charge=%.3fAh\n",
                voltage, current, soc_percent, chargeRemaining_Ah);

  // Update OLED
  updateDisplay(voltage, current, soc_percent);

  delay(500);   // 2 Hz update rate
}
