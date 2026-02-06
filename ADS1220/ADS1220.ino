#include <ADS1220_WE.h>
#include <SPI.h>

/* ------------------- ADS1220 Pins ------------------- */
#define ADS1220_CS_PIN     10
#define ADS1220_DRDY_PIN    9

ADS1220_WE ads(ADS1220_CS_PIN, ADS1220_DRDY_PIN);

/* ------------------- MUX Pins (CD74HC4067) ------------------- */
#define MUX_S0 2
#define MUX_S1 3
#define MUX_S2 4
#define MUX_S3 5

#define NUM_SENSORS 2   // <-- USER SETS THIS (1–16)
#define MA_WINDOW 100      // moving average window size
#define EMA_ALPHA    0.1   // 0 < alpha <= 1
#define MEDIAN_WIN   3


float ma_buffer[NUM_SENSORS][MA_WINDOW];
uint8_t ma_index = 0;
bool ma_filled = false;

float ema_state[NUM_SENSORS];
bool ema_initialized[NUM_SENSORS] = {false};

float median_buf[NUM_SENSORS][MEDIAN_WIN];
uint8_t median_idx[NUM_SENSORS] = {0};
bool median_initialized[NUM_SENSORS] = {false};



/* Select MUX channel 0–15 */
void selectMux(uint8_t ch) {
  digitalWrite(MUX_S0, (ch & 0x01) ? HIGH : LOW);
  digitalWrite(MUX_S1, (ch & 0x02) ? HIGH : LOW);
  digitalWrite(MUX_S2, (ch & 0x04) ? HIGH : LOW);
  digitalWrite(MUX_S3, (ch & 0x08) ? HIGH : LOW);
}

float convertReadingtoResistance(float reading) {
  float V_ex = 5.0;
  float R = 1000.;
  float TrimPot = 2.8;

  float Tuned = R + TrimPot;
  float V_bridge = reading / 1000.;

  float R_x =  - R * (V_bridge * (R + Tuned) + R * V_ex) / (V_bridge * (R + Tuned) - R * V_ex);
  R_x += TrimPot;

  return abs(R_x - 1000);

}


float movingAverage(uint8_t ch, float new_value) {
  ma_buffer[ch][ma_index] = new_value;

  float sum = 0.0;
  uint8_t count = ma_filled ? MA_WINDOW : (ma_index + 1);

  for (uint8_t i = 0; i < count; i++) {
    sum += ma_buffer[ch][i];
  }

  return sum / count;
}

float emaFilter(uint8_t ch, float x) {
  if (!ema_initialized[ch]) {
    ema_state[ch] = x;          // initialize per channel
    ema_initialized[ch] = true;
    return x;
  }

  ema_state[ch] = EMA_ALPHA * x + (1.0 - EMA_ALPHA) * ema_state[ch];
  return ema_state[ch];
}

inline float median3(float a, float b, float c) {
  if (a > b) {
    float t = a;
    a = b;
    b = t;
  }
  if (b > c) {
    float t = b;
    b = c;
    c = t;
  }
  if (a > b) {
    float t = a;
    a = b;
    b = t;
  }
  return b;
}

float medianFilter(uint8_t ch, float x) {

  // --- Initialize median buffer with first real sample ---
  if (!median_initialized[ch]) {
    for (uint8_t i = 0; i < MEDIAN_WIN; i++) {
      median_buf[ch][i] = x;
    }
    median_initialized[ch] = true;
    return x;
  }

  median_buf[ch][median_idx[ch]] = x;
  median_idx[ch] = (median_idx[ch] + 1) % MEDIAN_WIN;

  return median3(
    median_buf[ch][0],
    median_buf[ch][1],
    median_buf[ch][2]
  );
}





/* ------------------- Setup ------------------- */
void setup() {
  Serial.begin(115200);
  delay(200);

  /* MUX pins */
  pinMode(MUX_S0, OUTPUT);
  pinMode(MUX_S1, OUTPUT);
  pinMode(MUX_S2, OUTPUT);
  pinMode(MUX_S3, OUTPUT);

  /* Init SPI + ADS1220 */
  SPI.begin();
  if (!ads.init()) {
    Serial.println("ADS1220 is not connected!");
    while (1);
  }

  // ads.bypassPGA(false);                      // disable PGA for safety
  /* ADS1220 configuration */

  ads.setCompareChannels(ADS1220_MUX_1_2);
  ads.setVRefSource(ADS1220_VREF_REFP1_REFN1);
  ads.setVRefValue_V(5.0);
  ads.setGain(ADS1220_GAIN_8);              // gain = 1
  ads.setDataRate(ADS1220_DR_LVL_5);        // ~20 SPS
  ads.setOperatingMode(ADS1220_TURBO_MODE);
  ads.setConversionMode(ADS1220_CONTINUOUS);
  ads.setFIRFilter(ADS1220_50HZ_60HZ);


  Serial.println("ADS1220 initialized.");

  delay(100);

  //   selectMux(0);
  //   delay(100);
  //
}

/* ------------------- Main Loop ------------------- */
//void loop() { // Moving Average
//
//  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
//
//    selectMux(i);
//    delay(50);
//
//    float mv = ads.getVoltage_mV();
//    float R  = convertReadingtoResistance(mv);
//
//    /* ---------- Apply moving average ---------- */
//    float R_filt = movingAverage(i, R);
//
//    Serial.print(R_filt, 7);
//
//    if (i < NUM_SENSORS - 1) {
//      Serial.print(",");
//    }
//  }
//
//  Serial.println();
//
//  /* ---------- Update MA index ---------- */
//  ma_index++;
//  if (ma_index >= MA_WINDOW) {
//    ma_index = 0;
//    ma_filled = true;
//  }
//}

void loop() {  // Exponential Moving Average (LPF)

  for (uint8_t i = 0; i < NUM_SENSORS; i++) {

    selectMux(i);
    delay(3);

    float mv = ads.getVoltage_mV();
    float R  = convertReadingtoResistance(mv);

    // ---- Median → EMA ----
    float R_med  = medianFilter(i, R);
    float R_filt = emaFilter(i, R_med);

    Serial.print(R_filt, 5);
    if (i < NUM_SENSORS - 1) Serial.print(",");
  }

  Serial.println();
}
