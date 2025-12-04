#include <Arduino.h>
#include <TFT_eSPI.h>
#include <SPI.h>
#include <Wire.h>
#include <driver/adc.h>

// =============================================================================
// PIN DEFINITIONS
// =============================================================================

#define TFT_BL 32       // LCD Backlight control

#define I2C_SDA 21      // I2C Data
#define I2C_SCL 22      // I2C Clock

#define DAC_CS   5      // DAC8554 Chip Select (software SPI)
#define DAC_MOSI 26     // DAC8554 Data Out (avoids LCD conflict)
#define DAC_SCK  27     // DAC8554 Clock (avoids LCD conflict)

#define ADC_INPUT 34    // ADC1_CH6 - Photodiode/error signal input

// =============================================================================
// DAC ADDRESSES AND REGISTERS
// =============================================================================

#define DAC60501_ADDR     0x48  // Reference DAC I2C address
#define DAC60501_CONFIG   0x03
#define DAC60501_GAIN     0x04
#define DAC60501_TRIGGER  0x05
#define DAC60501_DAC_DATA 0x08

#define DAC8554_ADDR_BITS 0x00  // Device address (A1=0, A0=0)
#define DAC8554_LD_UPDATE 0x01  // Load and update mode

// =============================================================================
// GLOBAL OBJECTS
// =============================================================================

TFT_eSPI tft = TFT_eSPI();

// =============================================================================
// DAC60501 - 2.5V REFERENCE GENERATOR
// =============================================================================

void writeDAC60501(uint8_t reg, uint16_t value) {
  Wire.beginTransmission(DAC60501_ADDR);
  Wire.write(reg);
  Wire.write((uint8_t)(value >> 8));
  Wire.write((uint8_t)value);
  Wire.endTransmission();
}

void initReferenceDAC() {
  Serial.println("Configuring 2.5V reference...");
  writeDAC60501(DAC60501_CONFIG, 0x0000);    // Enable internal ref
  delay(10);
  writeDAC60501(DAC60501_GAIN, 0x0101);      // 1.25V × 2 = 2.5V
  delay(10);
  writeDAC60501(DAC60501_DAC_DATA, 0xFFF0);  // Full scale
  delay(10);
  writeDAC60501(DAC60501_TRIGGER, 0x0010);   // Trigger update
  delay(10);
  Serial.println("✓ 2.5V reference ready");
}

// =============================================================================
// DAC8554 - 4-CHANNEL OUTPUT DAC (SOFTWARE SPI)
// =============================================================================

void softSPI_begin() {
  pinMode(DAC_MOSI, OUTPUT);
  pinMode(DAC_SCK, OUTPUT);
  pinMode(DAC_CS, OUTPUT);
  digitalWrite(DAC_CS, HIGH);
  digitalWrite(DAC_SCK, LOW);
}

// Fast SPI bit-bang transfer
void softSPI_transfer_fast(uint8_t data) {
  for(int i = 7; i >= 0; i--) {
    digitalWrite(DAC_MOSI, (data >> i) & 1);
    digitalWrite(DAC_SCK, HIGH);
    digitalWrite(DAC_SCK, LOW);
  }
}

// Write to DAC8554 channel (0-3) with 16-bit value (0-65535 = 0-2.5V)
void writeDAC8554_fast(uint8_t channel, uint16_t value) {
  // Build 24-bit frame: [address|load|channel|powerdown|data]
  uint32_t frame = 0;
  frame |= ((uint32_t)DAC8554_ADDR_BITS << 22);
  frame |= ((uint32_t)DAC8554_LD_UPDATE << 20);
  frame |= ((uint32_t)channel << 17);
  frame |= value;
  
  digitalWrite(DAC_CS, LOW);
  softSPI_transfer_fast((frame >> 16) & 0xFF);
  softSPI_transfer_fast((frame >> 8) & 0xFF);
  softSPI_transfer_fast(frame & 0xFF);
  digitalWrite(DAC_CS, HIGH);
}

// =============================================================================
// ADC - CONTINUOUS SAMPLING WITH FIR FILTER
// =============================================================================

#define FIR_TAPS 17     // 17-tap symmetric filter
#define FIR_CENTER 8    // Middle tap

// Normalized FIR coefficients (sum = 1.0 for unity gain)
const float fir_coeff[FIR_CENTER + 1] = {
  0.0142, 0.0230, 0.0353, 0.0498, 0.0650, 0.0790, 0.0898, 0.0958, 0.0964
};

uint16_t fir_buffer[FIR_TAPS];
volatile uint8_t fir_index = 0;

hw_timer_t *adc_timer = NULL;

// Timer ISR - samples ADC at 10kHz
void IRAM_ATTR onTimer() {
  fir_buffer[fir_index] = adc1_get_raw(ADC1_CHANNEL_6);
  fir_index = (fir_index + 1) % FIR_TAPS;
}

void initADC() {
  Serial.println("Configuring ADC...");
  
  // Configure ADC for 12-bit, 0-3.3V range
  adc1_config_width(ADC_WIDTH_12Bit);
  adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_11db);
  
  // Initialize FIR buffer
  for(int i = 0; i < FIR_TAPS; i++) {
    fir_buffer[i] = 0;
  }
  
  // Setup 10kHz sampling timer
  adc_timer = timerBegin(0, 80, true);
  timerAttachInterrupt(adc_timer, &onTimer, true);
  timerAlarmWrite(adc_timer, 100, true);  // 100µs period
  timerAlarmEnable(adc_timer);
  
  Serial.println("✓ ADC: 10kHz sampling, 17-tap FIR");
}

// Apply symmetric FIR filter to get clean ADC reading
uint16_t applyFIR() {
  float output = 0.0;
  uint8_t idx = fir_index;
  
  for(int i = 0; i <= FIR_CENTER; i++) {
    int idx1 = (idx - i - 1 + FIR_TAPS) % FIR_TAPS;
    int idx2 = (idx - (FIR_TAPS - i) + FIR_TAPS) % FIR_TAPS;
    
    if(i == FIR_CENTER) {
      output += fir_buffer[idx1] * fir_coeff[i];
    } else {
      output += (fir_buffer[idx1] + fir_buffer[idx2]) * fir_coeff[i];
    }
  }
  
  return (uint16_t)output;
}

// Read filtered voltage (0-3.3V)
float readVoltage() {
  return (applyFIR() / 4095.0) * 3.3;
}

// =============================================================================
// SETUP
// =============================================================================

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n╔════════════════════════════════╗");
  Serial.println("║  Laser Lock - BGMAGLAB v1.0   ║");
  Serial.println("╚════════════════════════════════╝\n");
  
  // Initialize I2C and 2.5V reference
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);
  initReferenceDAC();
  
  // Initialize LCD
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);
  tft.init();
  tft.setRotation(3);
  tft.fillScreen(TFT_BLACK);
  
  // Splash screen
  const char* text1 = "Laser Lock";
  const char* text2 = "by Zoran D. Grujic";
  uint16_t colors[] = {TFT_GREEN, 0x07E0, TFT_CYAN, 0x05E6, TFT_DARKGREEN, 0x0560};
  
  tft.setTextSize(3);
  int x = 50, y = 50;
  for (size_t i = 0; i < strlen(text1); i++) {
    tft.setTextColor(colors[i % 6]);
    tft.setCursor(x, y);
    tft.print(text1[i]);
    x += 18;
  }
  
  tft.setTextSize(2);
  x = 30; y = 85;
  for (size_t i = 0; i < strlen(text2); i++) {
    tft.setTextColor(colors[i % 6]);
    tft.setCursor(x, y);
    tft.print(text2[i]);
    x += 12;
  }
  
  delay(3000);
  
  // Initialize 4-channel DAC
  softSPI_begin();
  for(int ch = 0; ch < 4; ch++) {
    writeDAC8554_fast(ch, 0);
    delay(10);
  }
  
  // Initialize ADC with FIR filter
  initADC();
  
  Serial.println("\n✓ System ready!\n");
}

// =============================================================================
// MAIN LOOP
// =============================================================================

void loop() {
  static int counter = 0;
  
  // Maintain 2.5V reference
  if(counter % 10 == 0) {
    writeDAC60501(DAC60501_CONFIG, 0x0000);
    writeDAC60501(DAC60501_GAIN, 0x0101);
    writeDAC60501(DAC60501_DAC_DATA, 0xFFF0);
  }
  
  // Test: sweep DAC output
  uint16_t dacValue = (counter * 4096) % 65536;
  writeDAC8554_fast(0, dacValue);
  float dacVoltage = (dacValue / 65535.0) * 2.5;
  
  delayMicroseconds(1600);  // Wait for FIR filter settling
  
  // Read filtered ADC value
  float adcVoltage = readVoltage();
  float error = adcVoltage - dacVoltage;
  
  // Update display
  tft.fillScreen(TFT_BLACK);
  tft.setTextSize(2);
  
  tft.setTextColor(TFT_YELLOW);
  tft.setCursor(10, 10);
  tft.print("System Test");
  
  tft.setTextColor(TFT_GREEN);
  tft.setCursor(10, 40);
  tft.printf("DAC: %.3fV", dacVoltage);
  
  tft.setTextColor(TFT_CYAN);
  tft.setCursor(10, 65);
  tft.printf("ADC: %.3fV", adcVoltage);
  
  tft.setTextColor(TFT_WHITE);
  tft.setCursor(10, 90);
  tft.printf("Err: %+.3fV", error);
  
  Serial.printf("%d,%.4f,%.4f,%+.4f\n", counter, dacVoltage, adcVoltage, error);
  
  counter++;
  delay(500);
}