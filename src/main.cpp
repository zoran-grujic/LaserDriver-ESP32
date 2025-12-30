#include <Arduino.h>
#include <TFT_eSPI.h>
#include <SPI.h>
#include <Wire.h>
#include <driver/adc.h>
#include <PID_v1.h>

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
  Serial.println("Configuring 1V reference...");
  writeDAC60501(DAC60501_CONFIG, 0x0000);    // Enable internal ref
  delay(10);
  writeDAC60501(DAC60501_GAIN, 0x0100);      // 1.25V × 1 = 1.25V (buffered)
  delay(10);
  writeDAC60501(DAC60501_DAC_DATA, 0xCCC0);  // 80% of full scale = 1.0V
  delay(10);
  writeDAC60501(DAC60501_TRIGGER, 0x0010);   // Trigger update
  delay(10);
  Serial.println("✓ 1V reference ready");
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

// Write to DAC8554 channel (0-3) with 16-bit value (0-65535 = 0-1.0V)
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
uint16_t _fir_buffer[FIR_TAPS];
const uint8_t fir_buffer_size = sizeof(fir_buffer);
int fir_index = 0;

hw_timer_t *adc_timer = NULL;
bool locked = false;

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
  memcpy(_fir_buffer, fir_buffer, fir_buffer_size);
  uint8_t idx = fir_index;
  
  for(int i = 0; i <= FIR_CENTER; i++) 
  {
    int idx1 = (idx - i - 1 + FIR_TAPS) % FIR_TAPS;
    int idx2 = (idx - (FIR_TAPS - i) + FIR_TAPS) % FIR_TAPS;
    
    if(i == FIR_CENTER) {
      output += _fir_buffer[idx1] * fir_coeff[i];
    } else {
      output += (_fir_buffer[idx1] + _fir_buffer[idx2]) * fir_coeff[i];
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
// SERIAL COMMAND INTERFACE
// =============================================================================

// Control parameters
uint16_t sweepStart = 0;
uint16_t sweepStop = 65535;
uint16_t lockPointDAC = 32768;  // DAC setpoint (value x)
uint16_t lockPointADC = 2048;   // ADC target (value y)
bool lockEnabled = false;
float pidP = 1.0;
float pidI = 0.0;
float pidD = 0.0;

// PID variables
double pidSetpoint, pidInput, pidOutput;
PID myPID(&pidInput, &pidOutput, &pidSetpoint, pidP, pidI, pidD, DIRECT);

void printHelp() {
  Serial.println("Laser lock by BGMAGLAB");
  Serial.println("\nCommands:");
  Serial.println("  whois?              - Identify device");
  Serial.println("  sweep <start> <stop> - Set sweep limits (0-65535)");
  Serial.println("  lock <x> <y>        - Lock at DAC x, ADC target y");
  Serial.println("  lock OFF            - Disable lock, return to sweep");
  Serial.println("  lock?               - Get lock status");
  Serial.println("  PID <P*100> <I*100> <D*100> - Set PID (values/100)");
  Serial.println("  PID?                - Get current PID parameters");
  Serial.println("  help or ?           - Show this help");
}

void processSerialCommand() {
  if(!Serial.available()) return;
  
  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  
  if(cmd.length() == 0) return;
  
  // Echo command
  Serial.print("# ");
  Serial.println(cmd);
  
  // Parse command
  if(cmd.equals("whois?")) {
    Serial.println("Laser lock by BGMAGLAB");
  }
  else if(cmd.startsWith("sweep ")) {
    int idx1 = cmd.indexOf(' ');
    int idx2 = cmd.indexOf(' ', idx1 + 1);
    if(idx2 > 0) {
      uint16_t start = cmd.substring(idx1 + 1, idx2).toInt();
      uint16_t stop = cmd.substring(idx2 + 1).toInt();
      sweepStart = start;
      sweepStop = stop;
      Serial.printf("Sweep set: %u to %u\n", sweepStart, sweepStop);
    }
  }
  else if(cmd.startsWith("lock ")) {
    if(cmd.substring(5).equalsIgnoreCase("OFF")) {
      lockEnabled = false;
      myPID.SetMode(MANUAL);
      Serial.println("Lock disabled, returning to sweep mode");
    } else {
      int idx1 = cmd.indexOf(' ');
      int idx2 = cmd.indexOf(' ', idx1 + 1);
      if(idx2 > 0) {
        lockPointDAC = cmd.substring(idx1 + 1, idx2).toInt();
        lockPointADC = cmd.substring(idx2 + 1).toInt();
        lockEnabled = true;
        
        // Configure PID
        pidSetpoint = lockPointADC;
        myPID.SetTunings(pidP, pidI, pidD);
        myPID.SetOutputLimits(0, 65535);
        pidOutput = lockPointDAC;
        myPID.SetMode(AUTOMATIC);
        pidOutput = lockPointDAC;
        
        Serial.printf("Lock enabled: DAC=%u, ADC target=%u\n", lockPointDAC, lockPointADC);
      }
    }
  }
  else if(cmd.startsWith("PID ")) {
    int idx1 = cmd.indexOf(' ');
    int idx2 = cmd.indexOf(' ', idx1 + 1);
    int idx3 = cmd.indexOf(' ', idx2 + 1);
    if(idx3 > 0) {
      pidP = cmd.substring(idx1 + 1, idx2).toFloat() / 100.0;
      pidI = cmd.substring(idx2 + 1, idx3).toFloat() / 100.0;
      pidD = cmd.substring(idx3 + 1).toFloat() / 100.0;
      if(lockEnabled) {
        myPID.SetTunings(pidP, pidI, pidD);
      }
      Serial.printf("PID: %d %d %d\n", (int)(pidP * 100), (int)(pidI * 100), (int)(pidD * 100));
    }
  }
  else if(cmd.equals("PID?")) {
    Serial.printf("PID: %d %d %d\n", (int)(pidP * 100), (int)(pidI * 100), (int)(pidD * 100));
  }
  else if(cmd.equals("lock?")) {
    if(lockEnabled) {
      Serial.printf("%u %u\n", lockPointDAC, lockPointADC);
    } else {
      Serial.println("unlocked");
    }
  }
  else if(cmd.equals("help") || cmd.equals("?")) {
    printHelp();
  }
  else {
    Serial.println("Unknown command. Type 'help' or '?' for usage.");
  }
}

// =============================================================================
// SWEEP AND PLOT
// =============================================================================

#define SWEEP_POINTS 200
#define SAMPLE_RATE 1000  // Hz (200Hz = 1 sweep/second)
#define SAMPLE_PERIOD_US (1000000 / SAMPLE_RATE)

float dacData[SWEEP_POINTS];
float adcData[SWEEP_POINTS];

void performSweep() {
  Serial.println("# Starting sweep...");
  Serial.println("# Point,DAC_Raw,ADC_Raw");
  
  // Maintain 1V reference
  writeDAC60501(DAC60501_CONFIG, 0x0000);
  writeDAC60501(DAC60501_GAIN, 0x0100);
  writeDAC60501(DAC60501_DAC_DATA, 0xCCC0);
  
  for(int i = 0; i < SWEEP_POINTS; i++) {
    // Calculate DAC value for this point using sweep limits
    uint16_t dacValue = sweepStart + ((uint32_t)(sweepStop - sweepStart) * i) / (SWEEP_POINTS - 1);
    writeDAC8554_fast(0, dacValue);
    
    // Calculate voltages for display
    dacData[i] = (dacValue / 65535.0) * 1.0;
    
    // Wait for settling and read ADC
    delayMicroseconds(100);
    uint16_t adcRaw = applyFIR();
    adcData[i] = (adcRaw / 4095.0) * 3.3;
    
    // Send raw integer data over serial
    Serial.printf("%d,%u,%u\n", i, dacValue, adcRaw);
    
    // Wait for next sample (1000µs period = 1kHz)
    delayMicroseconds(SAMPLE_PERIOD_US);
  }
  
  Serial.println("# Sweep complete");
  
  // Set output to first value of next sweep and wait
  writeDAC8554_fast(0, sweepStart);
  delay(10);
}

void plotSweep() {
  tft.fillScreen(TFT_BLACK);
  
  // Display title
  tft.setTextSize(2);
  tft.setTextColor(TFT_YELLOW);
  tft.setCursor(10, 5);
  tft.print("Sweep Plot");
  
  // Graph area: 300x140 pixels, offset (10,25)
  int gx = 10, gy = 25;
  int gw = 300, gh = 140;
  
  // Draw axes
  tft.drawRect(gx, gy, gw, gh, TFT_WHITE);
  
  // Find min/max for scaling
  float minV = 3.3, maxV = 0.0;
  for(int i = 0; i < SWEEP_POINTS; i++) {
    if(adcData[i] < minV) minV = adcData[i];
    if(adcData[i] > maxV) maxV = adcData[i];
  }
  
  // Add margin to voltage range
  float vRange = maxV - minV;
  minV -= vRange * 0.1;
  maxV += vRange * 0.1;
  if(minV < 0) minV = 0;
  if(maxV > 3.3) maxV = 3.3;
  
  // Plot ADC (cyan)
  for(int i = 1; i < SWEEP_POINTS; i++) {
    int x1 = gx + ((i - 1) * gw) / (SWEEP_POINTS - 1);
    int x2 = gx + (i * gw) / (SWEEP_POINTS - 1);
    
    // ADC plot
    int y1_adc = gy + gh - ((adcData[i-1] - minV) / (maxV - minV)) * gh;
    int y2_adc = gy + gh - ((adcData[i] - minV) / (maxV - minV)) * gh;
    tft.drawLine(x1, y1_adc, x2, y2_adc, TFT_CYAN);
  }
  
  // Display stats
  tft.setTextSize(1);
  tft.setTextColor(TFT_CYAN);
  tft.setCursor(10, gy + gh + 5);
  tft.print("ADC Signal");
  
  tft.setTextColor(TFT_WHITE);
  tft.setCursor(100, gy + gh + 5);
  tft.printf("Range: %.2f-%.2fV", minV, maxV);
}

// =============================================================================
// MAIN LOOP
// =============================================================================

void loop() {
  // Process serial commands
  processSerialCommand();
  
  if(lockEnabled) {
    // PID lock mode at 1kHz
    static uint8_t serialCounter = 0;
    
    uint16_t adcRaw = applyFIR();
    pidInput = adcRaw;
    
    myPID.Compute();
    
    uint16_t dacOutput = (uint16_t)pidOutput;
    writeDAC8554_fast(0, dacOutput);
    
    // Send serial data every 10th cycle (100Hz output rate)
    if(++serialCounter >= 10) {
      serialCounter = 0;
      int16_t error = (int16_t)adcRaw - (int16_t)lockPointADC;
      Serial.printf("lock,%u,%u,%d\n", dacOutput, adcRaw, error);
    }
    
    delay(1);  // 1kHz update rate
  } else {
    // Sweep mode
    performSweep();
    plotSweep();
  }
}