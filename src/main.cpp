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

#define DAC_CS   5      // DAC8554 Chip Select
#define DAC_MOSI 26     // DAC8554 Data Out
#define DAC_SCK  27     // DAC8554 Clock

#define ADC_INPUT 34    // ADC1_CH6 - Photodiode input

// =============================================================================
// DAC ADDRESSES AND REGISTERS
// =============================================================================

#define DAC60501_ADDR     0x48
#define DAC60501_CONFIG   0x03
#define DAC60501_GAIN     0x04
#define DAC60501_TRIGGER  0x05
#define DAC60501_DAC_DATA 0x08

#define DAC8554_ADDR_BITS 0x00
#define DAC8554_LD_UPDATE 0x01

// =============================================================================
// LOCK-IN DETECTION CONSTANTS
// =============================================================================

#define SAMPLE_RATE       10000   // 10 kHz ISR rate
#define SAMPLE_PERIOD_US  100     // 100 µs
#define SINE_LUT_SIZE     256     // Sine lookup table size
#define SWEEP_POINTS      200     // Points per sweep

// Fixed-point scaling for ISR math (Q16.16 format)
#define FP_SHIFT          16
#define FP_ONE            (1 << FP_SHIFT)

// =============================================================================
// GLOBAL OBJECTS
// =============================================================================

TFT_eSPI tft = TFT_eSPI();
hw_timer_t *lockinTimer = NULL;

// =============================================================================
// LOCK-IN STATE (volatile for ISR access)
// =============================================================================

// Sine lookup table (Q15 fixed-point: -32767 to +32767)
// Single LUT used for both 1f dither and 3f reference
int16_t sineLUT[SINE_LUT_SIZE];

// Phase offset for demodulation (in LUT steps, 0-255)
volatile uint8_t isrPhaseOffset = 64;  // 64 = 90 degrees (cosine)
volatile uint8_t isrHarmonic = 1;      // Demodulation harmonic: 1=1f, 3=3f

// ISR state variables
volatile uint16_t isrDacBase = 32768;     // Base DAC value (sweep position)
volatile uint16_t isrDitherAmp = 0;       // Dither amplitude (0 = disabled)
volatile uint16_t isrPhaseInc = 0;        // Phase increment per sample (Q8.8)
volatile uint16_t isrPhase = 0;           // Current phase accumulator (Q8.8, wraps at 256)

volatile int32_t isrLockinAccum = 0;      // Lock-in accumulator (Q16 scaled)
volatile uint16_t isrSampleCount = 0;     // Samples accumulated
volatile uint16_t isrSamplesPerPoint = 32;// Samples per integration period

// Double-buffered results
volatile int32_t isrLockinResult = 0;     // Latest completed lock-in result
volatile uint16_t isrAdcResult = 0;       // Latest ADC reading
volatile bool isrDataReady = false;       // Flag: new result available
volatile bool isrZeroCrossing = false;    // Flag: dither phase at zero crossing

// DC offset tracking for input signal
volatile int32_t isrDcAccum = 0;          // DC accumulator
volatile int32_t isrDcOffset = 2048 << 4; // Running DC estimate (Q4 fixed point)

// Precomputed next DAC value (computed at end of ISR, written at start of next)
volatile uint16_t isrNextDacValue = 32768;

// =============================================================================
// CONTROL PARAMETERS (non-volatile, set by main loop)
// =============================================================================

uint16_t sweepStart = 0;
uint16_t sweepStop = 65535;
uint16_t ditherFreq = 1000;       // Hz
uint16_t ditherAmp = 0;           // DAC units
uint16_t ditherPhaseShift = 0;    // Degrees (0-360)
bool lockEnabled = false;
uint16_t lockPointDAC = 32768;
uint16_t lockPointADC = 2048;
float pidP = 1.0, pidI = 0.0, pidD = 0.0;

// Sweep data storage
float dacData[SWEEP_POINTS];
float adcData[SWEEP_POINTS];
float lockinData[SWEEP_POINTS];

// PID
double pidSetpoint, pidInput, pidOutput;
PID myPID(&pidInput, &pidOutput, &pidSetpoint, pidP, pidI, pidD, DIRECT);

// =============================================================================
// DAC60501 - REFERENCE GENERATOR
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
  writeDAC60501(DAC60501_CONFIG, 0x0000);
  delay(10);
  writeDAC60501(DAC60501_GAIN, 0x0100);
  delay(10);
  writeDAC60501(DAC60501_DAC_DATA, 0xCCC0);
  delay(10);
  writeDAC60501(DAC60501_TRIGGER, 0x0010);
  delay(10);
  Serial.println("✓ 1V reference ready");
}

// =============================================================================
// DAC8554 - FAST ISR-SAFE DAC WRITE (direct GPIO)
// =============================================================================

// Inline DAC write for ISR - uses direct GPIO register access
void IRAM_ATTR writeDAC_ISR(uint16_t value) {
  uint32_t frame = 0;
  frame |= ((uint32_t)DAC8554_ADDR_BITS << 22);
  frame |= ((uint32_t)DAC8554_LD_UPDATE << 20);
  frame |= ((uint32_t)0 << 17);  // Channel 0
  frame |= value;
  
  GPIO.out_w1tc = (1 << DAC_CS);  // CS LOW
  
  for(int i = 23; i >= 0; i--) {
    if((frame >> i) & 1) {
      GPIO.out_w1ts = (1 << DAC_MOSI);
    } else {
      GPIO.out_w1tc = (1 << DAC_MOSI);
    }
    GPIO.out_w1ts = (1 << DAC_SCK);
    GPIO.out_w1tc = (1 << DAC_SCK);
  }
  
  GPIO.out_w1ts = (1 << DAC_CS);  // CS HIGH
}

// Standard DAC write for main loop
void writeDAC8554(uint8_t channel, uint16_t value) {
  uint32_t frame = 0;
  frame |= ((uint32_t)DAC8554_ADDR_BITS << 22);
  frame |= ((uint32_t)DAC8554_LD_UPDATE << 20);
  frame |= ((uint32_t)channel << 17);
  frame |= value;
  
  digitalWrite(DAC_CS, LOW);
  for(int i = 23; i >= 0; i--) {
    digitalWrite(DAC_MOSI, (frame >> i) & 1);
    digitalWrite(DAC_SCK, HIGH);
    digitalWrite(DAC_SCK, LOW);
  }
  digitalWrite(DAC_CS, HIGH);
}

// =============================================================================
// LOCK-IN ISR - Runs at 10 kHz
// =============================================================================
// 
// This ISR performs the complete lock-in detection loop:
// 1. Write DAC with precomputed value (from previous cycle)
// 2. Read ADC
// 3. Apply high-pass filter
// 4. Multiply by 3f reference and accumulate
// 5. Check if integration complete, update result buffer
// 6. Compute next dither value
//
// All math uses integer/fixed-point to avoid FPU issues in ISR
// =============================================================================

void IRAM_ATTR onLockinTimer() {
  // 1. Write DAC immediately (value computed in previous ISR)
  writeDAC_ISR(isrNextDacValue);
  
  // 2. Read ADC (12-bit, 0-4095)
  uint16_t adcRaw = adc1_get_raw(ADC1_CHANNEL_6);
  
  // Skip lock-in processing if dither disabled
  if(isrDitherAmp == 0) {
    isrAdcResult = adcRaw;
    isrNextDacValue = isrDacBase;
    return;
  }
  
  // 3. Get reference from LUT at selected harmonic
  // Phase accumulator is 16-bit (0-65535 = one cycle)
  // Multiply phase by harmonic number
  uint16_t phaseRef = isrPhase * isrHarmonic;
  // Phase offset must also scale with harmonic (physical delay scales with frequency)
  uint8_t scaledOffset = (isrPhaseOffset * isrHarmonic) & 0xFF;
  uint8_t lutIndexRef = ((phaseRef >> 8) + scaledOffset) & 0xFF;
  int16_t refSig = sineLUT[lutIndexRef];
  
  // 4. Multiply raw ADC by reference (demodulate)
  // adcRaw is 0-4095, refSig is -32767 to +32767
  int32_t demodulated = (int32_t)adcRaw * refSig;
  
  // 5. Accumulate (this IS the low-pass filter)
  isrLockinAccum += demodulated;
  isrSampleCount++;
  
  // 6. Check if integration period complete
  if(isrSampleCount >= isrSamplesPerPoint) {
    // Average and store result (scale down for reasonable output range)
    isrLockinResult = - isrLockinAccum / ((int32_t)isrSamplesPerPoint * 1920);
    isrAdcResult = adcRaw;
    isrDataReady = true;
    
    // Reset accumulator
    isrLockinAccum = 0;
    isrSampleCount = 0;
  }
  
  // 7. Advance phase and compute next DAC value
  uint16_t prevPhase = isrPhase;
  isrPhase += isrPhaseInc;  // Wraps naturally at 65536
  
  // Detect zero crossing (phase wraps from high to low)
  if(prevPhase > isrPhase) {
    isrZeroCrossing = true;
  }
  
  // Get next dither value from LUT (fundamental frequency)
  uint8_t nextLutIndex = (isrPhase >> 8) & 0xFF;
  int16_t ditherSin = sineLUT[nextLutIndex];
  
  // Calculate next DAC output: base + dither
  int32_t nextDac = (int32_t)isrDacBase + (((int32_t)ditherSin * isrDitherAmp) >> 15);
  if(nextDac < 0) nextDac = 0;
  if(nextDac > 65535) nextDac = 65535;
  isrNextDacValue = (uint16_t)nextDac;
}

// =============================================================================
// INITIALIZE LOCK-IN PARAMETERS
// =============================================================================

void updateLockinParams() {
  // Disable ISR during parameter update
  if(lockinTimer) timerAlarmDisable(lockinTimer);
  
  // Calculate phase increment for desired frequency
  if(ditherFreq > 0 && ditherAmp > 0) {
    // Calculate samples per cycle (must be integer)
    uint16_t samplesPerCycle = SAMPLE_RATE / ditherFreq;
    if(samplesPerCycle < 4) samplesPerCycle = 4;
    
    // Adjust frequency to get exact integer samples per cycle
    // This ensures integration over N complete cycles works correctly
    uint16_t actualFreq = SAMPLE_RATE / samplesPerCycle;
    
    // Phase increment for exact integer cycles
    // One cycle = 65536 phase units, divided by samples per cycle
    isrPhaseInc = 65536 / samplesPerCycle;
    
    // Integration period = N complete cycles (more for 3f since signal is weaker)
    uint8_t numCycles = (isrHarmonic >= 3) ? 8 : 4;
    isrSamplesPerPoint = samplesPerCycle * numCycles;
    
    Serial.printf("Lock-in: requested %uHz, actual %uHz\n", ditherFreq, actualFreq);
    Serial.printf("  samples/cycle=%u, cycles/point=%u, samples/point=%u\n", 
                  samplesPerCycle, numCycles, isrSamplesPerPoint);
  } else {
    isrPhaseInc = 0;
    isrSamplesPerPoint = 32;
  }
  
  isrDitherAmp = ditherAmp;
  
  // Generate sine lookup table (just one LUT, used for both 1f and 3f)
  for(int i = 0; i < SINE_LUT_SIZE; i++) {
    float phase = (2.0f * PI * i) / SINE_LUT_SIZE;
    sineLUT[i] = (int16_t)(sinf(phase) * 32767.0f);
  }
  
  // Calculate phase offset for demodulation
  // 64 LUT steps = 90 degrees (cosine), add user phase shift
  // User phase shift in degrees -> LUT steps: degrees * 256 / 360
  uint8_t userPhaseSteps = (uint8_t)((ditherPhaseShift * 256UL) / 360);
  isrPhaseOffset = (64 + userPhaseSteps) & 0xFF;  // 64 = cosine baseline
  Serial.printf("  Harmonic: %uf, phase offset: %u steps (%u deg)\n", 
                isrHarmonic, isrPhaseOffset, ditherPhaseShift);
  
  // Reset state
  isrLockinAccum = 0;
  isrDcAccum = 0;
  isrDcOffset = 2048 << 4;  // Start at mid-scale
  isrSampleCount = 0;
  isrPhase = 0;
  
  // Re-enable ISR
  if(lockinTimer) timerAlarmEnable(lockinTimer);
}

// =============================================================================
// INITIALIZE SYSTEM
// =============================================================================

void initDAC8554() {
  pinMode(DAC_MOSI, OUTPUT);
  pinMode(DAC_SCK, OUTPUT);
  pinMode(DAC_CS, OUTPUT);
  digitalWrite(DAC_CS, HIGH);
  digitalWrite(DAC_SCK, LOW);
  
  // Initialize all channels to 0
  for(int ch = 0; ch < 4; ch++) {
    writeDAC8554(ch, 0);
    delay(10);
  }
}

void initLockinTimer() {
  // Configure ADC
  adc1_config_width(ADC_WIDTH_12Bit);
  adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_11db);
  
  // Create timer at 10 kHz
  lockinTimer = timerBegin(0, 80, true);  // 80MHz / 80 = 1MHz tick
  timerAttachInterrupt(lockinTimer, &onLockinTimer, true);
  timerAlarmWrite(lockinTimer, 100, true);  // 100 ticks = 100µs = 10kHz
  timerAlarmEnable(lockinTimer);
  
  Serial.println("✓ Lock-in timer: 10kHz ISR");
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n╔════════════════════════════════╗");
  Serial.println("║  Laser Lock - BGMAGLAB v2.0   ║");
  Serial.println("║  ISR-based Lock-In Detection  ║");
  Serial.println("╚════════════════════════════════╝\n");
  
  // Initialize I2C and reference DAC
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
  tft.setTextColor(TFT_GREEN);
  tft.setTextSize(2);
  tft.setCursor(50, 50);
  tft.print("Laser Lock v2");
  tft.setTextColor(TFT_CYAN);
  tft.setCursor(30, 80);
  tft.print("ISR Lock-In Engine");
  delay(2000);
  
  // Initialize DAC
  initDAC8554();
  
  // Initialize lock-in parameters
  updateLockinParams();
  
  // Start lock-in timer
  initLockinTimer();
  
  Serial.println("\n✓ System ready!\n");
}

// =============================================================================
// SERIAL COMMAND INTERFACE
// =============================================================================

void printHelp() {
  Serial.println("Laser lock by BGMAGLAB");
  Serial.println("\nCommands:");
  Serial.println("  whois?              - Identify device");
  Serial.println("  sweep <start> <stop> - Set sweep limits (0-65535)");
  Serial.println("  lock <x> <y>        - Lock at DAC x, ADC target y");
  Serial.println("  lock OFF            - Disable lock");
  Serial.println("  lock?               - Get lock status");
  Serial.println("  PID <P*100> <I*100> <D*100> - Set PID");
  Serial.println("  PID?                - Get PID parameters");
  Serial.println("  DITHER_FREQ <Hz>    - Set dither frequency");
  Serial.println("  DITHER_FREQ?        - Get dither frequency");
  Serial.println("  DITHER_AMP <amp>    - Set dither amplitude");
  Serial.println("  DITHER_AMP?         - Get dither amplitude");
  Serial.println("  DITHER_PH <deg>     - Set phase shift 0-360");
  Serial.println("  DITHER_PH?          - Get phase shift");
  Serial.println("  help or ?           - Show this help");
}

void processSerialCommand() {
  if(!Serial.available()) return;
  
  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  if(cmd.length() == 0) return;
  
  if(cmd.equals("whois?")) {
    Serial.println("Laser lock by BGMAGLAB");
  }
  else if(cmd.startsWith("sweep ")) {
    int idx1 = cmd.indexOf(' ');
    int idx2 = cmd.indexOf(' ', idx1 + 1);
    if(idx2 > 0) {
      sweepStart = cmd.substring(idx1 + 1, idx2).toInt();
      sweepStop = cmd.substring(idx2 + 1).toInt();
    }
  }
  else if(cmd.startsWith("lock ")) {
    if(cmd.substring(5).equalsIgnoreCase("OFF")) {
      lockEnabled = false;
      myPID.SetMode(MANUAL);
    } else {
      int idx1 = cmd.indexOf(' ');
      int idx2 = cmd.indexOf(' ', idx1 + 1);
      if(idx2 > 0) {
        lockPointDAC = cmd.substring(idx1 + 1, idx2).toInt();
        lockPointADC = cmd.substring(idx2 + 1).toInt();
        lockEnabled = true;
        pidSetpoint = 0;  // Lock-in signal target is 0 (zero crossing)
        pidOutput = lockPointDAC;
        myPID.SetTunings(pidP, pidI, pidD);
        myPID.SetOutputLimits(0, 65535);
        myPID.SetMode(AUTOMATIC);
      }
    }
  }
  else if(cmd.equals("lock?")) {
    if(lockEnabled) {
      Serial.printf("%u %u\n", lockPointDAC, lockPointADC);
    } else {
      Serial.println("unlocked");
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
      if(lockEnabled) myPID.SetTunings(pidP, pidI, pidD);
      Serial.printf("%d %d %d\n", (int)(pidP*100), (int)(pidI*100), (int)(pidD*100));
    }
  }
  else if(cmd.equals("PID?")) {
    Serial.printf("%d %d %d\n", (int)(pidP*100), (int)(pidI*100), (int)(pidD*100));
  }
  else if(cmd.startsWith("DITHER_FREQ ")) {
    ditherFreq = cmd.substring(12).toInt();
    updateLockinParams();
  }
  else if(cmd.equals("DITHER_FREQ?")) {
    Serial.printf("%u\n", ditherFreq);
  }
  else if(cmd.startsWith("DITHER_AMP ")) {
    ditherAmp = cmd.substring(11).toInt();
    updateLockinParams();
  }
  else if(cmd.equals("DITHER_AMP?")) {
    Serial.printf("%u\n", ditherAmp);
  }
  else if(cmd.startsWith("DITHER_PH ")) {
    ditherPhaseShift = cmd.substring(10).toInt();
    if(ditherPhaseShift > 360) ditherPhaseShift = 360;
    updateLockinParams();
  }
  else if(cmd.equals("DITHER_PH?")) {
    Serial.printf("%u\n", ditherPhaseShift);
  }
  else if(cmd.startsWith("HARMONIC ")) {
    uint8_t h = cmd.substring(9).toInt();
    if(h == 1 || h == 3) {
      isrHarmonic = h;
      updateLockinParams();  // Recalculate integration time
      Serial.printf("%u\n", isrHarmonic);
    }
  }
  else if(cmd.equals("HARMONIC?")) {
    Serial.printf("%u\n", isrHarmonic);
  }
  else if(cmd.equals("help") || cmd.equals("?")) {
    printHelp();
  }
}

// =============================================================================
// SWEEP AND PLOT
// =============================================================================

void performSweep() {
  Serial.println("# Starting sweep...");
  if(ditherAmp > 0) {
    Serial.println("# Point,DAC,ADC,LockIn");
  } else {
    Serial.println("# Point,DAC,ADC");
  }
  
  // Refresh reference DAC
  writeDAC60501(DAC60501_CONFIG, 0x0000);
  writeDAC60501(DAC60501_GAIN, 0x0100);
  writeDAC60501(DAC60501_DAC_DATA, 0xCCC0);
  
  for(int i = 0; i < SWEEP_POINTS; i++) {
    // Calculate DAC value for this point
    uint16_t dacValue = sweepStart + ((uint32_t)(sweepStop - sweepStart) * i) / (SWEEP_POINTS - 1);
    
    // Set ISR base value
    isrDacBase = dacValue;
    
    if(ditherAmp > 0) {
      // Wait for lock-in integration to complete
      isrDataReady = false;
      isrZeroCrossing = false;
      isrLockinAccum = 0;
      isrSampleCount = 0;
      
      // Wait for integration complete
      while(!isrDataReady) {
        delayMicroseconds(10);
      }
      
      // Wait for zero crossing sync (ensures consistent phase)
      while(!isrZeroCrossing) {
        delayMicroseconds(10);
      }
      
      // Read results from ISR
      int32_t lockinVal = isrLockinResult;
      uint16_t adcVal = isrAdcResult;
      
      // Store data
      dacData[i] = dacValue / 65535.0f;
      adcData[i] = (adcVal / 4095.0f) * 3.3f;
      lockinData[i] = (float)lockinVal / 1000.0f;  // Scale for display
      
      Serial.printf("%d,%u,%u,%d\n", i, dacValue, adcVal, lockinVal);
    } else {
      // No dither - just read ADC
      delay(1);
      uint16_t adcVal = isrAdcResult;
      
      dacData[i] = dacValue / 65535.0f;
      adcData[i] = (adcVal / 4095.0f) * 3.3f;
      lockinData[i] = 0;
      
      Serial.printf("%d,%u,%u\n", i, dacValue, adcVal);
    }
  }
  
  Serial.println("# Sweep complete");
  
  // Return to start position (with dither offset if needed)
  if(ditherAmp > 0 && sweepStart < ditherAmp) {
    isrDacBase = sweepStart + (ditherAmp * 3 / 2);
  } else {
    isrDacBase = sweepStart;
  }
}

void plotSweep() {
  tft.fillScreen(TFT_BLACK);
  
  // Title
  tft.setTextSize(2);
  tft.setTextColor(TFT_YELLOW);
  tft.setCursor(10, 5);
  tft.print(ditherAmp > 0 ? "Sweep + Lock-in" : "Sweep");
  
  // Graph area
  int gx = 10, gy = 25, gw = 300, gh = 140;
  tft.drawRect(gx, gy, gw, gh, TFT_WHITE);
  
  // Find ADC min/max
  float minV = 3.3f, maxV = 0.0f;
  for(int i = 0; i < SWEEP_POINTS; i++) {
    if(adcData[i] < minV) minV = adcData[i];
    if(adcData[i] > maxV) maxV = adcData[i];
  }
  float vRange = maxV - minV;
  if(vRange < 0.01f) vRange = 0.01f;
  minV -= vRange * 0.1f;
  maxV += vRange * 0.1f;
  
  // Find lock-in min/max
  float minLI = 0, maxLI = 0;
  if(ditherAmp > 0) {
    minLI = maxLI = lockinData[0];
    for(int i = 1; i < SWEEP_POINTS; i++) {
      if(lockinData[i] < minLI) minLI = lockinData[i];
      if(lockinData[i] > maxLI) maxLI = lockinData[i];
    }
    float liRange = maxLI - minLI;
    if(liRange < 0.001f) liRange = 0.001f;
    minLI -= liRange * 0.1f;
    maxLI += liRange * 0.1f;
  }
  
  // Plot ADC (cyan)
  for(int i = 1; i < SWEEP_POINTS; i++) {
    int x1 = gx + ((i-1) * gw) / (SWEEP_POINTS - 1);
    int x2 = gx + (i * gw) / (SWEEP_POINTS - 1);
    int y1 = gy + gh - (int)((adcData[i-1] - minV) / (maxV - minV) * gh);
    int y2 = gy + gh - (int)((adcData[i] - minV) / (maxV - minV) * gh);
    tft.drawLine(x1, y1, x2, y2, TFT_CYAN);
  }
  
  // Plot lock-in (magenta)
  if(ditherAmp > 0) {
    for(int i = 1; i < SWEEP_POINTS; i++) {
      int x1 = gx + ((i-1) * gw) / (SWEEP_POINTS - 1);
      int x2 = gx + (i * gw) / (SWEEP_POINTS - 1);
      int y1 = gy + gh - (int)((lockinData[i-1] - minLI) / (maxLI - minLI) * gh);
      int y2 = gy + gh - (int)((lockinData[i] - minLI) / (maxLI - minLI) * gh);
      tft.drawLine(x1, y1, x2, y2, TFT_MAGENTA);
    }
  }
  
  // Labels
  tft.setTextSize(1);
  tft.setTextColor(TFT_CYAN);
  tft.setCursor(10, gy + gh + 5);
  tft.printf("ADC: %.2f-%.2fV", minV, maxV);
  
  if(ditherAmp > 0) {
    tft.setTextColor(TFT_MAGENTA);
    tft.setCursor(160, gy + gh + 5);
    tft.printf("LI: %.1f", maxLI - minLI);
  }
}

// =============================================================================
// MAIN LOOP
// =============================================================================

void loop() {
  processSerialCommand();
  
  if(lockEnabled) {
    // PID lock mode
    static uint8_t counter = 0;
    
    if(ditherAmp > 0) {
      // Set DAC base to current PID output
      isrDacBase = (uint16_t)pidOutput;
      
      // Wait for new lock-in data
      if(isrDataReady) {
        isrDataReady = false;
        
        // Use lock-in signal as error (target = 0 at line center)
        pidInput = (double)isrLockinResult;
        myPID.Compute();
        
        // Output
        if(++counter >= 10) {
          counter = 0;
          Serial.printf("lock,%u,%u,%d\n", (uint16_t)pidOutput, isrAdcResult, isrLockinResult);
        }
      }
    } else {
      // No dither - use ADC directly
      isrDacBase = (uint16_t)pidOutput;
      pidInput = isrAdcResult;
      myPID.Compute();
      
      if(++counter >= 100) {
        counter = 0;
        Serial.printf("lock,%u,%u\n", (uint16_t)pidOutput, isrAdcResult);
      }
    }
  } else {
    // Sweep mode
    performSweep();
    plotSweep();
    
    // Pause between sweeps for settling
    delay(500);
  }
}
