#include <SoftwareSerial.h> 
#define ENC_A_BIT                                                              \
  PD3 
#define ENC_B_BIT PD2 
#define MTR_IN1_BIT PD4 
#define MTR_IN2_BIT PD5 
#define MTR_PWM_BIT PD6 
#define STP_IN1_BIT PB3 
#define STP_IN2_BIT PB2 
#define STP_IN3_BIT PB5 
#define STP_IN4_BIT PB4 
#define STP_MASK                                                               \
  ((1 << STP_IN1_BIT) | (1 << STP_IN2_BIT) | (1 << STP_IN3_BIT) |              \
   (1 << STP_IN4_BIT))
const uint8_t PIN_TFMINI_RX = 9; 
const uint8_t PIN_TFMINI_TX = 8; 
const float YAW_STEP_DEG = 2.0f;
const float PITCH_START_DEG = 0.0f;
const float TARGET_DISTANCE_CM = 48.0f; 
const float TARGET_VERTICAL_RES_CM =
    0.5f; 
const float MAX_TARGET_HEIGHT_CM = 48.0f; 
const float PITCH_MANUAL_STEP_DEG = 15.0f; 
const int YAW_REVS_PER_PITCH = 1; 
const float ENCODER_COUNTS_PER_REV =
    660.0f; 
const float STEPPER_STEPS_PER_REV =
    4096.0f; 
const long WARMUP_COUNTS = 0;
float Kp = 0.6f;
float Ki = 0.15f;
float Kd = 0.02f;
const float TARGET_SPEED = 200.0f;
const int PWM_MAX = 200;
const int PWM_MIN_KICK = 80;
const float INTEGRAL_MAX = 400.0f;
float speedIntegral = 0.0f;
float speedPrevSpeed = 0.0f;          
float speedFilteredDerivative = 0.0f; 
unsigned long speedPrevTime = 0;
long speedPrevCount = 0;
static const uint8_t HALF_STEP_SEQ[8] = {
    (1 << STP_IN1_BIT),                      
    (1 << STP_IN1_BIT) | (1 << STP_IN2_BIT), 
    (1 << STP_IN2_BIT),                      
    (1 << STP_IN2_BIT) | (1 << STP_IN3_BIT), 
    (1 << STP_IN3_BIT),                      
    (1 << STP_IN3_BIT) | (1 << STP_IN4_BIT), 
    (1 << STP_IN4_BIT),                      
    (1 << STP_IN4_BIT) | (1 << STP_IN1_BIT), 
};
volatile int8_t stepIndex = 0;
volatile int32_t stepPosition = 0;
volatile int32_t stepTarget = 0;
SoftwareSerial tfmini(PIN_TFMINI_RX, PIN_TFMINI_TX);
volatile long encoderCount = 0;
volatile bool encoderForward = true; 
volatile bool targetRevsReached =
    false; 
volatile long encoderCountAtTrigger =
    0; 
volatile long encoderThreshold = 0;
enum ScannerState { IDLE, SPINNING_YAW, MOVING_PITCH, FINISHED };
ScannerState state = IDLE;
float currentYawDeg = 0.0f; 
float currentPitchDeg = 0.0f;
int currentPitchLevel = 0; 
uint16_t lidarDistanceCm = 0;
uint16_t lidarStrength = 0;
bool lidarValid = false;
unsigned long lastLidarFrameMs = 0;
bool lidarDebugSweep = false;
float lidarDebugYawDeg = 0.0f;
unsigned long lastLidarDebugPrintMs = 0;
void uart_init_115200() {
  UCSR0A = (1 << U2X0); 
  UBRR0H = 0;
  UBRR0L = 16;                            
  UCSR0B = (1 << RXEN0) | (1 << TXEN0);   
  UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); 
}
void uart_putc(char c) {
  while (!(UCSR0A & (1 << UDRE0)))
    ;       
  UDR0 = c; 
}
void uart_puts(const char *s) {
  while (*s)
    uart_putc(*s++);
}
void uart_puts_P(const char *s) {
  char c;
  while ((c = pgm_read_byte(s++)))
    uart_putc(c);
}
void uart_print_int(long n) {
  char buf[12];
  ltoa(n, buf, 10);
  uart_puts(buf);
}
void uart_print_uint(uint16_t n) {
  char buf[8];
  utoa(n, buf, 10);
  uart_puts(buf);
}
void uart_print_float(float val, uint8_t decimals) {
  char buf[16];
  dtostrf(val, 0, decimals, buf);
  uart_puts(buf);
}
void uart_println() {
  uart_putc('\r');
  uart_putc('\n');
}
bool uart_available() { return (UCSR0A & (1 << RXC0)) != 0; }
char uart_read() { return (char)UDR0; }
void setup_gpio() {
  DDRD &= ~((1 << ENC_A_BIT) | (1 << ENC_B_BIT));
  PORTD |= ((1 << ENC_A_BIT) | (1 << ENC_B_BIT));
  DDRD |= (1 << MTR_IN1_BIT) | (1 << MTR_IN2_BIT) | (1 << MTR_PWM_BIT);
  PORTD &= ~((1 << MTR_IN1_BIT) | (1 << MTR_IN2_BIT) | (1 << MTR_PWM_BIT));
  DDRB |= STP_MASK;
  PORTB &= ~STP_MASK; 
}
void setDcMotorPwm_raw(uint8_t duty) {
  if (duty == 0) {
    TCCR0A &= ~(1 << COM0A1);
    PORTD &= ~(1 << MTR_PWM_BIT); 
  } else {
    TCCR0A |= (1 << COM0A1);
    OCR0A = duty; 
  }
}
void setDcMotor(int pwm) {
  if (pwm > 255)
    pwm = 255;
  if (pwm < -255)
    pwm = -255;
  if (pwm > 0) {
    PORTD |= (1 << MTR_IN1_BIT);  
    PORTD &= ~(1 << MTR_IN2_BIT); 
    setDcMotorPwm_raw((uint8_t)pwm);
  } else if (pwm < 0) {
    PORTD &= ~(1 << MTR_IN1_BIT); 
    PORTD |= (1 << MTR_IN2_BIT);  
    setDcMotorPwm_raw((uint8_t)(-pwm));
  } else {
    PORTD &= ~((1 << MTR_IN1_BIT) | (1 << MTR_IN2_BIT));
    setDcMotorPwm_raw(0);
  }
}
void setup_encoder_interrupt() {
  EICRA = (EICRA & ~((1 << ISC11) | (1 << ISC10))) | (1 << ISC10);
  EIMSK = (EIMSK & ~(1 << INT0)) | (1 << INT1);
}
ISR(INT1_vect) {
  uint8_t pd = PIND;              
  bool a = (pd >> ENC_A_BIT) & 1; 
  bool b = (pd >> ENC_B_BIT) & 1; 
  bool dir = (a == b);            
  encoderCount += dir ? 1 : -1;
  encoderForward = dir;
  if (!targetRevsReached && encoderThreshold > 0) {
    long absNow = encoderCount < 0 ? -encoderCount : encoderCount;
    if (absNow >= encoderThreshold) {
      targetRevsReached = true;
      encoderCountAtTrigger = encoderCount;
    }
  }
}
long readEncoderCount() {
  uint8_t sreg = SREG; 
  cli();               
  long c = encoderCount;
  SREG = sreg; 
  return c;
}
void setup_stepper_timer() {
  TCCR1A = 0;
  TCCR1B = (1 << WGM12) | (1 << CS11) | (1 << CS10); 
  TCCR1B = (1 << WGM12) | (1 << CS12) | (1 << CS10); 
  OCR1A = 520;                                       
  TIMSK1 |= (1 << OCIE1A);
}
ISR(TIMER1_COMPA_vect) {
  if (stepPosition < stepTarget) {
    stepIndex = (stepIndex + 1) & 0x07; 
    stepPosition++;
  } else if (stepPosition > stepTarget) {
    stepIndex = (stepIndex - 1) & 0x07;
    stepPosition--;
  } else {
    return; 
  }
  PORTB = (PORTB & ~STP_MASK) | HALF_STEP_SEQ[stepIndex];
}
void stepper_moveTo(int32_t target) {
  uint8_t sreg = SREG;
  cli();
  stepTarget = target;
  SREG = sreg;
}
int32_t stepper_distanceToGo() {
  uint8_t sreg = SREG;
  cli();
  int32_t d = stepTarget - stepPosition;
  SREG = sreg;
  return d;
}
void stepper_setCurrentPosition(int32_t pos) {
  uint8_t sreg = SREG;
  cli();
  stepPosition = pos;
  stepTarget = pos;
  SREG = sreg;
}
void stepper_stop() {
  uint8_t sreg = SREG;
  cli();
  stepTarget = stepPosition;
  SREG = sreg;
}
long stepperAngleToSteps(float angleDeg) {
  return lround(-angleDeg * STEPPER_STEPS_PER_REV / 360.0f);
}
bool readTfminiFrame() {
  static uint8_t frame[9];
  static uint8_t idx = 0;
  while (tfmini.available() > 0) {
    uint8_t b = tfmini.read();
    if (idx == 0 && b != 0x59)
      continue;
    if (idx == 1 && b != 0x59) {
      idx = 0;
      continue;
    }
    frame[idx++] = b;
    if (idx == 9) {
      idx = 0;
      uint8_t sum = 0;
      for (uint8_t i = 0; i < 8; i++)
        sum += frame[i];
      if (sum == frame[8]) {
        lidarDistanceCm = frame[2] + (frame[3] << 8);
        lidarStrength = frame[4] + (frame[5] << 8);
        lidarValid = true;
        lastLidarFrameMs = millis();
        return true;
      }
    }
  }
  return false;
}
void printMeasurement() {
  uart_puts_P(PSTR("SCAN,"));
  uart_print_float(currentYawDeg, 2);
  uart_putc(',');
  uart_print_float(currentPitchDeg, 2);
  uart_putc(',');
  uart_print_uint(lidarDistanceCm);
  uart_putc(',');
  uart_print_uint(lidarStrength);
  uart_println();
}
void printSyncMeasurement(unsigned long t, long enc, int32_t step,
                          uint16_t dist, uint16_t str) {
  uart_puts_P(PSTR("SYNC,"));
  uart_print_int(t);
  uart_putc(',');
  uart_print_int(enc);
  uart_putc(',');
  uart_print_int(step);
  uart_putc(',');
  uart_print_uint(dist);
  uart_putc(',');
  uart_print_uint(str);
  uart_println();
}
void updateMeasurement() {
  if (!targetRevsReached)
    return;
  setDcMotor(0);
  uint8_t sreg = SREG;
  cli();
  targetRevsReached = false;
  encoderThreshold = 0; 
  SREG = sreg;
  long currentAbsCount = abs(readEncoderCount());
  long overshoot = currentAbsCount - abs(encoderCountAtTrigger);
  currentPitchLevel++;
  float targetHeightCm = currentPitchLevel * TARGET_VERTICAL_RES_CM;
  if (targetHeightCm > MAX_TARGET_HEIGHT_CM + 0.001f) {
    state = FINISHED;
    uart_puts_P(PSTR("STATUS,DONE\r\n"));
  } else {
    currentPitchDeg = atan2(targetHeightCm, TARGET_DISTANCE_CM) * 180.0f / PI;
    stepper_moveTo(stepperAngleToSteps(currentPitchDeg));
    state = MOVING_PITCH;
    uart_puts_P(PSTR("STATUS,NEXT_PITCH,"));
    uart_print_float(currentPitchDeg, 2);
    uart_puts_P(PSTR(",OVERSHOOT="));
    uart_print_int(overshoot);
    uart_println();
  }
}
void updateSpeedPID() {
  unsigned long now = millis();
  float dt = (now - speedPrevTime) / 1000.0f;
  if (dt < 0.020f)
    return;
  speedPrevTime = now;
  long currentAbsCount = abs(readEncoderCount());
  float currentSpeed = (float)(currentAbsCount - speedPrevCount) / dt;
  speedPrevCount = currentAbsCount;
  float error = TARGET_SPEED - currentSpeed;
  speedIntegral += error * dt;
  if (speedIntegral > INTEGRAL_MAX)
    speedIntegral = INTEGRAL_MAX;
  if (speedIntegral < -INTEGRAL_MAX)
    speedIntegral = -INTEGRAL_MAX;
  float rawDerivative = -(currentSpeed - speedPrevSpeed) / dt;
  speedPrevSpeed = currentSpeed;
  const float alpha = 0.3f; 
  speedFilteredDerivative =
      alpha * rawDerivative + (1.0f - alpha) * speedFilteredDerivative;
  float Kf = 0.5f; 
  float feedforward = Kf * TARGET_SPEED;
  int pwm = (int)(feedforward + Kp * error + Ki * speedIntegral +
                  Kd * speedFilteredDerivative);
  if (pwm > 0 && pwm < PWM_MIN_KICK)
    pwm = PWM_MIN_KICK;
  pwm = constrain(pwm, 0, PWM_MAX);
  setDcMotor(pwm);
}
void startYawSpin() {
  if (currentPitchLevel == 0) {
    uint8_t sreg = SREG;
    cli();
    encoderCount = 0;
    SREG = sreg;
  }
  long initCount = abs(readEncoderCount());
  long threshold =
      ((initCount / lround(ENCODER_COUNTS_PER_REV)) + YAW_REVS_PER_PITCH) *
      lround(ENCODER_COUNTS_PER_REV);
  uint8_t sreg = SREG;
  cli();
  targetRevsReached = false;
  encoderThreshold = threshold; 
  SREG = sreg;
  speedIntegral = 0.0f;
  speedPrevSpeed = 0.0f;
  speedFilteredDerivative = 0.0f;
  speedPrevTime = millis();
  speedPrevCount = initCount;
  setDcMotor(PWM_MIN_KICK);
  state = SPINNING_YAW;
  uart_puts_P(PSTR("STATUS,SPINNING,PITCH="));
  uart_print_float(currentPitchDeg, 1);
  uart_puts_P(PSTR(",ENC_INIT="));
  uart_print_int(initCount);
  uart_println();
}
void beginScan() {
  uint8_t sreg = SREG;
  cli();
  encoderCount = 0;
  SREG = sreg;
  currentYawDeg = 0.0f;
  currentPitchLevel = 0;
  currentPitchDeg = 0.0f;
  stepper_moveTo(stepperAngleToSteps(currentPitchDeg));
  state = MOVING_PITCH;
  uart_puts_P(PSTR("STATUS,START\r\n"));
}
void haltScan() {
  setDcMotor(0);
  speedIntegral = 0.0f;
  stepper_stop();
  state = IDLE;
  uart_puts_P(PSTR("STATUS,HALT\r\n"));
}
void printDebugSweepMeasurement() {
  currentYawDeg = lidarDebugYawDeg;
  currentPitchDeg = 0.0f;
  printMeasurement();
  lidarDebugYawDeg += 5.0f;
  if (lidarDebugYawDeg >= 360.0f)
    lidarDebugYawDeg -= 360.0f;
}
void printDiagnostic() {
  uint8_t sreg = SREG;
  cli();
  int32_t sp = stepPosition;
  int32_t st = stepTarget;
  SREG = sreg;
  uart_puts_P(PSTR("STATUS,DIAG,ENCODER_COUNT="));
  uart_print_int(readEncoderCount());
  uart_puts_P(PSTR(",ABS_COUNT="));
  uart_print_int(abs(readEncoderCount()));
  uart_puts_P(PSTR(",LIDAR_VALID="));
  uart_print_int(lidarValid ? 1 : 0);
  uart_puts_P(PSTR(",DISTANCE_CM="));
  uart_print_uint(lidarDistanceCm);
  uart_puts_P(PSTR(",STRENGTH="));
  uart_print_uint(lidarStrength);
  uart_puts_P(PSTR(",LIDAR_AGE_MS="));
  uart_print_int(lidarValid ? (long)(millis() - lastLidarFrameMs) : -1);
  uart_puts_P(PSTR(",PITCH_DEG="));
  uart_print_float(currentPitchDeg, 1);
  uart_puts_P(PSTR(",STEP_POS="));
  uart_print_int(sp);
  uart_puts_P(PSTR(",STEP_TGT="));
  uart_print_int(st);
  uart_puts_P(PSTR(",STATE="));
  uart_print_int((int)state);
  uart_println();
}
void movePitchManual(float deltaDeg) {
  state = IDLE;
  setDcMotor(0);
  currentPitchDeg += deltaDeg;
  stepper_moveTo(stepperAngleToSteps(currentPitchDeg));
  uart_puts_P(PSTR("STATUS,PITCH_TARGET,"));
  uart_print_float(currentPitchDeg, 1);
  uart_println();
}
void stepperLedTest() {
  state = IDLE;
  setDcMotor(0);
  stepper_setCurrentPosition(stepTarget);
  uart_puts_P(PSTR("STATUS,STEPPER_LED_TEST_START\r\n"));
  for (uint8_t round = 0; round < 3; round++) {
    PORTB = (PORTB & ~STP_MASK) | (1 << STP_IN1_BIT);
    delay(300);
    PORTB = (PORTB & ~STP_MASK) | (1 << STP_IN2_BIT); 
    delay(300);
    PORTB = (PORTB & ~STP_MASK) | (1 << STP_IN3_BIT);
    delay(300);
    PORTB = (PORTB & ~STP_MASK) | (1 << STP_IN4_BIT);
    delay(300);
  }
  PORTB &= ~STP_MASK;
  uart_puts_P(PSTR("STATUS,STEPPER_LED_TEST_DONE\r\n"));
}
void handleCommand(char cmd) {
  if (cmd == 's') {
    beginScan();
  } else if (cmd == 'h') {
    haltScan();
  } else if (cmd == 'c') {
    uint8_t sreg = SREG;
    cli();
    encoderCount = 0;
    SREG = sreg;
    uart_puts_P(PSTR("STATUS,CLEAR\r\n"));
  } else if (cmd == 'l') {
    state = IDLE;
    setDcMotor(-80);
  } else if (cmd == 'r') {
    state = IDLE;
    setDcMotor(80);
  } else if (cmd == 'x') {
    setDcMotor(0);
  } else if (cmd == 'u') {
    movePitchManual(PITCH_MANUAL_STEP_DEG);
  } else if (cmd == 'd') {
    movePitchManual(-PITCH_MANUAL_STEP_DEG);
  } else if (cmd == 'z') {
    state = IDLE;
    setDcMotor(0);
    currentPitchLevel = 0;
    currentPitchDeg = PITCH_START_DEG;
    stepper_moveTo(stepperAngleToSteps(PITCH_START_DEG));
    uart_puts_P(PSTR("STATUS,PITCH_ZERO\r\n"));
  } else if (cmd == 'p') {
    stepperLedTest();
  } else if (cmd == 'v') {
    state = IDLE;
    setDcMotor(0);
    lidarDebugSweep = !lidarDebugSweep;
    lidarDebugYawDeg = 0.0f;
    uart_puts_P(PSTR("STATUS,LIDAR_DEBUG_SWEEP,"));
    uart_puts_P(lidarDebugSweep ? PSTR("ON\r\n") : PSTR("OFF\r\n"));
  } else if (cmd == 't') {
    printDiagnostic();
  }
}
void setup() {
  setup_gpio();
  setup_encoder_interrupt();
  uart_init_115200();
  tfmini.begin(115200);
  setup_stepper_timer();
  stepper_setCurrentPosition(stepperAngleToSteps(PITCH_START_DEG));
  setDcMotor(0);
  sei();
  uart_puts_P(PSTR("STATUS,READY\r\n"));
}
void loop() {
  while (uart_available()) {
    handleCommand(uart_read());
  }
  bool newLidarFrame = readTfminiFrame();
  if (newLidarFrame && (state == SPINNING_YAW || state == MOVING_PITCH)) {
    uint8_t sreg = SREG;
    cli();
    long sync_enc = encoderCount;
    int32_t sync_step = stepPosition;
    unsigned long sync_time = millis();
    SREG = sreg;
    long absEnc = sync_enc < 0 ? -sync_enc : sync_enc;
    if (absEnc >= WARMUP_COUNTS) {
      printSyncMeasurement(sync_time, sync_enc, sync_step, lidarDistanceCm,
                           lidarStrength);
    }
  }
  if (lidarDebugSweep && newLidarFrame &&
      millis() - lastLidarDebugPrintMs >= 80) {
    lastLidarDebugPrintMs = millis();
    printDebugSweepMeasurement();
  }
  if (state == MOVING_PITCH && stepper_distanceToGo() == 0) {
    delay(200);     
    startYawSpin(); 
  } else if (state == SPINNING_YAW) {
    updateMeasurement(); 
    if (state == SPINNING_YAW) {
      updateSpeedPID(); 
    }
  } else if (state == FINISHED) {
    setDcMotor(0);
    state = IDLE;
  }
}