/*
* Differential Drive Tracked Robot - Arduino Mega 2560 Firmware
*
* Required Libraries:
*   - Wire.h        (built-in)
*   - MPU6050.h     (Install from Library Manager: "MPU6050 by Electronic Cats"
*                    or "I2Cdevlib-MPU6050" by Jeff Rowberg)
*   - I2Cdev.h      (bundled with MPU6050 library above)
*
* Hardware:
*   - Encoder LEFT:  C1->Pin18(INT5), C2->Pin19(INT4), 3960 PPR
*   - Encoder RIGHT: C1->Pin2(INT0),  C2->Pin3(INT1),  3960 PPR
*   - Motor LEFT:    RPWM->Pin5, LPWM->Pin6  (BTS7960, EN hardwired 5V)
*   - Motor RIGHT:   RPWM->Pin9, LPWM->Pin10 (BTS7960, EN hardwired 5V)
*   - MPU6050:       SDA->Pin20, SCL->Pin21
*
* Serial Protocol (115200 baud):
*   Pi -> Arduino:  "CMD,<v_left>,<v_right>\n"  (m/s, float)
*   Arduino -> Pi:  "DATA,<tL>,<tR>,<ax>,<ay>,<az>,<gx>,<gy>,<gz>\n"
*   Data rate: 20 Hz (every 50 ms)
*/




#include <Servo.h>
#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050.h>




// ============================================================
// Pin Definitions
// ============================================================




// Encoder LEFT
#define ENC_L_C1  18  // INT5
#define ENC_L_C2  19  // INT4




// Encoder RIGHT
#define ENC_R_C1  2   // INT0
#define ENC_R_C2  3   // INT1




// Motor LEFT (BTS7960)
#define MOT_L_RPWM  5   // Forward
#define MOT_L_LPWM  6   // Reverse




// Motor RIGHT (BTS7960)
#define MOT_R_RPWM  9   // Forward
#define MOT_R_LPWM  10  // Reverse


// Pan-Tilt Servos
#define SERVO_PAN_PIN   12
#define SERVO_TILT_PIN  11




// ============================================================
// Robot Parameters
// ============================================================




#define WHEEL_RADIUS    0.03f   // meters
#define WHEELBASE       0.22f   // meters
#define ENCODER_PPR     3960L   // pulses per revolution (output shaft)
#define MAX_SPEED       0.3f    // m/s




// Conversion: v = (delta_ticks / PPR) * 2 * PI * r / dt
// At 50ms interval: v = (delta_ticks / 3960) * 2 * PI * 0.03 / 0.05
#define TICKS_TO_MPS    (2.0f * PI * WHEEL_RADIUS / (float)ENCODER_PPR)




// ============================================================
// Timing
// ============================================================




#define CONTROL_INTERVAL_MS   50    // 20 Hz
#define CMD_TIMEOUT_MS        1000  // Safety stop if no command




// ============================================================
// PID Parameters
// ============================================================




#define KP  350.0f
#define KI  80.0f
#define KD  4.0f
#define PWM_DEADBAND_L 50
#define PWM_DEADBAND_R 62
#define INTEGRAL_LIMIT  250.0f
#define PWM_MAX         255




// ============================================================
// MPU6050 Calibration Offsets (adjust after calibration)
// ============================================================




#define ACCEL_OFFSET_X  0
#define ACCEL_OFFSET_Y  0
#define ACCEL_OFFSET_Z  0
#define GYRO_OFFSET_X   0
#define GYRO_OFFSET_Y   0
#define GYRO_OFFSET_Z   0




// ============================================================
// Global Variables
// ============================================================




// Encoder tick counters (volatile, accessed in ISRs)
volatile long enc_ticks_L = 0;
volatile long enc_ticks_R = 0;




// Previous tick snapshots for velocity calculation
long prev_ticks_L = 0;
long prev_ticks_R = 0;




// PID state - LEFT
float pid_setpoint_L = 0.0f;
float pid_integral_L = 0.0f;
float pid_prev_error_L = 0.0f;




// PID state - RIGHT
float pid_setpoint_R = 0.0f;
float pid_integral_R = 0.0f;
float pid_prev_error_R = 0.0f;




// MPU6050
MPU6050 mpu;
int16_t raw_ax, raw_ay, raw_az;
int16_t raw_gx, raw_gy, raw_gz;




// Timing
unsigned long last_control_time = 0;
unsigned long last_cmd_time = 0;




// Serial parsing buffer
#define SERIAL_BUF_SIZE 64
char serial_buf[SERIAL_BUF_SIZE];
uint8_t serial_idx = 0;


// Pan-Tilt States
Servo panServo;
Servo tiltServo;

int targetPan = 85;
int targetTilt = 70;
int currentPan = 85;
int currentTilt = 70;
unsigned long lastServoUpdateTime = 0;
#define SERVO_UPDATE_INTERVAL_MS 15




// ============================================================
// Encoder ISRs - Quadrature X4 Decoding
// ============================================================




// LEFT encoder - C1 (Pin 18) change
void isr_enc_L_C1() {
 uint8_t c1 = digitalRead(ENC_L_C1);
 uint8_t c2 = digitalRead(ENC_L_C2);
 if (c1 == c2) {
   enc_ticks_L++;
 } else {
   enc_ticks_L--;
 }
}




// LEFT encoder - C2 (Pin 19) change
void isr_enc_L_C2() {
 uint8_t c1 = digitalRead(ENC_L_C1);
 uint8_t c2 = digitalRead(ENC_L_C2);
 if (c1 == c2) {
   enc_ticks_L--;
 } else {
   enc_ticks_L++;
 }
}




// RIGHT encoder - C1 (Pin 2) change
void isr_enc_R_C1() {
 uint8_t c1 = digitalRead(ENC_R_C1);
 uint8_t c2 = digitalRead(ENC_R_C2);
 if (c1 == c2) {
   enc_ticks_R--;
 } else {
   enc_ticks_R++;
 }
}




// RIGHT encoder - C2 (Pin 3) change
void isr_enc_R_C2() {
 uint8_t c1 = digitalRead(ENC_R_C1);
 uint8_t c2 = digitalRead(ENC_R_C2);
 if (c1 == c2) {
   enc_ticks_R++;
 } else {
   enc_ticks_R--;
 }
}




// ============================================================
// Setup Functions
// ============================================================




void setupEncoders() {
pinMode(ENC_L_C1, INPUT);
pinMode(ENC_L_C2, INPUT);
pinMode(ENC_R_C1, INPUT);
pinMode(ENC_R_C2, INPUT);




 // Attach interrupts on CHANGE for full quadrature (X4)
 attachInterrupt(digitalPinToInterrupt(ENC_L_C1), isr_enc_L_C1, CHANGE);
 attachInterrupt(digitalPinToInterrupt(ENC_L_C2), isr_enc_L_C2, CHANGE);
 attachInterrupt(digitalPinToInterrupt(ENC_R_C1), isr_enc_R_C1, CHANGE);
 attachInterrupt(digitalPinToInterrupt(ENC_R_C2), isr_enc_R_C2, CHANGE);
}




void setupMotors() {
 pinMode(MOT_L_RPWM, OUTPUT);
 pinMode(MOT_L_LPWM, OUTPUT);
 pinMode(MOT_R_RPWM, OUTPUT);
 pinMode(MOT_R_LPWM, OUTPUT);




 // Start with motors stopped
 analogWrite(MOT_L_RPWM, 0);
 analogWrite(MOT_L_LPWM, 0);
 analogWrite(MOT_R_RPWM, 0);
 analogWrite(MOT_R_LPWM, 0);
}




void setupMPU() {
 Wire.begin();
 Wire.setClock(400000);  // 400 kHz I2C




 mpu.initialize();




 if (!mpu.testConnection()) {
   Serial.println("ERROR: MPU6050 not found!");
 } else {
   Serial.println("MPU6050 connected.");
 }




 // Apply calibration offsets
 mpu.setXAccelOffset(ACCEL_OFFSET_X);
 mpu.setYAccelOffset(ACCEL_OFFSET_Y);
 mpu.setZAccelOffset(ACCEL_OFFSET_Z);
 mpu.setXGyroOffset(GYRO_OFFSET_X);
 mpu.setYGyroOffset(GYRO_OFFSET_Y);
 mpu.setZGyroOffset(GYRO_OFFSET_Z);




 // Set ranges: ±2g accel, ±250 deg/s gyro
 mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
 mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
}




// ============================================================
// Motor Control
// ============================================================




// Set PWM for a motor side. pwm_value is signed: positive=forward, negative=reverse.
// side: 0 = LEFT, 1 = RIGHT
void setPWM(uint8_t side, int16_t pwm_value) {
 // Clamp to valid range
 if (pwm_value > PWM_MAX) pwm_value = PWM_MAX;
 if (pwm_value < -PWM_MAX) pwm_value = -PWM_MAX;




 uint8_t rpwm_pin, lpwm_pin;
 if (side == 0) {
   rpwm_pin = MOT_L_RPWM;
   lpwm_pin = MOT_L_LPWM;
 } else {
   rpwm_pin = MOT_R_RPWM;
   lpwm_pin = MOT_R_LPWM;
 }




 // Deadband compensation: map non-zero PWM to above motor threshold
 if (pwm_value != 0) {
   int deadband = (side == 0) ? PWM_DEADBAND_L : PWM_DEADBAND_R;
   int abs_pwm = (pwm_value > 0) ? pwm_value : -pwm_value;
   // Scale abs_pwm (1-255) to (deadband-255)
   abs_pwm = deadband + (int)((255 - deadband) * (abs_pwm - 1) / 254.0f);
   abs_pwm = constrain(abs_pwm, deadband, 255);
   pwm_value = (pwm_value > 0) ? abs_pwm : -abs_pwm;
 }




 if (pwm_value > 0) {
   analogWrite(lpwm_pin, 0);
   analogWrite(rpwm_pin, (uint8_t)pwm_value);
 } else if (pwm_value < 0) {
   analogWrite(rpwm_pin, 0);
   analogWrite(lpwm_pin, (uint8_t)(-pwm_value));
 } else {
   analogWrite(rpwm_pin, 0);
   analogWrite(lpwm_pin, 0);
 }
}




// ============================================================
// PID Controller
// ============================================================




// Compute PID for one wheel. Returns signed PWM output.
// setpoint and measured are in m/s. dt is in seconds.
int16_t computePID(float setpoint, float measured, float dt,
                  float *integral, float *prev_error) {
 float error = setpoint - measured;




 // Proportional
 float p_term = KP * error;




 // Integral with anti-windup
 // Boost integral accumulation when error is large
 float integral_gain = (fabsf(error) > 0.05f) ? 2.0f : 1.0f;
 *integral += error * dt * integral_gain;
 if (*integral > INTEGRAL_LIMIT) *integral = INTEGRAL_LIMIT;
 if (*integral < -INTEGRAL_LIMIT) *integral = -INTEGRAL_LIMIT;
 float i_term = KI * (*integral);




 // Derivative
 float d_term = 0.0f;
 if (dt > 0.0f) {
   d_term = KD * (error - *prev_error) / dt;
 }
 *prev_error = error;




 // Sum and clamp
 float output = p_term + i_term + d_term;




 // Direction is determined by setpoint sign; PID output magnitude drives PWM
 int16_t pwm_out;
 if (setpoint >= 0.0f) {
   pwm_out = (int16_t)constrain(output, 0, PWM_MAX);
 } else {
   // For negative setpoint, error and output are negative
   // output will be negative, so we pass it through directly
   pwm_out = (int16_t)constrain(output, -PWM_MAX, 0);
 }




 // If setpoint is zero, force output to zero (coast)
 if (fabsf(setpoint) < 0.001f) {
   *integral = 0.0f;
   *prev_error = 0.0f;
   pwm_out = 0;
 }




 return pwm_out;
}




// ============================================================
// MPU6050 Reading
// ============================================================




// Accel scale: ±2g  -> 16384 LSB/g  -> 1 LSB = 9.80665/16384 m/s²
// Gyro scale:  ±250 -> 131 LSB/°/s  -> 1 LSB = (PI/180)/131 rad/s
#define ACCEL_SCALE  (9.80665f / 16384.0f)
#define GYRO_SCALE   (PI / 180.0f / 131.0f)




float ax_mps2, ay_mps2, az_mps2;  // m/s²
float gx_rads, gy_rads, gz_rads;  // rad/s




void readMPU() {
 mpu.getMotion6(&raw_ax, &raw_ay, &raw_az, &raw_gx, &raw_gy, &raw_gz);




 ax_mps2 = (float)raw_ax * ACCEL_SCALE;
 ay_mps2 = (float)raw_ay * ACCEL_SCALE;
 az_mps2 = (float)raw_az * ACCEL_SCALE;




 gx_rads = (float)raw_gx * GYRO_SCALE;
 gy_rads = (float)raw_gy * GYRO_SCALE;
 gz_rads = (float)raw_gz * GYRO_SCALE;
}




// ============================================================
// Serial Communication
// ============================================================




// Parse incoming serial commands. Uses char array + strtok, no String class.
void parseSerial() {
 while (Serial.available() > 0) {
   char c = (char)Serial.read();




   if (c == '\n' || c == '\r') {
     if (serial_idx > 0) {
       serial_buf[serial_idx] = '\0';
       processCommand(serial_buf);
       serial_idx = 0;
     }
   } else {
     if (serial_idx < SERIAL_BUF_SIZE - 1) {
       serial_buf[serial_idx++] = c;
     } else {
       // Buffer overflow, reset
       serial_idx = 0;
     }
   }
 }
}




void processCommand(char *buf) {
 // Expected format: "CMD,<v_left>,<v_right>"
 char *token = strtok(buf, ",");
 if (token == NULL) return;




 // Check header
 if (strcmp(token, "CMD") == 0) {
   // Parse v_left
   token = strtok(NULL, ",");
   if (token == NULL) return;
   float v_left = atof(token);

   // Parse v_right
   token = strtok(NULL, ",");
   if (token == NULL) return;
   float v_right = atof(token);

   // Clamp to max speed
   if (v_left > MAX_SPEED) v_left = MAX_SPEED;
   if (v_left < -MAX_SPEED) v_left = -MAX_SPEED;
   if (v_right > MAX_SPEED) v_right = MAX_SPEED;
   if (v_right < -MAX_SPEED) v_right = -MAX_SPEED;

   // Reset integral when direction changes to prevent overshoot
   if ((v_left > 0.0f && pid_setpoint_L < 0.0f) || (v_left < 0.0f && pid_setpoint_L > 0.0f)) {
     pid_integral_L = 0.0f;
   }
   if ((v_right > 0.0f && pid_setpoint_R < 0.0f) || (v_right < 0.0f && pid_setpoint_R > 0.0f)) {
     pid_integral_R = 0.0f;
   }
   pid_setpoint_L = v_left;
   pid_setpoint_R = v_right;
   last_cmd_time = millis();
 }
 else if (strcmp(token, "SERVO") == 0) {
   // Parse pan angle
   token = strtok(NULL, ",");
   if (token == NULL) return;
   int pan = atoi(token);

   // Parse tilt angle
   token = strtok(NULL, ",");
   if (token == NULL) return;
   int tilt = atoi(token);

   // Clamp to safe angles
   targetPan = constrain(pan, 0, 180);
   targetTilt = constrain(tilt, 0, 180);
 }
}




// Send data packet to Pi
void sendData(long ticks_L, long ticks_R) {
 char ax_s[10], ay_s[10], az_s[10];
 char gx_s[10], gy_s[10], gz_s[10];
 float ax = isnan(ax_mps2) ? 0.0f : ax_mps2;
 float ay = isnan(ay_mps2) ? 0.0f : ay_mps2;
 float az = isnan(az_mps2) ? 0.0f : az_mps2;
 float gx = isnan(gx_rads) ? 0.0f : gx_rads;
 float gy = isnan(gy_rads) ? 0.0f : gy_rads;
 float gz = isnan(gz_rads) ? 0.0f : gz_rads;


 dtostrf(ax, 7, 4, ax_s);
 dtostrf(ay, 7, 4, ay_s);
 dtostrf(az, 7, 4, az_s);
 dtostrf(gx, 7, 4, gx_s);
 dtostrf(gy, 7, 4, gy_s);
 dtostrf(gz, 7, 4, gz_s);


 Serial.print("DATA,");
 Serial.print(ticks_L);
 Serial.print(',');
 Serial.print(ticks_R);
 Serial.print(',');
 Serial.print(ax_s);
 Serial.print(',');
 Serial.print(ay_s);
 Serial.print(',');
 Serial.print(az_s);
 Serial.print(',');
 Serial.print(gx_s);
 Serial.print(',');
 Serial.print(gy_s);
 Serial.print(',');
 Serial.println(gz_s);
}




// ============================================================
// Main Setup
// ============================================================




void setup() {
 Serial.begin(115200);
 delay(100); // Wait for UART to stabilize (Mega uses hardware UART, not native USB)




 Serial.println("Robot firmware starting...");




 setupEncoders();
 setupMotors();
 setupMPU();

 // Attach and init Servos
 panServo.attach(SERVO_PAN_PIN);
 tiltServo.attach(SERVO_TILT_PIN);
 panServo.write(currentPan);
 tiltServo.write(currentTilt);




 last_control_time = millis();
 last_cmd_time = millis();




 Serial.println("Robot firmware ready.");
}




// ============================================================
// Main Loop
// ============================================================




void loop() {
 // Always parse incoming serial data
 parseSerial();




 unsigned long now = millis();




 // Run control loop at 20 Hz
 if (now - last_control_time >= CONTROL_INTERVAL_MS) {
   float dt = (float)(now - last_control_time) / 1000.0f;
   last_control_time = now;




   // --- Safety timeout ---
   if (now - last_cmd_time > CMD_TIMEOUT_MS) {
     pid_setpoint_L = 0.0f;
     pid_setpoint_R = 0.0f;
   }




   // --- Read encoder ticks atomically ---
   noInterrupts();
   long current_ticks_L = enc_ticks_L;
   long current_ticks_R = enc_ticks_R;
   interrupts();




   // --- Compute actual velocity (m/s) ---
   long delta_L = current_ticks_L - prev_ticks_L;
   long delta_R = current_ticks_R - prev_ticks_R;
   prev_ticks_L = current_ticks_L;
   prev_ticks_R = current_ticks_R;




   float velocity_L = (float)delta_L * TICKS_TO_MPS / dt;
   float velocity_R = (float)delta_R * TICKS_TO_MPS / dt;




   // --- PID control ---
   int16_t pwm_L = computePID(pid_setpoint_L, velocity_L, dt,
                               &pid_integral_L, &pid_prev_error_L);
   int16_t pwm_R = computePID(pid_setpoint_R, velocity_R, dt,
                               &pid_integral_R, &pid_prev_error_R);




   // --- Apply PWM to motors ---
   setPWM(0, pwm_L);  // LEFT
   setPWM(1, pwm_R);  // RIGHT




   // --- Read IMU ---
   readMPU();




   // --- Send data to Pi ---
   sendData(current_ticks_L, current_ticks_R);
 }

 // --- Update servo angles smoothly (Non-blocking) ---
 if (now - lastServoUpdateTime >= SERVO_UPDATE_INTERVAL_MS) {
   lastServoUpdateTime = now;
    
   // Smoothly step Pan angle
   if (currentPan < targetPan) currentPan++;
   else if (currentPan > targetPan) currentPan--;
   panServo.write(currentPan);
    
   // Smoothly step Tilt angle
   if (currentTilt < targetTilt) currentTilt++;
   else if (currentTilt > targetTilt) currentTilt--;
   tiltServo.write(currentTilt);
 }
}
