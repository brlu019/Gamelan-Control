// puppet.cpp
// ESP32 puppet controller: LSM6DSO IMU + DRV8833 + MQTT beat sync
// State machine: IDLE → STRIKE (fixed torque) → DAMP_BACK (PD damping)

#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_Sensor.h>

// ————— Wi-Fi / MQTT Settings —————
const char*  WIFI_SSID    = "Berkeley-IoT";
const char*  WIFI_PASS    = "YOUR_PWD";
const char*  MQTT_BROKER  = "192.168.1.100";
const uint16_t MQTT_PORT  = 1883;
const char*  BEAT_TOPIC   = "gamelan/beat";

WiFiClient   netClient;
PubSubClient mqtt(netClient);

// ————— IMU —————
Adafruit_LSM6DSOX imu;

// ————— Motor / DRV8833 Pins & PWM —————
const int   PIN_IN1    = 16;
const int   PIN_IN2    = 17;
const int   PWM_CH     = 0;       // LEDC channel
const int   PWM_FREQ   = 5000;    // 5 kHz
const int   PWM_RES    = 8;       // 8-bit resolution

// ————— PD Gains —————
const float Kp = 1.8f;
const float Kd = 0.12f;

// ————— Strike & Impact Parameters —————
enum ArmState { IDLE, STRIKE, DAMP_BACK };
ArmState armState = IDLE;

// fixed strike torque command (0–255), + or – dir
const int   STRIKE_PWM    = 200;
const int   STRIKE_DIR    = 1;     // +1 = forward, –1 = reverse

// impact detection threshold (deg/s)
const float IMPACT_RATE   = 200.0f;
// settle thresholds for re-idling
const float SETTLE_ANGLE  = 2.0f;  // degrees
const float SETTLE_RATE   = 5.0f;  // deg/s

volatile bool beatFlag    = false;

// PD loop state
float targetAngle = 0.0f;           // desired angle (deg)
unsigned long prevMicros = 0;

// ——— MQTT callback ———
void onBeat(char* topic, byte* payload, unsigned int len) {
  beatFlag = true;
}

// ——— MQTT reconnect ———
void reconnectMQTT() {
  while (!mqtt.connected()) {
    if (mqtt.connect("esp32-puppet")) {
      mqtt.subscribe(BEAT_TOPIC);
    } else {
      delay(500);
    }
  }
}

// ——— Motor drive helper ———
void applyMotor(int command) {
  uint8_t duty = constrain(abs(command), 0, 255); // -255 ~ 255
  if (command > 0) {
    // forward
    digitalWrite(PIN_IN2, LOW);
    ledcAttachPin(PIN_IN1, PWM_CH);
    ledcWrite(PWM_CH, duty);
  } else if (command < 0) {
    // reverse
    digitalWrite(PIN_IN1, LOW);
    ledcAttachPin(PIN_IN2, PWM_CH);
    ledcWrite(PWM_CH, duty);
  } else {
    // brake
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, LOW);
  }
}

// ——— PD damping controller ———
void pdDamp(float pitch, float rate) {
  unsigned long now = micros();
  float dt = (now - prevMicros) * 1e-6f;
  prevMicros = now;

  float error = targetAngle - pitch;
  float u = Kp * error + Kd * (-rate);

  // scale control signal to motor command range
  int cmd = (int)(u * 3.0f);  // tuning factor
  applyMotor(cmd);
}

void setup() {
  Serial.begin(115200);

  // Init I2C + IMU
  Wire.begin(21, 22); // TODO: define imu pins SDA/SDL
  if (!imu.begin_I2C()) {
    Serial.println("Failed to find LSM6DSOX");
    while (1) delay(10);
  }
  imu.setAccelRange(LSM6DS_ACCEL_RANGE_4_G);
  imu.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);
  imu.setAccelDataRate(LSM6DS_RATE_104_HZ);
  imu.setGyroDataRate(LSM6DS_RATE_104_HZ);

  // Init Motor Pins & PWM
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  ledcSetup(PWM_CH, PWM_FREQ, PWM_RES);

//   // Init Wi-Fi & MQTT
//   WiFi.begin(WIFI_SSID, WIFI_PASS);
//   while (WiFi.status() != WL_CONNECTED) {
//     Serial.print('.');
//     delay(500);
//   }
//   Serial.println("\nWi-Fi connected");
//   mqtt.setServer(MQTT_BROKER, MQTT_PORT);
//   mqtt.setCallback(onBeat);
//   reconnectMQTT();

  prevMicros = micros();
}

void loop() {
  // maintain MQTT
//   if (!mqtt.connected()) reconnectMQTT();
//   mqtt.loop();

  // on-beat trigger
  if (beatFlag && armState == IDLE) {
    beatFlag = false;
    armState = STRIKE;
  }

  // read IMU once per iteration
  sensors_event_t accelEvt, gyroEvt, tempEvt;
  imu.getEvent(&accelEvt, &gyroEvt, &tempEvt);

  // compute pitch (°) from accel
  float pitch = atan2(
    accelEvt.acceleration.x,
    sqrt(accelEvt.acceleration.y * accelEvt.acceleration.y +
         accelEvt.acceleration.z * accelEvt.acceleration.z)
  ) * 180.0f / PI;

  // gyro.y → deg/s
  float rate = gyroEvt.gyro.y * 180.0f / PI;

  // state machine
  switch (armState) {
    case IDLE:
      targetAngle = 0.0f;
      pdDamp(pitch, rate);
      break;

    case STRIKE:
      // apply constant torque until impact
      applyMotor(STRIKE_DIR * STRIKE_PWM);

      if (fabs(rate) > IMPACT_RATE) {
        // impact sensed → switch to damping
        armState = DAMP_BACK;
        targetAngle = 0.0f;
        // reset PD timer
        prevMicros = micros();
      }
      break;

    case DAMP_BACK:
      // let PD loop damp back to neutral
      targetAngle = 0.0f;
      pdDamp(pitch, rate);

      // once settled, return to IDLE
      if (fabs(pitch) < SETTLE_ANGLE && fabs(rate) < SETTLE_RATE) {
        armState = IDLE;
        // brake motor
        applyMotor(0);
      }
      break;
  }

  delay(5);  // ~200 Hz
}
