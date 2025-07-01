#include <string>
#include <Ticker.h>
#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <QTRSensors.h>
#include "NimBLEDevice.h"

#define NUM_SENSORS  8    
#define EMITTER_PIN   16     
#define LEFT_MOTOR_FORWARD 14
#define LEFT_MOTOR_BACKWARD 13
#define RIGHT_MOTOR_FORWARD 11
#define RIGHT_MOTOR_BACKWARD 12
#define NEOPIXEL_PIN 48
#define NUM_PIXELS 1

#define DEVICE_NAME "LineFollower"

float Kp = 0.119810; 
float Ki = 0.01;  
float Kd = 1.059049;
float Speed = 125;
float TurnSpeed = 120;

#define ENC1_A 35
#define ENC1_B 36
#define ENC2_A 1
#define ENC2_B 21

// Gear ratio n: 5.47:1 14 84 impulses per revolution
const float wheelCircumference = 3.1416 * 29.0; // mm
const int ENCODER_IMPULSES_PER_REV = 84;
// const float GEAR_RATIO = 5.47; 
float distancePerPulse = wheelCircumference / ENCODER_IMPULSES_PER_REV * 1.096;

volatile int32_t encoderLeftCount = 0;
volatile int32_t encoderRightCount = 0;

static NimBLEServer* pServer;
NimBLECharacteristic* kpChar;
NimBLECharacteristic* kiChar;
NimBLECharacteristic* kdChar;
NimBLECharacteristic* speedChar;
NimBLECharacteristic* turnSpeedChar;
NimBLECharacteristic* sensorDataChar;
NimBLECharacteristic* calibrateChar;
NimBLECharacteristic* startStopChar;

float lost_threshold = 750;
float LeftEncoderStatus = 0;
float RightEncoderStatus = 0;

QTRSensors qtr;

uint16_t sensorValues[NUM_SENSORS];
int ready = 0;

Adafruit_NeoPixel pixels(NUM_PIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

void IRAM_ATTR encoder1ISR() {
  bool a = digitalRead(ENC1_A);
  bool b = digitalRead(ENC1_B);
  if (a == b) encoderLeftCount++;
  else encoderLeftCount--;
}

void IRAM_ATTR encoder2ISR() {
  bool a = digitalRead(ENC2_A);
  bool b = digitalRead(ENC2_B);
  if (a == b) encoderRightCount++;
  else encoderRightCount--;
}

void setMotor(int pwm, int in1, int in2) {
    if (pwm > 0) {
        analogWrite(in1, pwm);
        analogWrite(in2, 0);
    } else {
        analogWrite(in1, 0);
        analogWrite(in2, -pwm);
    }
}

void setupBLE() {
    NimBLEDevice::init(DEVICE_NAME);
    NimBLEDevice::setMTU(517);
    NimBLEDevice::setPower(ESP_PWR_LVL_P9);
    NimBLEDevice::setSecurityAuth(true, true, true);
    NimBLEDevice::setSecurityPasskey(123456);
    NimBLEDevice::setSecurityIOCap(BLE_HS_IO_DISPLAY_YESNO);

    pServer = NimBLEDevice::createServer();
    NimBLEService* pService = pServer->createService("12345678-1234-1234-1234-1234567890ab");

    kpChar = pService->createCharacteristic("0001", NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
    kiChar = pService->createCharacteristic("0002", NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
    kdChar = pService->createCharacteristic("0003", NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
    speedChar = pService->createCharacteristic("0004", NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
    turnSpeedChar = pService->createCharacteristic("0006", NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
    sensorDataChar = pService->createCharacteristic("0007", NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
    calibrateChar = pService->createCharacteristic("0009", NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::READ);
    startStopChar = pService->createCharacteristic("000A", NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::READ);

    kpChar->setValue(std::to_string(Kp));
    kiChar->setValue(std::to_string(Ki));
    kdChar->setValue(std::to_string(Kd));
    speedChar->setValue(std::to_string(Speed));
    turnSpeedChar->setValue(std::to_string(TurnSpeed));
    calibrateChar->setValue("0");
    startStopChar->setValue("0");

    pService->start();
    NimBLEAdvertising* pAdvertising = NimBLEDevice::getAdvertising();
    pAdvertising->setName(DEVICE_NAME);
    pAdvertising->addServiceUUID(pService->getUUID());

    pAdvertising->enableScanResponse(true);
    pAdvertising->start();
    Serial.println("BLE Service started, waiting for connections...");
}

void setup() {
    pixels.begin();
    Serial.begin(115200);

    pinMode(LEFT_MOTOR_FORWARD, OUTPUT);
    pinMode(LEFT_MOTOR_BACKWARD, OUTPUT);
    pinMode(RIGHT_MOTOR_FORWARD, OUTPUT);
    pinMode(RIGHT_MOTOR_BACKWARD, OUTPUT);
    analogWrite(LEFT_MOTOR_FORWARD, 0);
    analogWrite(LEFT_MOTOR_BACKWARD, 0);
    analogWrite(RIGHT_MOTOR_FORWARD, 0);
    analogWrite(RIGHT_MOTOR_BACKWARD, 0);
    analogReadResolution(12);

    qtr.setTypeAnalog();
    // qtr.setSensorPins((const uint8_t[]){ 9, 3, 8, 6, 10, 5, 4, 7}, NUM_SENSORS);
    qtr.setSensorPins((const uint8_t[]){ 7, 4, 5, 10, 6, 8, 3, 9}, NUM_SENSORS);
    qtr.setEmitterPin(EMITTER_PIN);

    pinMode(ENC1_A, INPUT_PULLUP);
    pinMode(ENC1_B, INPUT_PULLUP);
    pinMode(ENC2_A, INPUT_PULLUP);
    pinMode(ENC2_B, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(ENC1_A), encoder1ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC2_A), encoder2ISR, CHANGE);

    setupBLE();

    // Write header for serial output
    // Serial.println("millis\tleftDistance\trightDistance\taccel.x\taccel.y\tgyro.x\tgyro.y\tposition\tsensor1\tsensor2\tsensor3\tsensor4\tsensor5\tsensor6\tsensor7\tsensor8\terror\tleftPWM\trightPWM\tlost\tlast_sighted\tKp\tKi\tKd\tSpeed\tready");
}

int lastError = 0;
int rightPWM = 0;
int leftPWM = 0;
int last_sighted = 0;
int lost;
int lost_sensors;
int last_detection_time = 0;
int sumError = 0;
float leftDistance = 0;
float rightDistance = 0;

unsigned long lastNotifyTime = 0;

void pid_autotune() {
    Serial.println("Autotune: Twiddle start");

    float p[3] = {0.1, 0.0, 1.0}; // start Kp, Ki, Kd
    float dp[3] = {0.1, 0.01, 0.1};
    float best_err = 1e9;
    int n_iter = 8; // liczba iteracji twiddle
    int twiddle_loops = 0;

    for (int iter = 0; iter < n_iter; iter++) {
        for (int param = 0; param < 3; param++) {
            p[param] += dp[param];

            // Testuj aktualne PID
            float avg_err = 0;
            int samples = 0;
            int lastError = 0, sumError = 0;
            unsigned long start = millis();
            while (millis() - start < 3000) { // 3 sekundy testu
                int position = qtr.readLineBlack(sensorValues);
                int error = position - 3500;
                sumError += error;
                int correction = p[0] * error + p[2] * (error - lastError) + p[1] * sumError;
                lastError = error;

                int baseSpeed = 125;
                int leftPWM = constrain(baseSpeed - correction, -baseSpeed, baseSpeed);
                int rightPWM = constrain(baseSpeed + correction, -baseSpeed, baseSpeed);

                setMotor(leftPWM, LEFT_MOTOR_FORWARD, LEFT_MOTOR_BACKWARD);
                setMotor(rightPWM, RIGHT_MOTOR_FORWARD, RIGHT_MOTOR_BACKWARD);

                avg_err += abs(error);
                samples++;
                delay(20);
            }
            setMotor(0, LEFT_MOTOR_FORWARD, LEFT_MOTOR_BACKWARD);
            setMotor(0, RIGHT_MOTOR_FORWARD, RIGHT_MOTOR_BACKWARD);

            avg_err = (samples > 0) ? avg_err / samples : 1e9;

            if (avg_err < best_err) {
                best_err = avg_err;
                dp[param] *= 1.1;
            } else {
                p[param] -= 2 * dp[param];
                if (p[param] < 0) p[param] = 0;
                // Testuj w drugą stronę
                avg_err = 0;
                samples = 0;
                lastError = 0; sumError = 0;
                start = millis();
                while (millis() - start < 3000) {
                    int position = qtr.readLineBlack(sensorValues);
                    int error = position - 3500;
                    sumError += error;
                    int correction = p[0] * error + p[2] * (error - lastError) + p[1] * sumError;
                    lastError = error;

                    int baseSpeed = 125;
                    int leftPWM = constrain(baseSpeed - correction, -baseSpeed, baseSpeed);
                    int rightPWM = constrain(baseSpeed + correction, -baseSpeed, baseSpeed);

                    setMotor(leftPWM, LEFT_MOTOR_FORWARD, LEFT_MOTOR_BACKWARD);
                    setMotor(rightPWM, RIGHT_MOTOR_FORWARD, RIGHT_MOTOR_BACKWARD);

                    avg_err += abs(error);
                    samples++;
                    delay(20);
                }
                setMotor(0, LEFT_MOTOR_FORWARD, LEFT_MOTOR_BACKWARD);
                setMotor(0, RIGHT_MOTOR_FORWARD, RIGHT_MOTOR_BACKWARD);

                avg_err = (samples > 0) ? avg_err / samples : 1e9;

                if (avg_err < best_err) {
                    best_err = avg_err;
                    dp[param] *= 1.1;
                } else {
                    p[param] += dp[param];
                    dp[param] *= 0.9;
                }
            }
            Serial.print("Twiddle iter "); Serial.print(iter);
            Serial.print(" param "); Serial.print(param);
            Serial.print(" Kp="); Serial.print(p[0], 6);
            Serial.print(" Ki="); Serial.print(p[1], 6);
            Serial.print(" Kd="); Serial.print(p[2], 6);
            Serial.print(" best_err="); Serial.println(best_err, 6);
            delay(500);
        }
    }
    // Ustaw najlepsze PID
    Kp = p[0];
    Ki = p[1];
    Kd = p[2];
    Serial.print("Autotune zakończone. Kp="); Serial.print(Kp, 6);
    Serial.print(" Ki="); Serial.print(Ki, 6);
    Serial.print(" Kd="); Serial.println(Kd, 6);
}

// Twiddle parameters
float p[3] = {Kp, Ki, Kd}; // Kp, Ki, Kd
float dp[3] = {0.08, 0.0, 0.1};
float best_err = 1e9;
int twiddle_step = 0, twiddle_param = 0;
unsigned long lastTwiddleTime = 0;
bool twiddle_increase = true;
int twiddle_count = 0;
const int twiddle_interval = 3000; // 3 seconds
int twiddle_error_sum = 0;
int twiddle_error_count = 0;

void twiddleUpdate(int error, int lost) {
    if (lost == 1) return; // Ignore measurements when lost
    twiddle_error_sum += abs(error);
    twiddle_error_count++;
    unsigned long now = millis();
    if (now - lastTwiddleTime < twiddle_interval) return;

    float avg_err = (twiddle_error_count > 0) ? (float)twiddle_error_sum / twiddle_error_count : 0;
    lastTwiddleTime = now;
    twiddle_error_sum = 0;
    twiddle_error_count = 0;

    float sum_dp = dp[0] + dp[1] + dp[2];
    // if (sum_dp < 0.0001) return; // Stop if changes are too small

    if (twiddle_step == 0) {
        p[twiddle_param] += dp[twiddle_param];
        twiddle_step = 1;
    } else if (twiddle_step == 1) {
        if (avg_err < best_err) {
            best_err = avg_err;
            dp[twiddle_param] *= 1.1;
            twiddle_param = (twiddle_param + 1) % 3;
            twiddle_step = 0;
        } else {
            p[twiddle_param] -= 2 * dp[twiddle_param];
            twiddle_step = 2;
        }
    } else if (twiddle_step == 2) {
        if (avg_err < best_err) {
            best_err = avg_err;
            dp[twiddle_param] *= 1.1;
        } else {
            p[twiddle_param] += dp[twiddle_param];
            dp[twiddle_param] *= 0.9;
        }
        twiddle_param = (twiddle_param + 1) % 3;
        twiddle_step = 0;
    }

    // Clamp to non-negative
    for (int i = 0; i < 3; i++) if (p[i] < 0) p[i] = 0;

    // Update global PID
    Kp = p[0];
    Ki = p[1];
    Kd = p[2];
    // Synchronize BLE characteristics with new Twiddle values
    kpChar->setValue(std::to_string(Kp));
    kiChar->setValue(std::to_string(Ki));
    kdChar->setValue(std::to_string(Kd));
}

void calibrate(){
    // Serial.println("Calibrating sensors...");
    qtr.resetCalibration();
    for (uint16_t i = 0; i < 400; i++){
        qtr.calibrate();
    }
    for (uint8_t i = 0; i < NUM_SENSORS; i++){
        // Serial.print(qtr.calibrationOn.minimum[i]);
        // Serial.print(' ');
    }
    // pid_autotune();
}

void loop() {
    int position = qtr.readLineBlack(sensorValues);

    leftDistance = encoderLeftCount * distancePerPulse;
    rightDistance = encoderRightCount * distancePerPulse;

    unsigned long currentMillis = millis();
    if (currentMillis - lastNotifyTime >= 200) {
        lastNotifyTime = currentMillis;

        String sensorData = String(position);
        for(int i = 0; i < NUM_SENSORS; i++){
            sensorData += "," + String(sensorValues[i]);
        }
        sensorDataChar->setValue(sensorData.c_str());
        sensorDataChar->notify();
    }

    if (startStopChar->getValue() == "1") {
        ready = 1;
        sumError = 0;
    } else if (startStopChar->getValue() == "0") {
        ready = 0;
    }

    if (calibrateChar->getValue() == "1") {
        calibrate();
        calibrateChar->setValue("0");
    }

    // Aktualizacja parametrów z BLE
    // Nie nadpisuj Kp, Ki, Kd podczas aktywnego Twiddle
    if (twiddle_step == 0) { // Twiddle nieaktywny, pozwalamy BLE nadpisać
        if (kpChar->getValue().length() > 0) Kp = std::stof(kpChar->getValue());
        if (kiChar->getValue().length() > 0) Ki = std::stof(kiChar->getValue());
        if (kdChar->getValue().length() > 0) Kd = std::stof(kdChar->getValue());
    }
    if (speedChar->getValue().length() > 0) Speed = std::stof(speedChar->getValue());
    if (turnSpeedChar->getValue().length() > 0) TurnSpeed = std::stof(turnSpeedChar->getValue());

    if (sensorValues[0] >= 800 && sensorValues[NUM_SENSORS-1] < 800) {
        if (last_sighted != 1 && millis() - last_detection_time >= 100) {
            last_sighted = 1;
            last_detection_time = millis();
            pixels.setPixelColor(0, pixels.Color(0, 255, 0)); // Zielony dla lewej
            pixels.show();
        }
    } else if (sensorValues[0] < 800 && sensorValues[NUM_SENSORS-1] >= 800) {
        if (last_sighted != 2 && millis() - last_detection_time >= 100) {
            last_sighted = 2;
            last_detection_time = millis();
            pixels.setPixelColor(0, pixels.Color(255, 0, 0)); // Czerwony dla prawej
            pixels.show();
        }
    }

    lost_sensors = 0;
    lost = 0;
    for(int i = 0; i < NUM_SENSORS; i++){
        if(sensorValues[i] <= lost_threshold){
            lost_sensors += 1;
        }
    }
    if(lost_sensors >= (NUM_SENSORS)){
    lost = 1;
    }

    int error = position - 3500;

    // Twiddle PID tuning
    // twiddleUpdate(error, lost);

    sumError += error; // accumulate error for integral term
    int correction = Kp * error + Kd * (error - lastError) + Ki * sumError;
    lastError = error;

    leftPWM  = constrain(Speed - correction, 0, Speed);
    rightPWM = constrain(Speed + correction, 0, Speed);

    if (ready == 1) {
        if (lost == 1 && last_sighted == 2) {
            setMotor(-TurnSpeed, LEFT_MOTOR_FORWARD, LEFT_MOTOR_BACKWARD);
            setMotor(TurnSpeed, RIGHT_MOTOR_FORWARD, RIGHT_MOTOR_BACKWARD);
        } else if (lost == 1 && last_sighted == 1) {
            setMotor(TurnSpeed, LEFT_MOTOR_FORWARD, LEFT_MOTOR_BACKWARD);
            setMotor(-TurnSpeed, RIGHT_MOTOR_FORWARD, RIGHT_MOTOR_BACKWARD);
        } else {
            setMotor(leftPWM,  LEFT_MOTOR_FORWARD,  LEFT_MOTOR_BACKWARD);
            setMotor(rightPWM, RIGHT_MOTOR_FORWARD, RIGHT_MOTOR_BACKWARD);
        }
    } else {
        setMotor(0, LEFT_MOTOR_FORWARD, LEFT_MOTOR_BACKWARD);
        setMotor(0, RIGHT_MOTOR_FORWARD, RIGHT_MOTOR_BACKWARD);
    }

    char line[200];
    snprintf(line, sizeof(line), "%lu\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%d",
        millis(), leftDistance, rightDistance, 0, 0,
        0, 0, position);
    Serial.print(line);

    for (int i = 0; i < NUM_SENSORS; i++) {
        Serial.print('\t');
        Serial.print(sensorValues[i]);
    }

    snprintf(line, sizeof(line), "\t%d\t%d\t%d\t%d\t%d\t%.2f\t%.6f\t%.6f\t%.6f\t%d\n",
        error, leftPWM, rightPWM, lost, last_sighted,
        Kp, Ki, Kd, Speed, ready);
    Serial.print(line);
    // delay(10);

    // Serial.println("Twiddle Step: " + String(twiddle_step) + 
    //     ", Param: " + String(twiddle_param) + 
    //     ", Kp: " + String(Kp, 6) + 
    //     ", Ki: " + String(Ki, 6) + 
    //     ", Kd: " + String(Kd, 6) + 
    //     ", Best Error: " + String(best_err, 6));
    // Serial.println("dp: " + String(dp[0], 6) + ", " + String(dp[1], 6) + ", " + String(dp[2], 6) + 
    //     ", Twiddle Count: " + String(twiddle_count) + 
    //     ", Twiddle Interval: " + String(twiddle_interval) + 
    //     ", Twiddle Error Sum: " + String(twiddle_error_sum) + 
    //     ", Twiddle Error Count: " + String(twiddle_error_count));
}
