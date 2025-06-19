#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <QTRSensors.h>
#include <AS5600.h>
#include "NimBLEDevice.h"

#define SDA_1 17
#define SCL_1 15
#define SDA_2 18
#define SCL_2 46
#define LEFT_DIR_PIN 20
#define RIGHT_DIR_PIN 19

#define NUM_SENSORS  8    
#define EMITTER_PIN   16     
#define LEFT_MOTOR_FORWARD 14
#define LEFT_MOTOR_BACKWARD 13
#define RIGHT_MOTOR_FORWARD 12
#define RIGHT_MOTOR_BACKWARD 11
#define NEOPIXEL_PIN 48
#define NUM_PIXELS 1

#define DEVICE_NAME "LineFollower"

float Kp = 0.5; 
float Ki = 0.0;  
float Kd = 5;
float Speed = 50;
float TurnSpeed = 50;

static NimBLEServer* pServer;
NimBLECharacteristic* kpChar;
NimBLECharacteristic* kiChar;
NimBLECharacteristic* kdChar;
NimBLECharacteristic* speedChar; // Add this for BLE
NimBLECharacteristic* turnSpeedChar;
NimBLECharacteristic* sensorDataChar;
NimBLECharacteristic* calibrateChar;
NimBLECharacteristic* startStopChar;

float lost_threshold = 450;
float LeftEncoderStatus = 0;
float RightEncoderStatus = 0;

TwoWire I2C_1(0);
TwoWire I2C_2(1);

AS5600 as5600_left(&I2C_1); // left motor
AS5600 as5600_right(&I2C_2); // right motor

QTRSensors qtr;

uint16_t sensorValues[NUM_SENSORS];
int ready = 0;

Adafruit_NeoPixel pixels(NUM_PIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

void calibrate(){
    Serial.println("Calibrating sensors...");
    qtr.resetCalibration();
    for (uint16_t i = 0; i < 400; i++){
        qtr.calibrate();
    }
    for (uint8_t i = 0; i < NUM_SENSORS; i++){
        Serial.print(qtr.calibrationOn.minimum[i]);
        Serial.print(' ');
    }
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
    calibrateChar->setValue("0");

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

    I2C_1.begin(SDA_1, SCL_1, 100000);
    I2C_2.begin(SDA_2, SCL_2, 100000);

    as5600_left.begin(LEFT_DIR_PIN);
    as5600_right.begin(RIGHT_DIR_PIN);
    as5600_left.setDirection(AS5600_COUNTERCLOCK_WISE);
    as5600_right.setDirection(AS5600_CLOCK_WISE);

    LeftEncoderStatus = as5600_left.isConnected();
    RightEncoderStatus = as5600_right.isConnected();

    Serial.println(as5600_left.getAddress());
    Serial.println(as5600_right.getAddress());
    Serial.println(String(LeftEncoderStatus) + " " + String(RightEncoderStatus));

    setupBLE();
}

int lastError = 0;
int rightPWM = 0;
int leftPWM = 0;
int last_sighted = 0;
int lost;
int lost_sensors;
int last_detection_time = 0;
int sumError = 0;

unsigned long lastNotifyTime = 0;

void loop() {
    int position = qtr.readLineBlack(sensorValues);

    unsigned long currentMillis = millis();
    if (currentMillis - lastNotifyTime >= 200) {
        lastNotifyTime = currentMillis;

        // Przygotowanie i wysłanie danych czujników
        String sensorData = String(position);
        for(int i = 0; i < NUM_SENSORS; i++){
            sensorData += "," + String(sensorValues[i]);
        }
        sensorDataChar->setValue(sensorData.c_str());
        sensorDataChar->notify();
    }

    if (startStopChar->getValue() == "1") {
        ready = 1;
    } else if (startStopChar->getValue() == "0") {
        ready = 0;
    }

    if (calibrateChar->getValue() == "1") {
        calibrate();
        calibrateChar->setValue("0");
    }

    // Aktualizacja parametrów z BLE
    if (kpChar->getValue().length() > 0) Kp = std::stof(kpChar->getValue());
    if (kiChar->getValue().length() > 0) Ki = std::stof(kiChar->getValue());
    if (kdChar->getValue().length() > 0) Kd = std::stof(kdChar->getValue());
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

    sumError += error; // accumulate error for integral term
    int correction = Kp * error + Kd * (error - lastError) + Ki * sumError;
    lastError = error;

    leftPWM  = constrain(Speed - correction, Speed/10, Speed);
    rightPWM = constrain(Speed + correction, Speed/10, Speed);

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
    delay(10);
}
