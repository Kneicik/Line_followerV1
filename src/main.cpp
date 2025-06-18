#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <QTRSensors.h>
#include <AS5600.h>

float Kp = 0.5; 
float Ki = 0.0;  
float Kd = 5;
float MaxSpeed = 80; 
float BaseSpeed = 50;
float TurnSpeed = 70;
float lost_threshold = 450;
float LeftEncoderStatus = 0;
float RightEncoderStatus = 0;

#define SDA_1 17
#define SCL_1 15
#define SDA_2 18
#define SCL_2 46
#define LEFT_DIR_PIN 20
#define RIGHT_DIR_PIN 19

#define NUM_SENSORS  8    
#define EMITTER_PIN   16     
#define LEFT_MOTOR_FORWARD 12
#define LEFT_MOTOR_BACKWARD 11
#define RIGHT_MOTOR_FORWARD 14
#define RIGHT_MOTOR_BACKWARD 13
#define NEOPIXEL_PIN 48
#define NUM_PIXELS 1

TwoWire I2C_1(0);
TwoWire I2C_2(1);

AS5600 as5600_left(&I2C_1); // left motor
AS5600 as5600_right(&I2C_2); // right motor

QTRSensors qtr;

uint16_t sensorValues[NUM_SENSORS];
int ready = 0;

Adafruit_NeoPixel pixels(NUM_PIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

void calibrate(){
  qtr.resetCalibration();
  for (uint16_t i = 0; i < 400; i++){
    qtr.calibrate();
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

void setup() {
    pixels.begin();
    Serial.begin(115200);

    pinMode(LEFT_MOTOR_FORWARD, OUTPUT);
    pinMode(LEFT_MOTOR_BACKWARD, OUTPUT);
    pinMode(RIGHT_MOTOR_FORWARD, OUTPUT);
    pinMode(RIGHT_MOTOR_BACKWARD, OUTPUT);
    digitalWrite(LEFT_MOTOR_FORWARD, LOW);
    digitalWrite(LEFT_MOTOR_BACKWARD, LOW);
    digitalWrite(RIGHT_MOTOR_FORWARD, LOW);
    digitalWrite(RIGHT_MOTOR_BACKWARD, LOW);
    analogReadResolution(12);

    qtr.setTypeAnalog();
    qtr.setSensorPins((const uint8_t[]){ 9, 3, 8, 6, 10, 5, 4, 7}, NUM_SENSORS);
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
    Serial.print(String(LeftEncoderStatus) + " " + String(RightEncoderStatus));
    calibrate();
    ready = 1;
}

int lastError = 0;
int rightPWM = 0;
int leftPWM = 0;
int last_sighted = 0;
int lost;
int lost_sensors;
int last_detection_time = 0;

void loop() {
    int position = qtr.readLineBlack(sensorValues);

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

    int motorSpeed = Kp * error + Kd * (error - lastError);
    lastError = error;

    leftPWM  = constrain(BaseSpeed - motorSpeed, -MaxSpeed, MaxSpeed);
    rightPWM = constrain(BaseSpeed + motorSpeed, -MaxSpeed, MaxSpeed);

    setMotor(leftPWM,  LEFT_MOTOR_FORWARD,  LEFT_MOTOR_BACKWARD);
    setMotor(rightPWM, RIGHT_MOTOR_FORWARD, RIGHT_MOTOR_BACKWARD); 

    if (ready == 1) {
        if (lost == 1 && last_sighted == 1) {
            setMotor(-TurnSpeed, LEFT_MOTOR_FORWARD, LEFT_MOTOR_BACKWARD);
            setMotor(TurnSpeed, RIGHT_MOTOR_FORWARD, RIGHT_MOTOR_BACKWARD);
        } else if (lost == 1 && last_sighted == 2) {
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
}
