Created by Dor and Daniel

// ===================== BLE =====================

#include <BLEDevice.h>   //  Initializes and manages the ESP32 BLE stack, device name, advertising, and server creation.
#include <BLEServer.h>   //  Provides the BLE GATT server used to host services and characteristics.
#include <BLEUtils.h>    //  Utility definitions and helpers required by the ESP32 BLE library.
#include <BLE2902.h>     //  Enables Client Characteristic Configuration Descriptor (CCCD) for BLE notifications.


// ===================== Pins =====================
#define IN1_LEFT  26
#define IN2_LEFT  25
#define IN3_LEFT  18
#define IN4_LEFT  27

#define IN1_RIGHT 13
#define IN2_RIGHT 12
#define IN3_RIGHT 32
#define IN4_RIGHT 14

#define EN_LEFT   21
#define EN_RIGHT  22

#define CNY70_LEFT_PIN   34
#define CNY70_RIGHT_PIN  35
#define CNY70_SPEED_PIN  33

// ===================== Moving Average (3 sensors) =====================
// Moving average for stable sensor readings.
const int SENSOR_MA_WINDOW = 10;

uint16_t leftBuf[SENSOR_MA_WINDOW];
uint16_t rightBuf[SENSOR_MA_WINDOW];
uint16_t speedBuf[SENSOR_MA_WINDOW];

uint32_t leftSum = 0;
uint32_t rightSum = 0;
uint32_t speedSum = 0;

int maIndex = 0;
bool maReady = false;

void initMovingAverage() {
  leftSum = 0;
  rightSum = 0;
  speedSum = 0;

  for (int i = 0; i < SENSOR_MA_WINDOW; i++) {
    uint16_t l = (uint16_t)analogRead(CNY70_LEFT_PIN);
    uint16_t r = (uint16_t)analogRead(CNY70_RIGHT_PIN);
    uint16_t s = (uint16_t)analogRead(CNY70_SPEED_PIN);

    leftBuf[i] = l;   leftSum += l;
    rightBuf[i] = r;  rightSum += r;
    speedBuf[i] = s;  speedSum += s;

    delay(2);
  }

  maIndex = 0;
  maReady = true;
}

int readLeftSensorMA() {
  if (!maReady) initMovingAverage();

  uint16_t newVal = (uint16_t)analogRead(CNY70_LEFT_PIN);
  leftSum = leftSum - leftBuf[maIndex] + newVal;
  leftBuf[maIndex] = newVal;
  return (int)(leftSum / (uint32_t)SENSOR_MA_WINDOW);
}

int readRightSensorMA() {
  if (!maReady) initMovingAverage();

  uint16_t newVal = (uint16_t)analogRead(CNY70_RIGHT_PIN);
  rightSum = rightSum - rightBuf[maIndex] + newVal;
  rightBuf[maIndex] = newVal;
  return (int)(rightSum / (uint32_t)SENSOR_MA_WINDOW);
}

int readSpeedSensorMA() {
  if (!maReady) initMovingAverage();

  uint16_t newVal = (uint16_t)analogRead(CNY70_SPEED_PIN);
  speedSum = speedSum - speedBuf[maIndex] + newVal;
  speedBuf[maIndex] = newVal;
  return (int)(speedSum / (uint32_t)SENSOR_MA_WINDOW);
}

void advanceMAIndex() {
  maIndex++;
  if (maIndex >= SENSOR_MA_WINDOW) maIndex = 0;
}

// ===================== Thresholds =====================
const int THRESHOLD_LEFT  = 2800;
const int THRESHOLD_RIGHT = 2800;
const int THRESHOLD_SPEED = 2500;

// ===================== Speeds / Modes =====================
// If speed sensor reads "white" -> FAST, else -> SLOW
int SPEED_FAST = 160;
int SPEED_SLOW = 90;

// Mode presets (only FAST/SLOW change per mode)
const int SPEED_FAST_NORMAL = 150;
const int SPEED_SLOW_NORMAL = 90;

const int SPEED_FAST_SPORT  = 180;
const int SPEED_SLOW_SPORT  = 100;

const int SPEED_FAST_ECHO   = 140;
const int SPEED_SLOW_ECHO   = 80;

// Turning speed
const int SPEED_TURN = 150;

// ===================== Runtime state =====================
int currentForwardSpeed = 0;  // Updated every loop based on speed sensor

bool sportFlag = false;  // Used only to disable the startup boost in SPORT/ECHO
bool startFlag = true;

// ===================== BLE UUIDs =====================
#define SERVICE_UUID        "12345678-1234-1234-1234-1234567890ab"
#define COMMAND_CHAR_UUID   "abcdefab-1234-5678-9abc-1234567890ab"
#define SENSOR_CHAR_UUID    "fedcba98-7654-4321-9abc-1234567890ab"

// ===================== State machine =====================
enum CarState { CAR_IDLE = 0, CAR_RUN = 1 };
volatile CarState globalState = CAR_IDLE;

// "Memory" of the last correction direction when both sensors see white.
enum LastSide { SIDE_NONE = 0, SIDE_LEFT = 1, SIDE_RIGHT = 2 };
LastSide lastSide = SIDE_NONE;

BLEServer *pServer = nullptr;
BLECharacteristic *pCommandCharacteristic = nullptr;
BLECharacteristic *pSensorCharacteristic = nullptr;

// ===================== Motor helpers =====================
// Direction is controlled by IN pins, and speed by EN PWM.

void setLeftForward() {
  digitalWrite(IN1_LEFT, HIGH); digitalWrite(IN2_LEFT, LOW);
  digitalWrite(IN3_LEFT, HIGH); digitalWrite(IN4_LEFT, LOW);
}

void setLeftBackward() {
  digitalWrite(IN1_LEFT, LOW); digitalWrite(IN2_LEFT, HIGH);
  digitalWrite(IN3_LEFT, LOW); digitalWrite(IN4_LEFT, HIGH);
}

void setRightForward() {
  digitalWrite(IN1_RIGHT, HIGH); digitalWrite(IN2_RIGHT, LOW);
  digitalWrite(IN3_RIGHT, HIGH); digitalWrite(IN4_RIGHT, LOW);
}

void setRightBackward() {
  digitalWrite(IN1_RIGHT, LOW); digitalWrite(IN2_RIGHT, HIGH);
  digitalWrite(IN3_RIGHT, LOW); digitalWrite(IN4_RIGHT, HIGH);
}

void setAllStop() {
  digitalWrite(IN1_LEFT, LOW);  digitalWrite(IN2_LEFT, LOW);
  digitalWrite(IN3_LEFT, LOW);  digitalWrite(IN4_LEFT, LOW);

  digitalWrite(IN1_RIGHT, LOW); digitalWrite(IN2_RIGHT, LOW);
  digitalWrite(IN3_RIGHT, LOW); digitalWrite(IN4_RIGHT, LOW);
}

void moveForward() {
  setLeftForward();
  setRightForward();
  analogWrite(EN_LEFT, currentForwardSpeed);
  analogWrite(EN_RIGHT, currentForwardSpeed);
}

void turnLeft() {
  setLeftBackward();
  setRightForward();
  analogWrite(EN_LEFT, SPEED_TURN);
  analogWrite(EN_RIGHT, SPEED_TURN);
}

void turnRight() {
  setLeftForward();
  setRightBackward();
  analogWrite(EN_LEFT, SPEED_TURN);
  analogWrite(EN_RIGHT, SPEED_TURN);
}

void stopMotors() {
  setAllStop();
  analogWrite(EN_LEFT, 0);
  analogWrite(EN_RIGHT, 0);
}

// ===================== Telemetry over BLE =====================
// Format: R=<left>,L=<right>,M=<speed>
void sendSensorValues(int leftSensor, int rightSensor, int speedSensor) {
  if (!pSensorCharacteristic) return;

  String data = "R=" + String(leftSensor) +
                ",L=" + String(rightSensor) +
                ",M=" + String(speedSensor);

  pSensorCharacteristic->setValue(data.c_str());
  pSensorCharacteristic->notify();
}

// ===================== BLE callbacks =====================
class CommandCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) override {
    String cmd = pCharacteristic->getValue();
    if (cmd.length() == 0) return;

    cmd.toUpperCase();

    if (cmd == "D") {
      globalState = CAR_RUN;
    } else if (cmd == "P") {
      globalState = CAR_IDLE;
      startFlag = true;
    } else if (cmd == "NORMAL") {
      sportFlag = false;
      SPEED_FAST = SPEED_FAST_NORMAL;
      SPEED_SLOW = SPEED_SLOW_NORMAL;
    } else if (cmd == "SPORT") {
      sportFlag = true;
      SPEED_FAST = SPEED_FAST_SPORT;
      SPEED_SLOW = SPEED_SLOW_SPORT;
    } else if (cmd == "ECHO") {
      sportFlag = false;
      SPEED_FAST = SPEED_FAST_ECHO;
      SPEED_SLOW = SPEED_SLOW_ECHO;
    }
  }
};

// ===================== Driving logic =====================
void runCarLogic() {
  // Read all sensors , then advance once
  int leftSensor  = readLeftSensorMA();
  int rightSensor = readRightSensorMA();
  int speedSensor = readSpeedSensorMA();
  advanceMAIndex();

  bool leftWhite  = (leftSensor  > THRESHOLD_LEFT);
  bool rightWhite = (rightSensor > THRESHOLD_RIGHT);

  // Speed marker logic: choose forward speed by 3rd sensor
  bool speedWhite = (speedSensor > THRESHOLD_SPEED);
  currentForwardSpeed = speedWhite ? SPEED_FAST : SPEED_SLOW;

  sendSensorValues(leftSensor, rightSensor, speedSensor);

  // 1) Straight when line is between side sensors (both are NOT white)
  if (!leftWhite && !rightWhite) {
    lastSide = SIDE_NONE;

    // Startup boost only once, only in NORMAL and ECHO
    if (startFlag && !sportFlag) {
      currentForwardSpeed = 255;
      moveForward();
      delay(50);
      startFlag = false;

      // Restore speed based on speed sensor
      currentForwardSpeed = speedWhite ? SPEED_FAST : SPEED_SLOW;
    }

    moveForward();
    return;
  }

  // 2) Corrections are allowed only when speed marker is NOT white (your original rule)
  if (!speedWhite) {
    if (leftWhite && !rightWhite) {
      lastSide = SIDE_LEFT;
      turnLeft();
    } else if (!leftWhite && rightWhite) {
      lastSide = SIDE_RIGHT;
      turnRight();
    } else {
      // Both white: continue turning in last known direction
      if (lastSide == SIDE_LEFT) turnLeft();
      else if (lastSide == SIDE_RIGHT) turnRight();
      // else do nothing (keeps previous motor state)
    }
  }
}

// ===================== setup / loop =====================
void setup() {
  Serial.begin(115200);

  pinMode(IN1_LEFT, OUTPUT);  pinMode(IN2_LEFT, OUTPUT);
  pinMode(IN3_LEFT, OUTPUT);  pinMode(IN4_LEFT, OUTPUT);

  pinMode(IN1_RIGHT, OUTPUT); pinMode(IN2_RIGHT, OUTPUT);
  pinMode(IN3_RIGHT, OUTPUT); pinMode(IN4_RIGHT, OUTPUT);

  pinMode(EN_LEFT, OUTPUT);
  pinMode(EN_RIGHT, OUTPUT);

  pinMode(CNY70_LEFT_PIN, INPUT);
  pinMode(CNY70_RIGHT_PIN, INPUT);
  pinMode(CNY70_SPEED_PIN, INPUT);

  initMovingAverage();

  BLEDevice::init("The Best DnD");
  pServer = BLEDevice::createServer();
  BLEService *pService = pServer->createService(SERVICE_UUID);

  pCommandCharacteristic = pService->createCharacteristic(
    COMMAND_CHAR_UUID,
    BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_READ
  );
  pCommandCharacteristic->setCallbacks(new CommandCallbacks());
  pCommandCharacteristic->addDescriptor(new BLE2902());

  pSensorCharacteristic = pService->createCharacteristic(
    SENSOR_CHAR_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
  );
  pSensorCharacteristic->addDescriptor(new BLE2902());
  pSensorCharacteristic->setValue("L=0,R=0,M=0"); // removed C=

  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  BLEDevice::startAdvertising();
}

void loop() {
  if (globalState == CAR_RUN) {
    runCarLogic();
  } else {
    stopMotors();

    int leftSensorIdle  = readLeftSensorMA();
    int rightSensorIdle = readRightSensorMA();
    int speedSensorIdle = readSpeedSensorMA();
    advanceMAIndex();

    sendSensorValues(leftSensorIdle, rightSensorIdle, speedSensorIdle);
  }
}