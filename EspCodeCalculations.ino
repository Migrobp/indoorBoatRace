#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <ESP32_Servo.h>
#include <Wire.h>


// MPU6050
float RateRoll, RatePitch, RateYaw;
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;

float TotalYawAngle = 0.0;

int RateCalibrationNumber;

BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;
BLECharacteristic* pBattery = NULL;
BLECharacteristic* pServo = NULL;
BLE2902* pBLE2902;

bool deviceConnected = false;
bool oldDeviceConnected = false;
uint32_t value = 0;

int rudderPosition = 90;
int sailPosition = 90;

#define SERVICE_UUID "d67d9418-63a3-4cd5-ad66-7a5ff5405bdb"
#define BATTERY_CHARACTERISTIC_UUID "0968cb64-9577-48f9-a03d-2a0f2f0fe42c"
#define SERVO_CHARACTERISTIC_UUID "0746f868-e741-4506-a71f-7851d6ce2f18"


// Define servothings
#define rudderPin 25
#define sailPin 26

Servo servoRudder;
Servo servoSail;

#define MOTORMODE 5
#define MOTOR_PIN_FORWARD 12
#define MOTOR_PIN_BACKWARD 14

#define BUTTON_PIN 19
#define LED_BUILTIN 2
bool blinkState = false;

unsigned long lastTime = 0;  // Variable to store the last loop time
unsigned long lastPrintFunctionRunTime = 0;
unsigned long lastServoRunTime = 0;
unsigned long lastBluetoothRunTime = 0;
unsigned long currentTime = 0;

unsigned int loopTime = 0;

// Millis
unsigned long previousMillis = 0;               // Variable to store the previous timestamp
const unsigned long interval10Seconds = 10000;  // Interval in milliseconds (10 seconds)

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
  };

  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
  }
};

const int polarDiagram[] = { 0, 327, 638, 916, 1145, 1312, 1429, 1528, 1664, 1747, 1801, 1829, 1856, 1832, 1723, 1406, 1205, 1075, 947 };

const int sailTrimDiagram[] = { 0, 0, 111, 166, 222, 277, 333, 388, 444, 500, 555, 611, 666, 722, 777, 833, 888, 944, 1000 };

const float polarDiagramMaxValue = 1856;
const int tableSize = sizeof(polarDiagram) / sizeof(polarDiagram[0]);


// COURSE Things //
int courseValue = 0;
int currentAngle = 0;
int sailControl = 0;
int sailAngles[100] = { 30, 40, 50, 60, 70, 80, 90, 100, 110, 120, 130, 140, 150, 160, 170, 180 };
int trueSailPos = 0;
int speedFactor = 0;
int motorPin = 3;
int motorSpeed = 0;

bool portSide = true;

float trimFactor = 0;
float polarSpeedFactor = 0;

int oldSailServoValue = 0;
int oldRudderServoValue = 0;

float smoothedSailServoValue = 0;
float smoothedRudderServoValue = 0;

void setup(void) {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(MOTORMODE, OUTPUT);
  pinMode(MOTOR_PIN_FORWARD, OUTPUT);
  pinMode(MOTOR_PIN_BACKWARD, OUTPUT);

  digitalWrite(MOTOR_PIN_FORWARD, LOW);
  digitalWrite(MOTOR_PIN_BACKWARD, LOW);
  digitalWrite(MOTORMODE, LOW);


  // MPU6050
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  Wire.setClock(400000);
  Wire.begin();
  delay(250);
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  for (RateCalibrationNumber = 0; RateCalibrationNumber < 2000; RateCalibrationNumber++) {
    gyro_signals();
    RateCalibrationRoll += RateRoll;
    RateCalibrationPitch += RatePitch;
    RateCalibrationYaw += RateYaw;
    delay(1);
  }
  RateCalibrationRoll /= 2000;
  RateCalibrationPitch /= 2000;
  RateCalibrationYaw /= 2000;

  // Create the BLE Device
  BLEDevice::init("Land Sailor - RED");

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService* pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pBattery = pService->createCharacteristic(
    BATTERY_CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY);

  pServo = pService->createCharacteristic(
    SERVO_CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_WRITE);

  // https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.descriptor.gatt.client_characteristic_configuration.xml
  // Create a BLE Descriptor
  BLEDescriptor* pBatteryDescr;
  pBatteryDescr = new BLEDescriptor((uint16_t)0x2901);
  pBatteryDescr->setValue("Battery Indicator");

  BLEDescriptor* pServoDescr;
  pServoDescr = new BLEDescriptor((uint16_t)0x2901);
  pServoDescr->setValue("Combined Servos");

  pBLE2902 = new BLE2902();
  //pBLE2902->setNotifications(true);
  pBattery->addDescriptor(pBatteryDescr);
  pBattery->addDescriptor(pBLE2902);

  // Added at combined servo characteristic to minimize amount packages.
  pServo->addDescriptor(pServoDescr);
  pServo->addDescriptor(new BLE2902());

  // Start the service
  pService->start();

  // Start advertising
  BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);  // set value to 0x00 to not advertise this parameter
  BLEDevice::startAdvertising();
  Serial.println("Waiting a client connection to notify...");

  servoRudder.attach(rudderPin, 500, 2500);
  servoSail.attach(sailPin, 500, 2500);
  servoRudder.write(rudderPosition);
  servoSail.write(sailPosition);
  //servoRudder.detach();
  //servoSail.detach();
  delay(100);
}

void loop() {
  currentTime = millis();
  gyroRotation();
  bluetooth();
  blink();
  setCourse();
  calculateWind();
  if (deviceConnected) {
    //servoRudder.attach(rudderPin, 500, 2500);
    //servoSail.attach(sailPin, 500, 2500);
    digitalWrite(MOTORMODE, HIGH);
    controlServos();
    controlMotor();
  } else {
    //servoRudder.detach();
    //servoSail.detach();
    digitalWrite(MOTOR_PIN_FORWARD, LOW);
    digitalWrite(MOTOR_PIN_BACKWARD, LOW);
    digitalWrite(MOTORMODE, LOW);
  }

  printFunction();
}

void bluetooth() {
  // notify changed value
  if (deviceConnected) {
    // Calculate the elapsed time since the last function run
    //unsigned long currentTime = millis();
    //unsigned long elapsedTime = currentTime - lastBluetoothRunTime;

    // Check if enough time has elapsed since the last run
    //if (elapsedTime >= 20) {  // Run every 20 milliseconds
    lastBluetoothRunTime = currentTime;
    sendBatteryVoltage();
    uint8_t* servoValue = pServo->getData();

    rudderPosition = servoValue[0];
    sailPosition = servoValue[1];

    //servoRudder.write(rudderPosition);
    //servoSail.write(sailPosition);
    //}

    delay(20);  // bluetooth stack will go into congestion, if too many packets are sent, in 6 hours test i was able to go as low as 3ms
  }
  // disconnecting
  if (!deviceConnected && oldDeviceConnected) {
    delay(500);  // give the bluetooth stack the chance to get things ready

    pServer->startAdvertising();  // restart advertising
    Serial.println("start advertising");
    oldDeviceConnected = deviceConnected;
  }
  // connecting
  if (deviceConnected && !oldDeviceConnected) {
    // do stuff here on connecting
    oldDeviceConnected = deviceConnected;
  }
}

void sendBatteryVoltage() {
  if (currentTime - previousMillis >= interval10Seconds) {
    previousMillis = currentTime;

    float batteryVoltage = 4.2;
    int sendBatteryVoltage = map(batteryVoltage * 10, 33, 42, 0, 100);
    //Serial.println(sendBatteryVoltage);
    pBattery->setValue(sendBatteryVoltage);
    pBattery->notify();
    batteryVoltage += 0.1;
  }
}

void controlServos() {
  // Calculate the elapsed time since the last function run
  //unsigned long elapsedTime = currentTime - lastServoRunTime;
  // Check if enough time has elapsed since the last run
  //if (elapsedTime >= 15 - loopTime) {  // Run every 15 milliseconds
  //lastServoRunTime = currentTime;
  //servoSail.write(sailPosition);
  //Serial.println(sailPosition);


  //servoRudder.write(rudderPosition);
  //servoSail.write(trueSailPos);


  //if (trueSailPos != oldSailServoValue) {
  smoothedSailServoValue = (sailPosition * 0.05) + (oldSailServoValue * 0.95);
  oldSailServoValue = smoothedSailServoValue;
  //}

  //if (rudderPosition != oldRudderServoValue) {
  smoothedRudderServoValue = (rudderPosition * 0.05) + (oldRudderServoValue * 0.95);
  oldRudderServoValue = smoothedRudderServoValue;
  //}

  if (portSide) {
    trueSailPos = map(smoothedSailServoValue, 0, 180, 90, 0);
  } else {
    trueSailPos = map(smoothedSailServoValue, 0, 180, 90, 180);
  }

  servoRudder.write(smoothedRudderServoValue);
  servoSail.write(trueSailPos);



  //}
}

void calculateWind() {
  if (currentAngle > 20) {
    trimFactor = abs((180 - smoothedSailServoValue - currentAngle) / 180);
  }
  polarSpeedFactor = polarDiagram[currentAngle / 10] / polarDiagramMaxValue;
  //Serial.println(polarSpeedFactor);
  motorSpeed = 255 * polarSpeedFactor * trimFactor;
  if (motorSpeed == 0) {
    motorSpeed = 50;
  }
}

void gyroRotation() {
  gyro_signals();
  float dt = (currentTime - lastTime) / 1000.0;  // Convert milliseconds to seconds

  // Check if enough time has elapsed since the last update
  if (dt >= 0.03) {  // Update every 30 milliseconds (adjust as needed)
    lastTime = currentTime;

    // Subtract the calibration value
    RateYaw -= RateCalibrationYaw;

    // Integrate the yaw rate to get the total angle
    TotalYawAngle += RateYaw * dt;

    // Wrap the angle around to [0, 360) range
    //TotalYawAngle = fmod(TotalYawAngle, 360.0);

    // Handle the case when the angle becomes negative
    if (TotalYawAngle < -180.0) {
      TotalYawAngle += 360.0;
    }

    if (TotalYawAngle > 180.0) {
      TotalYawAngle += -360.0;
    }
    if (TotalYawAngle > 0) {
      portSide = true;
    } else {
      portSide = false;
    }

    currentAngle = abs(TotalYawAngle);
  }
}


void setCourse() {
  if (!digitalRead(BUTTON_PIN)) {
    TotalYawAngle = 0;
    //courseValue = currentAngle;
  }
}

void blink() {
  if (currentAngle <= 20) {
    digitalWrite(LED_BUILTIN, HIGH);
  } else {
    digitalWrite(LED_BUILTIN, LOW);
  }
}

void controlMotor() {
  analogWrite(MOTOR_PIN_FORWARD, motorSpeed);
  digitalWrite(MOTOR_PIN_BACKWARD, LOW);
}

void gyro_signals(void) {
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  int16_t GyroX = Wire.read() << 8 | Wire.read();
  int16_t GyroY = Wire.read() << 8 | Wire.read();
  int16_t GyroZ = Wire.read() << 8 | Wire.read();
  RateRoll = (float)GyroX / 65.5;
  RatePitch = (float)GyroY / 65.5;
  RateYaw = (float)GyroZ / 65.5;
}

void printFunction() {
  if (Serial) {
    unsigned long elapsedTime = currentTime - lastPrintFunctionRunTime;

    // Check if enough time has elapsed since the last run
    if (elapsedTime >= 200) {  // Run every 200 milliseconds
      lastPrintFunctionRunTime = currentTime;
      loopTime = millis() - currentTime;

      Serial.println("");
      Serial.print("CurrentAngle: ");
      Serial.print(currentAngle);
      Serial.print(" | Yaw Angle [°]= ");
      Serial.print(TotalYawAngle);
      Serial.print(" | ");
      Serial.print("Course Value: ");
      Serial.print(courseValue);
      Serial.print(" | ");
      Serial.print("RudderPOS: ");
      Serial.print(smoothedRudderServoValue);
      Serial.print(" | ");
      Serial.print("SailPOS: ");
      Serial.print(smoothedSailServoValue);
      Serial.print(" | ");
      Serial.print("SF: ");
      Serial.print(speedFactor);
      Serial.print(" | ");
      Serial.print("MotorSpeed: ");
      Serial.print(motorSpeed);

      Serial.print(" | ");
      Serial.print("trimFactor: ");
      Serial.print(abs(trimFactor));
      Serial.print(" | ");
      Serial.print("polarFactor: ");
      Serial.print(polarSpeedFactor);
      Serial.print(" | Loop time: ");
      Serial.print(loopTime);
      Serial.print(" | PortSide: ");
      Serial.print(portSide);
    }
  }
}
