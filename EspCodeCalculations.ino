#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <ESP32Servo.h>
#include <Wire.h>

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

/* MPU6050 default I2C address is 0x68*/
MPU6050 mpu;

#define OUTPUT_READABLE_YAWPITCHROLL

int const INTERRUPT_PIN = 15;  // Define the interruption #0 pin

/*---MPU6050 Control/Status Variables---*/
bool DMPReady = false;   // Set true if DMP init was successful
uint8_t MPUIntStatus;    // Holds actual interrupt status byte from MPU
uint8_t devStatus;       // Return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;     // Expected DMP packet size (default is 42 bytes)
uint8_t FIFOBuffer[64];  // FIFO storage buffer

/*---Orientation/Motion Variables---*/
Quaternion q;         // [w, x, y, z]         Quaternion container
VectorInt16 aa;       // [x, y, z]            Accel sensor measurements
VectorInt16 gy;       // [x, y, z]            Gyro sensor measurements
VectorInt16 aaReal;   // [x, y, z]            Gravity-free accel sensor measurements
VectorInt16 aaWorld;  // [x, y, z]            World-frame accel sensor measurements
VectorFloat gravity;  // [x, y, z]            Gravity vector
float euler[3];       // [psi, theta, phi]    Euler angle container
float ypr[3];         // [yaw, pitch, roll]   Yaw/Pitch/Roll container and gravity vector

/*------Interrupt detection routine------*/
volatile bool MPUInterrupt = false;  // Indicates whether MPU6050 interrupt pin has gone high
void DMPDataReady() {
  MPUInterrupt = true;
}


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

#define MOTORMODE 4
#define MOTOR_PIN_FORWARD 14
#define MOTOR_PIN_BACKWARD 12

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
int windDirection = 0;
int courseValue = 0;
int currentAngle = 0;
int sailControl = 0;
int sailAngles[100] = { 30, 40, 50, 60, 70, 80, 90, 100, 110, 120, 130, 140, 150, 160, 170, 180 };
int trueSailPos = 0;
int speedFactor = 0;
int motorSpeed = 0;
int minimumMotorSpeed = 50;

bool portSide = true;

float trimFactor = 0;
float polarSpeedFactor = 0;

int oldSailServoValue = 0;
int oldRudderServoValue = 0;

float smoothedSailServoValue = 0;
float smoothedRudderServoValue = 0;

void setup(void) {
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000);  // 400kHz I2C clock. Comment on this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif


  Serial.begin(115200);
  while (!Serial)
    ;

  /*Initialize device*/
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  /*Verify connection*/
  Serial.println(F("Testing MPU6050 connection..."));
  if (mpu.testConnection() == false) {
    Serial.println("MPU6050 connection failed");
    while (true)
      ;
  } else {
    Serial.println("MPU6050 connection successful");
  }

  devStatus = mpu.dmpInitialize();

  /* Supply your gyro offsets here, scaled for min sensitivity */
  mpu.setXGyroOffset(0);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(0);
  mpu.setXAccelOffset(0);
  mpu.setYAccelOffset(0);
  mpu.setZAccelOffset(0);

  /* Making sure it worked (returns 0 if so) */
  if (devStatus == 0) {
    mpu.CalibrateAccel(6);  // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateGyro(6);
    Serial.println("These are the Active offsets: ");
    mpu.PrintActiveOffsets();
    Serial.println(F("Enabling DMP..."));  //Turning ON DMP
    mpu.setDMPEnabled(true);

    /*Enable Arduino interrupt detection*/
    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), DMPDataReady, RISING);
    MPUIntStatus = mpu.getIntStatus();

    /* Set the DMP Ready flag so the main loop() function knows it is okay to use it */
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    DMPReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();  //Get expected DMP packet size for later comparison
  } else {
    Serial.print(F("DMP Initialization failed (code "));  //Print the error code
    Serial.print(devStatus);
    Serial.println(F(")"));
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
  }


  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(MOTORMODE, OUTPUT);
  pinMode(MOTOR_PIN_FORWARD, OUTPUT);
  pinMode(MOTOR_PIN_BACKWARD, OUTPUT);

  digitalWrite(MOTOR_PIN_FORWARD, LOW);
  digitalWrite(MOTOR_PIN_BACKWARD, LOW);
  digitalWrite(MOTORMODE, LOW);

  // **************************************************************************
  digitalWrite(LED_BUILTIN, LOW);
  delay(500);
  digitalWrite(LED_BUILTIN, HIGH);

  // Create the BLE Device
  BLEDevice::init("Land Sailor - RED");
  // **************************************************************************

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
    BLECharacteristic::PROPERTY_WRITE_NR);

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
  if (!DMPReady) return;

  currentTime = millis();
  gyroRotation();
  bluetooth();
  upWind();
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
    analogWrite(MOTOR_PIN_FORWARD, 0);
    analogWrite(MOTOR_PIN_BACKWARD, 0);
    digitalWrite(MOTORMODE, LOW);
    //blink();
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
    trueSailPos = map(sailPosition, 0, 180, 90, 0);
  } else {
    trueSailPos = map(sailPosition, 0, 180, 90, 180);
  }

  servoRudder.write(rudderPosition);
  servoSail.write(trueSailPos);



  //}
}

void calculateWind() {
  if (currentAngle > 20) {
    trimFactor = abs((180 - smoothedSailServoValue - currentAngle) / 180);
  }
  polarSpeedFactor = polarDiagram[currentAngle / 10] / polarDiagramMaxValue;
  //Serial.println(polarSpeedFactor);

  motorSpeed = (255 - minimumMotorSpeed) * polarSpeedFactor * trimFactor + minimumMotorSpeed;
}

void gyroRotation() {
  /* Read a packet from FIFO */
  if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) {  // Get the Latest packet
#ifdef OUTPUT_READABLE_YAWPITCHROLL
    /* Display Euler angles in degrees */
    mpu.dmpGetQuaternion(&q, FIFOBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    // Serial.print("ypr\t");
    // Serial.print(ypr[0] * 180 / M_PI);
    // Serial.print("\t");
    // Serial.print(ypr[1] * 180 / M_PI);
    // Serial.print("\t");
    // Serial.println(ypr[2] * 180 / M_PI);
    //currentAngle = abs(ypr[0] * 180 / M_PI - windDirection);
#endif

    currentAngle = ypr[0] * 180 / M_PI - windDirection;

    if (currentAngle > 180) {
      currentAngle -= 360;
      portSide = true;
    }
    if (currentAngle < -180) {
      currentAngle += 360;
      portSide = false;
    }
    currentAngle = abs(currentAngle);
    // if (ypr[0] * 180 / M_PI > 0) {  // This shit broken *******************************************************************************
    //   portSide = true;
    //   currentAngle = abs(ypr[0] * 180 / M_PI - windDirection);
    // } else {
    //   portSide = false;
    //   currentAngle = abs(ypr[0] * 180 / M_PI + windDirection);
    // }
  }
}


void setCourse() {
  if (!digitalRead(BUTTON_PIN)) {
    windDirection = ypr[0] * 180 / M_PI;
    //courseValue = currentAngle;
  }
}

void upWind() {
  if (currentAngle <= 20) {
    digitalWrite(LED_BUILTIN, HIGH);
  } else {
    digitalWrite(LED_BUILTIN, LOW);
  }
}

void controlMotor() {
  analogWrite(MOTOR_PIN_FORWARD, motorSpeed);
  analogWrite(MOTOR_PIN_BACKWARD, 0);
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
      Serial.print(" | TrueYaw = ");
      Serial.print(ypr[0] * 180 / M_PI);
      Serial.print(" | ");
      Serial.print("Wind Direction: ");
      Serial.print(windDirection);
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

void blink() {
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
  delay(100);
}