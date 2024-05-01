/*
Climbing Patient-Aid Robot (CPAR)
This project implemented by the students listed below:
1. Ayat Ahmed Khalaf
2. Entisar Jumaa Naemaa
3. Sara Mohammed Foud
Under supervision of
Dr. Raaed Sadoon
in The University of Technology, Control and systems engineering department, Mechatronics Branch – Spring 2024
Graduation Project Submitted to the Control and Systems Engineering Department in Partial fulfillment of B. Sc. Degree in Mechatronics Engineering.
 */
#include <PID_v1.h>
/*servo*/
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
#include <Servo.h>
MPU6050 mpu;

// Define the 3 servo motors
Servo servo0;
float correct;
int j = 0;

#define OUTPUT_READABLE_YAWPITCHROLL

#define INTERRUPT_PIN 19  // use pin 2 on Arduino Uno & most boards

bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };



// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}
/*servo end*/
double kP = 24;
double kI = 200;
double kD = 1.2;

double setpoint, input, output;   // PID variables
PID pid(&input, &output, &setpoint, kP, kI, kD, DIRECT); // PID setup
#include <nRF24L01.h>
#include <SPI.h>
#include <RF24.h>

RF24 radio(7, 8);    // CE, CSN
const byte address[6] = "00001";
float angleV = 0, turnV = 0; // values from remote
float controlValues[2]; // array to receive both data values
float pitch;
float trimAngle;
//Motor Control

// Motor A connections
#define motorLeft_IN1 2
#define motorLeft_IN2 3
#define motorRight_IN1 4
#define motorRight_IN2 5
int motorSpeed, steeringValue, leftMotorSpeed, rightMotorSpeed = 0;

void setup()
{
    // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
  Serial.begin(9600);

/*servo*/
   // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  Serial.begin(38400);
  while (!Serial); // wait for Leonardo enumeration, others continue immediately

  // initialize device
  //Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);
  devStatus = mpu.dmpInitialize();
  // supply your own gyro offsets here, scaled for min sensitivity
 mpu.setXGyroOffset(24);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(59);
  mpu.setZAccelOffset(906); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    // Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    //Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    // Serial.print(F("DMP Initialization failed (code "));
    //Serial.print(devStatus);
    //Serial.println(F(")"));
  }

  // Define the pins to which the 3 servo motors are connected
  servo0.attach(10);
/*servo end*/

// Left track
  digitalWrite(motorLeft_IN1, LOW);   // PWM value
  digitalWrite(motorLeft_IN2, LOW); // Forward

  // Right track
  digitalWrite(motorRight_IN1, LOW);   // PWM value
  digitalWrite(motorRight_IN2, LOW); // Forward


        radio.begin();
    radio.openReadingPipe(0, address);
    radio.setPALevel(RF24_PA_MIN);
    radio.startListening();
}

void loop()
{
  
 // only read remote data if remote is available, otherwise balance normally
  if (radio.available()) {
    radio.read(&controlValues, sizeof(controlValues)); // read values of array

    angleV = controlValues[0]; // assign array values to control variables
    turnV = controlValues[1];

  }  
  
  pitch = (ypr[1] * 180/M_PI); // adjust to degrees
    if (abs(turnV) < 15) { // turnV threshold
    turnV = 0;
  }

  if (abs(angleV) < .17) { // angleV threshold
    angleV = 0;
  }


  trimAngle = (-1/100) + 5 + angleV; // adjust to degrees
    // PID vars
  setpoint = trimAngle + angleV; 
  input = pitch;
    pid.Compute();

  // set motor speed with adjusted turn values
  //To read Joysticks Input
  float x_data = angleV;
  float y_data = turnV;



// convert the incoming date into suitable PWM value

  steeringValue = y_data;
  motorSpeed = abs(x_data);

  leftMotorSpeed = 70 + motorSpeed - steeringValue; // 70 + (0-185) + (0 - 185 ) =  70 - 255 so this range from 70 to 255 is used as PWM value
  rightMotorSpeed = 70 + motorSpeed + steeringValue;

  leftMotorSpeed = constrain(leftMotorSpeed, 0, 255); // constrain the PWM value from 0 to 255
  rightMotorSpeed = constrain(rightMotorSpeed, 0, 255);

  // if PWM is lower than 72, set PWM value to 0

  if (leftMotorSpeed < 72) {
    leftMotorSpeed = 0;
  }
  if (rightMotorSpeed < 72) {
    rightMotorSpeed = 0;
  }
  // if right joystick goes up > move Backward
  if (x_data >= 80) {
    analogWrite(motorLeft_IN1, leftMotorSpeed);  // PWM input
    digitalWrite(motorLeft_IN2, LOW); // Direction - Backward

    analogWrite(motorRight_IN1, rightMotorSpeed);  // PWM input
    digitalWrite(motorRight_IN2, LOW); // Direction - Backward
  }

  // if right joystick goes down > move Forward
  else if (x_data < 70) {
    digitalWrite(motorLeft_IN1, LOW);  // Direction - Forward
    analogWrite(motorLeft_IN2, leftMotorSpeed); // PWM input

    digitalWrite(motorRight_IN1, LOW);  // Direction - Forward
    analogWrite(motorRight_IN2, rightMotorSpeed); // PWM input
  }
  // if right joystick is in the middle, don't move
else if (x_data = 0) {
      digitalWrite(motorLeft_IN1, LOW);
      digitalWrite(motorLeft_IN2, LOW);

      digitalWrite(motorRight_IN1, LOW);
      digitalWrite(motorRight_IN2, LOW);
    }
    // if right joystick move just left or right, without going up or down, move the tank left or right (only 1 motor move)
    else {
      analogWrite(motorLeft_IN1, leftMotorSpeed);  // PWM input
      digitalWrite(motorLeft_IN2, LOW); // Direction - Forward

      analogWrite(motorRight_IN1, rightMotorSpeed);  // PWM input
      digitalWrite(motorRight_IN2, LOW); // Direction - Forward
    }
    pid.SetMode(AUTOMATIC);
    pid.SetOutputLimits(-255,255);
    pid.SetSampleTime(10);

/*servo*/
     // if programming failed, don't try to do anything
  if (!dmpReady) return;

  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize) {
    if (mpuInterrupt && fifoCount < packetSize) {
      // try to get out of the infinite loop
      fifoCount = mpu.getFIFOCount();
    }
  }

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    fifoCount = mpu.getFIFOCount();
    Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

    // Get Yaw, Pitch and Roll values
#ifdef OUTPUT_READABLE_YAWPITCHROLL
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    // Yaw, Pitch, Roll values - Radians to degrees
    ypr[0] = ypr[0] * 180 / M_PI;
    ypr[1] = ypr[1] * 180 / M_PI;
    ypr[2] = ypr[2] * 180 / M_PI;
    
    // Skip 300 readings (self-calibration process)
    if (j <= 300) {
      correct = ypr[0]; // Yaw starts at random value, so we capture last value after 300 readings
      j++;
    }
    // After 300 readings
    else {
      ypr[0] = ypr[0] - correct; // Set the Yaw to 0 deg - subtract  the last random Yaw value from the currrent value to make the Yaw 0 degrees
      // Map the values of the MPU6050 sensor from -90 to 90 to values suatable for the servo control from 0 to 180
      int servo0Value = map(ypr[1], -90, 90, 0, 180);
      
      // Control the servos according to the MPU6050 orientation
      servo0.write(servo0Value);

    }
#endif
  }
/*servo*/    
}

 
