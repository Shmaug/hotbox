#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
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
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
VectorFloat rotation;      // [x, y, z]

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}
/////////////////////////////////
///        end mpu vars      ///
////////////////////////////////
#define m1 9
#define m2 10
#define minThro 136 // full reverse
#define midThro 191 // neutral
#define maxThro 243 // full forward
#define revDif 55 // mid - min
#define forDif 52 // max - mid
bool useMPU=false;

void getMPUData(){
  String output="";
  
  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  
  // get current FIFO count
  fifoCount = mpu.getFIFOCount();
  
  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    //Serial.println(F("FIFO overflow!"));
  
  // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    
    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetEuler(euler, &q);
    
    rotation.x=euler[0] * 180/M_PI;
    rotation.y=euler[1] * 180/M_PI;
    rotation.z=euler[2] * 180/M_PI;
    
    rotation.x-=107;
    rotation.z-=110;
    
    output+="rotxyz\t";
    output+=rotation.x;
    output+="\t";
    output+=rotation.y;
    output+="\t";
    output+=rotation.z;
  
    // display initial world-frame acceleration, adjusted to remove gravity
    // and rotated based on known orientation from quaternion
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
    
    aaWorld.z-=390;
    
    output+="\t\taccxyz\t";
    output+=aaWorld.x;
    output+="\t";
    output+=aaWorld.y;
    output+="\t";
    output+=aaWorld.z;
    
    Serial.println(output);
    
    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
  }
}

void initMPU(){
  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
  
  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  
  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  
  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();
  
  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);
  
    // enable Arduino interrupt detection
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
  
    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;
  
    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
}

void setup() {
  Serial.begin(9600);
  
  if (useMPU){
    initMPU();
  }
  pinMode(m1,OUTPUT);
  pinMode(m2,OUTPUT);
  
  pinMode(LED_PIN, OUTPUT);
  
  // arm
  digitalWrite(13,HIGH);
  analogWrite(m1,midThro);
  analogWrite(m2,midThro);
  delay(6000);
  digitalWrite(13,LOW);
}

void b(int thr, int m){ // go backward at (thr)% throttle
  float f=(float)thr/100;
  analogWrite(m,midThro-(revDif*f));
}
void f(int thr, int m){ // go forward at (thr)% throttle
  float f=(float)thr/100;
  analogWrite(m,midThro-(forDif*f));
}
void n(int m){
  analogWrite(m,midThro);
}

int last=millis();
int cTimer;
void doAI(){
  int cur=millis();
  int delta=cur-last;
  
  cTimer-=delta;
  if (cTimer <= 0){
    n(m1);
    n(m2);
    
    cTimer=random(1000,3000);
    delay(random(1000,2000));
    
    if (random(-10,10)>0){
      f(m1,random(0,100));
    }else{
      b(m1,random(0,100));
    }
    if (random(-10,10)>0){
      f(m2,random(0,100));
    }else{
      b(m2,random(0,100));
    }
  }
  
  last=cur;
}

void loop() {
  if (useMPU){
    // if programming failed, don't try to do anything
    if (!dmpReady) return;
  
    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
      doAI();
    }
    getMPUData();
  }else{
    doAI();
  }
}
