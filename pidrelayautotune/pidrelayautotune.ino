#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
#include<Servo.h>
#include<PID_v1.h>
#define INTERRUPT_PIN 2 

int limit = 30;
int relay = 30;

unsigned long aktualnyCzas;

double Kp = 1.5;
double Ki = 0;
double Kd = 0;

double Setpointx = 0;
double Setpointy = 0;
double Setpointz = 0;
double Outputx;
double Outputy;
double Outputz;


MPU6050 mpu;

Servo motor1;
Servo motor2;
Servo motor3;
Servo motor4;
int pinmotor1 = 9;
int pinmotor2 = 10;
//int ponmotor3 = ;
//int pinmotor4 = ;

/////
int power = 0;
int powermotor1 = 0;
int powermotor2 = 0;
int powermotor3 = 0;
int powermotor4 = 0;




/////
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float degreeposition[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
double yaw = 0;
double pitch = 0;
double roll = 0;

//PID PIDZ(&yaw, &Outputz, &Setpointz, Kp, Ki, Kd, DIRECT);
//PID PIDX(&pitch, &Outputx, &Setpointx, Kp, Ki, Kd, DIRECT);
PID PIDY(&roll, &Outputy, &Setpointy, Kp, Ki, Kd, DIRECT);

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}
void setup() {
 #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
    Serial.begin(115200);
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();
    mpu.setXGyroOffset(52);
    mpu.setYGyroOffset(-20);
    mpu.setZGyroOffset(-8);
    mpu.setXAccelOffset(937);
    mpu.setYAccelOffset(-895);
    mpu.setZAccelOffset(925);
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
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
    motor1.attach(pinmotor1);
    motor2.attach(pinmotor2);
    //motor3.attach(pinmotor3);
    //motor4.attach(pinmotor4);

    PIDY.SetMode(AUTOMATIC);
    PIDY.SetOutputLimits(-limit,limit);
} 
void loop() {
  if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
    //sthxD
    }
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;
        
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(degreeposition, &q, &gravity);
        
        yaw = degreeposition[0]*180/M_PI;
        pitch = degreeposition[1]*180/M_PI;
        roll = degreeposition[2]*180/M_PI;

        if(roll<0)
        {
          Outputy=relay;
        }
        if(roll>=0)
        {
          Outputy=-relay;
        }
        /*Serial.print("ypr\t");
        Serial.print(yaw);
        Serial.print("\t");
        Serial.print(pitch);
        Serial.print("\t");
        Serial.println(roll);*/
        if(Serial.available())
        {
          power = Serial.parseInt();
        }
        
        powermotor1 = power;
        powermotor2 = power;
        
        //PIDY.Compute();
        //Serial.println(Outputy);
        Serial.print(roll);
        Serial.print("\t");
        Serial.println(millis());
        powermotor1 -= Outputy/2;
        powermotor2 += Outputy/2;
        //Serial.println(powermotor1);
        motor1.writeMicroseconds(powermotor1);
        motor2.writeMicroseconds(powermotor2);
    }
}
  // put your main code here, to run repeatedly:

