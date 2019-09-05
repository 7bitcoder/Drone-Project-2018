#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
#include<Servo.h>
#include<PID_v1.h>

#define INTERRUPT_PIN 2 //pin do sprawdzania popranosci komunikacji
#define ERROR_PIN 13 //pin do sugnalizowania krytycznego błędu kodu

// zrobic next symulacja fifo owerflow dać go to początek
//przemyslec limity pid
short int limit = 1024;
short int ab= 0;
int data;
float data2;
int option;
// nastawy pid
double Kp = 1;
double Ki = 0;
double Kd = 0;
// wejscie pid i wyjscie
double Setpointx = 0;
//double Setpointy = 0;
//double Setpointz = 0;
double Outputx = 0;
//double Outputy = 0;
//double Outputz = 0;


MPU6050 mpu;

Servo motor1;
Servo motor2;
//Servo motor3;
//Servo motor4;
//piny do silniczkow
short int pinmotor1 = 9;
short int pinmotor2 = 10;
//int ponmotor3 = ;
//int pinmotor4 = ;

//moc silniczkow thrust
int power = 0;
int powermotor1 = 0;
int powermotor2 = 0;
//int powermotor3 = 0;
//int powermotor4 = 0;



char DataF[64];
/////
bool dmpReady = false;  // potwierdzenie gotowosci dmp
uint8_t mpuIntStatus = 0;   // bajt błędów mpu 3A rejestr
uint8_t devStatus = 0;      // inicjalizacja dmp (0 = success, !0 = error)
uint16_t packetSize = 0;    // oczekiwana wartosc paczki danych (domyslnie 42 bytes)
uint16_t fifoCount = 0;     // licznik bajtów w fifo
uint8_t fifoBuffer[64]; // fifo danych z czujnika

Quaternion q;           // [w, x, y, z]         dane z czujnika
VectorFloat gravity;    // [x, y, z]            wektor grawitacji
float degreeposition[3];           // [yaw, pitch, roll]   yaw/pitch/roll

double yaw = 0;
double pitch = 0;
double roll = 0;

// pid pitch rolli yaw
//PID PIDZ(&yaw, &Outputz, &Setpointz, Kp, Ki, Kd, DIRECT);
PID PIDX(&pitch, &Outputx, &Setpointx, Kp, Ki, Kd, DIRECT);
//PID PIDY(&roll, &Outputy, &Setpointy, Kp, Ki, Kd, DIRECT);

volatile bool mpuInterrupt = false;

// funkcja sprawdzająca komunkację z czujnikiem wywoływana pin interupt
void dmpDataReady()
{
  mpuInterrupt = true;
}

void setup()
{
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C czas
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
  Serial.begin(115200);
  while (!Serial) {
    ;
  }
  // Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);
  pinMode(ERROR_PIN, OUTPUT);
  digitalWrite(ERROR_PIN, LOW);
  // Serial.println(F("Testing device connections..."));
  // Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  //Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  if (devStatus == 0)
  {
    // turn on the DMP, now that it's ready
    // Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    //  Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    //Serial.println(digitalRead(INTERRUPT_PIN));
    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    // Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
    // Serial.println(F("packet size"));
    //Serial.println(packetSize);

    mpu.setXGyroOffset(52);
    mpu.setYGyroOffset(-20);
    mpu.setZGyroOffset(-8);
    mpu.setXAccelOffset(937);
    mpu.setYAccelOffset(-895);
    mpu.setZAccelOffset(925);
  }
  else
  {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    digitalWrite(ERROR_PIN, HIGH);
    //Serial.print(F("DMP Initialization failed (code "));
    //Serial.print(devStatus);
    //Serial.println(F(")"));
  }
  motor1.attach(pinmotor1);
  motor2.attach(pinmotor2);
  //motor3.attach(pinmotor3);
  //motor4.attach(pinmotor4);

  PIDX.SetMode(AUTOMATIC);
  PIDX.SetOutputLimits(-limit, limit);
}

void loop()
{
  digitalWrite(ERROR_PIN, LOW);
  if (!dmpReady) return;
  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize)
  {
    digitalWrite(ERROR_PIN, HIGH);
    // Serial.println(F("jestem w while interupt tzn komunikacja nie zostala rozpoczeta mpu6050"));
  }

  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  if ((mpuIntStatus & 0x10) || fifoCount == 1024)
  {
    // reset aby dalej działać
    mpu.resetFIFO();
    //Serial.println(F("FIFO overflow!"));
    digitalWrite(ERROR_PIN, HIGH);
    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  }
  else if (mpuIntStatus & 0x02)
  {
    // czekamy na całą paczkę danych
    //Serial.println(fifoCount);
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // czytamy paczke
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(degreeposition, &q, &gravity);

    yaw = degreeposition[0] * 180 / M_PI;
    pitch = degreeposition[1] * 180 / M_PI;
    roll = degreeposition[2] * 180 / M_PI;

    //Serial.print("ypr\t");
    if(ab == 3){
    //Serial.print(yaw);
    //Serial.print("\t");
    Serial.print(pitch);
    //Serial.print("\t");
    //Serial.println(roll);
    Serial.print("\t");
    Serial.print(Kp);
    Serial.print("\t");
    Serial.print(Ki);
    Serial.print("\t");
    Serial.print(Kd);
    Serial.print("\t");
    Serial.print(power);
    Serial.print("\t");
    Serial.print(powermotor1);
    Serial.print("\t");
    Serial.println(powermotor2);
    ab = 0;
    }
    ab++;
    if (Serial.available() > 0)
    {
      int len = Serial.readBytesUntil('\n',DataF,100);
      Serial.read();
      DataF[len] = '\n';
      data = atoi(DataF);
      if(data == 9)
      {   
        while(Serial.available() == 0)
           ;
          int len = Serial.readBytesUntil('\n',DataF,100);
          Serial.read();
          DataF[len] = '\n';
          data = atoi(DataF);
           while(Serial.available() == 0)
            ;
          len = Serial.readBytesUntil('\n',DataF,100);
          Serial.read();
          DataF[len] = '\n';
          data2 = atof(DataF);
          if(data == 1)
          Kp = data2;
          if(data == 2)
          Ki = data2;
          if(data == 3)
          Kd = data2;
          PIDX.SetTunings(Kp,Ki,Kd);
      }
      else
      //Serial.read();
      power = data;
    }

   
    powermotor1 = power;
    powermotor2 = power;

    PIDX.Compute();
    // Serial.println(Outputx);
    //Serial.println(roll);
    powermotor1 += round(Outputx / 2); //round okolo
    powermotor2 -= round(Outputx / 2);

  //      Serial.print(powermotor1);
    //  Serial.print("\t");
    //Serial.println(powermotor2);
    motor1.writeMicroseconds(powermotor1);
    motor2.writeMicroseconds(powermotor2);
  }
}
// put your main code here, to run repeatedly:
