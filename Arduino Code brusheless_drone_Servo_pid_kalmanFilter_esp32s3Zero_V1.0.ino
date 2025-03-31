//Drone V1.0 Project created by Mhamdi Ghaith 
//facebook page: HardSoftRoboticsMh: https://www.facebook.com/profile.php?id=61574058525266
//Github: ghaithmhamd: https://github.com/ghaithmhamd
//All thanks to https://github.com/CarbonAeronautics https://www.youtube.com/@carbonaeronautics
#include <Wire.h>
#include <SPI.h>           
#include <nRF24L01.h>
#include <RF24.h>
#include <ESP32Servo.h>
Servo esc1;
Servo esc2;
Servo esc3;
Servo esc4;

SPIClass mySPI(HSPI);
RF24 radio(9, 10);   // nRF24L01 (CE, CSN)
const byte address[6] = "00001";

int ESCfreq=500;
float t=0.004;

uint32_t LoopTimer,lastReceiveTime;

struct Data_Package {
  volatile byte j1PotX;
  volatile byte j1PotY;
  byte j1Button;
  volatile byte j2PotX;
  volatile byte j2PotY;
  byte j2Button;
  byte pot1;
  volatile byte pot2;
  byte tSwitch1;
  byte tSwitch2;
  byte button1;
  byte button2;
  byte button3;
  byte button4;
};
Data_Package data;

volatile float RateRoll, RatePitch, RateYaw;
volatile float AngleRoll, AnglePitch;
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
int RateCalibrationNumber;

volatile float AccX, AccY, AccZ;

volatile float DesiredRateRoll, DesiredRatePitch, DesiredRateYaw;
volatile float ErrorRateRoll, ErrorRatePitch, ErrorRateYaw;
volatile float PrevErrorRateRoll, PrevErrorRatePitch, PrevErrorRateYaw;
volatile float PrevItermRateRoll, PrevItermRatePitch, PrevItermRateYaw;

volatile float DesiredAngleRoll, DesiredAnglePitch;
volatile float ErrorAngleRoll, ErrorAnglePitch;
volatile float PrevErrorAngleRoll, PrevErrorAnglePitch;
volatile float PrevItermAngleRoll, PrevItermAnglePitch;

volatile float InputRoll, InputThrottle, InputPitch, InputYaw;
volatile float PIDReturn[]={0, 0, 0};

float PRateRoll=0.625 ; float PRatePitch=PRateRoll; float PRateYaw=4;   
float IRateRoll=2.1 ; float IRatePitch=IRateRoll; float IRateYaw=3;  
float DRateRoll=0.0088 ; float DRatePitch=DRateRoll; float DRateYaw=0;  

float PAngleRoll=2; float PAnglePitch=PAngleRoll;
float IAngleRoll=0.5; float IAnglePitch=IAngleRoll;
float DAngleRoll=0.007; float DAnglePitch=DAngleRoll;

volatile float MotorInput1, MotorInput2, MotorInput3, MotorInput4;

volatile float KalmanAngleRoll=0, KalmanUncertaintyAngleRoll=2*2;
volatile float KalmanAnglePitch=0, KalmanUncertaintyAnglePitch=2*2;
volatile float Kalman1DOutput[]={0,0};

void resetData(){
  data.j1PotX = 127;
  data.j1PotY = 127;
  data.j2PotX = 127;
  data.j2PotY = 0;   //
  data.j1Button = 1;
  data.j2Button = 1;
  data.pot1 = 0;
  data.pot2 = 0;
  data.tSwitch1 = 1;
  data.tSwitch2 = 1;
  data.button1 = 1;
  data.button2 = 1;
  data.button3 = 1;
  data.button4 = 1;
}

void pid_equation(volatile float Error, float P , float I, float D, volatile float PrevError, volatile float PrevIterm) {
  volatile float Pterm=P*Error;
  volatile float Iterm=PrevIterm+I*(Error+PrevError)* t/2;
  if (Iterm > 400) Iterm=400;
  else if (Iterm <-400) Iterm=-400;
  volatile float Dterm=D*(Error-PrevError)/t;
  volatile float PIDOutput= Pterm+Iterm+Dterm;
  if (PIDOutput>400) PIDOutput=400;
  else if (PIDOutput <-400) PIDOutput=-400;
  PIDReturn[0]=PIDOutput;
  PIDReturn[1]=Error;
  PIDReturn[2]=Iterm;
}

void reset_pid(void) {
  PrevErrorRateRoll=0; PrevErrorRatePitch=0; PrevErrorRateYaw=0;
  PrevItermRateRoll=0; PrevItermRatePitch=0; PrevItermRateYaw=0;
  PrevErrorAngleRoll=0; PrevErrorAnglePitch=0;    
  PrevItermAngleRoll=0; PrevItermAnglePitch=0;
}

void kalman_1d(volatile float KalmanState, volatile float KalmanUncertainty, volatile float KalmanInput, volatile float KalmanMeasurement) {
  KalmanState=KalmanState+ t*KalmanInput;
  KalmanUncertainty=KalmanUncertainty + t*t*4*4; //here 4 is the vairnece of IMU i.e 4 deg/s
  volatile float KalmanGain=KalmanUncertainty * 1/(1*KalmanUncertainty + 3 * 3); //std deviation of error is 3 deg
  KalmanState=KalmanState+KalmanGain * (
  KalmanMeasurement-KalmanState);
  KalmanUncertainty=(1-KalmanGain) * KalmanUncertainty;
  Kalman1DOutput[0]=KalmanState; 
  Kalman1DOutput[1]=KalmanUncertainty;
}
/*
float complementaryAngleRoll = 0.0f;  //KalmanAngleRoll<--complementaryAngleRoll
float complementaryAnglePitch = 0.0f; //KalmanAnglePitch<--complementaryAnglePitch
*/

void gyro_signals(void) {
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(); 
  Wire.requestFrom(0x68,6);
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();
  Wire.beginTransmission(0x68);
  Wire.write(0x1B); 
  Wire.write(0x8);
  Wire.endTransmission();                                                   
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68,6);
  int16_t GyroX=Wire.read()<<8 | Wire.read();
  int16_t GyroY=Wire.read()<<8 | Wire.read();
  int16_t GyroZ=Wire.read()<<8 | Wire.read();
  RateRoll=(float)GyroX/65.5;
  RatePitch=(float)GyroY/65.5;
  RateYaw=(float)GyroZ/65.5;
  AccX=(float)AccXLSB/4096-0.05; //-0.05 5ater lguitha 1.05 //ken edrone maawja w tmchy maa y+ nhot - snn +
  AccY=(float)AccYLSB/4096+0.02; //+0.02 5ater lguitha 0.98 //ken edrone maawja w tmchy maa y+ nhot - snn +
  AccZ=(float)AccZLSB/4096+0.08; //+0.08 5ater lguitha 0.91
  AngleRoll=atan(AccY/sqrt(AccX*AccX+AccZ*AccZ))*1/(3.142/180);
  AnglePitch=-atan(AccX/sqrt(AccY*AccY+AccZ*AccZ))*1/(3.142/180);
}

void setup() {
  //Serial.begin(921600);
  Wire.begin(8, 7);  //(I2C_SDA,I2C_SCL)
  Wire.setClock(400000);
  delay(250);
  Wire.beginTransmission(0x68); 
  Wire.write(0x6B);
  Wire.write(0x00);   
  Wire.endTransmission();
  delay(250);
  for (RateCalibrationNumber=0; 
        RateCalibrationNumber<2000;
        RateCalibrationNumber ++) {
    gyro_signals();
    RateCalibrationRoll+=RateRoll;
    RateCalibrationPitch+=RatePitch;
    RateCalibrationYaw+=RateYaw;
    delay(1);
  }
  RateCalibrationRoll/=2000;
  RateCalibrationPitch/=2000;
  RateCalibrationYaw/=2000;

  ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);
  delay(1000);

  esc1.attach(1,1000,2000);
  delay(1000);
  esc1.setPeriodHertz(ESCfreq);
  delay(100);
  esc2.attach(2,1000,2000);
  delay(1000);
  esc2.setPeriodHertz(ESCfreq);
  delay(100);
  esc3.attach(3,1000,2000);
  delay(1000);
  esc3.setPeriodHertz(ESCfreq);
  delay(100);
  esc4.attach(4,1000,2000);
  delay(1000);
  esc4.setPeriodHertz(ESCfreq);
  delay(100);

  esc1.writeMicroseconds(1000);
  esc2.writeMicroseconds(1000);
  esc3.writeMicroseconds(1000);
  esc4.writeMicroseconds(1000);
  delay(1500);

  mySPI.begin(12,13,11,10);   //(NRF_SCK, NRF_MISO, NRF_MOSI, NRF_CSN)
  radio.begin(&mySPI);
  radio.openReadingPipe(0, address);
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_MAX);
  radio.startListening(); 
  resetData();

  while(!radio.available()); 
  delay(100);
  radio.read(&data, sizeof(Data_Package));
  while (data.pot2>5) {
    radio.read(&data, sizeof(Data_Package));
    delay(4);
  }  
  LoopTimer=micros();
}

void loop() {
  gyro_signals();
  RateRoll-=RateCalibrationRoll;
  RatePitch-=RateCalibrationPitch;
  RateYaw-=RateCalibrationYaw;   
  /*Serial.print("AccX= ");
  Serial.print(AccX);
  Serial.print("      AccY= ");
  Serial.print(AccY);
  Serial.print("      AccZ= ");
  Serial.println(AccZ);*/
  
  kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
  KalmanAngleRoll=Kalman1DOutput[0]; 
  KalmanUncertaintyAngleRoll=Kalman1DOutput[1];
  //KalmanAngleRoll = (KalmanAngleRoll > 20) ? 20 : ((KalmanAngleRoll < -20) ? -20 : KalmanAngleRoll);
  
  kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);
  KalmanAnglePitch=Kalman1DOutput[0]; 
  KalmanUncertaintyAnglePitch=Kalman1DOutput[1];
  //KalmanAnglePitch = (KalmanAnglePitch > 20) ? 20 : ((KalmanAnglePitch < -20) ? -20 : KalmanAnglePitch);
  /*
  //KalmanAngleRoll<--complementaryAngleRoll
  //KalmanAnglePitch<--complementaryAnglePitch
  complementaryAngleRoll=0.991*(complementaryAngleRoll+RateRoll*t) + 0.009*AngleRoll;     
  complementaryAnglePitch=0.991*(complementaryAnglePitch+RatePitch*t) + 0.009*AnglePitch; 
  // Clamping complementary filter roll angle to ±20 degrees
  complementaryAngleRoll = (complementaryAngleRoll > 20) ? 20 : ((complementaryAngleRoll < -20) ? -20 : complementaryAngleRoll);
  complementaryAnglePitch = (complementaryAnglePitch > 20) ? 20 : ((complementaryAnglePitch < -20) ? -20 : complementaryAnglePitch);
  */

  if (radio.available()) {
    radio.read(&data, sizeof(Data_Package)); 
    lastReceiveTime = millis(); 
  }
  else if(millis()-lastReceiveTime>2000){
    resetData();
  }
  //DesiredAnglePitch   85-->115, 84-->114, 170-->140
  if (data.j1PotY>=140){
    DesiredAnglePitch = map(data.j1PotY, 140, 255, 0, 50);    
  }
  else if(data.j1PotY<115){
    DesiredAnglePitch =- map(data.j1PotY, 114, 0, 0, 50);
  }else{
    DesiredAnglePitch=0;
  }
  //DesiredAngleRoll
  if (data.j1PotX>=140){
    DesiredAngleRoll =- map(data.j1PotX, 140, 255, 0, 50);       
  }
  else if(data.j1PotX<115){
    DesiredAngleRoll = map(data.j1PotX, 114, 0, 0, 50);
  }else{
    DesiredAngleRoll=0;
  }
  //DesiredRateYaw
  if (data.j2PotX >= 140) {
    DesiredRateYaw =- map(data.j2PotX, 140, 255, 0, 75);                              
  }
  else if (data.j2PotX < 115) {
    DesiredRateYaw = map(data.j2PotX, 114, 0, 0, 75);
  }else{
    DesiredRateYaw=0;
  }
  //inputThrottle
  InputThrottle=map(data.pot2, 0, 255, 1000, 2000);   

  ErrorAngleRoll=DesiredAngleRoll-KalmanAngleRoll;
  pid_equation(ErrorAngleRoll, PAngleRoll, IAngleRoll, DAngleRoll, PrevErrorAngleRoll,PrevItermAngleRoll);     
  DesiredRateRoll=PIDReturn[0]; 
  PrevErrorAngleRoll=PIDReturn[1];
  PrevItermAngleRoll=PIDReturn[2];
 
  ErrorAnglePitch=DesiredAnglePitch-KalmanAnglePitch;
  pid_equation(ErrorAnglePitch, PAnglePitch, IAnglePitch, DAnglePitch, PrevErrorAnglePitch, PrevItermAnglePitch);
  DesiredRatePitch=PIDReturn[0]; 
  PrevErrorAnglePitch=PIDReturn[1];
  PrevItermAnglePitch=PIDReturn[2];

  ErrorRateRoll=DesiredRateRoll-RateRoll;
  pid_equation(ErrorRateRoll, PRateRoll, IRateRoll, DRateRoll, PrevErrorRateRoll, PrevItermRateRoll);
  InputRoll=PIDReturn[0];
  PrevErrorRateRoll=PIDReturn[1]; 
  PrevItermRateRoll=PIDReturn[2];
  
  ErrorRatePitch=DesiredRatePitch-RatePitch;
  pid_equation(ErrorRatePitch, PRatePitch, IRatePitch, DRatePitch, PrevErrorRatePitch, PrevItermRatePitch);
  InputPitch=PIDReturn[0]; 
  PrevErrorRatePitch=PIDReturn[1]; 
  PrevItermRatePitch=PIDReturn[2];
  
  ErrorRateYaw=DesiredRateYaw-RateYaw;
  pid_equation(ErrorRateYaw, PRateYaw,IRateYaw, DRateYaw, PrevErrorRateYaw,PrevItermRateYaw);
  InputYaw=PIDReturn[0]; 
  PrevErrorRateYaw=PIDReturn[1]; 
  PrevItermRateYaw=PIDReturn[2];
  
  if (InputThrottle > 1800) InputThrottle = 1800;
  
  MotorInput1= (InputThrottle-InputRoll-InputPitch-InputYaw);//FR 
  MotorInput2= (InputThrottle-InputRoll+InputPitch+InputYaw);//BR
  MotorInput3= (InputThrottle+InputRoll+InputPitch-InputYaw);//BL
  MotorInput4= (InputThrottle+InputRoll-InputPitch+InputYaw);//FL

  MotorInput1=constrain(MotorInput1,1150,1999);  //1150-->38.25
  MotorInput2=constrain(MotorInput2,1150,1999);
  MotorInput3=constrain(MotorInput3,1150,1999);
  MotorInput4=constrain(MotorInput4,1150,1999);
  
  if (InputThrottle <1050){ //data.pot2<13
    MotorInput1=1000;
    MotorInput2=1000;
    MotorInput3=1000;
    MotorInput4=1000;
    reset_pid();
  }
  
  esc1.writeMicroseconds(MotorInput1);  //FR
  esc2.writeMicroseconds(MotorInput2);  //BR
  esc3.writeMicroseconds(MotorInput3);  //BL
  esc4.writeMicroseconds(MotorInput4);  //FL

  /*
  //Reciever signals
  Serial.print("\n");
  Serial.print("data.j1PotX = ");
  Serial.println(data.j1PotX);
  Serial.print("data.j1PotY = ");
  Serial.println(data.j1PotY);
  Serial.print("data.j2PotX = ");
  Serial.println(data.j2PotX);
  Serial.print("data.j2PotY= ");
  Serial.println(data.j2PotY);
  Serial.print("data.j1Button = ");
  Serial.println(data.j1Button);
  Serial.print("data.j2Button = ");
  Serial.println(data.j2Button);
  Serial.print("data.pot1 = ");
  Serial.println(data.pot1);
  Serial.print("data.pot2 = ");
  Serial.println(data.pot2);
  Serial.print("data.tSwitch1 = ");
  Serial.println(data.tSwitch1);
  Serial.print("data.tSwitch2 = ");
  Serial.println(data.tSwitch2);
  Serial.print("data.button1 = ");
  Serial.println(data.button1);
  Serial.print("data.button2 = ");
  Serial.println(data.button2);
  Serial.print("data.button3 = ");
  Serial.println(data.button3);
  Serial.print("data.button4 = ");
  Serial.println(data.button4);
  
   //Motor PWMs in us
  Serial.print("MotVals-");
  Serial.print(MotorInput1);
  Serial.print("  ");
  Serial.print(MotorInput2);
  Serial.print("  ");
  Serial.print(MotorInput3);
  Serial.print("  ");
  Serial.print(MotorInput4);
  Serial.println(" ");

  //Reciever translated rates
  Serial.print(DesiredRateRoll);
  Serial.print("  ");
  Serial.print(DesiredRatePitch);
  Serial.print("  ");
  Serial.print(DesiredRateYaw);
  Serial.print(" -- ");

  //IMU values
  Serial.print("Acc values: ");
  Serial.print("AccX:");
  Serial.print(AccX);
  Serial.print("  ");
  Serial.print("AccY:");
  Serial.print(AccY);
  Serial.print("  ");
  Serial.print("AccZ:");
  Serial.print(AccZ);
  Serial.print(" -- ");
  Print the gyroscope values
  Serial.print("Gyro values: ");
  Serial.print(RateRoll);
  Serial.print("  ");
  Serial.print(RatePitch);
  Serial.print("  ");
  Serial.print(RateYaw);
  Serial.print("  ");
  Serial.print(" -- ");

  //PID outputs
  Serial.print("PID O/P ");
  Serial.print(InputPitch);
  Serial.print("  ");
  Serial.print(InputRoll);
  Serial.print("  ");
  Serial.print(InputYaw);
  Serial.print(" -- ");

  //Angles from MPU
  Serial.print("AngleRoll:");
  Serial.print(AngleRoll);
  serial.print("  ");
  Serial.print("AnglePitch:");
  Serial.print(AnglePitch);

  Serial.print("KalmanAngleRoll:");
  Serial.print(KalmanAngleRoll);
  serial.print("  ");
  Serial.print("KalmanAnglePitch:");
  Serial.print(KalmanAnglePitch);

  Serial.print("ComplementaryAngleRoll: ");
  Serial.print(complementaryAngleRoll);
  Serial.print("ComplementaryAnglePitch: ");
  Serial.print(complementaryAnglePitch);
  Serial.println(" ");  

  //  serial plotter comparison
  Serial.print(KalmanAngleRoll);
  Serial.print(" ");
  Serial.print(KalmanAnglePitch);
  Serial.print(" ");
  
  Serial.print(complementaryAngleRoll);
  Serial.print(" ");
  Serial.println(complementaryAnglePitch);
  */

  while (micros() - LoopTimer < (t*1000000));
  LoopTimer=micros();
}
