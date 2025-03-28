#include <Wire.h>
#include <SPI.h>           
#include <nRF24L01.h>
#include <RF24.h>
#include <ESP32Servo.h>
Servo esc1;
Servo esc2;
Servo esc3;
Servo esc4;

struct Data_Package {
  byte j1PotX;
  byte j1PotY;
  byte j1Button;
  byte j2PotX;
  byte j2PotY;
  byte j2Button;
  byte pot1;
  byte pot2;
  byte tSwitch1;
  byte tSwitch2;
  byte button1;
  byte button2;
  byte button3;
  byte button4;
};
Data_Package data;
const byte address[6] = "00001";

SPIClass mySPI(HSPI);
RF24 radio(9, 10);   // nRF24L01 (CE, CSN)


float RateRoll, RatePitch, RateYaw;
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw,AccXCalibration,AccYCalibration,AccZCalibration;
int RateCalibrationNumber;

uint32_t LoopTimer,lastReceiveTime;

float DesiredRateRoll, DesiredRatePitch, DesiredRateYaw;
float ErrorRateRoll, ErrorRatePitch, ErrorRateYaw;
float InputRoll, InputThrottle, InputPitch, InputYaw;
float PrevErrorRateRoll, PrevErrorRatePitch, PrevErrorRateYaw;
float PrevItermRateRoll, PrevItermRatePitch, PrevItermRateYaw;
float PIDReturn[]={0, 0, 0};

float PRateRoll=0.7 ; float PRatePitch=PRateRoll; float PRateYaw=2;   //>Parfaitee ema edrone maawja btbiaatha
float IRateRoll=8 ; float IRatePitch=IRateRoll; float IRateYaw=12;  
float DRateRoll=0.085 ; float DRatePitch=DRateRoll; float DRateYaw=0;  //d=0.1 talle3 edrone barcha w glebha, d=0.09 talle3ha brcha ema a9al

float MotorInput1, MotorInput2, MotorInput3, MotorInput4;

float AccX, AccY, AccZ;
float AngleRoll, AnglePitch;

float KalmanAngleRoll=0, KalmanUncertaintyAngleRoll=2*2;
float KalmanAnglePitch=0, KalmanUncertaintyAnglePitch=2*2;
float Kalman1DOutput[]={0,0};

float DesiredAngleRoll, DesiredAnglePitch;
float ErrorAngleRoll, ErrorAnglePitch;
float PrevErrorAngleRoll, PrevErrorAnglePitch;
float PrevItermAngleRoll, PrevItermAnglePitch;

float PAngleRoll=2; float PAnglePitch=PAngleRoll;
float IAngleRoll=0; float IAnglePitch=IAngleRoll;
float DAngleRoll=0.007/*0*/; float DAnglePitch=DAngleRoll;

uint16_t dig_T1, dig_P1;
int16_t  dig_T2, dig_T3, dig_P2, dig_P3, dig_P4, dig_P5;
int16_t  dig_P6, dig_P7, dig_P8, dig_P9; 
float AltitudeBarometer, AltitudeBarometerStartUp;
float AccZInertial;

#include <BasicLinearAlgebra.h>
using namespace BLA;
float AltitudeKalman, VelocityVerticalKalman;
BLA::Matrix<2,2> F; BLA::Matrix<2,1> G;
BLA::Matrix<2,2> P; BLA::Matrix<2,2> Q;
BLA::Matrix<2,1> S; BLA::Matrix<1,2> H;
BLA::Matrix<2,2> I; BLA::Matrix<1,1> Acc;
BLA::Matrix<2,1> K; BLA::Matrix<1,1> R;
BLA::Matrix<1,1> L; BLA::Matrix<1,1> M;

float DesiredVelocityVertical, ErrorVelocityVertical;
float PVelocityVertical=3.5; float IVelocityVertical=0.0015; float DVelocityVertical=0.01; 
float PrevErrorVelocityVertical, PrevItermVelocityVertical;

void kalman_2d(void){
  Acc = {AccZInertial};
  S=F*S+G*Acc;
  P=F*P*~F+Q;
  L=H*P*~H+R;
  K=P*~H*Invert(L);
  M = {AltitudeBarometer};
  S=S+K*(M-H*S);
  AltitudeKalman=S(0,0); 
  VelocityVerticalKalman=S(1,0); 
  P=(I-K*H)*P;
}

void barometer_signals(void){
  Wire.beginTransmission(0x76);
  Wire.write(0xF7);
  Wire.endTransmission();
  Wire.requestFrom(0x76,6);
  uint32_t press_msb = Wire.read();
  uint32_t press_lsb = Wire.read();
  uint32_t press_xlsb = Wire.read();
  uint32_t temp_msb = Wire.read();
  uint32_t temp_lsb = Wire.read();
  uint32_t temp_xlsb = Wire.read();
  unsigned long int adc_P = (press_msb << 12) | (press_lsb << 4) | (press_xlsb >>4);
  unsigned long int adc_T = (temp_msb << 12) | (temp_lsb << 4) | (temp_xlsb >>4);
  signed long int var1, var2;
  var1 = ((((adc_T >> 3) - ((signed long int )dig_T1 <<1)))* ((signed long int )dig_T2)) >> 11;
  var2 = (((((adc_T >> 4) - ((signed long int )dig_T1)) * ((adc_T>>4) - ((signed long int )dig_T1)))>> 12) * ((signed long int )dig_T3)) >> 14;
  signed long int t_fine = var1 + var2;
  unsigned long int p;
  var1 = (((signed long int )t_fine)>>1) - (signed long int )64000;
  var2 = (((var1>>2) * (var1>>2)) >> 11) * ((signed long int )dig_P6);
  var2 = var2 + ((var1*((signed long int )dig_P5)) <<1);
  var2 = (var2>>2)+(((signed long int )dig_P4)<<16);
  var1 = (((dig_P3 * (((var1>>2)*(var1>>2)) >> 13 ))>>3)+((((signed long int )dig_P2) * var1)>>1))>>18;
  var1 = ((((32768+var1))*((signed long int )dig_P1)) >>15);
  if (var1 == 0) { p=0;}    
  p = (((unsigned long int )(((signed long int ) 1048576)-adc_P)-(var2>>12)))*3125;
  if(p<0x80000000){ p = (p << 1) / ((unsigned long int ) var1);}
  else { p = (p / (unsigned long int )var1) * 2;  }
  var1 = (((signed long int )dig_P9) * ((signed long int ) (((p>>3) * (p>>3))>>13)))>>12;
  var2 = (((signed long int )(p>>2)) * ((signed long int )dig_P8))>>13;
  p = (unsigned long int)((signed long int )p + ((var1 + var2+ dig_P7) >> 4));
  double pressure=(double)p/100;
  AltitudeBarometer=44330*(1-pow(pressure
     /1013.25, 1/5.255))*100;
}

void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
  KalmanState=KalmanState+0.004*KalmanInput;
  KalmanUncertainty=KalmanUncertainty + 0.004 * 0.004 * 4 * 4;
  float KalmanGain=KalmanUncertainty * 1/(1*KalmanUncertainty + 3 * 3);
  KalmanState=KalmanState+KalmanGain * (
  KalmanMeasurement-KalmanState);
  KalmanUncertainty=(1-KalmanGain) * KalmanUncertainty;
  Kalman1DOutput[0]=KalmanState; 
  Kalman1DOutput[1]=KalmanUncertainty;
}

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
  AccX=(float)AccXLSB/4096-0.05-0.036; //-0.05 5ater lguitha 1.05  ;-0.02 5atr edrone maawja //ken edrone maawja w tmchy maa y+ nhot - snn +
  AccY=(float)AccYLSB/4096+0.02/*+0.01*/; //+0.02 5ater lguitha 0.98   ;+0.02 5atr edrone maawja //ken edrone maawja w tmchy maa y+ nhot - snn +
  AccZ=(float)AccZLSB/4096+0.08; //+0.08 5ater lguitha 0.91
  AngleRoll=atan(AccY/sqrt(AccX*AccX+AccZ*AccZ))*1/(3.142/180);
  AnglePitch=-atan(AccX/sqrt(AccY*AccY+AccZ*AccZ))*1/(3.142/180);
}

void pid_equation(float Error, float P , float I, float D, float PrevError, float PrevIterm) {
  float Pterm=P*Error;
  float Iterm=PrevIterm+I*(Error+PrevError)*0.004/2;
  if (Iterm > 400) Iterm=400;
  else if (Iterm <-400) Iterm=-400;
  float Dterm=D*(Error-PrevError)/0.004;
  float PIDOutput= Pterm+Iterm+Dterm;
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
  PrevErrorVelocityVertical=0; 
  PrevItermVelocityVertical=0;
}

void setup() {
  //Serial.begin(921600);
  esc1.attach(1,1000,2000);
  esc2.attach(2,1000,2000);
  esc3.attach(3,1000,2000);
  esc4.attach(4,1000,2000);
  delay(500);                                                  
  esc1.writeMicroseconds(1000);
  esc2.writeMicroseconds(1000);
  esc3.writeMicroseconds(1000);
  esc4.writeMicroseconds(1000);
  delay(2000);

  Wire.begin(8, 7);  //(I2C_SDA,I2C_SCL)
  Wire.setClock(400000);
  delay(250);
  Wire.beginTransmission(0x68); 
  Wire.write(0x6B);
  Wire.write(0x00);   
  Wire.endTransmission();
  Wire.beginTransmission(0x76); 
  Wire.write(0xF4);
  Wire.write(0x57);
  Wire.endTransmission();   
  Wire.beginTransmission(0x76);
  Wire.write(0xF5); 
  Wire.write(0x14);
  Wire.endTransmission();   
  uint8_t data_[24], i=0;
  Wire.beginTransmission(0x76);
  Wire.write(0x88);
  Wire.endTransmission();
  Wire.requestFrom(0x76,24);      
  while(Wire.available()){
    data_[i] = Wire.read();
    i++;
  } 
  dig_T1 = (data_[1] << 8) | data_[0]; 
  dig_T2 = (data_[3] << 8) | data_[2];
  dig_T3 = (data_[5] << 8) | data_[4];
  dig_P1 = (data_[7] << 8) | data_[6]; 
  dig_P2 = (data_[9] << 8) | data_[8];
  dig_P3 = (data_[11]<< 8) | data_[10];
  dig_P4 = (data_[13]<< 8) | data_[12];
  dig_P5 = (data_[15]<< 8) | data_[14];
  dig_P6 = (data_[17]<< 8) | data_[16];
  dig_P7 = (data_[19]<< 8) | data_[18];
  dig_P8 = (data_[21]<< 8) | data_[20];
  dig_P9 = (data_[23]<< 8) | data_[22]; delay(250);
  for (RateCalibrationNumber=0; 
        RateCalibrationNumber<2000;
        RateCalibrationNumber ++) {
    gyro_signals();
    RateCalibrationRoll+=RateRoll;
    RateCalibrationPitch+=RatePitch;
    RateCalibrationYaw+=RateYaw;
    barometer_signals();
    AltitudeBarometerStartUp+=AltitudeBarometer; 
    delay(1);
  }
  RateCalibrationRoll/=2000;
  RateCalibrationPitch/=2000;
  RateCalibrationYaw/=2000;
  AltitudeBarometerStartUp/=2000;

  F = {1, 0.004,
            0, 1};  
  G = {0.5*0.004*0.004,
            0.004};
  H = {1, 0};
  I = {1, 0,
           0, 1};
  Q = G * ~G*10*10;
  R = {30*30};
  P = {0, 0,
           0, 0};
  S = {0,
           0};

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
  
  kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);
  KalmanAnglePitch=Kalman1DOutput[0]; 
  KalmanUncertaintyAnglePitch=Kalman1DOutput[1];
  
  AccZInertial=-sin(AnglePitch*(3.142/180))*AccX+cos(AnglePitch*(3.142/180))*sin(AngleRoll*(3.142/180))* AccY+cos(AnglePitch*(3.142/180))*cos(AngleRoll*(3.142/180))*AccZ;   
  AccZInertial=(AccZInertial-1)*9.81*100;
  
  barometer_signals();
  AltitudeBarometer-=AltitudeBarometerStartUp;
  kalman_2d();

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
  //DesiredVelocityVertical  
  if (data.j2PotY>=140){
    DesiredVelocityVertical = map(data.j2PotY, 140, 255, 0, 150);    
  }
  else if(data.j2PotY<115){
    DesiredVelocityVertical =- map(data.j2PotY, 114, 0, 0, 150);
  }else{
    DesiredVelocityVertical=0;
  }
  
  InputThrottle=map(data.pot2, 0, 255, 1000, 2000);   ///////


  ErrorVelocityVertical=DesiredVelocityVertical-VelocityVerticalKalman;
  pid_equation(ErrorVelocityVertical, PVelocityVertical, IVelocityVertical, DVelocityVertical, PrevErrorVelocityVertical, PrevItermVelocityVertical);
  //InputThrottle=1500+PIDReturn[0];  
  PrevErrorVelocityVertical=PIDReturn[1]; 
  PrevItermVelocityVertical=PIDReturn[2];

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

  MotorInput1=constrain(MotorInput1,1150,1953);  //1150-->38.25
  MotorInput2=constrain(MotorInput2,1150,1953);
  MotorInput3=constrain(MotorInput3,1150,1953);
  MotorInput4=constrain(MotorInput4,1150,1953);
  if (data.pot2<13){
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

  while (micros() - LoopTimer < 4000);
  LoopTimer=micros();
}

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