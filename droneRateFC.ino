#include <Wire.h>
#include <SoftwareSerial.h>

#define YAW_CHANNEL 3
#define THROTTLE_CHANNEL 2
#define ROLL_CHANNEL 0
#define PITCH_CHANNEL 1
#define CALIBRATION_CNT 400
#define MOTOR_MAX_VAL 1999
#define MAX_THROTTLE 1999

float MAX_RATE = 0.2;

SoftwareSerial mySerial(8, 11); // RX, TX

float ratePitch, rateRoll, rateYaw;
float rateCalibrationPitch, rateCalibrationRoll, rateCalibrationYaw;
int rateCalibrationNumber;

float receiverValue[]={0, 0, 0, 0, 0, 0, 0, 0};
int channelNumber=0; 

uint32_t loopTimer;
float desiredRollRate, desiredPitchRate, desiredYawRate;
float errorRollRate, errorPitchRate, errorYawRate;
float inputRoll, inputThrottle, inputPitch, inputYaw;
float prevErrorRollRate, prevErrorPitchRate, prevErrorYawRate;
float prevItermRollRate, prevItermPitchRate, prevItermYawRate;
float PIDReturn[]={0, 0, 0};
float PRateRoll= 2.2 ; float PRatePitch=PRateRoll; float PYawRate=2;
float IRateRoll=1.2 ; float IRatePitch=IRateRoll; float IYawRate=12;
float DRateRoll=0.001 ; float DRatePitch=DRateRoll; float DYawRate=0;
float motorInput1, motorInput2, motorInput3, motorInput4;


#define BUFFER_SIZE 30

uint8_t buffer[BUFFER_SIZE];

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
  Wire.requestFrom(0x68,6);
  int16_t GyroX=Wire.read()<<8 | Wire.read();
  int16_t GyroY=Wire.read()<<8 | Wire.read();
  int16_t GyroZ=Wire.read()<<8 | Wire.read();
  rateRoll=(float)GyroX/65.5;
  ratePitch=(float)GyroY/65.5;
  rateYaw=(float)GyroZ/65.5;
}

#define MAX_PID_VALUE 200
void pid_equation(float Error, float P , float I, float D, float PrevError, float PrevIterm) {
  float Pterm=P*Error;
  
  float Iterm=PrevIterm+I*(Error+PrevError)*0.004/2;
  if (Iterm > MAX_PID_VALUE) 
      Iterm=MAX_PID_VALUE;
  else if (Iterm <-MAX_PID_VALUE) 
      Iterm=-MAX_PID_VALUE;
      
  float Dterm=D*(Error-PrevError)/0.004;
  
  float PIDOutput= Pterm+Iterm+Dterm;
  
  if (PIDOutput>MAX_PID_VALUE) 
      PIDOutput=MAX_PID_VALUE;
  else if (PIDOutput <-MAX_PID_VALUE) 
      PIDOutput=-MAX_PID_VALUE;
  PIDReturn[0]=PIDOutput;
  PIDReturn[1]=Error;
  PIDReturn[2]=Iterm;
}

void reset_pid(void) {
  prevErrorRollRate=0; prevErrorPitchRate=0; prevErrorYawRate=0;
  prevItermRollRate=0; prevItermPitchRate=0; prevItermYawRate=0;
}

void waitForPacket(){
  char n;
  
    while(true){
    n = mySerial.read();
    if(n == 32){
//        Serial.println();
      delayMicroseconds(10);
        n = mySerial.read();
        
        if(n == 64){
            break;
        }
    }
  }
}

void savePacketBuffer(){
  uint8_t k = 0;
  
  while(k < BUFFER_SIZE){
      if(mySerial.available())
        buffer[k++] = mySerial.read();
  }
}

typedef struct RadioData{
  int16_t roll;
  int16_t pitch;
  int16_t throttle;
  int16_t yaw;
  int16_t arm;
  int16_t mode;
} RadioData;

RadioData data;

void createPacket(){
    uint16_t val = buffer[0] + (buffer[1] << 8);
    if(val > 2000)
      return;
    data.roll =  val;
    
    val = buffer[2] + (buffer[3] << 8);
    if(val > 2000)
      return;
    data.pitch = val;

    val = buffer[4]+ (buffer[5] << 8);
    if(val > 2000)
      return;
    data.throttle = val;

    val = buffer[6]+ (buffer[7] << 8);
    if(val > 2000)
      return;
    data.yaw = val;

    val = buffer[12]+ (buffer[13] << 8);
    if(val > 2000)
      return;
    data.mode = val;

    val = buffer[18]+ (buffer[19] << 8);
    if(val > 2000)
      return;
    data.arm = val;
}

bool bufferCorrect = true;

void validatePacket(){
      for(int i = 0; i < 10; i++){
            if(i % 2 == 1 && buffer[i] > 7){
                bufferCorrect = false;
                break;    
            }
      }
      bufferCorrect = true;
}

void readRadio(){
  waitForPacket();
  savePacketBuffer();
  validatePacket();

  if(bufferCorrect == false)
    return;
    
  createPacket();
}

void setup() {
  Serial.begin(1000000);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  mySerial.begin(115200);

  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
     
  Wire.setClock(400000);
  Wire.begin();
  delay(250);
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();


  for (rateCalibrationNumber=0; rateCalibrationNumber< CALIBRATION_CNT; rateCalibrationNumber ++) {
    gyro_signals();
    rateCalibrationRoll+=rateRoll;
    rateCalibrationPitch+=ratePitch;
    rateCalibrationYaw+=rateYaw;
    delay(1);
  }
  
  rateCalibrationRoll/=CALIBRATION_CNT;
  rateCalibrationPitch/=CALIBRATION_CNT;
  rateCalibrationYaw/=CALIBRATION_CNT;
  

  while(mySerial.available()){
      char n = mySerial.read();
      if(n == 32){
//        Serial.println("32");
        char n = mySerial.read();
        
        if(n == 64){
//          Serial.println("64");
          break;
        }
      }
  }

//  readRadio();
//  while(data.arm < 1200){
//    readRadio();
//  }

  Serial.println("start2");

  loopTimer=micros();
}

int idx = 1;
void loop() {
  //    read until buffercorect == true?
    readRadio();

    if(data.mode > 1600){
        MAX_RATE = 0.05;
        
        gyro_signals();
        rateRoll-=rateCalibrationRoll;
        ratePitch-=rateCalibrationPitch;
        rateYaw-=rateCalibrationYaw;
    
        
        desiredRollRate = MAX_RATE * (data.roll - 1500);
        desiredPitchRate = MAX_RATE * (data.pitch - 1500);
        desiredYawRate = 0.2 * (data.yaw - 1500);
        errorRollRate = desiredRollRate - rateRoll;
        errorPitchRate = desiredPitchRate - ratePitch;
        errorYawRate = desiredYawRate - rateYaw;
    
        pid_equation(errorRollRate, PRateRoll, IRateRoll, DRateRoll, prevErrorRollRate, prevItermRollRate);
        inputRoll=PIDReturn[0];
        prevErrorRollRate=PIDReturn[1]; 
        prevItermRollRate=PIDReturn[2];
    
        pid_equation(errorPitchRate, PRatePitch, IRatePitch, DRatePitch, prevErrorPitchRate, prevItermPitchRate);
        inputPitch=PIDReturn[0]; 
        prevErrorPitchRate=PIDReturn[1]; 
        prevItermPitchRate=PIDReturn[2];
        
        pid_equation(errorYawRate, PYawRate, IYawRate, DYawRate, prevErrorYawRate, prevItermYawRate);
        inputYaw=PIDReturn[0]; 
        prevErrorYawRate=PIDReturn[1]; 
        prevItermYawRate=PIDReturn[2];
    } else {
        MAX_RATE = 0.15;
        
        inputRoll=desiredRollRate = MAX_RATE * (data.roll - 1500);
        inputPitch=desiredPitchRate = MAX_RATE * (data.pitch - 1500);
        inputYaw=desiredYawRate = 0.6 * (data.yaw - 1500);
        errorRollRate = desiredRollRate - rateRoll;
        errorPitchRate = desiredPitchRate - ratePitch;
        errorYawRate = desiredYawRate - rateYaw;
    }

    inputThrottle= data.throttle;
    if (inputThrottle > MAX_THROTTLE) 
        inputThrottle = MAX_THROTTLE;

//    String s = String(inputThrottle) + " " + String(inputRoll) + " " + String(inputPitch) + " " + String(inputYaw);
//    String s2 = String(data.roll) + " " + String(rateRoll) + " " + String(desiredRollRate) + " " + String(errorRollRate);


//    M4 /\ M1
//       X     
//    M3    M2
//
    float MOTOR_MULTIPLIER = 1.03;
    
    motorInput1= MOTOR_MULTIPLIER * (inputThrottle-inputRoll-inputPitch-inputYaw);
    motorInput2= MOTOR_MULTIPLIER * (inputThrottle-inputRoll+inputPitch+inputYaw);
    motorInput3= MOTOR_MULTIPLIER * (inputThrottle+inputRoll+inputPitch-inputYaw);
    motorInput4= MOTOR_MULTIPLIER * (inputThrottle+inputRoll-inputPitch+inputYaw);

    if (motorInput1 > MOTOR_MAX_VAL)motorInput1 = MOTOR_MAX_VAL;
    if (motorInput2 > MOTOR_MAX_VAL)motorInput2 = MOTOR_MAX_VAL; 
    if (motorInput3 > MOTOR_MAX_VAL)motorInput3 = MOTOR_MAX_VAL; 
    if (motorInput4 > MOTOR_MAX_VAL)motorInput4 = MOTOR_MAX_VAL;

    int throttleIdle = 1050;
    if (motorInput1 < throttleIdle) 
        motorInput1 =  throttleIdle;
    if (motorInput2 < throttleIdle) 
        motorInput2 =  throttleIdle;
    if (motorInput3 < throttleIdle) 
        motorInput3 =  throttleIdle;
    if (motorInput4 < throttleIdle) 
        motorInput4 =  throttleIdle;

    int throttleCutOff=1000;
    if (data.throttle<1030) {
      motorInput1=throttleCutOff; 
      motorInput2=throttleCutOff;
      motorInput3=throttleCutOff; 
      motorInput4=throttleCutOff;
      reset_pid();
    }

    if(idx++ > 25){
//        String s = String(motorInput1) + " " + String(motorInput2) + " " + String(motorInput3) + " " + String(motorInput4);

          String s = 
                  String(inputPitch) + "\t\t" + 
                  String(map(motorInput1, 1000, 2000, 0, 255)) + " " + 
                  String(motorInput2) + "\t\t" + 
                  String(motorInput3) + " " + 
                  String(motorInput4) //+ " " + 
                  ;
        
        if(data.mode > 1600){
            Serial.print("acro ON | ");
            
             s = 
//                  String(rateRoll) + "\t" + 
              String(ratePitch) + "\t" + 
//                  String(errorRollRate) + " " +
              String(errorPitchRate) + "\t" + 
//                  String(inputRoll) + "\t\t" + 
              String(inputPitch) + "\t\t" + 
              String(map(motorInput1, 1000, 2000, 0, 255)) + " " + 
              String(motorInput2) + "\t\t" + 
              String(motorInput3) + " " + 
              String(motorInput4) //+ " " + 
              ;
        }
        Serial.println(s); 
        
        idx = 0;
    }

    analogWrite(5,map(motorInput1, 1000, 2000, 0, 255));
    analogWrite(6,map(motorInput2, 1000, 2000, 0, 255));
    analogWrite(9,map(motorInput3, 1000, 2000, 0, 255)); 
    analogWrite(10,map(motorInput4, 1000, 2000, 0, 255));

    while (micros() - loopTimer < 4000);
    loopTimer = micros();
}
