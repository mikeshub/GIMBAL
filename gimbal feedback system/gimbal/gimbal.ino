#include <SPI.h>
#include <Streaming.h>
#include "PIDL.h" 

#define GEAR_RATIO_YAW 3092.783505154639f//currently the motor drive ratio of 9.7:1 * 30,000 - 
//30,000 is to simplfly the calculations
#define GEAR_RATIO_PITCH 1.0f//change to gear ratios * 30,000

//general SPI defines
#define READ 0x80
#define WRITE 0x00
#define MULTI 0x40
#define SINGLE 0x00

//gyro defines
#define L3G_CTRL_REG1 0x20
#define L3G_CTRL_REG2 0x21
#define L3G_CTRL_REG3 0x22
#define L3G_CTRL_REG4 0x23
#define L3G_CTRL_REG5 0x24
#define L3G_OUT_Y_L 0x2A

//gyro calibration vars
int32_t gyroSumY,gyroSumZ;
int16_t offsetY,offsetZ;
//gyro vars
typedef union{
  struct{
    int16_t y;
    int16_t z;
  }
  v;
  byte buffer[4];
}
Sensor_t;

Sensor_t gyro;



//macros for SS line
#define GyroSSOutput() DDRB |= 1<<0 
#define GyroSSHigh() PORTB |= 1<<0 
#define GyroSSLow() PORTB &= ~(1<<0)

//PWM
#define FREQ 100
#define PRESCALE 8
#define PERIOD ((F_CPU/PRESCALE/FREQ) - 1)

#define yawMotorWriteMicros(x) OCR1A = x * 2//motor 1 is attached to pin9
#define pitchMotorWriteMicros(x) OCR1B = x * 2//motor 2 is attached to pin10

//encoder velocity vars
float yawEncoderSpeed;
uint32_t yawEncoderCount;
boolean yawDirectionFlag;
float pitchEncoderSpeed;
uint32_t pitchEncoderCount;
boolean pitchDirectionFlag;

//PID
float kpYawOuter = 1,kiYawOuter = 0,kdYawOuter = 0;//gains
float kpYawInner = 1,kiYawInner = 0,kdYawInner = 0;
float kpPitchOuter = 1,kiPitchOuter = 0,kdPitchOuter = 0;
float kpPitchInner = 1,kiPitchInner = 0,kdPitchInner = 0;
float outerIntegralLimit = 300,outerLimit = 500;
float innerIntegralLimit = 300,innerLimit = 500;

float yawSetPoint,pitchSetPoint;//control vars
float degreeGyroY,degreeGyroZ;
float yawDPSCommand,pitchDPSCommand;


float yawMotorCommand,pitchMotorCommand;
float yawMotorAdj,pitchMotorAdj;

boolean integrate = true;

float DT;
uint32_t previousControlTime;

PID YawOuter(&yawSetPoint,&degreeGyroZ,&yawDPSCommand,&integrate,&kpYawOuter,&kiYawOuter,&kdYawOuter,&DT,outerIntegralLimit,outerLimit);
PID PitchOuter(&pitchSetPoint,&degreeGyroY,&pitchDPSCommand,&integrate,&kpPitchOuter,&kiPitchOuter,&kdPitchOuter,&DT,outerIntegralLimit,outerLimit);

PID YawInner(&yawDPSCommand,&yawEncoderSpeed,&yawMotorAdj,&integrate,&kpYawInner,&kiYawInner,&kdYawInner,&DT,innerIntegralLimit,innerLimit);
PID PitchInner(&pitchDPSCommand,&pitchEncoderSpeed,&pitchMotorAdj,&integrate,&kpPitchInner,&kiPitchInner,&kdPitchInner,&DT,outerIntegralLimit,outerLimit);



//GP loops vars
int i,j;

//SBUS / RC
uint8_t readState,inByte,bufferIndex;
boolean newRC = false;

typedef union{
  struct{
    uint16_t aileron;//A8
    uint16_t aux1;//A9
    uint16_t elevator;//A10
    uint16_t gear;//A11
    uint16_t rudder;//A12
    uint16_t aux2;//A13
    uint16_t throttle;//A14
    uint16_t aux3;//A15 only for futaba or standard RC
  }
  values;
  byte buffer[16];
  uint16_t standardRCBuffer[8];
}
RadioControl_t;


RadioControl_t rcCommands;

uint8_t sBusData[25];


void setup(){
  Serial.begin(100000);//SBUS serial data rate
  pinMode(6,INPUT);
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV2);  
  SPI.setDataMode(SPI_MODE0);
  GyroSSOutput();
  GyroInit();
  MotorInit();
  DDRD = 0;//pin 2,3,4,5
  PORTD |= 0x3C;//turn on pullups
  EICRA |= 0x03;//rising edge for both interrupts
  EIMSK |= 0x03;//enable the interrupts
  previousControlTime = millis();
}

void loop(){
  SBusParser();
  if (millis() - previousControlTime >= 10){
    cli();//disable interrupts for calculations
    DT = millis() - previousControlTime;
    yawEncoderSpeed = ( yawEncoderCount * GEAR_RATIO_YAW) / DT;
    if (yawDirectionFlag != true){
      yawEncoderSpeed *= -1.0;
    }
    yawEncoderCount = 0;
    pitchEncoderSpeed = ( pitchEncoderCount * GEAR_RATIO_PITCH) / DT;
    if (pitchDirectionFlag != true){
      pitchEncoderSpeed *= -1.0;
    }
    pitchEncoderCount = 0;
    GetGyro();
    YawOuter.calculate();
    YawInner.calculate();
    PitchOuter.calculate();
    PitchInner.calculate();
    yawMotorCommand = 1500 + yawMotorAdj;
    pitchMotorCommand = 1500 + pitchMotorAdj;
    
    yawMotorWriteMicros(yawMotorCommand);
    pitchMotorWriteMicros(pitchMotorCommand);
    
    previousControlTime = millis();
    sei();//enable the interrupts
  }  

  //this section is used to program the max and mins for the JRK 12v12
  /*Serial<<"Writing 2000us\r\n";
   Motor1WriteMicros(2000);
   Pause();
   Serial<<"Writing 1000us\r\n";
   Motor1WriteMicros(1000);
   Pause();
   Serial<<"Writing 1500us\r\n";
   Motor1WriteMicros(1500);
   Pause();*/
}

ISR(INT0_vect){
  yawEncoderCount ++;
  yawDirectionFlag = digitalRead(4);
}
ISR(INT1_vect){
  pitchEncoderCount ++;
  pitchDirectionFlag = digitalRead(5);
}

void Pause(){
  while(digitalRead(6)==0){
  }//wait for the toggle
  //delayMicroseconds(100000);//debounce
  delay(1000);
}

void MotorInit(){
  DDRB |= 0x06; //set as output

  //PWM mode 14
  //top ICR1 - updated OCR at bottom
  //Clear OC1A/OC1B on Compare Match, set OC1A/OC1B at BOTTOM (non-inverting mode)
  //clock prescaler to 8
  TCCR1A = (1<<COM1A1)|(1<<COM1B1)|(1<<WGM11);
  TCCR1B = (1<<WGM12)|(1<<WGM13)|(1<<CS11);
  ICR1 = PERIOD;

}


void SBusParser(){
  if (Serial.available() > 24){
    while(Serial.available() > 0){
      inByte = Serial.read();
      switch (readState){
      case 0:
        if (inByte != 0x0f){
          while(Serial.available() > 0){//read the contents of in buffer this should resync the transmission
            inByte = Serial.read();
          }
          return;
        }
        else{
          bufferIndex = 0;
          sBusData[bufferIndex] = inByte;
          sBusData[24] = 0xff;
          readState = 1;
        }
        break;
      case 1:
        bufferIndex ++;
        sBusData[bufferIndex] = inByte;
        if (bufferIndex < 24 && Serial.available() == 0){
          readState = 0;
        }
        if (bufferIndex == 24){
          readState = 0;
          if (sBusData[0]==0x0f && sBusData[24] == 0x00){
            newRC = true;
          }
        }
        break;
      }
    }
  }  

  if (newRC == true){
    rcCommands.values.aileron  = constrain(((sBusData[1]|sBusData[2]<< 8) & 0x07FF),352,1695) ;
    rcCommands.values.aileron  = (rcCommands.values.aileron  - 352) * 0.7446 + 1000;
    rcCommands.values.elevator  = constrain(((sBusData[2]>>3|sBusData[3]<<5) & 0x07FF),352,1695);
    rcCommands.values.elevator  = (rcCommands.values.elevator  - 352) * 0.7446 + 1000;
    //only using aileron and elevator
    /*rcCommands.values.throttle  = constrain(((sBusData[3]>>6|sBusData[4]<<2|sBusData[5]<<10) & 0x07FF),352,1695);
     rcCommands.values.throttle  = (rcCommands.values.throttle  - 352) * 0.7446 + 1000;
     rcCommands.values.rudder  = constrain(((sBusData[5]>>1|sBusData[6]<<7) & 0x07FF),352,1695);
     rcCommands.values.rudder  = (rcCommands.values.rudder  - 352) * 0.7446 + 1000;
     rcCommands.values.gear = constrain(((sBusData[6]>>4|sBusData[7]<<4) & 0x07FF),352,1695);
     rcCommands.values.gear  = (rcCommands.values.gear  - 352) * 0.7446 + 1000;
     rcCommands.values.aux1 = constrain(((sBusData[7]>>7|sBusData[8]<<1|sBusData[9]<<9) & 0x07FF),352,1695);
     rcCommands.values.aux1  = (rcCommands.values.aux1  - 352) * 0.7446 + 1000;
     rcCommands.values.aux2  = constrain(((sBusData[9]>>2|sBusData[10]<<6) & 0x07FF),352,1695);
     rcCommands.values.aux2  = (rcCommands.values.aux2  - 352) * 0.7446 + 1000;
     rcCommands.values.aux3  = constrain(((sBusData[10]>>5|sBusData[11]<<3) & 0x07FF),352,1695);
     rcCommands.values.aux3  = (rcCommands.values.aux3  - 352) * 0.7446 + 1000;*/
    if (rcCommands.values.aileron < 1560 && rcCommands.values.aileron > 1540){
      yawSetPoint = 0;
    }
    else{
      MapVar(&rcCommands.values.aileron,&yawSetPoint,1000.0,2000.0,-300.0,300.0);
    }
    if (rcCommands.values.elevator < 1560 && rcCommands.values.elevator > 1540){
      pitchSetPoint = 0;
    }
    else{
      MapVar(&rcCommands.values.elevator,&pitchSetPoint,1000.0,2000.0,-300.0,300.0);

    }
  }

}

void MapVar (float *x, float *y, float in_min, float in_max, float out_min, float out_max){
  *y = (*x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
void MapVar (int16_t *x, float *y, float in_min, float in_max, float out_min, float out_max){
  *y = (*x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void MapVar (uint16_t *x, float *y, float in_min, float in_max, float out_min, float out_max){
  *y = (*x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
void GyroInit(){
  GyroSSLow();
  SPI.transfer(L3G_CTRL_REG2 | WRITE | SINGLE);
  SPI.transfer(0x00); //high pass filter disabled
  GyroSSHigh();

  GyroSSLow();
  SPI.transfer(L3G_CTRL_REG3 | WRITE | SINGLE);
  SPI.transfer(0x00); //not using interrupts
  GyroSSHigh();

  GyroSSLow();
  SPI.transfer(L3G_CTRL_REG4 | WRITE | SINGLE);
  SPI.transfer(0x20); //2000dps scale
  GyroSSHigh();

  GyroSSLow();
  SPI.transfer(L3G_CTRL_REG5 | WRITE | SINGLE);
  SPI.transfer(0x02); //out select to use the second LPF
  GyroSSHigh();

  GyroSSLow();
  SPI.transfer(L3G_CTRL_REG1 | WRITE | SINGLE);
  SPI.transfer(0xCF); //fastest update rate 30Hz cutoff
  GyroSSHigh();
  //this section takes an average of 500 samples to calculate the offset
  //if this step is skipped the IMU will still work, but this simple step gives better results
  offsetY = 0;
  offsetZ = 0;
  for (j = 0; j < 500; j ++){//give the internal LPF time to warm up
    GetGyro();
    delay(3);
  }
  for (j = 0; j < 500; j ++){//give the internal LPF time to warm up
    GetGyro();
    gyroSumY += gyro.v.y;
    gyroSumZ += gyro.v.z;
    delay(3);
  }
  offsetY = gyroSumY / 500.0;
  offsetZ = gyroSumZ / 500.0;

}

void GetGyro(){
  SPI.setDataMode(SPI_MODE0);
  GyroSSLow();
  SPI.transfer(L3G_OUT_Y_L  | READ | MULTI);
  for (i = 0; i < 4; i++){//the endianness matches as does the axis order
    gyro.buffer[i] = SPI.transfer(0x00);
  }
  GyroSSHigh();

  degreeGyroY = (gyro.v.y - offsetY) * 0.07;
  degreeGyroZ = (gyro.v.z - offsetZ) * 0.07;

}


