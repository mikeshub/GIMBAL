#include <Streaming.h>

#define GEAR_RATIO 3092.783505154639f//currently the motor drive ratio of 9.7:1 * 30,000 - 
//30,000 is to simplfly the calculations

unt32_t previousControlTime;
int32_t encoderSpeed;
int32_t encoderCount;
uint32_t DT;
boolean directionFlag;

void setup(){
  DDRD = 0;//pin 2,3,4,5
  PORTD |= 0x3C;//turn on pullups
  EICRA |= 0x03;//rising edge for both interrupts
  EIMSK |= 0x03;//enable the interrupts

  Serial.begin(115200);
  //attachInterrupt(0,YawEncoderRead,RISING);//pin 2 corresponds with interrupt 0

  
  previousControlTime = millis();
}

void loop(){
  if (millis() - previousControlTime >= 10){
    cli();//disable interrupts for calculations
    DT = millis() - previousControlTime;
    encoderSpeed = ( encoderCount * GEAR_RATIO) / (float)DT;
    if (directionFlag != true){
      encoderSpeed *= -1.0;
    }
    encoderCount = 0;
    previousControlTime = millis();
    sei();//enable the interrupts
  }
  Serial<<encoderSpeed<<"\r\n";
}

ISR(INT0_vect){
  encoderCount ++;
  directionFlag = digitalRead(4);
}
