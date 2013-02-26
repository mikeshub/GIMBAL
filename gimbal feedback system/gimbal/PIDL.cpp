#include "PIDL.h"

PID::PID(float *set, float *act, float *adj,boolean *intToggle,float *p, float *i, float *d,float *delta,float iLim,float lim){
  setPoint = set;
  actual = act;
  adjustment = adj;
  integrate = intToggle;
  kp = p;
  ki = i;
  kd = d;
  integralLimitHigh = iLim;
  integralLimitLow = -1*iLim;
  outputLimitHigh = lim;
  outputLimitLow = -1*lim;
  dt = delta;
  prevActual = 0;

}

void PID::calculate(){
  error = *setPoint - *actual;

  if (*integrate == true){
    iError += *ki * *dt * error;
  }
  if (iError > integralLimitHigh){
    iError = integralLimitHigh;
  }
  if (iError < integralLimitLow){
    iError = integralLimitLow;
  }
  dError = (error - prevError) / *dt;
  
  *adjustment = *kp * error  + iError + *kd * dError;

  if (*adjustment > outputLimitHigh){
    *adjustment  = outputLimitHigh;
  }
  if (*adjustment < outputLimitLow){
    *adjustment = outputLimitLow;
  }
  prevError = error;
}

void PID::reset(){
  error = 0;
  iError = 0;
  dError = 0;
  *adjustment = 0;
  prevActual = *actual;
  prevError =0;
}

