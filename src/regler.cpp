
#include<Arduino.h>
#include<PID_v1.h> //https://github.com/br3ttb/Arduino-PID-Library/
#include<Adafruit_ADS1015.h>
#include "regler.h"
#include "configuration.h"


Regler::Regler(int *K_Kurve, int zone, double* Input, double* Output, double* sollwert, double P, double I, double D)
  : pid(Input, Output, sollwert, P, I, D, ProponMeasure, REVERSE)
{
  kalibrierkurve = K_Kurve;
  _Input = Input;
  _Output = Output;
  _Sollwert = sollwert;
  _zone = zone;
  pid.SetMode(AUTOMATIC);
  pid.SetSampleTime(interval);
}

void Regler::init(Adafruit_ADS1015 &ads_reference, int bias) {
  ads = ads_reference;
  _Bias = bias;
  pid.SetOutputLimits(-bias,Output_max-bias);

}

void Regler::calculate_output() {
  
  #if (RAWDATA)
    *_Input =readadc();
  #else
    *_Input = adc2um(readadc());
    pid.Compute();
    *_Output+= _Bias;
  #endif
  //*_Input = readadc();
  //pid.Compute();
  //wert = readadc();
  /*else if ( _zone == 1)  {
    wert = analogRead(PA6);
    *_Input = wert;
  }
   else if ( _zone == 2)  {
    wert = analogRead(PA7);
    *_Input = wert;
  }*/
  //*myInput = readadc();
  //*_Input = adc2um(wert*32);
  //*_Input = wert;
  // pid.Compute();*/
  return;
}

int Regler::readadc() {
  #if(ADC)
    return ads.readADC_SingleEnded(_zone);
  #else
    return random(0,1000);
  #endif
  
}

int Regler::adc2um(int adc) {

  int oldraw = kalibrierkurve[0];
  int olddist = kalibrierkurve[1];
  int newraw, newdist = 0;
  if (adc <oldraw){
    return 0;
  }
  for (int i = 1; i < K_Punkte; i++) {
    newraw = kalibrierkurve[2 * i];
    newdist = kalibrierkurve[2 * i + 1];
    if (newraw > adc) {
      return (olddist + (float)(adc - oldraw) * (float)(newdist - olddist) / (newraw - oldraw)) + _offset;
    }
    olddist = newdist;
    oldraw = newraw;
  }
  return kalibrierkurve[K_Punkte*2-1];
}

void Regler::setTunings(double Kp, double Ki, double Kd, int POn) {
  pid.SetTunings(Kp, Ki, Kd, POn);
}

void Regler::setTunings(double Kp, double Ki, double Kd) {
  pid.SetTunings(Kp, Ki, Kd);
}

void Regler::setMode(int mode) {
  pid.SetMode(mode);
}

void Regler::setcurrentPosition(double position) {
  if (position <0){
    _offset = 0;
  }
  else{
     _offset = position - (*_Input-_offset);   
  }
 

}

void Regler::setBias(int bias){
  _Bias = bias;
  pid.SetOutputLimits(bias, Output_max-bias);
}
