#pragma once

#include "Arduino.h"
//#include "configuration.h"
#include <PID_v1.h>
#include <Adafruit_ADS1015.h>

class Regler
{
    public:
        Regler(int *K_Kurve,int zone,double* Input, double* Output,double* sollwert, double P,double I, double D);
        void init(Adafruit_ADS1015 &ads_reference,int bias);
        void calculate_output();
        void setTunings(double Kp,double Ki,double Kd,int POn);
        void setTunings(double Kp,double Ki,double Kd);
        void setMode(int mode);
        void setcurrentPosition(double position);
        void setBias(int);
    private:
        int *kalibrierkurve;
        int adc2um(int);
        int readadc();
        int _zone;
        double _offset=0;
        double _Bias;
        double* _Sollwert, *_Input, *_Output;
        Adafruit_ADS1015 ads;
        PID pid;
};
