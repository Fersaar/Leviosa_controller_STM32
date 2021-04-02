#pragma once

#include <Arduino.h>

#define DEBUG false
#define RAWDATA false
#define ADC false

// Pins für IO

#define Pin_Out_Zone0 PA6
#define Pin_Out_Zone1 PA7
#define Pin_Out_Zone2 PB0
#define Pin_Out_Zone3 PB1
#define Pin_Out_GND PA1

#define Pin_In_Zone0 0
#define Pin_In_Zone1 1
#define Pin_In_Zone2 2
#define Pin_In_Zone3 3
//#define sensepin PA1

//Kalibrierung
#define K_Punkte 101 // Anzahl an Kalibrierpunkten

// Regelintervall

#define interval 5           // interval (milliseconds), wenn zu schnell kommt PC nicht mehr hinterher


//PID

#define Output_max 4096
#define Output_min 0
#define ProponMeasure P_ON_E // P_ON_M
#define default_P 1
#define default_I 1
#define default_D 1

//Bias Spannung
//Spannung bei der elektrostatische Kraft im Sollabstand der Gewichtskraft entspricht
// U = sqrt((m*g)/(epsilon*A))*Sollabstand
// -> U = K_Bias * Sollabstand
// Sollabstand in um -> 1E-3
#define MASSE 0.00017199 //kg
#define G 9.81 //N/kg
#define EPSILON 8.85E-12
#define AREA_EFF 3981E-6 *1E6 //1E-6 von Meter in Mikrometer (+6 durch Quadrat und Quotient)
