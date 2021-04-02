
#include <arduino.h>
#include <Wire.h>
#include <Adafruit_ADS1015.h>
#include "configuration.h"
#include "regler.h"
#include "leviosa_magic.h"
#include "kalibierkurve.h"

double output[5]={1,2,3,4,0};
double input[4]={0,0,0,0};
double Sollwert=1000;      // Sollabstand in Mikrometer
double nullpunktoffset =0;
double P,I,D;
int PoE;
float amplitude;
bool Overwrite = false;
double K_Bias = sqrt((2*MASSE*G)/(EPSILON*AREA_EFF));
double bias = 0;

// Intervallsteuerung
unsigned long previousMillis = 0; // Zeitpunkt der letzten Ausführung (milliseconds)


// einkommende Steuersignale

String recvString = "";
bool newData = false;
byte ndx = 0;
double g_offset = 0.5;

Regler zone0 = Regler(Kalibrierkurve1,0,&input[0],&output[0],&Sollwert,default_P,default_I,default_D);
Regler zone1 = Regler(Kalibrierkurve2,1,&input[1],&output[1],&Sollwert,default_P,default_I,default_D);
Regler zone2 = Regler(Kalibrierkurve3,2,&input[2],&output[2],&Sollwert,default_P,default_I,default_D);
Regler zone3 = Regler(Kalibrierkurve4,3,&input[3],&output[3],&Sollwert,default_P,default_I,default_D);
Adafruit_ADS1015 ads;

HardwareTimer timer(3);
//HardwareTimer timeru2(2);

void setup() {
  Serial.begin(38400);
  #if DEBUG
    Serial1.begin(115200);
    delay(5);
    Serial1.println("STM32 gestartet");
  #endif


 // Wire.begin();
  ads.begin();
  bias = K_Bias * Sollwert;
  zone0.init(ads,bias);
  zone1.init(ads,bias);
  zone2.init(ads,bias);
  zone3.init(ads,bias);
  //analogWriteResolution(16);
  //analogWriteFrequency(10000);
  //https://github.com/stm32duino/STM32Examples/tree/master/examples/Peripherals/HardwareTimer
  //timer.setPeriod(60);
  timer.setPrescaleFactor(1);
  timer.setOverflow(4320);
  pinMode(Pin_Out_Zone0,PWM);
  pinMode(Pin_Out_Zone1,PWM);
  pinMode(Pin_Out_Zone2,PWM);
  pinMode(Pin_Out_Zone3,PWM);
  pinMode(Pin_Out_GND,PWM);
  Serial.println("STM32 gestartet");
  
  //pinMode(sensepin,PWM);
  //digitalWrite(sensepin,HIGH);
  //pwmWrite(sensepin,8000);
  
  
}

void loop() {
  
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    zone0.calculate_output();
    zone1.calculate_output();
    zone2.calculate_output();
    zone3.calculate_output();
    pwmWrite(Pin_Out_Zone0,output[0]);
    pwmWrite(Pin_Out_Zone1,output[1]);
    pwmWrite(Pin_Out_Zone2,output[2]);
    pwmWrite(Pin_Out_Zone3,output[3]);
    float val0 = input[0];
    float val1 = input[1];
    float val2 = input[2];
    float val3 = input[3];
    float out0 = float(output[0]);
    float out1 = float(output[1]);
    float out2 = float(output[2]);
    float out3 = float(output[3]);
    float out4 = float(output[4]);
    sendToPC(&val0,&val1, &val2,&val3,&out0,&out1,&out2,&out3,&out4);
  }
  getSerialData();
  if (newData){
    parseData();
  }
}


void sendToPC(int* data)
{
  byte* byteData = (byte*)(data);
  Serial.write(byteData, 2);
}

void sendToPC(float* data)
{
  byte* byteData = (byte*)(data);
  Serial.write(byteData, 4);
}

void sendToPC(float* data1, float* data2, float* data3, float* data4, float* data5, float* data6, float* data7, float* data8,float* data9)
{
  byte* byteData1 = (byte*)(data1);
  byte* byteData2 = (byte*)(data2);
  byte* byteData3 = (byte*)(data3);
  byte* byteData4 = (byte*)(data4);
  byte* byteData5 = (byte*)(data5);
  byte* byteData6 = (byte*)(data6);
  byte* byteData7 = (byte*)(data7);
  byte* byteData8 = (byte*)(data8);
  byte* byteData9 = (byte*)(data9);
  byte buf[36] = {byteData1[0], byteData1[1], byteData1[2], byteData1[3],
                 byteData2[0], byteData2[1], byteData2[2], byteData2[3],
                 byteData3[0], byteData3[1], byteData3[2], byteData3[3],
                 byteData4[0], byteData4[1], byteData4[2], byteData4[3],
                 byteData5[0], byteData5[1], byteData5[2], byteData5[3],
                 byteData6[0], byteData6[1], byteData6[2], byteData6[3],
                 byteData7[0], byteData7[1], byteData7[2], byteData7[3],
                 byteData8[0], byteData8[1], byteData8[2], byteData8[3],
                 byteData9[0], byteData9[1], byteData9[2], byteData9[3]};
  Serial.write(buf, 36);
}

void getSerialData()
{
  char input;
  while (Serial.available())
  {
    input = Serial.read();
    /*#ifdef DEBUG
    Serial1.println(input);
    #endif*/
    if (input == '%') // this is the end of message marker
    {
      ndx = 0;
      newData = true;
      break;
    }

    recvString+=input;
  }
}

void parseData(){
  #if DEBUG
    Serial1.println(recvString);
  #endif
  
  int typ = recvString.substring(0,2).toInt();
  switch (typ){
    case 0: // Sollwert setzen
      Sollwert = recvString.substring(2,8).toInt();
      bias = K_Bias * Sollwert;
      zone0.setBias(bias);
      zone1.setBias(bias);
      zone2.setBias(bias);
      zone3.setBias(bias);
      #if DEBUG
        Serial1.print("Sollwert: ");
        Serial1.println(Sollwert);
      #endif
      break;
    case 1:   // Nullpunkt offset
      nullpunktoffset = recvString.substring(2,8).toInt();
      zone0.setcurrentPosition(nullpunktoffset); 
      zone1.setcurrentPosition(nullpunktoffset); 
      zone2.setcurrentPosition(nullpunktoffset); 
      zone3.setcurrentPosition(nullpunktoffset); 
      #if DEBUG
        Serial1.print("Nullpunktoffset: ");
        Serial1.println(nullpunktoffset);
      #endif
      break;
    case 2:   // PID Werte
      P = recvString.substring(2,9).toFloat();
      I = recvString.substring(9,16).toFloat();
      D = recvString.substring(16,23).toFloat();
      PoE = recvString.substring(23,30).toInt();
      zone0.setTunings(P,I,D,PoE);
      zone1.setTunings(P,I,D,PoE);
      zone2.setTunings(P,I,D,PoE);
      zone3.setTunings(P,I,D,PoE);
      #if DEBUG
        Serial1.print("PID Werte bekommen, P: ");
        Serial1.print(P);
        Serial1.print(" I: ");
        Serial1.print(I);
        Serial1.print(" D: ");
        Serial1.print(D);
        Serial1.print(" PoE: ");
        Serial1.println(PoE);
      #endif
      break;
    case 3:     // overwrite
      zone0.setMode(0);
      zone1.setMode(0);
      zone2.setMode(0);
      zone3.setMode(0);
      output[0]=recvString.substring(2,8).toInt();
      output[1]=recvString.substring(8,14).toInt();
      output[2]=recvString.substring(14,20).toInt();
      output[3]=recvString.substring(20,26).toInt();
      output[4]=recvString.substring(26,32).toInt();
	    pwmWrite(Pin_Out_Zone0,output[0]);
      pwmWrite(Pin_Out_Zone1,output[1]);
      pwmWrite(Pin_Out_Zone2,output[2]);
      pwmWrite(Pin_Out_Zone3,output[3]);
      pwmWrite(Pin_Out_GND,output[4]);
      #if DEBUG
        Serial1.print("Overwrite aktiviert, Zone0: ");
        Serial1.print(output[0]);
        Serial1.print(" Zone1: ");
        Serial1.print(output[1]);
        Serial1.print(" Zone2: ");
        Serial1.print(output[2]);
        Serial1.print(" Zone3: ");
        Serial1.println(output[3]);
        Serial1.print(" GND: ");
        Serial1.println(output[4]);
      #endif
      break;
    case 4:   // automatik modus aktivieren
      output[0]=0;
      output[1]=0;
      output[2]=0;
      output[3]=0;
      zone0.setMode(1);
      zone1.setMode(1);
      zone2.setMode(1);
      zone3.setMode(1);
      #if DEBUG
        Serial1.println("Automatikmodus aktiviert");
      #endif
      break;
    case 5: // aus
      zone0.setMode(0);
      zone1.setMode(0);
      zone2.setMode(0);
      zone3.setMode(0);  
      output[0]=0;
      output[1]=0;
      output[2]=0;
      output[3]=0;
      output[4]=0;
	    pwmWrite(Pin_Out_Zone0,output[0]);
      pwmWrite(Pin_Out_Zone1,output[1]);
      pwmWrite(Pin_Out_Zone2,output[2]);
      pwmWrite(Pin_Out_Zone3,output[3]);
      pwmWrite(Pin_Out_GND,output[4]);
      #if DEBUG
        Serial1.println("Zonen abgeschaltet");
      #endif
      break;
    case 6: //aus mit sweep
      #if DEBUG
        Serial1.println("Zonen abgeschaltet mit Sweep");
      #endif
	    zone0.setMode(0);
      zone1.setMode(0);
      zone2.setMode(0);
      zone3.setMode(0);  
      amplitude = 4200;
      while (amplitude>0){
        pwmWrite(Pin_Out_Zone0,0);
        pwmWrite(Pin_Out_Zone1,0);
        pwmWrite(Pin_Out_Zone2,0);
        pwmWrite(Pin_Out_Zone3,0);
        pwmWrite(Pin_Out_GND,amplitude);
        delay(5);
        pwmWrite(Pin_Out_GND,0);
        pwmWrite(Pin_Out_Zone0,amplitude);
        pwmWrite(Pin_Out_Zone1,amplitude);
        pwmWrite(Pin_Out_Zone2,amplitude);
        pwmWrite(Pin_Out_Zone3,amplitude);
        amplitude -=4.2;
        delay(5);
        /*
        pwmWrite(Pin_Out_Zone1,amplitude);
        pwmWrite(Pin_Out_Zone0,amplitude);
        float val0 = 0;
        
        sendToPC(&val0,&val0, &val0,&val0,&amplitude,&val0,&val0,&val0);
        delay(5);
        pwmWrite(Pin_Out_Zone0,0);
        pwmWrite(Pin_Out_Zone1,amplitude);*/
        amplitude -=4.2;
        delay(5);
      }
      output[0]=0;
      output[1]=0;
      output[2]=0;
      output[3]=0;
      output[4]=0;
	    pwmWrite(Pin_Out_Zone0,output[0]);
      pwmWrite(Pin_Out_Zone1,output[1]);
      pwmWrite(Pin_Out_Zone2,output[2]);
      pwmWrite(Pin_Out_Zone3,output[3]);
      pwmWrite(Pin_Out_Zone3,output[4]);
      break;
    case 7: // full on
      zone0.setMode(0);
      zone1.setMode(0);
      zone2.setMode(0);
      zone3.setMode(0);  
      output[0]=Output_max;
      output[1]=Output_max;
      output[2]=Output_max;
      output[3]=Output_max;
	    pwmWrite(Pin_Out_Zone0,output[0]);
	    pwmWrite(Pin_Out_Zone1,output[1]);
      pwmWrite(Pin_Out_Zone2,output[2]);
      pwmWrite(Pin_Out_Zone3,output[3]);
      #if DEBUG
        Serial1.println("Zonen full on");
      #endif
	    break;
	
  }
  recvString="";
  newData = false;

}
