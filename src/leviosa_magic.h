
#include <arduino.h>
//#include "configuration.h"
#include "regler.h"



void read_distance();


void sendToPC(int* data);


void sendToPC(float* data);


void sendToPC(int* data1, int* data2, int* data3, int* data4);

 
void sendToPC(float* data1, float* data2, float* data3, float* data4);


void sendToPC(float* data1, float* data2, float* data3, float* data4, float* data5, float* data6, float* data7, float* data8);

void parseData();

void getSerialData();

