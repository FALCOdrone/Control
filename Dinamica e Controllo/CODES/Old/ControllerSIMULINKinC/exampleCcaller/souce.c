#include "header.h"

double gVec[5];
double sum=0;

void cumulativeAdder(void){
    int idx = 0, tmpSum=0;

    for(;idx<5;idx++){
        tmpSum = tmpSum + gVec[idx];
    }
    sum = tmpSum; //function write to sum as OUTPUT
}
