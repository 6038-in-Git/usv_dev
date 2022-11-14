#ifndef USVFunction_h
#define USVFunction_h
#include "Arduino.h" 
#include "Servo.h"
class USVFunction {
// class code goes here
public:
	Servo f_1;
  	Servo f_2;
    	USVFunction();
	float map_float(float val, float min1, float max1, float min2, float max2);
	void motor_control(float _thrust, float _moment);
private:
};
#endif