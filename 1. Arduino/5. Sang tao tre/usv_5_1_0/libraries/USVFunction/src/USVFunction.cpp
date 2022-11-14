#include "Arduino.h"
#include "USVFunction.h"
#include "Servo.h"

USVFunction::USVFunction() {
}
float USVFunction::map_float( float val, float min1, float max1, float min2, float max2)
{
  float val_2;
  if (val<min1)
    val = min1;
  if (val>max1)
    val = max1; 
  val_2 = min2 + (val-min1)*(max2-min2)/(max1-min1);
  return val_2;
}
void USVFunction::motor_control(float _thrust, float _moment)
{
  int speed_1 = _thrust + _moment; // PWM output, range 0-255
  int speed_2 = _thrust - _moment;
  if (speed_1>180) speed_1 = 180;
  if (speed_2>180) speed_2 = 180;
  if (speed_1<0) speed_1 = 0;
  if (speed_2<0) speed_2 = 0;
  Serial.print("Motor: ");
  Serial.print(speed_1);
  Serial.print(" ");
  Serial.print(speed_2);
  Serial.print("; ");
  f_1.write(speed_1);
  f_2.write(speed_2);
}
