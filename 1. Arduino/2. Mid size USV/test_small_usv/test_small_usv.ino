#include <Servo.h>

Servo f_1;
Servo f_2;   

int throttle;
int turn;

int thrust;
int moment;

int speed_1;
int speed_2;

void setup() {

  f_1.attach(4,1000,2000);
  f_2.attach(14,1000,2000); 
  pinMode(12, INPUT);
  pinMode(15, INPUT);
  Serial.begin(115200);
}

void loop() {
  
  throttle = pulseIn(12, HIGH); // PWM width, range 1000 - 2000 miroseconds
  Serial.print(throttle);
  Serial.print("  ");
   
  turn = pulseIn(15, HIGH);
  Serial.print(turn);
  Serial.print("  ");
  
  if (throttle<1400) throttle = 1400;
  thrust = map(throttle, 1500, 2000, 0, 180); 
  moment = map(turn, 1000, 2000, -50, 50);

  speed_1 = thrust + moment/2 - 10; // PWM output, range 
  
  speed_2 = thrust - moment/2;
  
  if (speed_1>180) speed_1 = 180;
  if (speed_2>180) speed_2 = 180;
  if (speed_1<0) speed_1 = 0;
  if (speed_2<0) speed_2 = 0;
  
  f_1.write(speed_1);
  f_2.write(speed_2);
  Serial.print(speed_1);
  Serial.print(" ");
  Serial.println(speed_2);
     
}
