/*************************************************** 
  This is an example for our Adafruit 16-channel PWM & Servo driver
  GPIO test - this will set a pin high/low

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/products/815

  These drivers use I2C to communicate, 2 pins are required to  
  interface.

  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/
   
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// called this way, it uses the default address 0x40
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
// you can also call it with a different address you want
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);
// you can also call it with a different address and I2C interface
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, Wire);

// TIME
  //double current_time = 0;

// MODE CHOICE
  //const int pin_mode = 14;        // D5
  const int pin_mode = 13;        // D7
  bool auto_mode = false;

// MOTOR VARIABLES
  const int pin_throttle = 12;    // D6
  const int pin_turn = 15;        // D8
  //const int pin_motor_1 = 13;     // D7   
  int throttle;
  int turn;
  int thrust;
  int moment;
  int speed_1;
  int speed_2;
  
void setup() {

  // Serial
  Serial.begin(115200);

  // MODE SETUP
  //pinMode(pin_mode, INPUT);
  
  // MOTOR SETUP
  //f_1.attach(pin_motor_1,1000,2000);    
  //f_2.attach(pin_motor_2,1000,2000);   
  pinMode(pin_throttle, INPUT);         
  pinMode(pin_turn, INPUT); 

  pwm.begin();
  pwm.setPWMFreq(50);  // Set to whatever you like, we don't use it in this demo!

  // if you want to really speed stuff up, you can go into 'fast 400khz I2C' mode
  // some i2c devices dont like this so much so if you're sharing the bus, watch
  // out for this!
  Wire.setClock(400000);
}

void loop() {
  // Drive each pin in a 'wave'
    // RADIO
    throttle = pulseIn(pin_throttle, HIGH); // PWM width, range 1000 - 2000 miroseconds
    //turn = pulseIn(pin_turn, HIGH);
    //throttle = 200;
    turn = 1500;
    if (throttle<1600) throttle = 1600;
    thrust = map(throttle, 1600, 2000, 0, 180); 
    //moment = map(turn, 1000, 2000, -120, 120);
    moment = 0;

    Serial.print("throttle: ");
    Serial.print(throttle);
    Serial.print("; ");
    Serial.print("turn: ");
    Serial.print(turn);
    Serial.print("; ");
    Serial.print("thrust: ");
    Serial.print(thrust);
    Serial.print("; ");
    Serial.print("moment: ");
    Serial.print(moment);
    Serial.print("; ");

    speed_1 = thrust + 3*moment; // PWM output, range 0-255
    speed_2 = thrust - 3*moment;
    if (speed_1>180) speed_1 = 200;
    if (speed_2>180) speed_2 = 200;
    if (speed_1<0) speed_1 = 0;
    if (speed_2<0) speed_2 = 0;
    Serial.print("Motor: ");
    Serial.print(speed_1);
    Serial.print(" ");
    Serial.print(speed_2);
    Serial.print("; ");

    speed_1 = map(speed_1, 0, 255, 4096/18, 4096/10);
    speed_2 = map(speed_2, 0, 255, 4096/18, 4096/10);
    Serial.print(speed_1);
    Serial.print(" ");
    Serial.print(speed_2);
    Serial.print("; ");

    pwm.setPWM(0, 0, speed_1);       // turns pin fully on
    pwm.setPWM(1, 0, speed_2);       // turns pin fully off

    Serial.println();
}
