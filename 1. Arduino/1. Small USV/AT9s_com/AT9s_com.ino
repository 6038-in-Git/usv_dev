//byte PWM_PIN = 6;
byte PWM_PIN_2 = 5;
byte PWM_PIN_3 = 6;
byte PWM_PIN_4 = 9;
byte PWM_PIN_5 = 10;

//int pwm_value;
int pwm_value_2;
int pwm_value_3;
int pwm_value_4;
int pwm_value_5;

void setup() {
  //pinMode(PWM_PIN, INPUT);
  pinMode(PWM_PIN_2, INPUT);
  pinMode(PWM_PIN_3, INPUT);
  pinMode(PWM_PIN_4, INPUT);
  pinMode(PWM_PIN_5, INPUT);
  Serial.begin(115200); 
}

void loop() {

  //pwm_value = pulseIn(PWM_PIN, HIGH);
  //Serial.print(" ");
  //Serial.print(pwm_value);

  pwm_value_2 = pulseIn(PWM_PIN_2, HIGH);
  Serial.print(" ");
  Serial.print(pwm_value_2);

  //pwm_value_3 = pulseIn(PWM_PIN_3, HIGH);
  //Serial.print(" ");
  //Serial.print(pwm_value_3);

  //pwm_value_4 = pulseIn(PWM_PIN_4, HIGH);
  //Serial.print(" ");
  //Serial.print(pwm_value_4);

  //pwm_value_5 = pulseIn(PWM_PIN_5, HIGH);
  //Serial.print(" ");
  //Serial.print(pwm_value_5);

  Serial.println();
  
}
