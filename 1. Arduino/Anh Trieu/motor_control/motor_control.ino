const float pi = 3.14159265359;
const float k_fix_lat = 113894;
const float k_fix_lng = 104225;
const float a = 20;
const float b = 20;
const float k_moment = (400/pi)*(100/255);
const float cruise = 90*(100/255);

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}

void motor_control (float usv_lat, float usv_lng, float target_lat, float target_lng, float compass_reading)
{
  int speed_1 = 0;
  int speed_2 = 0;
  float desire_yaw = 0;
  float usv_yaw = 0;
  float fix_lat_error = 0;
  float fix_lng_error = 0;

  // ------------FIX YAW ------------
  if (compass_reading<=180)
    usv_yaw = map_float(compass_reading, 0, 180, 0, pi);            
  if (compass_reading>180)
  {
    usv_yaw = map_float(compass_reading, 180, 360, -pi, 0);           // range -pi ~ pi
    compass_reading = map_float(compass_reading, 180, 360, -180, 0);  // range -180 ~ 180
  }

  fix_lat_error = k_fix_lat * (target_lat - usv_lat); 
  fix_lng_error = k_fix_lng * (target_lng - usv_lng);
  desire_yaw = atan2(fix_lng_error, fix_lat_error);                   // range -pi ~ pi
  
  yaw_error = desire_yaw - usv_yaw;                                   // range -2pi ~ 2pi
  
  // ------------FIX YAW ERROR------------
  if (yaw_error > pi)
  yaw_error = yaw_error - 2*pi;
  if (yaw_error < - pi)
  yaw_error = yaw_error + 2*pi;                                       // range -pi ~ pi
  
  // ------------GET DISTANCE------------
    distance = pow( (pow(fix_lat_error,2) + pow(fix_lng_error,2)), 0.5);

  // ------------DESIGN MOTOR THRUST------------
  k_yaw_error = 1 / (1 + a*pow(yaw_error,2));                   // range 1 ~ 0
  k_distance = pow(distance,2)/(pow(distance,2) + b);           // range 0 ~ 1
  thrust = k_yaw_error * k_distance * cruise;                   // range 0 ~ cruise
  moment = k_moment * yaw_error;

  speed_1 = _thrust + _moment;                                  // PWM output, range 0-100
  speed_2 = _thrust - _moment;
  if (speed_1>100) speed_1 = 100;
  if (speed_2>100) speed_2 = 100;
  if (speed_1<0) speed_1 = 0;
  if (speed_2<0) speed_2 = 0;
  
}
