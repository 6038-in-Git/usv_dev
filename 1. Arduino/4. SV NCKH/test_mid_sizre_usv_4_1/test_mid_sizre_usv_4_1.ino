
// -------------------------------CODE for ESP8266 Nodemcu-------------------------------

#include <Servo.h>
#include <QMC5883LCompass.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

// TIME
  //double current_time = 0;

// MODE CHOICE
  //const int pin_mode = 14;      // D5
  const int pin_mode = 13;      // D7
  bool auto_mode = false;

// MOTOR VARIABLES
  const int pin_throttle = 12;  // D6
  const int pin_turn = 15;      // D8
  //const int pin_motor_1 = 13;   // D7
  const int pin_motor_2 = 16;   // D0
  const int pin_motor_1 = 14;   // D5
   
  Servo f_1;
  Servo f_2;   
  int throttle;
  int turn;
  int thrust;
  int moment;
  int speed_1;
  int speed_2;

// COMPASS
  QMC5883LCompass compass;
  float compass_reading;          // Arduino reads QMC5883l, then send it to ESP8226 via Serial
  

// GPS
  const int RXPin = 0;          // D3
  const int TXPin = 2;          // D4
  const uint32_t GPSBaud = 9600;
  TinyGPSPlus gps;
  SoftwareSerial ss(RXPin, TXPin);

// UDP WIFI
  //const char* ssid = "MERCURY_2.4G_0122";
  //const char* password = "hch4pw7n";
  const char* ssid = "KTHK 207";
  const char* password = "hmt123456789xyz";
  const char* end_char = "; ";
  const char* terminate_char = "~";
  
  WiFiUDP Udp;
  unsigned int localUdpPort = 4210;  // local port to listen on
  char incomingPacket[255];  // buffer for incoming packets
  char  replyPacket[] = "USV Packet: ";  // a reply string to send back

// UDP char
  const char mode_char[] = "Mode: ";
  const char target_char[] = "Target: ";
  const char target_char_1234[] = "4pts: ";
  const char usv_char[] = "USV: ";
  const char valid_char[] = "Valid: ";
  const char arrival_char[] = "Arrival: ";
  const char yaw_char []= "Yaw: "; 
  const char desire_yaw_char []= "Desire_yaw: ";
  const char distance_char [] = "Distance: ";
  const char k_yaw_char [] = "k_yaw_e: ";
  const char k_distance_char [] = "k_distance: ";
  const char motor_char[] = "Motor: ";
  

// TRACKING PARAMETERS
  float distance_12 = 0;
  const float step_12 = 1;
  int step_no = 0;
  
  int current_target = 0;
  bool switch_target = 0;
  double arrival_start_time = 0;
  bool end_zigzag = false;
  
  double target_lat = 0;
  double target_lng = 0;
  double target_lat_1 = 0;
  double target_lng_1 = 0;
  double target_lat_2 = 0;
  double target_lng_2 = 0;
  double target_lat_3 = 0;
  double target_lng_3 = 0;
  double target_lat_4 = 0;
  double target_lng_4 = 0;
  double temp;
  double usv_lat = 0;
  double usv_lng = 0;
  bool valid = false;

// CONTROL PARAMETER
  float usv_yaw = 0;
  float fix_lat_error = 0;
  float fix_lng_error = 0;
  float yaw_error = 0;
  float desire_yaw = 0;
  float distance = 0;
  float k_yaw_error = 0;
  float k_distance = 0;
  bool arrival = false;
  bool arrival_prev = false;
  bool arrival_prev_prev = false;
  float temp2;
  
  const float pi = 3.14159265359;
  const float k_fix_lat = 113894;
  const float k_fix_lng = 104225;
  const float a = 20;
  const float b = 20;
  const float k_moment = 400/pi;
  const float cruise = 90;
  const float arrival_radius = 5;
  
  
//----------------------------------------------------------------------------    SETUP   -----------------------------------------------------------------------------
//-----------------------------------------------------------------------------***-----***----------------------------------------------------------------------------- 
void setup() {

  // Serial
  Serial.begin(115200);

  // MODE SETUP
  //pinMode(pin_mode, INPUT);
  
  // MOTOR SETUP
  f_1.attach(pin_motor_1,1000,2000);    
  f_2.attach(pin_motor_2,1000,2000);   
  pinMode(pin_throttle, INPUT);         
  pinMode(pin_turn, INPUT); 

  // COMPASS SETUP
  compass.init();
  
  // GPS SETUP
  ss.begin(GPSBaud);

  // UDP WIFI
  Serial.printf("Connecting to %s ", ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println(" connected");

  Udp.begin(localUdpPort);
  Serial.printf("Now listening at IP %s, UDP port %d\n", WiFi.localIP().toString().c_str(), localUdpPort);
  
}

//-----------------------------------------------------------------------------   LOOP    -----------------------------------------------------------------------------
//-----------------------------------------------------------------------------***-----***-----------------------------------------------------------------------------
void loop(){

  // ----------------------------------------------- READING ----------------------------------------------- 

  // UDP RECIEVE TARGET COORDINATES
    int packetSize = Udp.parsePacket();
    
    // receive incoming UDP packets
    int len = Udp.read(incomingPacket, 255);
    if (len > 0)
    {
      incomingPacket[len] = 0;
    }

    if (packetSize==109)
    {
      if (incomingPacket[0] == 33)
      {
        arrival_start_time = millis();
        current_target = 1;
        
        target_lat_1 = parse_target_lat(2);
        target_lng_1 = parse_target_lng(2);
  
        target_lat_2 = parse_target_lat(29);
        target_lng_2 = parse_target_lng(29);
  
        target_lat_3 = parse_target_lat(56);
        target_lng_3 = parse_target_lng(56);
  
        target_lat_4 = parse_target_lat(83);
        target_lng_4 = parse_target_lng(83);
      }
    }

    // SETTING MODE
    if (incomingPacket[0] == 33) 
      auto_mode = true;
    else
      auto_mode = false;
    Serial.print("Auto = ");
    Serial.print(auto_mode);
    Serial.print("; ");  
    
    // COMPASS
      compass.read();
      compass_reading = compass.getAzimuth();
      if (compass_reading> 270)
        compass_reading = map_float(compass_reading, 270, 360, 0, 90);
      else
        compass_reading = map_float(compass_reading, 0, 270, 90, 360);
      Serial.print("Compass: ");
      Serial.print(compass_reading);
      Serial.print("; ");
      
    // GPS
      valid = false;
      while (ss.available() > 0)
        if (gps.encode(ss.read())) 
          if (gps.location.isValid())
          {
            valid = true;
            usv_lat = gps.location.lat();
            usv_lng = gps.location.lng();
          }
          
      Serial.print("valid =  ");
      Serial.print(valid);
      Serial.print("; ");
          
      Serial.print("usv_lat =  ");
      Serial.print(usv_lat,6);
      Serial.print("; ");

      Serial.print("usv_lng =  ");
      Serial.print(usv_lng,6);
      Serial.print("; ");      

    // CALCULATE ZIGZAG STEP
        distance_12 = get_distance (target_lat_1, target_lat_2, target_lng_1, target_lng_2);
        step_no = get_step_no(distance_12, step_12);

        
    // ----------------------------------------------- CHOOSE TARGET -----------------------------------------------

      if (current_target >= ((step_no+1)*2))
        end_zigzag = true;
      Serial.print("end_zigzag =  ");
      Serial.print(end_zigzag);
      Serial.print("; ");  
      
      if ((arrival_prev>0)&&(arrival_prev_prev<1))
        arrival_start_time = millis();

      if (((millis()-arrival_start_time) > 5000)&&(arrival_prev>0))
      {
        arrival_start_time = millis();
        switch_target = true;
      }

      if ((switch_target>0)&&(end_zigzag<1))
      {
        switch_target = false;
        current_target = current_target +1;
      }
      Serial.print("current_target =  ");
      Serial.print(current_target);
      Serial.print("; "); 
        
      target_lat = get_target_list(current_target, step_no, target_lat_1, target_lat_2, target_lat_3, target_lat_4);
      target_lng = get_target_list(current_target, step_no, target_lng_1, target_lng_2, target_lng_3, target_lng_4);
      switch_target = false;

      Serial.print("target_lat =  ");
      Serial.print(target_lat,6);
      Serial.print("; ");

      Serial.print("target_lng =  ");
      Serial.print(target_lat,6);
      Serial.print("; "); 
      
    // ----------------------------------------------- GET YAW ERROR -----------------------------------------------
    //
      if (compass_reading<=180)
        usv_yaw = map_float(compass_reading, 0, 180, 0, pi);            
      if (compass_reading>180)
      {
        usv_yaw = map_float(compass_reading, 180, 360, -pi, 0);           // range -pi ~ pi
        compass_reading = map_float(compass_reading, 180, 360, -180, 0);  // range -180 ~ 180
      }
      temp2 = map_float(usv_yaw, -pi, pi, -180, 180);
      Serial.print("usv_yaw: ");
      Serial.print(temp2, 0);
      Serial.print("; "); 

      fix_lat_error = k_fix_lat * (target_lat - usv_lat); 
      fix_lng_error = k_fix_lng * (target_lng - usv_lng);
      desire_yaw = atan2(fix_lng_error, fix_lat_error);             // range -pi ~ pi
      //Serial.print("lat_e: ");
      //Serial.print(fix_lat_error, 2);
      //Serial.print("; "); 
      //Serial.print("lng_e: ");
      //Serial.print(fix_lng_error, 2);
      //Serial.print("; "); 
      
      temp2 = map_float(desire_yaw, -pi, pi, -180, 180);      
      Serial.print("yaw_d: ");
      Serial.print(temp2, 0);
      Serial.print("; "); 

      yaw_error = desire_yaw - usv_yaw;                             // range -2pi ~ 2pi

    // ------------FIX YAW ERROR------------
      if (yaw_error > pi)
        yaw_error = yaw_error - 2*pi;
      if (yaw_error < - pi)
        yaw_error = yaw_error + 2*pi;                               // range -pi ~ pi
      temp2 = map_float(yaw_error, -pi, pi, -180, 180);      
      Serial.print("yaw_e: ");
      Serial.print(temp2, 0);
      Serial.print("; "); 

    // ------------GET DISTANCE------------
      distance = pow( (pow(fix_lat_error,2) + pow(fix_lng_error,2)), 0.5);


    // ----------------------------------------------- DESIGN AUTO THRUST AND MOMENT -----------------------------------------------
    
      k_yaw_error = 1 / (1 + a*pow(yaw_error,2));                   // range 1 ~ 0
      k_distance = pow(distance,2)/(pow(distance,2) + b);           // range 0 ~ 1
      Serial.print("k_yaw_e: ");
      Serial.print(k_yaw_error, 3);
      Serial.print("; "); 
      Serial.print("k_dist: ");
      Serial.print(k_distance, 3);
      Serial.print("; "); 

    // ------------TARGET REACHED------------
      arrival = false;
      if (distance<arrival_radius)
      {
        arrival = true;
      }
 
  // ----------------------------------------------- AUTO MODE -----------------------------------------------
  if (incomingPacket[0] == 33)
  {
      thrust = k_yaw_error * k_distance * cruise;                   // range 0 ~ cruise
      moment = k_moment * yaw_error;

    // ------------CONTROL MOTOR------------
      motor_control(thrust, moment);

  }
  
  // ----------------------------------------------- MANUAL MODE -----------------------------------------------
  else
  {
    throttle = pulseIn(pin_throttle, HIGH); // PWM width, range 1000 - 2000 miroseconds
    turn = pulseIn(pin_turn, HIGH);
    
    if (throttle<1600) throttle = 1600;
    thrust = map(throttle, 1600, 2000, 0, 180); 
    moment = map(turn, 1000, 2000, -120, 120);
  
    // ------------CONTROL MOTOR------------
      motor_control(thrust, moment); 
  }

  // UDP SEND LOG DATA
    Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
    Udp.write(replyPacket);

    // Auto mode
    Udp.write(mode_char);
    udp_send(auto_mode, 1, 0);
    
    // Target coordinates
    Udp.write(target_char_1234);
    udp_send(target_lat_1, 2, 9);
    udp_send(target_lng_1, 3, 9);

    Udp.write(target_char);
    udp_send(step_no, 3,0);
    udp_send(current_target,3,0);
    udp_send(target_lat, 2, 9);
    udp_send(target_lng, 3, 9);

    // USV coordinates
    Udp.write(usv_char);
    udp_send(usv_lat,2,9);
    udp_send(usv_lng,3,9);
    Udp.write(valid_char);
    udp_send(valid,1,0);
       
    // Compass
    temp2 = map_float(usv_yaw, -pi, pi, -180, 180);
    Udp.write(yaw_char);
    udp_send(temp2,3,0);

    // Arival state
    Udp.write(arrival_char);
    udp_send(arrival,1,0);

    // Distance
    Udp.write(distance_char);
    udp_send(distance, 4, 0);

    // Desire yaw
    temp2 = map_float(desire_yaw, -pi, pi, -180, 180);
    Udp.write(desire_yaw_char);
    udp_send(temp2, 4, 0);

    // k_yaws
    Udp.write(k_yaw_char);
    udp_send(k_yaw_error, 1, 3);
    Udp.write(k_distance_char);
    udp_send(k_distance, 1, 3);

    // Motor
    Udp.write(motor_char);
    udp_send(speed_1, 3, 0);
    udp_send(speed_2, 3 ,0);
    
    Udp.write(terminate_char);
    Udp.endPacket();
  
  // ---------------------END LOOP---------------------
    Serial.println(" ");
    arrival_prev_prev = arrival_prev;
    arrival_prev = arrival;
}

//----------------------------------------------------------------------------- FUNCTIONS ----------------------------------------------------------------------------- 
//-----------------------------------------------------------------------------***-----***----------------------------------------------------------------------------- 

void udp_send (double data, int top_digit, int lowest_decimal)
{
  if (data<0)
  {
    data = abs(data);
    Udp.write(45);
  }
  
  int string_size = top_digit + lowest_decimal + 1;
  char udp_string [string_size];
  temp = data;
  
  for (int i = top_digit; i>=1; i--)
  {
   udp_string[top_digit-i] = temp/pow(10,(i-1));
   temp = temp - udp_string[top_digit-i] * pow(10,(i-1));
   udp_string[top_digit-i] = udp_string[top_digit-i] +48;
  }

  udp_string[top_digit] = 46;

  for (int i = 1; i <= lowest_decimal; i++)
  {
   udp_string[top_digit+i] = temp/pow(10,-i);
   temp = temp - udp_string[top_digit+i] * pow(10,-i);
   udp_string[top_digit+i] = udp_string[top_digit+i] +48;
  }

  for (int i=0; i<= (string_size-1); i++)
  {
    Udp.write(udp_string[i]);
  }
   
  Udp.write(end_char);
}

void motor_control(float _thrust, float _moment)
{
  speed_1 = _thrust + _moment; // PWM output, range 0-255
  speed_2 = _thrust - _moment;
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

float map_float( float val, float min1, float max1, float min2, float max2)
{
  float val_2;
  if (val<min1)
    val = min1;
  if (val>max1)
    val = max1; 
  val_2 = min2 + (val-min1)*(max2-min2)/(max1-min1);
  return val_2;
}

double parse_target_lat (int _start_char_no)
{     
      double _target_lat;
      int _k;
      _target_lat = 0;
      _k = _start_char_no+0;
      _target_lat = _target_lat + (incomingPacket[_start_char_no+0]-48)*1E1;
      _k = _start_char_no+1;
      _target_lat = _target_lat + (incomingPacket[_start_char_no+1]-48)*1E0;
      _k = _start_char_no+3;
      _target_lat = _target_lat + (incomingPacket[_start_char_no+3]-48)*1E-1;
      _k = _start_char_no+4;
      _target_lat = _target_lat + (incomingPacket[_start_char_no+4]-48)*1E-2;
      _k = _start_char_no+5;
      _target_lat = _target_lat + (incomingPacket[_start_char_no+5]-48)*1E-3;
      _k = _start_char_no+6;
      _target_lat = _target_lat + (incomingPacket[_start_char_no+6]-48)*1E-4;
      _k = _start_char_no+7;
      _target_lat = _target_lat + (incomingPacket[_start_char_no+7]-48)*1E-5;
      _k = _start_char_no+8;
      _target_lat = _target_lat + (incomingPacket[_start_char_no+8]-48)*1E-6;
      _k = _start_char_no+9;
      _target_lat = _target_lat + (incomingPacket[_start_char_no+9]-48)*1E-7;
      _k = _start_char_no+10;
      _target_lat = _target_lat + (incomingPacket[_start_char_no+10]-48)*1E-8;
      _k = _start_char_no+11;
      _target_lat = _target_lat + (incomingPacket[_start_char_no+11]-48)*1E-9;

      return _target_lat;    
}



double parse_target_lng (int _start_char_no)
{
      double _target_lng;
      int _k;
      _target_lng = 0;
      _k = _start_char_no+13;
      _target_lng = _target_lng + (incomingPacket[_k]-48)*1E2;
      _k = _start_char_no+14;
      _target_lng = _target_lng + (incomingPacket[_k]-48)*1E1;
      _k = _start_char_no+15;
      _target_lng = _target_lng + (incomingPacket[_k]-48)*1E0;
      _k = _start_char_no+17;
      _target_lng = _target_lng + (incomingPacket[_k]-48)*1E-1;
      _k = _start_char_no+18;
      _target_lng = _target_lng + (incomingPacket[_k]-48)*1E-2;
      _k = _start_char_no+19;
      _target_lng = _target_lng + (incomingPacket[_k]-48)*1E-3;
      _k = _start_char_no+20;
      _target_lng = _target_lng + (incomingPacket[_k]-48)*1E-4;
      _k = _start_char_no+21;
      _target_lng = _target_lng + (incomingPacket[_k]-48)*1E-5;
      _k = _start_char_no+22;
      _target_lng = _target_lng + (incomingPacket[_k]-48)*1E-6;
      _k = _start_char_no+23;
      _target_lng = _target_lng + (incomingPacket[_k]-48)*1E-7;
      _k = _start_char_no+24;
      _target_lng = _target_lng + (incomingPacket[_k]-48)*1E-8;
      _k = _start_char_no+25;
      _target_lng = _target_lng + (incomingPacket[_k]-48)*1E-9;

      return _target_lng;
}

float get_distance (double _target_lat_1, double _target_lat_2, double _target_lng_1, double _target_lng_2)
{
  float _distance;
  _distance = pow(pow(k_fix_lat*(_target_lat_2 - _target_lat_1) , 2) + pow(k_fix_lng*(target_lng_1 - target_lng_2) , 2) ,0.5);
  return _distance;
}

int get_step_no (float _distance, float _step_size)
{
  int _step_no;
  _step_no = _distance/_step_size;
  return _step_no;
}

double get_target (int i, int _step_no, double _target_1, double _target_2)
{
  double _target;
  double _step;

  _step = (_target_2 - _target_1)/_step_no;
  _target = _target_1 + _step*i;

  return _target;
}

double get_target_list (int i, int _step_no, double _target_1, double _target_2, double _target_3, double _target_4)
{
  double _target_list = 0;
  int k_int = 0;
  int mod_4 = 0;
  k_int = i/4;
  mod_4 = i%4;

  if (mod_4 == 0)
    _target_list = get_target((2*k_int), _step_no, _target_1, _target_2);

  if (mod_4 == 1)
    _target_list = get_target((2*k_int+1), _step_no, _target_1, _target_2);

  if (mod_4 == 2)
    _target_list = get_target((2*k_int), _step_no, _target_4, _target_3);

  if (mod_4 == 3)
    _target_list = get_target((2*k_int+1), _step_no, _target_4, _target_3);
  
  return _target_list;
}
