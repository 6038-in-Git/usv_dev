
// -------------------------------CODE for ESP8266 Nodemcu-------------------------------

//#include <Servo.h>
//#include <QMC5883LCompass.h>
//#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

// TIME
  //double current_time = 0;

// MODE CHOICE
  ////const int pin_mode = 14;      // D5
  int usv_mode = 0;

// MOTOR VARIABLES
  //const int pin_throttle = 12;  // D6
  //const int pin_turn = 15;      // D8
  ////const int pin_motor_1 = 13;   // D7
  //const int pin_motor_2 = 16;   // D0
  //const int pin_motor_1 = 14;   // D5

  //Servo f_1;
  //Servo f_2;   
  //int throttle;
  //int turn;
  int thrust;
  float moment;
  int speed_1;
  int speed_2;

// COMPASS
  //QMC5883LCompass compass;
  //float compass_reading;          // Arduino reads QMC5883l, then send it to ESP8226 via Serial
  

// GPS
  //const int RXPin = 0;          // D3
  //const int TXPin = 2;          // D4
  //const uint32_t GPSBaud = 9600;
  //TinyGPSPlus gps;
  //SoftwareSerial ss(RXPin, TXPin);

// UDP WIFI
  //const char* ssid = "MERCURY_2.4G_0122";
  //const char* password = "hch4pw7n";
  //const char* ssid = "KTHK 207";
  //const char* password = "hmt123456789xyz";
  const char* ssid = "HUAWEI_ADDD";
  const char* password = "YG0A94HM7B2";
  
  const char* end_char = "; ";
  const char* terminate_char = "~";
  
  WiFiUDP Udp;
  unsigned int localUdpPort = 4210;  // local port to listen on
  char incomingPacket[255];  // buffer for incoming packets
  char  replyPacket[] = "USV Packet: ";  // a reply string to send back
  int packetSize;
  int len;

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
  const char thrust_moment_char[] = "Thrust - Moment: ";
  
  const char along_e_char [] = "along_e: ";
  const char cross_e_char [] = "cross_e: ";
  const char x_path_char [] = "x_path: ";
  const char y_path_char [] = "y_path: ";
  const char path_yaw_char [] = "path_yaw: ";
  const char k_moment_char [] = "k_moment: ";
  const char fuzzy_low_char [] = "fuzzy_low: ";
  const char fuzzy_mid_char [] = "fuzzy_mid: ";
  const char fuzzy_high_char [] = "fuzzy_high: ";
  const char track_time_char [] = "track_time: ";
  const char thrust_char [] = "thrust: ";
  const char idle_char [] = "Standing by !";


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

  int initial_x = 0;
  int initial_y = 0;
  int r_x = 0;
  int r_y = 0;
  int omega_x = 0;
  int omega_y = 0;
  int path_type = 0;
  int track_time = 0;
  int track_duration = 0;
  float x_path = 0;
  float y_path = 0;
  float v_x_desire = 0;
  float v_y_desire = 0;
  float path_yaw = 0;
  float along_e = 0;
  float cross_e = 0;

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
  float temp2_2;
  float fuzzy_low = 0;
  float fuzzy_mid = 0;
  float fuzzy_high = 0;
  
  const float pi = 3.14159265359;
  const float k_fix_lat = 113894;
  const float k_fix_lng = 104225;
  const int a = 20;
  const int b = 20;
  const float k_moment = 500/pi;
  const float k_moment_2 = 300/pi;
  const int cruise = 90;
  const int arrival_radius = 5;
  const int look_ahead = 5;
  const int u_desire = 80;

// IP Address
  IPAddress laptop(192,168,8,104);
  IPAddress driver(192,168,8,111);
  IPAddress controller(192,168,8,112);
  IPAddress observer(192,168,8,110);
  
//----------------------------------------------------------------------------    SETUP   -----------------------------------------------------------------------------
//-----------------------------------------------------------------------------***-----***----------------------------------------------------------------------------- 
void setup() {

  // Serial
  Serial.begin(115200);

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

  // ----------------------------------------------- RECEIVING UDP -----------------------------------------------  
  // -------------- Parse from Ground Control Station --------------
    receive_udp();
    if (Udp.remoteIP() == observer)
    {
      usv_yaw = parse_integer_3_digit(3);
      thrust = parse_integer_3_digit(8);
      moment = parse_integer_3_digit(13);
    }
  
  // -------------- Parse from Ground Control Station --------------
    receive_udp();
    if (Udp.remoteIP( )== laptop)
    {
      if (incomingPacket[0] == 33)
        usv_mode = 1;
      
      if (incomingPacket[0] == 126)
        usv_mode = 0;
    }
    
  // 
  // ----------------------------------------------- UDP SEND -----------------------------------------------
  // ---------------------------------------- TO Ground Control Station -------------------------------------
  //
  
    Udp.beginPacket(laptop, 4210);
    Udp.write(replyPacket);

    // MODE
    Udp.write(mode_char);
    udp_send(usv_mode, 1, 0);

    // IDLE MODE
    if (usv_mode == 0)
    {
      Udp.write(idle_char);
    }

    // REPORT MODE
    if (usv_mode == 1)
    {
      // Compass
      Udp.write(yaw_char);
      udp_send(usv_yaw,3,0);
      
      // Thrust Moment
      Udp.write(thrust_moment_char);
      udp_send(thrust, 3, 0);
      udp_send(moment, 3 ,0);
    }
    
    // END OF PACKET
    Udp.write(terminate_char);
    Udp.endPacket();

  // 
  // ----------------------------------------------- UDP SEND -----------------------------------------------
  // ---------------------------------------------- To Driver -----------------------------------------------
  //
    Udp.beginPacket(driver, 4210);
    
    udp_send(thrust, 3, 0);
    udp_send(moment, 3 ,0);
    
    Udp.write(terminate_char);
    Udp.endPacket();
  
  // ---------------------------------------------- END LOOP ----------------------------------------------
  // ---------------------------------------------- ******** ----------------------------------------------
    Serial.println();
    //arrival_prev_prev = arrival_prev;
    //arrival_prev = arrival;
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

int pow_int (int _base, int _exp)
{
  int _pow = 1;
  for (int i = 1; i <= _exp ; i++)
  {
    _pow = _pow * _base;
  }
  return _pow;
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

int parse_int (int _start_char_no, int _last_char_no)
{
      int _val = 0;
      int _k;
      for (int i = 0; i <= (_last_char_no - _start_char_no); i++)
      {
        _k = _last_char_no - i;
        _val = _val + (incomingPacket[_k]-48) * pow_int(10,i);
      }
      return _val;
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

double get_zigzag_list (int i, int _step_no, double _target_1, double _target_2, double _target_3, double _target_4)
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


void zigzag_function()
{

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
      
    // CALCULATE ZIGZAG STEP
        distance_12 = get_distance (target_lat_1, target_lat_2, target_lng_1, target_lng_2);
        step_no = get_step_no(distance_12, step_12);

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
      
      // ------------TARGET REACHED------------
      arrival = false;
      if (distance<arrival_radius)
      {
        arrival = true;
      }
      
      //---------------------------------------
      
      if ((switch_target>0)&&(end_zigzag<1))
      {
        switch_target = false;
        current_target = current_target +1;
      }
      Serial.print("current_target =  ");
      Serial.print(current_target);
      Serial.print("; "); 

      // Arival state
      Udp.write(arrival_char);
      udp_send(arrival,1,0);
}


// ------------------------------------* TRAJECTORY TRACKING *------------------------------------
// -----------------------------------------------------------------------------------------------
//
float get_path_x (int _track_time, int _path_type, int _initial_x, int _r_x, int _omega_x)
{
  float _x;

  // ------------Straight------------
  if (_path_type == 1)
    _x =  _initial_x + _omega_x*0.001*_track_time*0.001;

  // ------------Circular------------
  if (_path_type == 2)
    _x =  _initial_x + _omega_x*0.001*_track_time*0.001;

  // ------------Sinusoidal------------
  if (_path_type == 3)
    _x =  _initial_x + r_x*sin(_omega_x*0.001*_track_time*0.001);

 return _x;
}


float get_path_y (int _track_time, int _path_type, int _initial_y, int _r_y, int _omega_y)
{
  float _y;

  // ------------Straight------------
  if (_path_type == 1)
    _y =  _initial_y + _omega_y*0.001*_track_time*0.001;

  // ------------Circular------------
  if (_path_type == 2)
    _y =  _initial_y + r_y*sin(_omega_y*0.001*_track_time*0.001);

  // ------------Sinusoidal------------
  if (_path_type == 3)
    _y =  _initial_y + r_y*cos(_omega_y*0.001*_track_time*0.001);

  return _y;
}

bool check_y_path_axis (double _target_lat_1, double _target_lng_1, double _target_lat_2, double _target_lng_2, double _target_lat_3, double _target_lng_3)
{
  
  float _a1 = (_target_lng_2 - _target_lng_1)*k_fix_lng/k_fix_lat;
  float _b1 = -(_target_lat_2 - _target_lat_1)*k_fix_lat/k_fix_lng;
  float _a2 = _target_lat_3 - _target_lat_1;
  float _b2 = _target_lng_3 - _target_lng_1;
  bool _axis_side = false;
  
  if (((_a1*_a2)+(_b1*_b2))>0)
    _axis_side = true;

  return _axis_side;
}

float delta_2(float _a, float _b)
{
  float _delta_2;
  _delta_2 = pow((pow(_a,2)+pow(_b,2)),0.5);
  return _delta_2;
}

double x_y_to_lat (float _x, float _y, double _target_lat_1, double _target_lng_1, double _target_lat_2, double _target_lng_2, double _target_lat_3, double _target_lng_3)
{
  double _lat;
  float _dot_x;
  float _dot_y;
  float _i_1;
  float _i_2;
  float _j_1;
  float _j_2;

  _i_1 = target_lat_2 - target_lat_1;
  _i_2 = target_lng_2 - target_lng_1;
  _j_1 = (k_fix_lng/k_fix_lat)*(target_lng_2 - target_lng_1);
  _j_2 = -(k_fix_lat/k_fix_lng)*(target_lat_2 - target_lat_1);

  if (check_y_path_axis(_target_lat_1, _target_lng_1, _target_lat_2, _target_lng_2, _target_lat_3, _target_lng_3) > 0)
  {
    _dot_x = 0.707 * _i_1 / delta_2(_i_1, _i_2); 
    _dot_y = 0.707 * _j_1 / delta_2(_j_1, _j_2);
  }
  else
  {
    _dot_x = 0.707 * _i_1 / delta_2(_i_1, _i_2); 
    _dot_y = -0.707 * _j_1 / delta_2(_j_1, _j_2);
  }

  _lat = (_x * _dot_x + _y * _dot_y)/k_fix_lat;

  return _lat;
}

double x_y_to_lng (float _x, float _y, double _target_lat_1, double _target_lng_1, double _target_lat_2, double _target_lng_2, double _target_lat_3, double _target_lng_3)
{
  double _lng;
  float _dot_x;
  float _dot_y;
  float _i_1;
  float _i_2;
  float _j_1;
  float _j_2;

  _i_1 = target_lat_2 - target_lat_1;
  _i_2 = target_lng_2 - target_lng_1;
  _j_1 = (k_fix_lng/k_fix_lat)*(target_lng_2 - target_lng_1);
  _j_2 = -(k_fix_lat/k_fix_lng)*(target_lat_2 - target_lat_1);

  if (check_y_path_axis(_target_lat_1, _target_lng_1, _target_lat_2, _target_lng_2, _target_lat_3, _target_lng_3) > 0)
  {
    _dot_x = 0.707 * _i_2 / delta_2(_i_1, _i_2); 
    _dot_y = 0.707 * _j_2 / delta_2(_j_1, _j_2);
  }
  else
  {
    _dot_x = 0.707 * _i_2 / delta_2(_i_1, _i_2); 
    _dot_y = -0.707 * _j_2 / delta_2(_j_1, _j_2);
  }

  _lng = (_x * _dot_x + _y * _dot_y)/k_fix_lng;

  return _lng;
}

float v_x_path (int _track_time, int _path_type, int _initial_x, int _r_x, int _omega_x)
{
  float _v_x;

  // ------------Straight------------
  if (_path_type == 1)
    _v_x =  _omega_x*0.001;

  // ------------Circular------------
  if (_path_type == 2)
    _v_x =  _omega_x*0.001;

  // ------------Sinusoidal------------
  if (_path_type == 3)
    _v_x =  _omega_x*0.001*r_x*cos(_omega_x*0.001*_track_time*0.001);

 return _v_x;
}

float v_y_path (int _track_time, int _path_type, int _initial_y, int _r_y, int _omega_y)
{
  float _v_y;

  // ------------Straight------------
  if (_path_type == 1)
    _v_y =  _omega_y*0.001;

  // ------------Circular------------
  if (_path_type == 2)
    _v_y =  _omega_y*0.001*r_y*cos(_omega_y*0.001*_track_time*0.001);

  // ------------Sinusoidal------------
  if (_path_type == 3)
    _v_y =  -_omega_y*0.001*r_y*sin(_omega_y*0.001*_track_time*0.001);

  return _v_y;
}

float wrap_2_pi (float _angle)
{
  float _wrap_angle = _angle;
  
  if (_angle > pi)
    _wrap_angle = _angle - 2*pi;
  if (_angle < - pi)
    _wrap_angle = _angle + 2*pi;    // range -pi ~ pi

  //_wrap_angle = atan2(cos(_angle),sin(_angle));  
  return _wrap_angle;
}

float fuzzy_member (int _level, float _in, float _in_1, float _in_2, float _in_3)
{
  float _member_deg;
  
  if (_level == 1)
  {
    if ((_in_1 <= _in)&&(_in <= _in_2))
      _member_deg = map_float (_in, _in_1, _in_2, 1, 0); 
    if ((_in_2 < _in)&&(_in <= _in_3))
      _member_deg = 0;
  }
  
  if (_level == 2)
  {
    if ((_in_1 <= _in)&&(_in <= _in_2))
      _member_deg = map_float (_in, _in_1, _in_2, 0, 1); 
    if ((_in_2 < _in)&&(_in <= _in_3))
      _member_deg = map_float (_in, _in_2, _in_3, 1, 0); 
  }
    
  if (_level == 3)
  {
    if ((_in_1 <= _in)&&(_in <= _in_2))
      _member_deg = 0; 
    if ((_in_2 < _in)&&(_in <= _in_3))
      _member_deg = map_float (_in, _in_2, _in_3, 0, 1); 
  }
  return _member_deg;
}

int parse_integer_3_digit (int _start_char_no)
{
      int _val;
      int _k;
      _val = 0;
      _k = _start_char_no;
      _val = _val + (incomingPacket[_k]-48)*1E2;
      _k = _start_char_no+1;
      _val = _val + (incomingPacket[_k]-48)*1E1;
      _k = _start_char_no+2;
      _val = _val + (incomingPacket[_k]-48)*1E0;

      return _val;
}

void receive_udp()
{
  packetSize = Udp.parsePacket();
    
  len = Udp.read(incomingPacket, 255);
  if (len > 0)
  {
    incomingPacket[len] = 0;
  }
}
