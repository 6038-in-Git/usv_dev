
// -------------------------------CODE for ESP8266 Nodemcu-------------------------------

#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
 
  int throttle;
  int turn;
  int thrust;
  int moment;
  int speed_1;
  int speed_2;
  int mode;
  

// UDP WIFI
  const char* ssid = "MERCURY_2.4G_0122";
  const char* password = "hch4pw7n";
  // const char* ssid = "Linh";
  // const char* password = "sinhnhatcuatao";
  // const char* ssid = "POCO F3";
  // const char* password = "12345789";
  // const char* ssid = "TP-LINK_3D77A0";
  // const char* password = "";
  // const char* ssid = "KTHK 207";
  // const char* password = "hmt123456789xyz";
  const char* end_char = "; ";
  const char* terminate_char = "~";
  
  
  WiFiUDP Udp;
  unsigned int localUdpPort = 4210;  // local port to listen on
  char incomingPacket[255];  // buffer for incoming packets
  char  replyPacket[] = "USV Packet: ";  // a reply string to send back


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
  double target_lat1 = 0;
  double target_lng1 = 0;
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
  int temp_3;

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
  int radio_thrust;
  int radio_moment;

  const float pi = 3.14159265359;
  const float k_fix_lat = 113894;
  const float k_fix_lng = 104225;
  const float a = 20;
  const float b = 20;
  const float k_moment = 400/pi;
  const float cruise = 90;
  const float arrival_radius = 5;

  // IP Address
  // IPAddress laptop(192,168,1,100);
  // IPAddress linhkien_3m(192,168,139,34);  //driver
  // IPAddress lolin(192,168,139,244);  //controller
  // IPAddress lolin_2(192,168,139,241);  //observer


  IPAddress laptop(192,168,1,102);
  IPAddress linhkien_3m(192,168,1,100);  //driver
  IPAddress lolin(192,168,1,103);  //controller
  IPAddress lolin_2(192,168,1,101);  //observer


  
//----------------------------------------------------------------------------    SETUP   -----------------------------------------------------------------------------
//-----------------------------------------------------------------------------***-----***----------------------------------------------------------------------------- 
void setup() {
 Serial.begin(115200);

  
  // Connect to Wi-Fi network with SSID and password
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  // Print local IP address and start web server
  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  Udp.begin(4210);
}

//-----------------------------------------------------------------------------   LOOP    -----------------------------------------------------------------------------
//-----------------------------------------------------------------------------***-----***-----------------------------------------------------------------------------
void loop(){

  // ----------------------------------------------- READING ----------------------------------------------- 

  int packetSize = Udp.parsePacket();

  int len = Udp.read(incomingPacket, 255);
  if (len > 0)
  {
    incomingPacket[len] = 0;
  }

if (Udp.remoteIP()==lolin_2)
    {     
        usv_lat = parse_target_lat(6);
        usv_lng = parse_target_lng(21);
        usv_yaw = parse_integer_3_digit(58);
        radio_thrust = parse_integer_3_digit(79);
        radio_moment = parse_integer_3_digit(100);
        mode = incomingPacket[113] - 48;
        valid = incomingPacket[44] -48;
    }
    Serial.print(usv_yaw);
    Serial.print(", ");
    Serial.print(mode);
    Serial.print(", ");
    Serial.print(valid);
    Serial.print(", ");
    Serial.print(radio_thrust);
    Serial.print(", ");
    Serial.print(radio_moment);
    Serial.print(", ");

  // UDP RECIEVE TARGET COORDINATE
if (Udp.remoteIP() == laptop)
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
      if(incomingPacket[0] == 64)
      {
                
        target_lat = parse_target_lat(2);
        target_lng = parse_target_lng(2);  
      }
    }
}
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
      
      if(incomingPacket[0] == 33){
      target_lat = get_target_list(current_target, step_no, target_lat_1, target_lat_2, target_lat_3, target_lat_4);
      target_lng = get_target_list(current_target, step_no, target_lng_1, target_lng_2, target_lng_3, target_lng_4);
      switch_target = false;
      }

      Serial.print("target_lat =  ");
      Serial.print(target_lat,6);
      Serial.print("; ");

      Serial.print("target_lng =  ");
      Serial.print(target_lng,6);
      Serial.print("; "); 
      
    // ----------------------------------------------- GET YAW ERROR -----------------------------------------------
    //
      usv_yaw = map_float(usv_yaw, -180, 180, -pi, pi);
      // Serial.print("usv_yaw: ");
      // Serial.print(temp2, 0);
      // Serial.print("; "); 

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
      Serial.print(distance);
      Serial.print(", ");


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
      if (distance < arrival_radius)
      {
        arrival = true;
      }
 
  // ----------------------------------------------- AUTO MODE -----------------------------------------------
      thrust = k_yaw_error * k_distance * cruise;                   // range 0 ~ cruise
      moment = k_moment * yaw_error;
  
  // UDP SEND LOG DATA

    Udp.beginPacket(laptop, 4210);

    // USV coordinates
    Udp.write("driver: ");
    udp_send(usv_lat,2,9);
    udp_send(usv_lng,3,9);
    Udp.write("Valid: ");
    udp_send(valid,1,0);

    //radio
    Udp.write("radio: ");
    udp_send(radio_thrust,3,0);
    udp_send(radio_moment,3,0);

    //auto thrust moment
    Udp.write("thrust,moment: ");
    udp_send(thrust,3,0);
    udp_send(moment,3,0);

    //target
    Udp.write("Target: ");
    udp_send(step_no, 3,0);
    udp_send(current_target,3,0);
    udp_send(target_lat, 2, 9);
    udp_send(target_lng, 3, 9);

    // Compass
    temp2 = map_float(usv_yaw, -pi, pi, -180, 180);
    Udp.write("usv_yaw");
    udp_send(temp2,3,0);

    // Distance
    Udp.write("Distance: ");
    udp_send(distance, 4, 0);
    
    Udp.write(terminate_char);
    Udp.endPacket();

  // send to driver:

    Udp.beginPacket(linhkien_3m, 4210);
    //Udp.beginPacket(laptop, 4210);

    Udp.write("Auto: ");
    udp_send(thrust,3,0);
    udp_send(moment,3,0);

    Udp.write("Manual: ");
    udp_send(radio_thrust,3,0);
    udp_send(radio_moment,3,0);

    Udp.write("Mode: ");
    udp_send(mode,1,0);


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
  else Udp.write(43);
  
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
      int _k = 1;
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
      int _k = 2;
      _target_lng = 0;
      _k = _start_char_no + 13;
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

      if((int)incomingPacket[_start_char_no - 1] == 45) 
        _val=-_val;
      
      return _val;
}
