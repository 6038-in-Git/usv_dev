
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
  //const int pin_mode = 14;        // D5
  const int pin_mode = 13;        // D7
  bool auto_mode = false;

// MOTOR VARIABLES
  const int pin_throttle = 12;    // D6
  const int pin_turn = 15;        // D8
  //const int pin_motor_1 = 13;     // D7
  const int pin_motor_2 = 16;     // D0
  const int pin_motor_1 = 14;     // D5
   
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
  //const char* ssid = "KTHK 207";
  //const char* password = "hmt123456789xyz";
  const char* ssid = "Hieu Thao 2,4g";
  const char* password = "0944012555";
  const char* end_char = "; ";
  const char* terminate_char = "~";
  
  WiFiUDP Udp;
  unsigned int localUdpPort = 4210;  // local port to listen on
  //char incomingPacket[255];  // buffer for incoming packets
  char  replyPacket[] = "USV Packet: ";  // a reply string to send back

// UDP char
  //const char mode_char[] = "Mode: ";
  //const char target_char[] = "Target: ";
  //const char target_char_1234[] = "4pts: ";
  const char usv_char[] = "USV: ";
  const char valid_char[] = "Valid: ";
  //const char arrival_char[] = "Arrival: ";
  const char yaw_char []= "Yaw: "; 
  //const char desire_yaw_char []= "Desire_yaw: ";
  //const char distance_char [] = "Distance: ";
  //const char k_yaw_char [] = "k_yaw_e: ";
  //const char k_distance_char [] = "k_distance: ";
  //const char motor_char[] = "Motor: ";
  const char radio_thrust_char[] = "Radio thrust: ";
  const char radio_moment_char[] = "Radio moment: ";

// TRACKING PARAMETERS
  //float distance_12 = 0;
  //const float step_12 = 1;
  //int step_no = 0;
  
  //int current_target = 0;
  //bool switch_target = 0;
  //double arrival_start_time = 0;
  //bool end_zigzag = false;
  
  //double target_lat = 0;
  //double target_lng = 0;
  //double target_lat_1 = 0;
  //double target_lng_1 = 0;
  //double target_lat_2 = 0;
  //double target_lng_2 = 0;
  //double target_lat_3 = 0;
  //double target_lng_3 = 0;
  //double target_lat_4 = 0;
  //double target_lng_4 = 0;
  double temp;
  double usv_lat = 0;
  double usv_lng = 0;
  bool valid = false;

// CONTROL PARAMETER
  float usv_yaw = 0;
  //float fix_lat_error = 0;
  //float fix_lng_error = 0;
  //float yaw_error = 0;
  //float desire_yaw = 0;
  //float distance = 0;
  //float k_yaw_error = 0;
  //float k_distance = 0;
  //bool arrival = false;
  //bool arrival_prev = false;
  //bool arrival_prev_prev = false;
  float temp2;
  
  const float pi = 3.14159265359;
  //const float k_fix_lat = 113894;
  //const float k_fix_lng = 104225;
  //const float a = 20;
  //const float b = 20;
  //const float k_moment = 400/pi;
  //const float cruise = 90;
  //const float arrival_radius = 5;

// IP Address
  IPAddress laptop(192,168,1,30);
  IPAddress linhkien_3m(192,168,1,21);
  IPAddress lolin(192,168,1,35);
  IPAddress lolin_2(192,168,1,47);
  
//----------------------------------------------------------------------------    SETUP   -----------------------------------------------------------------------------
//-----------------------------------------------------------------------------***-----***----------------------------------------------------------------------------- 
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
  
  // RADIO
    throttle = pulseIn(pin_throttle, HIGH); // PWM width, range 1000 - 2000 miroseconds
    turn = pulseIn(pin_turn, HIGH);
    
    if (throttle<1600) throttle = 1600;
    thrust = map(throttle, 1600, 2000, 0, 180); 
    moment = map(turn, 1000, 2000, -120, 120);
  

  // ------------------------------------------------UDP SEND LOG DATA-------------------------------------------
    Udp.beginPacket(lolin, 4210);
    Udp.write(replyPacket);

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

    // Radio
    Udp.write(radio_thrust_char);
    udp_send(thrust,3,0);
    Udp.write(radio_moment_char);
    udp_send(moment,3,0);

    Udp.write(terminate_char);
    Udp.endPacket();
  
  // ---------------------END LOOP---------------------
    Serial.println();
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
