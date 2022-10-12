
// -------------------------------CODE for ESP8266 Nodemcu-------------------------------

#include <Servo.h>
#include <QMC5883LCompass.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include "ping1d.h"

// PING VARIABLES
static const uint8_t arduinoRxPin = 1;
static const uint8_t arduinoTxPin = 3;
SoftwareSerial pingSerial = SoftwareSerial(arduinoRxPin, arduinoTxPin);
static Ping1D ping { pingSerial };

// MODE CHOICE
  const int pin_mode = 14;      // D5
  bool auto_mode = false;

// MOTOR VARIABLES
  const int pin_throttle = 12;  // D6
  const int pin_turn = 15;      // D8
  const int pin_motor_1 = 13;   // D7
  const int pin_motor_2 = 16;   // D0
   
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
  const char* ssid = "KTHK 207";
  const char* password = "hmt123456789xyz";
  const char* end_char = "; ";
  const char* terminate_char = "~";
  
  WiFiUDP Udp;
  unsigned int localUdpPort = 4210;  // local port to listen on
  char incomingPacket[255];  // buffer for incoming packets
  char  replyPacket[] = "USV Packet: ";  // a reply string to send back

// TRACKING PARAMETERS
  double target_lat = 0;
  double target_lng = 0;
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
  float temp2;
  
  const float pi = 3.14159265359;
  const float k_fix_lat = 113894;
  const float k_fix_lng = 104225;
  const float a = 20;
  const float b = 20;
  const float k_moment = 150/pi;
  const float cruise = 120;
  const float arrival_radius = 5;
  
  
//-------------------------------SETUP-------------------------------
void setup() {
  //Set up ping serial
  pingSerial.begin(9600);
  Serial.println("Blue Robotics ping1d-simple.ino");
  while (!ping.initialize()) {
        Serial.println("\nPing device failed to initialize!");
        Serial.println("Are the Ping rx/tx wired correctly?");
        Serial.print("Ping rx is the green wire, and should be connected to Arduino pin ");
        Serial.print(arduinoTxPin);
        Serial.println(" (Arduino tx)");
        Serial.print("Ping tx is the white wire, and should be connected to Arduino pin ");
        Serial.print(arduinoRxPin);
        Serial.println(" (Arduino rx)");
        delay(2000);
    }
    
  // Serial
  //Serial.begin(115200);

  // MODE SETUP
  pinMode(pin_mode, INPUT);
  
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

//-------------------------------LOOP-------------------------------
void loop(){
  // DISPLAY PING SONAR
    if (ping.update()) {
        Serial.print("Distance: ");
        Serial.print(ping.distance());
        Serial.print("\tConfidence: ");
        Serial.println(ping.confidence());
    } else {
        Serial.println("No update received!");
    }
  
  // SETTING MODE
    if (pulseIn(pin_mode, HIGH) >= 1500) 
      auto_mode = true;
    else
      auto_mode = false;
    Serial.print("Auto = ");
    Serial.print(auto_mode);
    Serial.print("; ");   

  // UDP RECIEVE TARGET COORDINATES
    int packetSize = Udp.parsePacket();
    
    // receive incoming UDP packets
    int len = Udp.read(incomingPacket, 255);
    if (len > 0)
    {
      incomingPacket[len] = 0;
    }

    //Serial.print("Ground Station: ");
    //Serial.print(incomingPacket);
    //Serial.print("; ");
    
    if (packetSize = 26)
    {
      // Decode target
      target_lat = 0;
      target_lat = target_lat + (incomingPacket[0]-48)*1E1;
      target_lat = target_lat + (incomingPacket[1]-48)*1E0;
      target_lat = target_lat + (incomingPacket[3]-48)*1E-1;
      target_lat = target_lat + (incomingPacket[4]-48)*1E-2;
      target_lat = target_lat + (incomingPacket[5]-48)*1E-3;
      target_lat = target_lat + (incomingPacket[6]-48)*1E-4;
      target_lat = target_lat + (incomingPacket[7]-48)*1E-5;
      target_lat = target_lat + (incomingPacket[8]-48)*1E-6;
      target_lat = target_lat + (incomingPacket[9]-48)*1E-7;
      target_lat = target_lat + (incomingPacket[10]-48)*1E-8;
      target_lat = target_lat + (incomingPacket[11]-48)*1E-9;
      //Serial.print("target_lat =  ");
      //Serial.print(target_lat,6);
      //Serial.print("; ");      

      target_lng = 0;
      target_lng = target_lng + (incomingPacket[13]-48)*1E2;
      target_lng = target_lng + (incomingPacket[14]-48)*1E1;
      target_lng = target_lng + (incomingPacket[15]-48)*1E0;
      target_lng = target_lng + (incomingPacket[17]-48)*1E-1;
      target_lng = target_lng + (incomingPacket[18]-48)*1E-2;
      target_lng = target_lng + (incomingPacket[19]-48)*1E-3;
      target_lng = target_lng + (incomingPacket[20]-48)*1E-4;
      target_lng = target_lng + (incomingPacket[21]-48)*1E-5;
      target_lng = target_lng + (incomingPacket[22]-48)*1E-6;
      target_lng = target_lng + (incomingPacket[23]-48)*1E-7;
      target_lng = target_lng + (incomingPacket[24]-48)*1E-8;
      target_lng = target_lng + (incomingPacket[25]-48)*1E-9;
      //Serial.print("target_lng =  ");
      //Serial.print(target_lng,6);
      //Serial.print("; "); 
    }
    

    // COMPASS
      compass.read();
      compass_reading = compass.getAzimuth();
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

    
    // ------------GET YAW ERROR------------
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
      Serial.print("lat_e: ");
      Serial.print(fix_lat_error, 6);
      Serial.print("; "); 
      Serial.print("lng_e: ");
      Serial.print(fix_lng_error, 6);
      Serial.print("; "); 
      
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


 
  // -------------------------------AUTO MODE-------------------------------
  if (pulseIn(pin_mode, HIGH) >= 1500)
  {
    // ------------DESIGN THRUST AND MOMENT------------
      k_yaw_error = 1 / (1 + a*pow(yaw_error,2));                   // range 1 ~ 0
      k_distance = pow(distance,2)/(pow(distance,2) + b);           // range 0 ~ 1
      Serial.print("k_yaw_e: ");
      Serial.print(k_yaw_error, 3);
      Serial.print("; "); 
      Serial.print("k_dist: ");
      Serial.print(k_distance, 3);
      Serial.print("; "); 
      
      thrust = k_yaw_error * k_distance * cruise;                   // range 0 ~ cruise
      moment = k_moment * yaw_error;

    // ------------TARGET REACHED------------
      arrival = false;
      if (distance<arrival_radius)
      {
        thrust = 0;
        moment = 0;
        arrival = true;
      }

    // ------------CONTROL MOTOR------------
      motor_control(thrust, moment); 

  }
  
  // -------------------------------MANUAL MODE-------------------------------
  else
  {
    throttle = pulseIn(pin_throttle, HIGH); // PWM width, range 1000 - 2000 miroseconds
    turn = pulseIn(pin_turn, HIGH);
    
    if (throttle<1400) throttle = 1400;
    thrust = map(throttle, 1500, 2000, 0, 180); 
    moment = map(turn, 1000, 2000, -50, 50);
  
    // ------------CONTROL MOTOR------------
      motor_control(thrust, moment); 
  }

  // UDP SEND LOG DATA
    Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
    Udp.write(replyPacket);

    // Target coordinates
    udp_send(target_lat, 2, 9);
    udp_send(target_lng, 3, 9);

    // USV coordinates
    udp_send(usv_lat,2,9);
    udp_send(usv_lng,3,9);
    udp_send(valid,1,0);t
       
    // Compass
    udp_send(compass_reading,3,0);

    // ping sonar:
    udp_send(ping_sonar, 5, 0);
    
    // Arival state
    udp_send(arrival,1,0);
    
    Udp.write(terminate_char);
    Udp.endPacket();
  
  // END LOOP
    Serial.println(" ");
}

void udp_send (double data, int top_digit, int lowest_decimal)
{
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
  speed_1 = _thrust + _moment - 10; // PWM output, range 0-255
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
