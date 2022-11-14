// -------------------------------CODE for ESP8266 Nodemcu-------------------------------

#include <QMC5883LCompass.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <USVFunction.h>

USVFunction usv;

// MODE CHOICE
  const int pin_mode = 14;        // D5
  // const int pin_mode = 13;        // D7
  bool auto_mode = false;


// MOTOR VARIABLES
  const int pin_throttle = 12;    // D6
  const int pin_turn = 15;        // D8
    
  int throttle;
  int turn;
  int thrust;
  int moment;


// COMPASS
  QMC5883LCompass compass;
  float compass_reading;          // Arduino reads QMC5883l, then send it to ESP8226 via Serial
  

// GPS
  const int RXPin = 2;          // D3
  const int TXPin = 0;          // D4
  const uint32_t GPSBaud = 9600;
  TinyGPSPlus gps;
  SoftwareSerial ss(RXPin, TXPin);

// UDP WIFI
  const char* ssid = "MERCURY_2.4G_0122";
  const char* password = "hch4pw7n";
  //const char* ssid = "KTHK 207";
  //const char* password = "hmt123456789xyz";
  // const char* ssid = "Hieu Thao 2,4g";
  // const char* password = "0944012555";
  // const char* ssid = "Linh";
  // // const char* password = "sinhnhatcuatao";
  // // const char* ssid = "POCO F3";
  // // const char* password = "12345789";
  // const char* ssid = "TP-LINK_3D77A0";
  // const char* password = "";
  const char* end_char = "; ";
  const char* terminate_char = "~";
  
  WiFiUDP Udp;
  unsigned int localUdpPort = 4210;  // local port to listen on
  char  replyPacket[] = "USV Packet: ";  // a reply string to send back

  double temp;
  double usv_lat = 0;
  double usv_lng = 0;
  bool valid = false;

// CONTROL PARAMETER
  float usv_yaw = 0;
  float temp2;
  const float pi = 3.14159265359;

  // IP Address
  // IPAddress laptop(192,168,139,214);
  // IPAddress linhkien_3m(192,168,139,51);  //driver
  // IPAddress lolin(192,168,139,244);  //controller
  // IPAddress lolin_2(192,168,139,241);  //observer
  
  IPAddress laptop(192,168,1,102);
  IPAddress linhkien_3m(192,168,1,100);  //driver
  IPAddress lolin(192,168,1,103);  //controller
  IPAddress lolin_2(192,168,1,101);  //observer

  
//----------------------------------------------------------------------------    SETUP   -----------------------------------------------------------------------------
//-----------------------------------------------------------------------------***-----***----------------------------------------------------------------------------- 
void setup() {

  // Serial
  Serial.begin(115200);

  // MODE SETUP
  pinMode(pin_mode, INPUT);
  
  // MOTOR SETUP  
  pinMode(pin_throttle, INPUT);         
  pinMode(pin_turn, INPUT); 

  // COMPASS SETUP
  compass.init();
  
  // GPS SETUP
  ss.begin(GPSBaud);

  
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
    
    // COMPASS
    compass.read();
    compass_reading = compass.getAzimuth();
    Serial.print("Compass: ");
    Serial.print(compass_reading);
    Serial.print("; ");
    //if (compass_reading> 270)
    //  compass_reading = usv.map_float(compass_reading, 270, 360, 0, 90);
    //else
    //  compass_reading = usv.map_float(compass_reading, 0, 270, 90, 360);
    //Serial.print("Compass: ");
    //Serial.print(compass_reading);
    //Serial.print("; ");

    if (compass_reading<=180)
        usv_yaw = usv.map_float(compass_reading, 0, 180, 0, pi);            
    if (compass_reading>180)
    {
      usv_yaw = usv.map_float(compass_reading, 180, 360, -pi, 0);           // range -pi ~ pi
      compass_reading = usv.map_float(compass_reading, 180, 360, -180, 0);  // range -180 ~ 180
    }
    Serial.print("usv_yaw: ");
    Serial.print(temp2, 0);
    Serial.print("; "); 
      
    // GPS
    valid = false;
    //   if (gps.encode(ss.read())) 
    //     if (gps.location.isValid())
    //     {
    //       valid = true;
  usv_lat = gps.location.lat();
  usv_lng = gps.location.lng();

  printInt(gps.charsProcessed(), true, 6);
  printInt(gps.sentencesWithFix(), true, 10);
  printInt(gps.failedChecksum(), true, 9);
  // Serial.println();
  
  // smartDelay(1000);

  if (millis() > 5000 && gps.charsProcessed() < 10)
    Serial.print(F("No GPS data received: check wiring"));

    //     }
    // Serial.print("valid =  ");
    // Serial.print(valid);
    // Serial.print("; ");
        
    Serial.print("usv_lat =  ");
    Serial.print(usv_lat,9);
    Serial.print("; ");

    Serial.print("usv_lng =  ");
    Serial.print(usv_lng,9);
    Serial.print("; ");      
  
  // RADIO
    if(pulseIn(pin_mode, HIGH)>1500){
      auto_mode = 1; 
    } 
    else auto_mode = 0;
    
    throttle = pulseIn(pin_throttle, HIGH); // PWM width, range 1000 - 2000 miroseconds
    turn = pulseIn(pin_turn, HIGH);
    
    if (throttle<1600) throttle = 1600;
    thrust = map(throttle, 1600, 2000, 0, 180); 
    moment = map(turn, 1000, 2000, -120, 120);
    Serial.print("thrust = ");
    Serial.print(thrust);
    Serial.print("; moment = ");
    Serial.print(moment);

  // ------------------------------------------------UDP SEND LOG DATA-------------------------------------------
    
  Udp.beginPacket(lolin, 4210);
  //Udp.beginPacket(laptop, 4210);
   
  // USV coordinates
    Udp.write("USV: ");
    udp_send(usv_lat,2,9);
    udp_send(usv_lng,3,9);
    Udp.write("valid: ");
    udp_send(valid,1,0);
       
    // Compass
    temp2 = usv.map_float(usv_yaw, -pi, pi, -180, 180);
    Udp.write("usv_yaw: ");
    udp_send(temp2,3,0);

    // Radio
    Udp.write("radio_thrust: ");
    udp_send(thrust,3,0);
    Udp.write("radio_moment: ");
    udp_send(moment,3,0);

    // Mode
    Udp.write("Mode: ");
    udp_send(auto_mode,1,0);

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

// This custom version of delay() ensures that the gps object
// is being "fed".
static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}

static void printFloat(float val, bool valid, int len, int prec)
{
  if (!valid)
  {
    while (len-- > 1)
      Serial.print('*');
    Serial.print(' ');
  }
  else
  {
    Serial.print(val, prec);
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1); // . and -
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
    for (int i=flen; i<len; ++i)
      Serial.print(' ');
  }
  smartDelay(0);
}

static void printInt(unsigned long val, bool valid, int len)
{
  char sz[32] = "*****************";
  if (valid)
    sprintf(sz, "%ld", val);
  sz[len] = 0;
  for (int i=strlen(sz); i<len; ++i)
    sz[i] = ' ';
  if (len > 0) 
    sz[len-1] = ' ';
  Serial.print(sz);
  smartDelay(0);
}