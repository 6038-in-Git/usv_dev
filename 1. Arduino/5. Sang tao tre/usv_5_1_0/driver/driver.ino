// -------------------------------CODE for ESP8266 Nodemcu-------------------------------

#include <Servo.h>
#include <SoftwareSerial.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <USVFunction.h>

USVFunction usv;

// MODE CHOICE
  //const int pin_mode = 14;      // D5
  const int pin_mode = 13;      // D7
  int mode = false;

// MOTOR VARIABLES

  const int pin_motor_2 = 12;   // D0
  const int pin_motor_1 = 15;   // D5
   
  Servo f_1;
  Servo f_2;   
  int throttle;
  int turn;
  int thrust;
  int moment;
  int radio_thrust;
  int radio_moment;
  
// UDP WIFI
  const char* ssid = "MERCURY_2.4G_0122";
  const char* password = "hch4pw7n";
  // const char* ssid = "KTHK 207";
  // const char* password = "hmt123456789xyz";
  // const char* ssid = "Linh";
  // const char* password = "sinhnhatcuatao";
  // const char* ssid = "POCO F3";
  // const char* password = "12345789";
  // const char* ssid = "TP-LINK_3D77A0";
  //const char* password = "";
  const char* end_char = "; ";
  const char* terminate_char = "~";
  
  WiFiUDP Udp;
  unsigned int localUdpPort = 4210;  // local port to listen on
  char incomingPacket[255];  // buffer for incoming packets
  char  replyPacket[] = "USV Packet: ";  // a reply string to send back

  double temp;
  float temp2;
  
  // IP Address
  // IPAddress laptop(192,168,139,214);
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

  // Serial
  Serial.begin(115200);

  // MOTOR SETUP
  f_1.attach(pin_motor_1,1000,2000);    
  f_2.attach(pin_motor_2,1000,2000);   

 // Configures static IP address

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

  // UDP RECIEVE TARGET COORDINATES
    int packetSize = Udp.parsePacket();
    
    // receive incoming UDP packets
    int len = Udp.read(incomingPacket, 255);
    if (len > 0)
    {
      incomingPacket[len] = 0;
    }

  // -------------------------------- Parse from Controller --------------------------------

    if (Udp.remoteIP()==lolin)
    {     
        thrust = parse_integer_3_digit(7);
        moment = parse_integer_3_digit(14);
        radio_thrust = parse_integer_3_digit(29);
        radio_moment = parse_integer_3_digit(36);
        mode = incomingPacket[49]-48;
    }
    Serial.print(thrust);
    Serial.print(", ");
    Serial.print(moment);
    Serial.print(", ");
    Serial.print(radio_thrust);
    Serial.print(", ");
    Serial.print(radio_moment);
    Serial.print(", ");
    Serial.print(mode);
 
  // -------------------------------- CONTROL MOTOR --------------------------------
    if(mode ==  1) motor_control(thrust, moment);
    else motor_control(radio_thrust, radio_moment);
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

void motor_control(float _thrust, float _moment)
{
  int speed_1 = _thrust*1.5 + _moment*3 + 6 ; // PWM output, range 0-255
  int speed_2 = _thrust*1.5 - _moment*3 - 15;
  if (speed_1>200) speed_1 = 200;
  if (speed_2>200) speed_2 = 200;
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