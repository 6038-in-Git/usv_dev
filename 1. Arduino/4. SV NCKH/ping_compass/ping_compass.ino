/**
 *  This example is targeted toward the arduino platform
 *
 *  This example demonstrates the most simple usage of the Blue Robotics
 *  Ping1D c++ API in order to obtain distance and confidence reports from
 *  the device.
 *
 *  This API exposes the full functionality of the Ping1D Echosounder
 *
 *  Communication is performed with a Blue Robotics Ping1D Echosounder
 */

// ------------------------------------------------------------------IMU ------------------------------------------------------------------

#include "ping1d.h"

#include "SoftwareSerial.h"

#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include "I2Cdev.h"

#include "Adafruit_MPU6050.h"
#include "Adafruit_Sensor.h"
#include <Wire.h>
#include <QMC5883LCompass.h>


// ------------------------------------------------------------------PING ------------------------------------------------------------------

// This serial port is used to communicate with the Ping device
// If you are using and Arduino UNO or Nano, this must be software serial, and you must use
// 9600 baud communication
// Here, we use pin 9 as arduino rx (Ping tx, white), 10 as arduino tx (Ping rx, green)

static const uint8_t arduinoRxPin = 14;
static const uint8_t arduinoTxPin = 12;
static const uint8_t infrared_1_pin = 15;
static const uint8_t infrared_2_pin = 2;

int infrared_1 = 1;
int infrared_2 = 1;
int yaw;
int depth;
int confidence;
bool ping_valid;

SoftwareSerial pingSerial = SoftwareSerial(arduinoRxPin, arduinoTxPin);
static Ping1D ping { pingSerial };


// ---------------------------------------Wifi-------------------------------------------------
// UDP WIFI
  //const char* ssid = "MERCURY_2.4G_0122";
  //const char* password = "hch4pw7n";
  //const char* ssid = "KTHK 207";
  //const char* password = "hmt123456789xyz";
  const char* ssid = "POCO F3";
  const char* password = "12345789";
  
  
  const char* end_char = "; ";
  const char* terminate_char = "~";
  
  WiFiUDP Udp;
  unsigned int localUdpPort = 4210;  // local port to listen on
  char incomingPacket[255];  // buffer for incoming packets
  char  replyPacket[] = "USV Packet: ";  // a reply string to send back

// UDP char
 
  char distance_ping_char [] ="distance_ping_char: ";
  char confidence_char [] = "confidence_char: ";
  char IRsensor_1 [] = "IRsensor_1:  ";
  char IRsensor_2 [] = "IRsensor_2:  ";
  char compass_char [] = "compass: ";

// COMPASS
QMC5883LCompass compass; 
  
void setup()
{
  pingSerial.begin(115200);
  Serial.begin(115200);
  
  pinMode(infrared_1_pin, INPUT);
  pinMode(infrared_2_pin, INPUT);
  pinMode(13, INPUT);
    
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
  // COMPASS
   compass.init(); 
  
    Serial.println("");
}

void loop()
{
    infrared_1 = digitalRead(infrared_1_pin);
    //Serial.print("infrared_1: ");
    //Serial.print(infrared_1);
    //Serial.print("; ");
    
    infrared_2 = digitalRead(infrared_2_pin);
    //Serial.print("infrared_2: ");
    //Serial.print(infrared_2);
    //Serial.print("; ");

 // COMPASS
  // Read compass values
  compass.read();

  // Return Azimuth reading
  yaw = compass.getAzimuth();
  
  Serial.print("Yaw: ");
  Serial.print(yaw);
  
 // UDP RECIEVE TARGET COORDINATES
    int packetSize = Udp.parsePacket();
    
    // receive incoming UDP packets
    int len = Udp.read(incomingPacket, 255);
    if (len > 0)
    {
      incomingPacket[len] = 0;
    }
 
 // -------------------------------- PING --------------------------------
    depth = ping.distance();
    confidence = ping.confidence();

    ping_valid = ping.update();
    //if (ping.update()) 
    //{
        Serial.print("Depth: ");
        Serial.print(depth);
        Serial.print("\tConfidence: ");
        Serial.print(confidence);
    //}

    // -------------------------------------------- UDP SEND LOG DATA --------------------------------------------
    Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
    Udp.write(replyPacket);


    // PING SONAR
    Udp.write(distance_ping_char);
    udp_send(depth, 5, 0 );
    
    Udp.write(confidence_char);
    udp_send(confidence, 3, 0 );
    
    Udp.write(IRsensor_1);
    udp_send(infrared_1, 1, 0 );

    Udp.write(IRsensor_2);
    udp_send(infrared_2, 1, 0 );

    Udp.write(compass_char);
    udp_send(yaw, 3, 0 );
    
    Udp.write(terminate_char);
    Udp.endPacket();
    
  // END LOOP
    Serial.println(" ");
}

void udp_send (double data, int top_digit, int lowest_decimal)
{
  if (data<0)
  {
    data = abs(data);
    Udp.write(45);
  }
  
  int string_size = top_digit + lowest_decimal + 1;
  char udp_string [string_size];
  double  temp = data;
  
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
