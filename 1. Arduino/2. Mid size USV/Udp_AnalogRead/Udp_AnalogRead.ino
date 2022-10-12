//#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

//const char* ssid = "MERCURY_2.4G_0122";
//const char* password = "hch4pw7n";
const char* ssid = "Hieu Thao 2,4g";
const char* password = "0944012555";
int pinReading = 0;

WiFiUDP Udp;
unsigned int localUdpPort = 4210;  // local port to listen on
char incomingPacket[255];  // buffer for incoming packets
char  replyPacket[] = "Hi there! Analog reading = ";  // a reply string to send back

void setup()
{
  Serial.begin(115200);
  Serial.println();

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


void loop()
{
  int packetSize = Udp.parsePacket();
    // receive incoming UDP packets
    Serial.printf("Received %d bytes from %s, port %d", packetSize, Udp.remoteIP().toString().c_str(), Udp.remotePort());
    Serial.print("; ");
    int len = Udp.read(incomingPacket, 255);
    if (len > 0)
    {
      incomingPacket[len] = 0;
    }
    Serial.printf("UDP packet contents: %s\n", incomingPacket); 
    Serial.print("; ");
    Serial.print(Udp.remoteIP());
    Serial.print("; ");
    Serial.print(Udp.remotePort()); 
    Serial.print("; ");


    // send back a reply, to the IP address and port we got the packet from
    pinReading = analogRead(A0);
    int digit_4 = pinReading/1000;
    int digit_3 = (pinReading - (digit_4*1000))/100;
    int digit_2 = (pinReading - (digit_4*1000)-(digit_3*100))/10;
    int digit_1 = pinReading - (digit_4*1000)-(digit_3*100) - (digit_2*10);
    digit_4 = digit_4 + 48;
    digit_3 = digit_3 + 48;
    digit_2 = digit_2 + 48;
    digit_1 = digit_1 + 48;
    
    Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
    Udp.write(replyPacket);
    Udp.write(digit_4);
    Udp.write(digit_3);
    Udp.write(digit_2);
    Udp.write(digit_1);
    Udp.endPacket();
    Serial.println();
}
