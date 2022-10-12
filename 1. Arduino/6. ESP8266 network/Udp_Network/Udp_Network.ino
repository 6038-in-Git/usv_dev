#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

//const char* ssid = "MERCURY_2.4G_0122";
//const char* password = "hch4pw7n";
const char* ssid = "Hieu Thao 2,4g";
const char* password = "0944012555";
int pinReading = 0;

WiFiUDP Udp;
unsigned int localUdpPort = 4210;  // local port to listen on
char incomingPacket[100];  // buffer for incoming packets
char packet_lolin[100];
char packet_3m[100];
char  replyPacket[] = "Hi there, I'm Lolin 192.168.1.35 ";  // a reply string to send back
char char_3m[] = "Linh kien 3m: ";
char char_lolin_2[] = "Lolin 2: ";
char char_tab[] = "; ";
IPAddress laptop(192,168,1,30);
IPAddress linhkien_3m(192,168,1,21);
IPAddress lolin(192,168,1,35);
IPAddress lolin_2(192,168,1,47);

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

  int len = Udp.read(incomingPacket, 255);
  if (len > 0)
  {
    incomingPacket[len] = 0;
  }
  Serial.print("linhkien_3m");
  Serial.print("; ");

  if (Udp.remoteIP()==linhkien_3m)
  {
    strcpy(packet_3m,incomingPacket);
    Serial.printf("UDP packet contents: %s", incomingPacket);
    Serial.print("; ");
    Serial.print(Udp.remoteIP());
    Serial.print("; ");
    Serial.print(Udp.remotePort()); 
    Serial.print("; ");
  }

  packetSize = Udp.parsePacket();

//--------------------------------------------
  len = Udp.read(incomingPacket, 255);
  if (len > 0)
  {
    incomingPacket[len] = 0;
  }
  Serial.print("Lolin_2");
  Serial.print("; ");
  
  if (Udp.remoteIP()==lolin_2)
  {
    strcpy(packet_lolin,incomingPacket);
    Serial.printf("UDP packet contents: %s", incomingPacket);
    Serial.print("; ");
    Serial.print(Udp.remoteIP());
    Serial.print("; ");
    Serial.print(Udp.remotePort()); 
    Serial.print("; ");
  } 

//-------------------------------------------

    Udp.beginPacket(laptop, 4210);
    Udp.write(replyPacket);
    Udp.write(char_tab);

    Udp.write(char_3m);
    Udp.write(packet_3m);
    Udp.write(char_tab);
    
    Udp.write(char_lolin_2);
    Udp.write(packet_lolin);
    Udp.write(char_tab);
    
    Udp.endPacket();
    
    Serial.println();
    Serial.println();
}

    // send back a reply, to the IP address and port we got the packet from
    //pinReading = analogRead(A0);
    //int digit_4 = pinReading/1000;
    //int digit_3 = (pinReading - (digit_4*1000))/100;
    //int digit_2 = (pinReading - (digit_4*1000)-(digit_3*100))/10;
    //int digit_1 = pinReading - (digit_4*1000)-(digit_3*100) - (digit_2*10);
    //digit_4 = digit_4 + 48;
    //digit_3 = digit_3 + 48;
    //digit_2 = digit_2 + 48;
    //digit_1 = digit_1 + 48;

    //Udp.write(digit_4);
    //Udp.write(digit_3);
    //Udp.write(digit_2);
    //Udp.write(digit_1);
