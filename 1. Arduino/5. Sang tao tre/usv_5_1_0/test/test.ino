// Load Wi-Fi library
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

// Replace with your network credentials
const char* ssid     = "Linh";
const char* password = "sinhnhatcuatao";

  
  WiFiUDP Udp;

 // IP Address
  IPAddress laptop(192,168,1,99);
  IPAddress linhkien_3m(192,168,1,102);  //driver
  IPAddress lolin(192,168,1,101);  //controller
  IPAddress lolin_2(192,168,1,100);  //observer

// Set your Static IP address
IPAddress local_IP(192, 168, 1, 100);
// Set your Gateway IP address
IPAddress gateway(192, 168, 1, 1);

IPAddress subnet(255, 255, 0, 0);
IPAddress primaryDNS(8, 8, 8, 8);   //optional
IPAddress secondaryDNS(8, 8, 4, 4); //optional

void setup() {
  Serial.begin(115200);

  // Configures static IP address
  if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) {
    Serial.println("STA Failed to configure");
  }
  
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
void loop(){
  Udp.beginPacket(laptop, 4210);
  Udp.write("usv_yaw: ");
  Udp.endPacket();
}