#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>

TinyGPSPlus gps;

SoftwareSerial ss(0, 2);

void setup(){
  Serial.begin(9600);
  ss.begin(9600);
}

void loop(){
  while (ss.available()){
    gps.encode(ss.read());
    if (gps.location.isUpdated()>0){
      Serial.print("Latitude= "); 
      Serial.print(gps.location.lat(), 6);
      Serial.print(" Longitude= "); 
      Serial.println(gps.location.lng(), 6);
    }
  }
}
