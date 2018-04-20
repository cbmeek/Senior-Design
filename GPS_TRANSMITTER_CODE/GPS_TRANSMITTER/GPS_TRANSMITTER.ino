#include <Arduino.h>   // required before wiring_private.h
#include "wiring_private.h" // pinPeripheral() function
#include <TinyGPS++.h>

// Instantiate objects
Uart Serial2(&sercom1, 12, 10, SERCOM_RX_PAD_3, UART_TX_PAD_2);
TinyGPSPlus gps;
TinyGPSCustom lat(gps, "PUBX", 3); // $PUBX sentence, 3rd element
TinyGPSCustom NS(gps, "PUBX", 4); // $PUBX sentence, 4rd element
TinyGPSCustom lon(gps, "PUBX", 5); // $PUBX sentence, 5th element
TinyGPSCustom EW(gps, "PUBX", 6); // $PUBX sentence, 6rd element
TinyGPSCustom ele(gps, "PUBX", 7); // $PUBX sentence,  7th element

#define VBATPIN A7
const float WEIGHT=0.05;
long elevation,oldElevation;
void SERCOM1_Handler()

{
  Serial2.IrqHandler();
}

void setup()
{
  Serial.begin(115200);
  Serial1.begin(19200);
  Serial2.begin(115200);
 
  // Assign pins 10 & 12 SERCOM functionality
  pinPeripheral(10, PIO_SERCOM);
  pinPeripheral(12, PIO_SERCOM);
  
}

void loop() {
  
  
  if(lat.isUpdated() || lon.isUpdated() || ele.isUpdated()){
    String tempEle = ele.value();
    float temp2Ele = tempEle.toFloat();
     
    Serial1.print("<"); //Start char for parsing
    Serial1.print(nmea2DD(lat.value(),NS.value()),9); Serial1.print(","); 
    Serial1.print(nmea2DD(lon.value(),EW.value()),9); Serial1.print(",");
    Serial1.print(ele.value()); Serial1.print(",");
    Serial1.print(batteryVoltage());
    Serial1.println(">"); //End char for parsing
  }
  
  while(Serial2.available()>0) gps.encode(Serial2.read());
}
float batteryVoltage(){
  float measuredvbat = analogRead(VBATPIN);
  measuredvbat *= 2;    // we divided by 2, so multiply back
  measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
  measuredvbat /= 1024; // convert to voltage
  return measuredvbat;
}

float nmea2DD(String nmea,String dir){
  String mm ="";
  float dd;
  if(nmea.length() == 10) //Latitude
  {    
    dd = nmea.substring(0,2).toFloat() + (nmea.substring(2).toFloat())/60;
    if(dir == "N") dd*=1;
    else if(dir == "S") dd*=-1;
    return dd; 
  }
  else if(nmea.length() == 11) //Longitude
  {
    dd = nmea.substring(0,3).toFloat() + (nmea.substring(3).toFloat())/60;
    if(dir == "E") dd *= 1;
    else if(dir == "W") dd *= -1;
    return dd;
  }
}
void movingAvg(float latitude, float longitude){
  totalLat = totalLat - latAr[idx]; 
  totalLon = totalLat - longAr[idx];
  latAr[idx] = latitude;
  longAr[idx] = longitude;
  totalLat = totalLat + latAr[idx]; 
  totalLon = totalLat + longAr[idx];
  idx++;
  if(idx>=avgWindow){
    idx = 0;  
  }
  avgLat = totalLat/avgWindow;
  avgLon = totalLon/avgWindow;
}

