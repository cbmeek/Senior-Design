#include <Arduino.h>   // required before wiring_private.h
#include "wiring_private.h" // pinPeripheral() function

#if defined(ARDUINO_SAMD_ZERO) && defined(SERIAL_PORT_USBVIRTUAL)
  // Required for Serial on Zero based boards
  #define Serial SERIAL_PORT_USBVIRTUAL
#endif

// Serial2 pin and pad definitions (in Arduino files Variant.h & Variant.cpp)
#define PIN_SERIAL2_RX       (34ul)               // Pin description number for PIO_SERCOM on D12
#define PIN_SERIAL2_TX       (36ul)               // Pin description number for PIO_SERCOM on D10
#define PAD_SERIAL2_TX       (UART_TX_PAD_2)      // SERCOM pad 2
#define PAD_SERIAL2_RX       (SERCOM_RX_PAD_3)    // SERCOM pad 3

// Instantiate the Serial2 class
Uart Serial2(&sercom1, PIN_SERIAL2_RX, PIN_SERIAL2_TX, PAD_SERIAL2_RX, PAD_SERIAL2_TX);

#define HC12_SET_PIN 11
#define PPS_PIN 13

void setup() {
  Serial.begin(115200);
  Serial1.begin(19200);
  Serial2.begin(9600);
  
  // Assign pins 10 & 12 SERCOM functionality
  pinPeripheral(10, PIO_SERCOM);
  pinPeripheral(12, PIO_SERCOM);
  
  pinMode(HC12_SET_PIN, OUTPUT);
  pinMode(PPS_PIN,INPUT);
}

void loop() {
 
  digitalWrite(HC12_SET_PIN, HIGH); //Set LOW to program HC12

 //HC12 <==> USB Serial Bridge
  if (Serial.available()) {      // If anything comes in Serial (USB)
    Serial1.write(Serial.read());   // read it and send it to HC12 Transceiver
  }

  if (Serial1.available()) {     // If anything comes in HC12 Transceiver
    Serial.write(Serial1.read());   // read it and send it to Serial (USB)
  }
  //GPS Module <==> USB Serial Bridge 
  if (Serial.available()){  // If anything comes in Serial (USB)
   Serial2.write(Serial.read());  // read it and send it to GPS Module
  }
  if (Serial2.available(){ //If anything comes in GPS Module
   Serial.write(Serial2.read());  // read it and send it to Serial (USB)
  }
}

void SERCOM1_Handler()    // Interrupt handler for SERCOM1
{
  Serial2.IrqHandler();
}
