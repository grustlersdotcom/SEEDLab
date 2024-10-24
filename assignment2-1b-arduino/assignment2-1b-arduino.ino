#include <Wire.h>
#define MY_ADDR 8

void setup() {
  // put your setup code here, to run once:
Wire.begin(MY_ADDR);
Wire.onReceive(receive); //Sets the ISR when the arduino receives information over i2c
Wire.onRequest(request); //Sets the ISR when the arduino receives an information request over i2c
}

//Set up all of the variables.
volatile uint8_t offset = 0;
volatile uint8_t value = 0;
volatile uint8_t msgLength = 0;
volatile uint8_t reply = 0;

void loop() {
  if (msgLength > 0) {
    //Sets the reply to whatever the arduino got + 100. Breaks if value>155
    reply = value + 100;
    msgLength = 0;
    
  }

}

//ISR function when the arduino receives i2c information
void receive() {
  offset = Wire.read();
  while (Wire.available()) {
    value = Wire.read();
    msgLength++;
    
  }
}

//ISR function when the arduino receives an i2c information request
void request() {
  Wire.write(reply);
  reply = 0;
}
