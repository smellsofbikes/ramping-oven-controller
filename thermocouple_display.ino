//arduino, based on code from multiple sources

#include <SPI.h>
#include "Adafruit_MAX31855.h"
#include <LiquidCrystal.h>

// hardware SPI
#define MAXDO   3
#define MAXCS   4
#define MAXCLK  5

// initialize the Thermocouple as hardware spi
Adafruit_MAX31855 thermocouple(MAXCLK, MAXCS, MAXDO);
LiquidCrystal lcd(11, 10, 9, 8, 7, 6);

void setup() {
  Serial.begin(9600);
  lcd.begin(20, 4);
 
  while (!Serial) delay(1); // wait for Serial on Leonardo/Zero, etc

  Serial.println("MAX31855 test");
  lcd.print("MAX31855 test");
  // wait for MAX chip to stabilize
  delay(500);
}

void loop() {
  // basic readout test, just print the current temp
   Serial.print("Internal Temp = ");
   Serial.println(thermocouple.readInternal());
   
   double c = thermocouple.readCelsius();
   if (isnan(c)) {
     Serial.println("Something wrong with thermocouple!");
   } else {
     Serial.print("C = "); 
     Serial.println(c);
     lcd.setCursor(0,1);
     lcd.print("C = ");
     lcd.setCursor(5,1);
     lcd.print(c);
   }
   //Serial.print("F = ");
   //Serial.println(thermocouple.readFarenheit());
 
   delay(1000);
}
