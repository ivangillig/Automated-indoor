#include <LiquidCrystal_I2C.h>
#include <Wire.h>

LiquidCrystal_I2C lcd(0x27, 16, 4);

void setup() {
  
lcd.init();
lcd.backlight();

lcd.setCursor(0, 0);
lcd.print("Hola");
  
}

void loop() {
  // put your main code here, to run repeatedly:

}
