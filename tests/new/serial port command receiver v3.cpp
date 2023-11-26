#include <VarSpeedServo.h>
#include <stdlib.h>

#define h_speed 15
#define g_speed 45
#define b_speed 30
#define garra_pin 2
#define haste1_pin 3
#define haste2_pin 4
#define base_pin 5

VarSpeedServo garra;
VarSpeedServo haste1;
VarSpeedServo haste2;
VarSpeedServo base;

void setup() {
  garra.attach(garra_pin);
  haste1.attach(haste1_pin);
  haste2.attach(haste2_pin);
  base.attach(base_pin);
  Serial.begin(9600);
}

void loop() {
  String comando_base;

  // Testa se tem alguma porta com comunicação ativa
  if(Serial.available() > 0){

    comando_base = Serial.readStringUntil('\n').toInt();
    base.slowmove(comando_base, b_speed);

    else {
      Serial.println("No command detected...");
    }
    
  }

}
