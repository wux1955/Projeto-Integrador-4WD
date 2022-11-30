
/*
#include <HCSR04.h>
UltraSonicDistanceSensor distanceSensor(5, 3);  // Initialize sensor that uses digital pins 5 and 3.
void setup () {
  Serial.begin(9600);  // We initialize serial connection so that we could print values from sensor.
}
void loop () {
  Serial.println(distanceSensor.measureDistanceCm());
  delay(500);
}
*/

#include <HCSR04.h>
UltraSonicDistanceSensor distanceSensor(5, 3);

int encoder_pin = 7;
unsigned int rpm = 0;
float velocity = 0;
volatile byte pulses = 0;
unsigned long timeold = 0;
unsigned int pulsesperturn = 20;
const int wheel_diameter = 64;
static volatile unsigned long debounce = 0;


void setup(){
  Serial.begin(9600);
  pinMode(encoder_pin, INPUT);
  attachInterrupt(0, counter, RISING);
  pulses = 0;
  rpm = 0;
  timeold = 0;
  Serial.print("Segundos ");
  Serial.print("RPM ");
  Serial.print("Pulses ");
  Serial.print("Velocidade[Km/h]");
  Serial.println("Distância");
  }

void loop(){
  if (millis() - timeold >= 1000){
    noInterrupts();
    rpm = (60 * 1000 / pulsesperturn )/ (millis() - timeold)* pulses;
    velocity = rpm * 3.1416 * wheel_diameter * 60 / 1000000;
    timeold = millis();
    Serial.print(millis()/1000);
    Serial.print("       ");
    Serial.print(rpm,DEC);
    Serial.print("   ");
    Serial.print(pulses,DEC)
    Serial.print("     ");
    Serial.print(velocity,2);
    Serial.print("           ");
    Serial.println(distanceSensor.measureDistanceCm());
    pulses = 0;
    interrupts();
  }
}

void counter(){
  if(digitalRead (encoder_pin) && (micros()-debounce > 500) && digitalRead (encoder_pin)){ 
    debounce = micros();
    pulses++;
  }
  else; 
}

