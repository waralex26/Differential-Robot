#include <CylinderRecuperation.h>

CylinderRecuperation mechanism;
bool CylinderRecuperation::s_isInterrupted = false;

void setup() {
  pinMode(2, INPUT);
  Serial.begin(115200);
  attachInterrupt(digitalPinToInterrupt(2), interrupt, RISING);
  mechanism.initialize(5,6);
  
}





void loop() {
  //Serial.println(50);
  //Serial.println(CylinderRecuperation::s_isInterrupted); 
  Serial.println(digitalRead(2));
  mechanism.loop();
  if (CylinderRecuperation::s_isInterrupted)
    mechanism.interrupted();
}

void interrupt()
{
  CylinderRecuperation::s_isInterrupted = true;
  //Serial.println(digitalRead(2));
  mechanism.interrupted();
  //Serial.println("Trigger");
}

