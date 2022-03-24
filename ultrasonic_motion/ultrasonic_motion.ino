#include <NewPing.h>
#include "DHT.h"


int temp;
const byte dhtPin = 4;
#define DHTTYPE DHT11
#define TRIGGER_PIN  10
#define ECHO_PIN     11
#define MAX_DISTANCE 200
long duration, distance, c;

DHT dht(dhtPin, DHTTYPE);

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

void setup() 
{
  dht.begin();
  Serial.begin(9600);
  delay(50);

}

void loop() 
{

Serial.println("Distance is:"  );
Serial.println(sonar.ping_cm());
 delay(500);

  duration = pulseIn(ECHO_PIN, HIGH);

  //get the temp in celcius (default)
  Serial.println("Temp  = ");
    temp = dht.readTemperature();
    Serial.println(temp);
    Serial.println(" Deg C  ");
    Serial.println("Temp  = ");

   //calculate the speed of sound based on value from temp sensor
  c = 331.3 + temp * 0.606;
  
  //calculate the distance from duration and speed of sound
  distance = c * duration;

  Serial.println("Temperature = ");
  Serial.print(temp);
  Serial.println("Speed of sound = ");
  Serial.print(c);
  Serial.println("Distance = ");
  Serial.print(distance);
  delay(2000);

if (distance < 20)

{ digitalWrite(led,HIGH);

}

else {

digitalWrite(led,LOW);

}
}
