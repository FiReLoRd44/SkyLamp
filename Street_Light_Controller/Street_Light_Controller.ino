#include <NewPing.h>
#include "DHT.h"


int temp;
const byte dhtPin = 4;
#define DHTTYPE DHT11
#define TRIGGER_PIN  10
#define ECHO_PIN     11
#define LED 2
#define MAX_DISTANCE 200
long duration, distance, c;
int light;

DHT dht(dhtPin, DHTTYPE);

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

void setup() 
{
  pinMode(LED,OUTPUT);
  dht.begin();
  //serial transmission begining on 9600 baud rate
  Serial.begin(9600);
  

}

void loop() 
{
 //reading the analog value from pin A0
  light = analogRead(A0);
  //printing the analog values on the serial monitor
  Serial.println(light);
  //Checking day and night
  if (light > 250)
  {
    Serial.println("Day");
  }
  else
  {
    Serial.println("Night");
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
    //Turn on the light if distance is less than 20 cm if greater than that keep it off. 
    if (distance < 20)
    { 
      digitalWrite(LED,HIGH);
    }
  
    else 
    {
      digitalWrite(LED,LOW);
    }
  }
  

  
}
