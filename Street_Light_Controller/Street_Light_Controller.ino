#include <NewPing.h>
#include "DHT.h"


int temp;
const byte dhtPin = 4;
#define DHTTYPE DHT11
#define TRIGGER_PIN  10
#define ECHO_PIN     11
#define LED 2
#define MAX_DISTANCE 200
float duration, distance, c;
int iterations = 5;
int light;
int humidity;
float counts, mv, perc;
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
  //Reading the voltage of battery 
  counts = analogRead(A1);
  Serial.println(counts);
  //Calculating the accurate voltage
  mv = counts * 4.63 / 1023;
  //Mapping the voltage to accurate battery percentage
  perc = mapb(mv, 3.6, 4.2, 0, 100);
  Serial.print("Voltage= ");
  Serial.println(mv);
  Serial.print("Battery level= ");
  Serial.print(perc);
  Serial.println(" %");
  
  //reading the analog value from pin A0
  light = analogRead(A0);
  //printing the analog values on the serial monitor
  Serial.println(light);
  //Checking day and night
  if (light > 150)
  {
    Serial.println("Day");
    delay(500);
  }
  else
  {
    Serial.println("Night");
    Serial.print("Distance is:");
    Serial.print(sonar.ping_cm()); 

    //duration = pulseIn(11, HIGH);
    duration = sonar.ping();
    Serial.println("Duration = ");
    Serial.print(duration);
   
    //get the temp in celcius (default)
    temp = dht.readTemperature();
    humidity = dht.readHumidity();
  
    //calculate the speed of sound based on value from temp sensor
    c = 331.4 + temp * 0.606 + humidity * 0.0124;
    c = c * 100 * 0.000001;
    //calculate the distance from duration and speed of sound
    distance = c * (duration/2) ;
  
    Serial.println("\nTemperature = ");
    Serial.print(temp);
    Serial.print(" Deg C  ");

    Serial.println("\nHumidity = ");
    Serial.print(humidity);
    
    Serial.println("\nSpeed of sound = ");
    Serial.print(c);
    
    Serial.println("\nDistance = ");
    Serial.print(distance);

    //Turn on the light if distance is less than 4 cm and greater than 0 cm, if greater than that keep it off. 
      if (distance > 0 && distance < 4)
      { 
        digitalWrite(LED,HIGH);
        Serial.println("\n LED on");
      }
  
      else 
      {
        digitalWrite(LED,LOW);
        Serial.println("\n LED off");
      }
  
    delay(2000);
  }
}
float mapb(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
