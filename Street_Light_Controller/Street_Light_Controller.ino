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
  }
  else
  {
    Serial.println("Night");
    Serial.println("Distance is:");
    Serial.println(sonar.ping_cm()); 

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
    
    //Turn on the light if distance is less than 20 cm if greater than that keep it off. 
    if (distance < 4)
    { 
      digitalWrite(LED,HIGH);
      Serial.println("\n LED on");
    }
  
    else 
    {
      digitalWrite(LED,LOW);
      Serial.println("\n LED off");
    }
  }
  
delay(2000);
  
}
