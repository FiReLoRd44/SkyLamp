
float counts = 0;
float mv = 0;
float perc; 
void setup(){
  Serial.begin(9600);
}

void loop(){
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
  delay(1000);
}
//function to map battery percentage
float mapb(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
