#define battery A0
int value = 0;
float voltage;
float perc;

void setup(){
  Serial.begin(9600);
}

void loop(){
  value = analogRead(A0);
  voltage = value * 5.0/1023;
  perc = map(voltage, 3.6, 4.2, 0, 100);
  Serial.print("Voltage= ");
  Serial.println(voltage);
  Serial.print("Battery level= ");
  Serial.print(perc);
  Serial.println(" %");
  delay(500);
}
