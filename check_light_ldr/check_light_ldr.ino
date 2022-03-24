//Code to get analog reading of LDR 
int light;
void setup() {
  // put your setup code here, to run once:
  //serial transmission begining on 9600 baud rate
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  //reading the analog value from pin A0
  light = analogRead(A0);
  //printing the analog values on the serial monitor
  Serial.println(light);
  //Checking day and night
  if (light > 300)
  {
    Serial.println("Day");
  }
  else
  {
    Serial.println("Night");
  }
  delay(100);
}
