//Code to get analog reading of LDR 
int light;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  //reading the analog value from pin A0
  light = analogRead(A0);
  Serial.println(light);
  delay(100);
}
