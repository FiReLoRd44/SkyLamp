#define LED D1
#define LED2 D2
#include <ESP8266WiFi.h>
void setup() {
  pinMode(LED,OUTPUT);
  pinMode(LED2, OUTPUT);
}

const char* ssid = "Shah Home";
const char* password = "hetami@5";
 
int ledPin = 13;
WiFiServer server(80);
 
void setup() {
  Serial.begin(115200);
  delay(10);
 
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);
 
  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
 
  WiFi.begin(ssid, password);
 
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  
  server.begin();
  Serial.println("Server started");
 
  Serial.print("Use this URL to connect: ");
  Serial.print("http://");
  Serial.print(WiFi.localIP());
  Serial.println("/");
}

void loop() {
  digitalWrite(LED,HIGH);
  delay(1000);
  digitalWrite(LED2,LOW);
  delay(1000);
}
