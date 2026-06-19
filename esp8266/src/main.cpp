#include <Arduino.h>           // Arduino core functions
#include <ESP8266WiFi.h>       // ESP8266 WiFi library

const char* ssid = "Ideapad320";      // Apne router ka WiFi name
const char* password = "Robo@#centos$";  // Apne router ka password

void setup() {

  Serial.begin(115200);        // Serial monitor start
  delay(1000);          // Thoda wait

  Serial.println("Hello");

  Serial.println();
  Serial.println("Starting ESP8266...");

  WiFi.mode(WIFI_STA);         // ESP ko station mode me set karta hai
  WiFi.begin(ssid, password);  // WiFi connect start

  Serial.print("Connecting to WiFi");

  while (WiFi.status() != WL_CONNECTED) { // Jab tak connect na ho
    delay(500);
    Serial.print(".");
  }

  Serial.println();
  Serial.println("WiFi Connected!");

  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());   // ESP ka IP address print
}

void loop() {

  Serial.print("Signal Strength: ");
  Serial.println(WiFi.RSSI());  // WiFi signal strength print

  delay(5000);                  // 5 sec delay

}
