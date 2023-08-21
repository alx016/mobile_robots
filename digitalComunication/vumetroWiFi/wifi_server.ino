#include "ESP8266WiFi.h"
#include "ESPAsyncWebServer.h"

const char* ssid = "preciado";
const char* password = "123456789";
String val="";

float lectura;
float volt = 0;

AsyncWebServer server(80);

void setup() {
  Serial.begin(115200);
  pinMode(A0, INPUT);
  delay(100);
  Serial.println();

  Serial.print("Setting AP (Access Point)...");

  WiFi.softAP(ssid, password);

  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.print(IP);
  server.on("/Hello", HTTP_GET, [](AsyncWebServerRequest *request){
    request -> send_P(200, "text/plain", val.c_str());
  });
  server.begin();

}

float counter = 0.5;
void loop() {
  // put your main code here, to run repeatedly:
  lectura = analogRead(A0);
  volt = lectura / 1023 * 5.0;
  val = String(volt);
  Serial.print("Volts: ");
  Serial.print(volt);
  delay(500);
}
