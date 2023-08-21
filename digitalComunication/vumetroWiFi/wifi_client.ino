#ifdef ESP32
  #include <WiFi.h>
  #include <HTTPClient.h>
#else
  #include <ESP8266WiFi.h>
  #include <ESP8266HTTPClient.h>
#endif

#define one 2
#define two 4
#define three 16

const char* ssid = "preciado";      //same name as given in access point
const char* password = "123456789";

//Your IP address or domain name with URL path
const char* serverNameTemp = "http://192.168.4.1/Hello";

void setup() {
  Serial.begin(115200);
  pinMode(one, OUTPUT);
  pinMode(two, OUTPUT);
  pinMode(three, OUTPUT);
  
  WiFi.begin(ssid, password);
  Serial.println("Connecting");
  while(WiFi.status() != WL_CONNECTED){ 
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected to WiFi network with IP Address: ");
  Serial.println(WiFi.localIP());
}

String inputStr;
float input;
unsigned long previousMillis = 0;
const long interval = 1000;

void loop(){
  unsigned long currentMillis = millis();
  
  if(currentMillis - previousMillis >= interval) 
  {
     // Check WiFi connection status
    if(WiFi.status()== WL_CONNECTED )
    { 
      inputStr = httpGETRequest(serverNameTemp);
      
      Serial.println("Received message: " + inputStr);
      input = inputStr.toInt();
      if(input < 1){
        digitalWrite(one, LOW);
        digitalWrite(two, LOW);
        digitalWrite(three, LOW);
      }
      else if (input < 2){
        digitalWrite(one, HIGH);
        digitalWrite(two, LOW);
        digitalWrite(three, LOW);
      }
      else if (input < 3){
        digitalWrite(one, HIGH);
        digitalWrite(two, HIGH);
        digitalWrite(three, LOW);
      }
      else if (input < 4){
        digitalWrite(one, HIGH);
        digitalWrite(two, HIGH);
        digitalWrite(three, HIGH);
      }
      else{
        digitalWrite(one, LOW);
        digitalWrite(two, LOW);
        digitalWrite(three, LOW);
      }
      
      // save the last HTTP GET Request
      previousMillis = currentMillis;
    }
    else 
    {
      Serial.println("WiFi Disconnected");
    }
  }
}

String httpGETRequest(const char* serverName) 
{
  WiFiClient client;
  HTTPClient http;
    
  // Your Domain name with URL path or IP address with path
  http.begin(client, serverName);
  
  // Send HTTP POST request
  int httpResponseCode = http.GET();
  
  String payload = "--"; 
  
  if (httpResponseCode>0) 
  {
    Serial.print("HTTP Response code: ");
    Serial.println(httpResponseCode);
    payload = http.getString();
  }
  else 
  {
    Serial.print("Error code: ");
    Serial.println(httpResponseCode);
  }
  // Free resources
  http.end();

  return payload;
}
