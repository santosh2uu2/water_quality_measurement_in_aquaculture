#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <DHT.h>
#define Relay1            D0
#define Relay2            D1
#define Relay3            D2
#define Relay4            D3
#define DHTPIN            D5
#define DHTTYPE DHT11    
DHT dht(DHTPIN, DHTTYPE);
const char* ssid = "SRKR_IDE";
const char* password = "Tech$9889";
const char* mqtt_server = "192.168.68.175";
const char* username = "MQTT_USERNAME";
const char* pass = "MQTT_PASSWORD";
#define sub1 "device2/relay1"
#define sub2 "device2/relay2"

char str_hum_data[10];
char str_temp_data[10];
char str_tds_data[10];
char str_ldr_data[10];
WiFiClient espClient;
PubSubClient client(espClient);
unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE  (50)
char msg[MSG_BUFFER_SIZE];
int value = 0;
int ldr = D4;
int bs = 0;
#define ss 5
#define rst 14
#define dio0 2
int counter = 0;
#define TdsSensorPin A0
#define VREF 5.0              // analog reference voltage(Volt) of the ADC
#define SCOUNT  30            // sum of sample point
int analogBuffer[SCOUNT];     // store the analog value in the array, read from ADC
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0;
int copyIndex = 0;
float averageVoltage = 0;
float tdsValue = 0;
float temperature = 16;       // current temperature for compensation
// median filtering algorithm
int getMedianNum(int bArray[], int iFilterLen){
  int bTab[iFilterLen];
  for (byte i = 0; i<iFilterLen; i++)
  bTab[i] = bArray[i];
  int i, j, bTemp;
  for (j = 0; j < iFilterLen - 1; j++) {
    for (i = 0; i < iFilterLen - j - 1; i++) {
      if (bTab[i] > bTab[i + 1]) {
        bTemp = bTab[i];
        bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
      }
    }
  }
  if ((iFilterLen & 1) > 0){
    bTemp = bTab[(iFilterLen - 1) / 2];
  }
  else {
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
  }
  return bTemp;
}
void setup_wifi()
{
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  randomSeed(micros());
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  if (strstr(topic, sub1))
  {
    for (int i = 0; i < length; i++) {
      Serial.print((char)payload[i]);
    }
    Serial.println();  
    if ((char)payload[0] == '1') {
      digitalWrite(Relay1, HIGH);
      digitalWrite(Relay2, LOW);
       Serial.println("good");
      delay(500);
       digitalWrite(Relay1, LOW);
      digitalWrite(Relay2, LOW);
      delay(500);    
    } else {
      digitalWrite(Relay1, LOW);
      digitalWrite(Relay2, HIGH);
      delay(700);
      digitalWrite(Relay1, LOW);
      digitalWrite(Relay2, LOW);
      delay(500);// Turn the LED off by making the voltage HIGH
    }
  }
  else if ( strstr(topic, sub2))
  {
    for (int i = 0; i < length; i++) {
      Serial.print((char)payload[i]);
    }
    Serial.println();
    // Switch on the LED if an 1 was received as first character
    if ((char)payload[0] == '1') {
      digitalWrite(Relay3, HIGH);
      digitalWrite(Relay4, LOW);
      Serial.println("good1");      
    } else {
      digitalWrite(Relay3, LOW);
      digitalWrite(Relay4, LOW);    // Turn the LED off by making the voltage HIGH
    }
  }
  else
  {
    Serial.println("unsubscribed topic");
  }
}
void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");    
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);  
    if (client.connect(clientId.c_str(), username, pass) ) {
      Serial.println("connected");    
      client.subscribe(sub1);
      client.subscribe(sub2);
     
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");    
      delay(5000);
    }
  }
}
void setup() {
  pinMode(Relay1, OUTPUT);
  pinMode(Relay2, OUTPUT);
  pinMode(Relay3, OUTPUT);
  pinMode(Relay4, OUTPUT);
  pinMode(ldr,INPUT);
  pinMode(TdsSensorPin,INPUT);
  Serial.begin(115200);
  dht.begin();
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
}
void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  unsigned long now = millis();
  if (now - lastMsg > 2000) {
    static unsigned long analogSampleTimepoint = millis();
  if(millis()-analogSampleTimepoint > 40U){     //every 40 milliseconds,read the analog value from the ADC
    analogSampleTimepoint = millis();
    analogBuffer[analogBufferIndex] = analogRead(TdsSensorPin);    //read the analog value and store into the buffer
    analogBufferIndex++;
    if(analogBufferIndex == SCOUNT){
      analogBufferIndex = 0;
    }
  }
 
  static unsigned long printTimepoint = millis();
  if(millis()-printTimepoint > 800U){
    printTimepoint = millis();
    for(copyIndex=0; copyIndex<SCOUNT; copyIndex++){
      analogBufferTemp[copyIndex] = analogBuffer[copyIndex];
     
      // read the analog value more stable by the median filtering algorithm, and convert to voltage value
      averageVoltage = getMedianNum(analogBufferTemp,SCOUNT) * (float)VREF / 1024.0;
     
      //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
      float compensationCoefficient = 1.0+0.02*(temperature-25.0);
      //temperature compensation
      float compensationVoltage=averageVoltage/compensationCoefficient;
     
      //convert voltage value to tds value
      tdsValue=(133.42*compensationVoltage*compensationVoltage*compensationVoltage - 255.86*compensationVoltage*compensationVoltage + 857.39*compensationVoltage)*0.5;
     
      //Serial.print("voltage:");
      //Serial.print(averageVoltage,2);
      //Serial.print("V   ");
      Serial.print("TDS Value:");
      Serial.print(tdsValue,0);
      Serial.println("ppm");
       dtostrf(tdsValue, 4, 2, str_tds_data);  
    bs = digitalRead(ldr);
     dtostrf(bs, 4, 2, str_ldr_data);
    float hum_data = dht.readHumidity();
    Serial.println(hum_data);  
    dtostrf(hum_data, 4, 2, str_hum_data);
    float temp_data = dht.readTemperature(); // or dht.readTemperature(true) for Fahrenheit
    dtostrf(temp_data, 4, 2, str_temp_data);
    lastMsg = now;
    Serial.print("Publish message: ");
    Serial.print("Temperature - "); Serial.println(str_temp_data);
    client.publish("device2/temp", str_temp_data);
    Serial.print("Humidity - "); Serial.println(str_hum_data);
    client.publish("device2/hum", str_hum_data);
     Serial.print("ldr - "); Serial.println(str_ldr_data);
    client.publish("device2/light", str_ldr_data);
    Serial.print("tds - "); Serial.println(str_tds_data);
    client.publish("device2/tds", str_tds_data);
  }
}}}
