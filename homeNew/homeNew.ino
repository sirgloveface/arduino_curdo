#define MQTT_VERSION MQTT_VERSION_3_1_1
#define MQTT_MAX_PACKET_SIZE 200
#define PUBLISH_DELAY   15000
//Libraries
#include <SPI.h>
#include <Ethernet.h>
#include <PubSubClient.h>
#include <dht.h>
#include <PID_v1.h>

dht DHT;
//Constants
#define DHT22_PIN 2     // DHT 22  (AM2302) - what pin we're connected to
#define tempRelay 4
// Update these with values suitable for your network.
byte mac[] = {  0xDE, 0xED, 0xBA, 0xFE, 0xFE, 0xED };
IPAddress ip(192, 168, 1, 151);
IPAddress server(35, 184, 80, 23);

EthernetClient ethClient;
PubSubClient client(ethClient);
unsigned long lastSend;

float temp = 25.00;
bool relayTempstate = LOW;
float h = 50.00;
float t = 25.00;

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
double Kp=5, Ki=3, Kd=3;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;
  // 
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
    messageTemp += (char)payload[i];
  }
  Serial.println();
  if (String(topic) == "home/sb/t" || String(topic) == "home/sb/up/t"  || String(topic) == "home/sb/down/t") {
    Serial.print("Changing output to ");
    if(messageTemp == "On")
      relayTempstate = HIGH;    
    else if(messageTemp == "Off")
      relayTempstate = LOW;   
    else if(messageTemp == "-1" || messageTemp == "1")
       temp += messageTemp.toFloat();  
    Serial.println(temp);
  }
  compute();
}

void compute() {
    if((float)t > (float)temp ) { 
       // Apaga relay de generacion de calor y deberia mantener la temperatura seteada
       Serial.print("Changing output to OFF");
       relayTempstate = LOW;
       digitalWrite(tempRelay, relayTempstate);
    }
    else {
       // Prende relay de generacion de calor y para intentar mantener la temperatura seteada
       Serial.print("Changing output to ON");
       relayTempstate = HIGH;
       digitalWrite(tempRelay, relayTempstate);
    }
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("arduinoClient")) {
      Serial.println("connected");
      // ... and resubscribe
      client.subscribe("agent/connected");
      client.subscribe("agent/message");
      client.subscribe("agent/disconnected");
      client.subscribe("home/sb/t");
      client.subscribe("home/sb/up/t");
      client.subscribe("home/sb/down/t");      
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void setup()
{   
  pinMode(tempRelay, OUTPUT);
  Serial.begin(9600);
  client.setServer("35.184.80.23", 1883);
  //client.setServer("192.168.1.101", 1883);
  client.setCallback(callback);
  Ethernet.begin(mac);
  // Allow the hardware to sort itself out
  delay(1500);
  Serial.println(Ethernet.localIP());
  lastSend = millis();
  //initialize the variables we're linked to
  Setpoint = 68;
  //tell the PID to range between 0 and the full window size
  myPID.SetOutputLimits(0, 10000);
  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  client.subscribe("home/sb/t");
  client.subscribe("home/sb/up/t");
  client.subscribe("home/sb/down/t"); 
  
}

void loop()
{
  if (!client.connected()) {
    reconnect();
  }
  // it's time to send new data?
  if (millis() - lastSend > PUBLISH_DELAY) {
    sendData();
    lastSend = millis();
  }
  client.loop();
}

void sendData(){
   Serial.println("Collecting temperature data.");
   int chk = DHT.read22(DHT22_PIN);
   h = DHT.humidity;  //Stores humidity value
   t = DHT.temperature; //Stores temperature value
   if (isnan(h) || isnan(t)) {
    Serial.println("DHT sensor fallo en la lectura!");
     return;
   }
   //Print temp and humidity values to serial monitor
   Serial.print("Humidity: ");
   Serial.print(h);
   Serial.print(" %, Temp: ");
   Serial.print(t);
   Serial.println(" Celsius");
   compute(); 
   // Prepare a JSON payload string
   String payload = "{\"temp\" :" + String(t) + ", \"hum\" : " + String(h) + ", \"ret\": " + (relayTempstate ? "true" : "false") + ", \"ts\":" + String(temp) + "}";
   // Send payload
   char attributes[108];
   payload.toCharArray( attributes, 108 );
   client.publish("home/nb/temp", attributes);
   Serial.println(attributes);
}


