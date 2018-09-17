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
#define DHT22_PIN 2     // DHT22 what pin we're connected to
#define tempRelay 4     // Relay Temperature connect to
#define fanRelay 5      // Relay Temperature connect to
// Update these with values suitable for your network.
byte mac[] = {  0xDE, 0xED, 0xBA, 0xFE, 0xFE, 0xED };
IPAddress ip(192, 168, 1, 151);
IPAddress server(35, 184, 80, 23);

EthernetClient ethClient;
PubSubClient client(ethClient);
unsigned long lastSend;

float temp = 22.00;
bool relayTempstate = LOW;
bool relayFanState = LOW;
float h = 50.00;
float t = 21.00;

//Medir luz
const long A = 1000;     //Resistencia en oscuridad en KΩ
const int B = 15;        //Resistencia a la luz (10 Lux) en KΩ
const int Rc = 10;       //Resistencia calibracion en KΩ
const int LDRPin = A0; 
int V;
int ilum;

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
double Kp=2, Ki=5, Kd=1;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
int WindowSize = 5000;
unsigned long windowStartTime;

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
  sendData();
}

void compute() {
    /*if((float)t > (float)temp ) { 
       // Apaga relay de generacion de calor y deberia mantener la temperatura seteada
       Serial.print("Changing output to OFF");
       relayTempstate = LOW;
       //relayFanState = HIGH;
       digitalWrite(tempRelay, relayTempstate);
       //digitalWrite(fanRelay, relayFanState);
    }
    else {
       // Prende relay de generacion de calor y para intentar mantener la temperatura seteada
       Serial.print("Changing output to ON");
       relayTempstate = HIGH;
       //relayFanState = LOW;
       digitalWrite(tempRelay, relayTempstate);
       //digitalWrite(fanRelay, relayFanState);
    }*/
    Input = t;
    myPID.Compute();
    Serial.print(Output);
    Serial.print("\n");
    /************************************************
     turn the output pin on/off based on pid output
    ************************************************/
    unsigned long now = millis();
    if (now - windowStartTime > WindowSize)
    { //time to shift the Relay Window
      windowStartTime += WindowSize;
    }
    if (Output > now - windowStartTime) {
      relayTempstate = HIGH;
      digitalWrite(tempRelay, relayTempstate);  
    }
    else {
      relayTempstate = LOW;
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
 // pinMode(fanRelay, OUTPUT);
  Serial.begin(9600);
  client.setServer("35.184.80.23", 1883);
  //client.setServer("192.168.1.101", 1883);
  client.setCallback(callback);
  Ethernet.begin(mac);
  // Allow the hardware to sort itself out
  delay(1500);
  Serial.println(Ethernet.localIP());
  lastSend = millis();
  windowStartTime = millis();
  //initialize the variables we're linked to
  Setpoint = 21.00;
  //tell the PID to range between 0 and the full window size
  myPID.SetOutputLimits(0, WindowSize);
  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  client.subscribe("home/sb/t");
  client.subscribe("home/sb/up/t");
  client.subscribe("home/sb/down/t"); 
  
}

void loop()
{
  compute(); 
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
   readLight();
   compute(); 
   // Prepare a JSON payload string
   // t:  temperatura
   // h:  humedad
   // rt: relay temperatura
   // ts: temperatura seteada
   // l:  cantidad de luz o lux o lumenes
   Setpoint = temp;
   String payload = "{\"t\" :" + String(t) + ", \"h\" : " + String(h) + ", \"rt\": " + (relayTempstate ? "true" : "false") + ", \"ts\":" + String(temp) + ", \"l\": " + String(ilum) + "}";
   // Send payload
   char attributes[108];
   payload.toCharArray( attributes, 108 );
   client.publish("home/nb/temp", attributes);
   Serial.println(attributes);
}


void readLight(){
   Serial.println("Collecting light data.");
   V = analogRead(LDRPin);         
   //ilum = ((long)(1024-V)*A*10)/((long)B*Rc*V);  //usar si LDR entre GND y A0 
   //ilum = ((long)V*A*10)/((long)B*Rc*(1024-V));    //usar si LDR entre A0 y Vcc (como en el esquema anterior)
   ilum = map (V, 0,1023, 0, 100); 
}




