#define MQTT_VERSION MQTT_VERSION_3_1_1
#define MQTT_MAX_PACKET_SIZE 200
#define PUBLISH_DELAY   15000

//Libraries
#include <SPI.h>
#include <Ethernet.h>
#include <PubSubClient.h>
#include <dht.h>
#include <PID_v1.h>
#include <LiquidCrystal_I2C.h>

// Initialization
dht DHT;
LiquidCrystal_I2C lcd(0x27, 20,4);

//Constants
#define LDRPin    A0 // Fdr
#define DHT22_PIN  2 // DHT22 what pin we're connected to
#define tempRelay  4 // Relay Temperature connect to
#define fanRelay   5 // Relay Temperature connect to
#define backLight  3 // Back_light_pin


// Update these with values suitable for your network.
byte mac[] = {  0xDE, 0xED, 0xBA, 0xFE, 0xFE, 0xED };
IPAddress ip(192, 168, 1, 151);
IPAddress server(35, 184, 80, 23);

EthernetClient ethClient;
PubSubClient client(ethClient);
unsigned long lastSend;
// Temperature vars
float temp = 22.00;
bool relayTempstate = LOW;
float h = 50.00;
float t = 22.00;
// Define Variables we'll be connecting to
double Setpoint, Input, Output;
//S pecify the links and initial tuning parameters
double Kp=2, Ki=5, Kd=1;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
int WindowSize = 5000;
unsigned long windowStartTime;
// TODO: Estudiar inteligencia artifical y aplicar aqui para control de temperatura
// Control de temp manual, automatico o inteligente aplicando inteligencia artificial
// 0 Manual 1 Automatico con pid 2 Automatico con inteligencia y control pid
int tempControl = 0;

// Light vars
int V;
int ilum;

// Fan cooler vars
bool relayFanState = LOW;

// Temp. Icon °C
byte measure[8] =
 {
    0b00001100,     // Los definimos como binarios 0bxxxxxxx
    0b00010010,
    0b00010010,
    0b00001100,
    0b00000000,
    0b00000000,
    0b00000000,
    0b00000000
 };
// Temp. Icon 
byte temperature[8] = {
        B00100,
        B01010,
        B01010,
        B01010,
        B01110,
        B11111,
        B11111,
        B01110
};
// Moisture icon 
byte moisture[8] = {
        B00100,
        B00100,
        B01010,
        B01010,
        B10001,
        B10001,
        B10001,
        B01110
};

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print("Message: ");
  String messageTemp;
  // 
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
    messageTemp += (char)payload[i];
  }
  Serial.println();
  Serial.print(messageTemp);
  if (String(topic) == "home/sb/t" || String(topic) == "home/sb/up/t"  || String(topic) == "home/sb/down/t") {
    if(messageTemp == "On")
      relayTempstate = HIGH;    
    else if(messageTemp == "Off")
      relayTempstate = LOW;   
    else if(messageTemp == "-1" || messageTemp == "1")
       temp += messageTemp.toFloat();  
  }
  sendData();
}

void compute() {

    Input = t;
    myPID.Compute();
    /************************************************
     turn the output pin on/off based on pid output
     in this case 4 pin
    ************************************************/
    unsigned long now = millis();
    if(tempControl == 0) {
      if((float)t > (float)temp ) { 
        // Apaga relay de generacion de calor y deberia mantener la temperatura seteada
        Serial.print("Changing output to OFF");
        Serial.println();
        relayTempstate = LOW;
        //relayFanState = HIGH;
        //digitalWrite(fanRelay, relayFanState);
      }
      else {
        // Prende relay de generacion de calor y para intentar mantener la temperatura seteada
        Serial.print("Changing output to ON");
        Serial.println();
        relayTempstate = HIGH;
        //relayFanState = LOW;
        //digitalWrite(fanRelay, relayFanState);
      }
    } else if(tempControl == 1) {
       if (now - windowStartTime > WindowSize)
       { //time to shift the Relay Window
          windowStartTime += WindowSize;
       }
       if (Output > now - windowStartTime) {
          relayTempstate = HIGH;
       }
       else {
          relayTempstate = LOW;     
        }
    } else if(tempControl == 2) {
      // TODO: Control inteligente inteligencia artifical      
    } 
    digitalWrite(tempRelay, relayTempstate);
      // Print PID Value
    lcd.setCursor ( 0, 1 );
    lcd.print("P.I.D:");
    lcd.setCursor( 8, 1);
    lcd.print(Output);
    lcd.setCursor(15, 1); 
    lcd.print("%"); 
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
   // Lcd Configuration
  lcd.begin();
  lcd.backlight();
  lcd.createChar(1, measure);
  lcd.createChar(2, temperature);
  lcd.createChar(3, moisture);
  
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
   readLight();
   compute(); 
   // Prepare a JSON payload string
   // t:  temperatura
   // h:  humedad
   // rt: relay temperatura
   // ts: temperatura seteada
   // l:  cantidad de luz o lux o lumenes
   // tc: control de temperatura 0 manual 1 automatico pid 2 inteligente ia
   Setpoint = temp;
   String payload = "{\"t\" :" + String(t) + ", \"h\" : " + String(h) + ", \"rt\": " + (relayTempstate ? "1" : "0") + ", \"ts\":" + String(temp) + ", \"l\": " + String(ilum) + ", \"tc\": " 
   + String(tempControl) + "}";
   // Send payload
   char attributes[108];
   payload.toCharArray( attributes, 108 );
   client.publish("home/nb/temp", attributes);
   Serial.println(attributes);
   printScreen();
}

void readLight(){
   Serial.println("Collecting light data.");
   V = analogRead(LDRPin);
   ilum = map (V, 0,1023, 0, 100); 
}

void printScreen(){
  
  // clear display, set cursor position to zero
  lcd.clear(); 
  // Backlight on
  lcd.setBacklight(HIGH);     
  // go home  
  lcd.home();
  // Print Temperature                      
  lcd.write(2);
  lcd.setCursor ( 2, 0 );  
  lcd.print(t); 
  lcd.setCursor ( 7, 0 ); 
  lcd.write(1);
  lcd.setCursor ( 8, 0 ); 
  lcd.write('C');  

  // Print moisture
  lcd.setCursor ( 11, 0 ); 
  lcd.write(3); 
  lcd.setCursor ( 13, 0 ); 
  lcd.print(h); 
  lcd.setCursor ( 18, 0 );
  lcd.write('%'); 
}


