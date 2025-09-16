// Include the libraries we need
#include <OneWire.h>
#include <DallasTemperature.h>
#include <WiFi.h>
#include "ThingSpeak.h" // always include thingspeak header file after other header files and custom macros
#include "EmonLib.h"             // Include Emon Library
EnergyMonitor emon1;             // Create an instance

const char ssid[] = "Galaxy A51 917E";  // your network SSID (name)
const char pass[] = "tosin@345";   // your network password 
WiFiClient  client;

// SoftwareSerial bluetoothSerial(13, 12);
String voiceCommand;
//parameters for energy
float realPower, apparentPower, powerFactor, supplyVoltage, Irms, actualCurrent, actualVoltage;
float actualPower, actualEnergy;

const int pirPin = 26;
const int socketPin = 2;
const int lightBulbPin = 18;
const int dcFanPin = 15;
const int voltageSensorPin = 34;
const int currentSensorPin = 35;
//states to hold the voice and IoT inputs
bool bulbState, socketState, dcFanState, autoSwitch;
bool bulbState1, socketState1, dcFanState1;

unsigned long counterChannelNumber = 2536471;
unsigned long counterChannelNumber2 = 2558070;
unsigned long readChannelNumber = 2558070;             // Channel ID  
const char * myWriteAPIKey = "5NYF7DAJ5CPNOJW9";
const char * myWriteAPIKey2 = "XWA4T3G0P2MKHGYY";
const char * myCounterReadAPIKey = "UHVB76ZF2XX8EJVA"; // Read API Key
const int FieldNumber1 = 1;  // The field you wish to read
const int FieldNumber2 = 2;  // The field you wish to read
const int FieldNumber3 = 3;
const int FieldNumber4 = 4;

int number, statusCode = 0;
int field[4] = {2,3,4,5};
//for temp sensor
// Data wire is plugged into port 2 on the Arduino
#define ONE_WIRE_BUS 4
// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);
float tempC;


void setup() {
  Serial.begin(9600);
  sensors.begin();
  //begin the bluetooth connections
  // bluetoothSerial.begin(115200);
  WiFi.mode(WIFI_STA);   
  ThingSpeak.begin(client);  // Initialize ThingSpeak
  //the outputs and inputs
  pinMode(socketPin, OUTPUT);
  pinMode(lightBulbPin, OUTPUT);
  pinMode(dcFanPin, OUTPUT);
  pinMode(pirPin, INPUT);
  //turn them all off first
  digitalWrite(socketPin, LOW);
  digitalWrite(lightBulbPin, LOW);
  digitalWrite(dcFanPin, LOW);
//use the emon lib to call the voltage and current parameters
  emon1.voltage(voltageSensorPin, 55.26, 1.7);  // Voltage: input pin, calibration, phase_shift
  emon1.current(currentSensorPin, 107.1);       // Current: input pin, calibration.
  //check if conected to wiFi connection 
  if(WiFi.status() != WL_CONNECTED){
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    while(WiFi.status() != WL_CONNECTED){
      WiFi.begin(ssid, pass); // Connect to WPA/WPA2 network. Change this line if using open or WEP network
      Serial.print(".");
      delay(5000);     
    } 
    Serial.println("\nConnected.");
  }
}

//function for check A.C voltage level
double voltageCurrentSensors(){
  emon1.calcVI(20,2000);         // Calculate all. No.of half wavelengths (crossings), time-out
  //emon1.serialprint();           // Print out all variables (realpower, apparent power, Vrms, Irms, power factor)
  
  realPower       = emon1.realPower;        //extract Real Power into variable
  apparentPower   = emon1.apparentPower;    //extract Apparent Power into variable
  powerFactor     = emon1.powerFactor;      //extract Power Factor into Variable
  supplyVoltage   = emon1.Vrms;             //extract Vrms into Variable
  Irms            = emon1.Irms;             //extract Irms into Variable
  actualCurrent = (Irms/0.707)/1000;
  actualVoltage = supplyVoltage/0.707;
  actualPower = actualCurrent*actualVoltage;
  actualEnergy = actualPower* (millis()/1000);
  return actualCurrent, actualVoltage, actualPower, actualEnergy, powerFactor;
}

float tempSensor(){
  // call sensors.requestTemperatures() to issue a global temperature 
  // request to all devices on the bus
  // Serial.print("Requesting temperatures...");
  sensors.requestTemperatures(); // Send the command to get temperatures
  // Serial.println("DONE");
  // After we got the temperatures, we can print them here.
  // We use the function ByIndex, and as an example get the temperature from the first sensor only.
  tempC = sensors.getTempCByIndex(0);

  // Check if reading was successful
  if(tempC != DEVICE_DISCONNECTED_C) {
    Serial.print("Temperature for the device 1 (index 0) is: ");
    Serial.println(tempC);
  } 
  else  {
    Serial.println("Error: Could not read temperature data");
  }
//put condition for turning on Fan Automatically using temperature
if(tempC <= 10.20){
  digitalWrite(dcFanPin, LOW);
}

else if(tempC >= 55.00){
  digitalWrite(dcFanPin, HIGH);
}
  return tempC;
}

float DecimalRound(float input, int decimals){
  float scale=pow(10,decimals);
  return round(input*scale)/scale;
}

void writeToThingspeak(){
  voltageCurrentSensors();
  actualVoltage = DecimalRound(actualVoltage, 2);
  actualCurrent = DecimalRound(actualCurrent, 2);
  actualPower = DecimalRound(actualPower, 2);
  actualEnergy = DecimalRound(actualEnergy, 2);
   // set the fields with the values , , , , powerFactor
  ThingSpeak.setField(1, actualVoltage);
  ThingSpeak.setField(2, actualCurrent);
  ThingSpeak.setField(3, actualPower);
  ThingSpeak.setField(4, actualEnergy);
   /*print out the values*/
   Serial.print("Voltage: ");
  Serial.print(actualVoltage);
  Serial.print(" current: ");
  Serial.println(actualCurrent);
   // write to the ThingSpeak channel
  int x = ThingSpeak.writeFields(counterChannelNumber, myWriteAPIKey);
  if(x == 200){
    Serial.println("Channel update successful.");
  }
  else{
    Serial.println("Problem updating channel. HTTP error code " + String(x));
  }
  
  // change the value
  // number++;
  // if(number > 99){
  //   number = 0;
  // }
  }

int readFromThingspeak(){
  //---------------- Channel 1 ----------------//
  bulbState = ThingSpeak.readLongField(readChannelNumber, FieldNumber1, myCounterReadAPIKey);
  statusCode = ThingSpeak.getLastReadStatus();
      if (statusCode == 200){
        Serial.print("A.C. Bulb: ");
        Serial.println(bulbState);
      }
      else{
        Serial.println("Unable to read channel / No internet connection");
      }
  delay(100);
  //---------------- Channel 2 ----------------//
  socketState = ThingSpeak.readLongField(readChannelNumber, FieldNumber2, myCounterReadAPIKey);
  statusCode = ThingSpeak.getLastReadStatus();
  if (statusCode == 200) {
    Serial.print("A.C. Socket: ");
    Serial.println(socketState); 
  }
  else {
    Serial.println("Unable to read channel / No internet connection");
  }
  delay(100);
   //---------------- Channel 3 ----------------//
  dcFanState = ThingSpeak.readLongField(readChannelNumber, FieldNumber3, myCounterReadAPIKey);
  statusCode = ThingSpeak.getLastReadStatus();
  if (statusCode == 200) {
    Serial.print("DC Fan: ");
    Serial.println(dcFanState); 
  }
  else {
    Serial.println("Unable to read channel / No internet connection");
  }
   //---------------- Channel 4 ----------------//
  autoSwitch = ThingSpeak.readLongField(readChannelNumber, FieldNumber4, myCounterReadAPIKey);
  statusCode = ThingSpeak.getLastReadStatus();
  if (statusCode == 200) {
    Serial.print("Auto Switch Control: ");
    Serial.println(autoSwitch); 
  }
  else {
    Serial.println("Unable to read channel / No internet connection");
  }
  //use what is read to control appliances if the automatic switch is not turned on
if(autoSwitch == 0){ 
if(bulbState == 1){
  digitalWrite(lightBulbPin, HIGH); 
      }
if (bulbState == 0) {
  digitalWrite(lightBulbPin, LOW);
    }
    //control the socket point
if(socketState == 1){
  digitalWrite(socketPin, HIGH);
      }
if (socketState == 0) {
  digitalWrite(socketPin, LOW);
    }
if(dcFanState == 1){
  digitalWrite(dcFanPin, HIGH);
      }
if (dcFanState == 0) {
  digitalWrite(dcFanPin, LOW);
    }
  }
  if(autoSwitch == 1){
     Serial.println(digitalRead(pirPin));
    if(digitalRead(pirPin) == 1){
     digitalWrite(lightBulbPin, HIGH);
      delay(2000);
      digitalWrite(dcFanPin, HIGH);
      delay(2000);
      digitalWrite(socketPin, HIGH);
    }
  }
  return dcFanState, socketState, bulbState;
}

//function for checking bluetooth voice control
void btVoiceControl() {
  tempSensor();
  readFromThingspeak();
  while (Serial.available()) {
    delay(10);
    char c = Serial.read();
    if (c == '#') {
      break;
    }
    voiceCommand += c;
    // Serial.write(c);
  }
  if (voiceCommand.length() > 0) {
     Serial.println(voiceCommand);

    if((voiceCommand == "turn light on") || (voiceCommand == "light on") || (voiceCommand == "light bulb on")  || (voiceCommand == "turn bulb on") || (voiceCommand == "bulb on")|| (voiceCommand == "turn bob on") || (voiceCommand == "Bob on")){
      bulbState1 = 1;
    }

    if((voiceCommand == "turn light off") || (voiceCommand == "lights off") || (voiceCommand == "light bulb off") || (voiceCommand == "turn bulb off") || (voiceCommand == "bulb off") || (voiceCommand == "turn bob off") || (voiceCommand == "Bob off")){
      bulbState1 = 0;
    }

    if((voiceCommand == "turn socket on") || (voiceCommand == "socket on")){
      socketState1 = 1;
    }

    if((voiceCommand == "turn socket off") || (voiceCommand == "socket off")){
      bulbState1 = 0;
    }

    if((voiceCommand == "turn fan on") || (voiceCommand == "fan on")){
      dcFanState1 = 1;
    }

    if((voiceCommand == "turn fan off") || (voiceCommand == "fan off")){
      dcFanState1 = 0;
    }
     if((voiceCommand == "turn off all") || (voiceCommand == "off all") || (voiceCommand == "off all appliances")){
      dcFanState1 = 0;
      bulbState1 = 0;
      socketState1 = 0;
    }

    if((voiceCommand == "turn on all") || (voiceCommand == "on all") || (voiceCommand == "on all appliances") || (voiceCommand == "on all loads")){
      dcFanState1 = 1;
      bulbState1 = 1;
      socketState1 = 1;
    }
    //use voice to control appliance
    if(bulbState1 == 1){
  digitalWrite(lightBulbPin, HIGH); 
      }
if (bulbState1 == 0) {
  digitalWrite(lightBulbPin, LOW);
    }
    //control the light bulb
if(socketState1 == 1){
  digitalWrite(socketPin, HIGH);
      }
if (socketState1 == 0) {
  digitalWrite(socketPin, LOW);
    }
if(dcFanState1 == 1){
  digitalWrite(dcFanPin, HIGH);
      }
if (dcFanState1 == 0) {
  digitalWrite(dcFanPin, LOW);
    }
//send these states to thingspeak
  ThingSpeak.setField(1, bulbState1);
  ThingSpeak.setField(2, socketState1);
  ThingSpeak.setField(3, dcFanState1);
   // write to the ThingSpeak channel
  int x = ThingSpeak.writeFields(counterChannelNumber2, myWriteAPIKey2);
  if(x == 200){
    Serial.println("Channel update successful.");
  }
  else{
    Serial.println("error updating channel");
  }
  }
    Serial.print("A.C. Socket: ");
    Serial.print(socketState); 
    Serial.print(" A.C. Bulb: ");
    Serial.print(bulbState); 
    Serial.print(" DC Fan: ");
    Serial.println(dcFanState); 
    
  voiceCommand = "";
  // return bulbState1, socketState1, dcFanState1;
}


void loop() {
  writeToThingspeak();
  btVoiceControl(); 
  
   delay(15000);
 
  // voltageSensor();
}
