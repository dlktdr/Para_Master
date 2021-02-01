#include <ArduinoBLE.h>
#include <Arduino.h>
#include "PPMOut.h"
#include "opentxbt.h"

#define DEBUG

uint16_t ppmInput[8];

int decodeBLEData(uint8_t *buffer, int len, uint16_t *channelvals);
void processTrainerByte(uint8_t data);

bool scanning = false;
BLEDevice peripheral;
BLECharacteristic fff6;

// PPM Output on D10
PpmOut ppmout(digitalPinToPinName(D10),8);

void setup() 
{
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(LED_BLUE, OUTPUT);
    digitalWrite(LED_BLUE,HIGH); // Connected If Blue

    Serial.begin(115200);
    
    if(!BLE.begin()) {
        Serial.println("Could not start BLE");
        while(1);
    }
        
    Serial.print("Local BLE Address: "); 
    Serial.println(BLE.address());
}

void loop() 
{
    digitalWrite(LED_BUILTIN,HIGH);

    // Start Scan for PARA Slaves
    if(!BLE.connected() && !scanning) {
        Serial.println("Starting Scan");
        BLE.scan();
        scanning = true;           
    }

    // If scanning see if there is a BLE device available
    if(!BLE.connected() && scanning) {
        bool fault = false;
        peripheral = BLE.available(); 
        if(peripheral) {
            if (peripheral.address() == "") {
                Serial.print(" <no advertised address> ");
            } else {
                Serial.print("Advertised Device Address: ");
                Serial.print(peripheral.address());
            }
            if (peripheral.localName() == "") {
                Serial.print(" <no advertised local name> ");
            } else {
                Serial.print(" Local Name: ");
                Serial.print(peripheral.localName());
            }

            if (peripheral.advertisedServiceUuid() == "") {
                Serial.print(" <no advertised service UUID> ");
            } else {
                Serial.print(" Advertised Service UUID ");
                Serial.print(peripheral.advertisedServiceUuid());
            }
            Serial.println();

            if(peripheral.localName() == "Hello" &&
               peripheral.advertisedServiceUuid() == "fff0") {
                Serial.println("Looks like this is a PARA Slave Device");
                Serial.println("Stopping scan");
                BLE.stopScan();
                scanning = false;
                Serial.println("Connecting...");       
                if(peripheral.connect()) {
                    Serial.println("Connected");
                    delay(100); // Attribute discovery fails if done right away
                    Serial.println("Discovering Attributes");
                    if(peripheral.discoverAttributes()) {
                        Serial.println("Discovered Attributes");
                        fff6 = peripheral.service("fff0").characteristic("fff6");
                        if(fff6) {
                            Serial.println("Attaching Event Handler");
                            fff6.setEventHandler(BLEWritten, fff6Written);  // Call this function on data received
                            Serial.println("Got Characteristic");
                            Serial.println("Subscribing...");
                            delay(100); // Doesn't always send data if done right away
                            if(fff6.subscribe()) {
                                Serial.println("Subscribed!");                                
                            } else {
                                Serial.println("Subscribe Failed");
                                fault = true;
                            }
                        } else  {
                            Serial.println("Couldn't find characteristic");
                            fault = true;                        
                            }
                    } else {
                        Serial.println("Attribute Discovery Failed");
                        fault = true;                    
                        }
                } else {
                    Serial.println("Couldn't connect to Para Slave, Rescanning");
                    fault = true;
                }
            }
        }
        // On any faults, disconnect and start scanning again
        if(fault) {
            peripheral.disconnect();
            BLE.scan();
            scanning = true;
        }

    }

    // Notify not connected
    if(!BLE.connected()) {
        digitalWrite(LED_BLUE,HIGH);
    }

    BLE.poll();
    digitalWrite(LED_BUILTIN,LOW);
    
}


void printHex(uint8_t *addr, int len)
{
    for(int i=0;i<len;i++) {
        Serial.print("0x");
        Serial.print(addr[i], HEX);
        Serial.print(" ");        
    }
}

// Called when Radio Outputs new Data
void fff6Written(BLEDevice central, BLECharacteristic characteristic) {
    // Got Data Must Be Connected
    digitalWrite(LED_BLUE,LOW);
  
    uint8_t buffer1[BLUETOOTH_LINE_LENGTH+1];
    int len = characteristic.readValue(buffer1,32);

    // Simulate sending byte by byte like opentx uses
    for(int i=0;i<len;i++) {
        processTrainerByte(buffer1[i]);
    }
  
     // Got the Channel Data, Set PPM Output
    for(int i=0;i<8;i++) {
        // Limit channels to 1000 - 2000us
        ppmInput[i] = MAX(MIN(ppmInput[i],2000),1000); 
        ppmout.setChannel(i,ppmInput[i]);
    }    

#ifdef DEBUG       
    for(int i=0;i<8;i++) {
        Serial.print("Ch");Serial.print(i+1);Serial.print(":");Serial.print(ppmInput[i]);Serial.print(" ");
    }
    Serial.println("");
#endif        
}