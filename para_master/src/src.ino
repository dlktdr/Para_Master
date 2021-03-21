#include <ArduinoBLE.h>
#include <Arduino.h>
#include "PPMOut.h"
#include "PPMIn.h"
#include "opentxbt.h"
#include "mbed.h"
#include "sbus.h"
#include <nrf_uarte.h>

// Version 1.3

//#define DEBUG
#define CHANNEL_COUNT 8
#define PPM_CENTER 1500
#define SBUS_CENTER 992
#define SBUS_UPDATE_RATE 10000 // 10ms
#define WATCHDOG_TIMEOUT 10000
#define IO_PERIOD 10

uint16_t chanoverrides = 0xFFFF;
uint16_t ppmChannels[16];
int ppminchcnt=0;

int decodeBLEData(uint8_t *buffer, int len, uint16_t *channelvals);
void processTrainerByte(uint8_t data);
void fff6Written(BLEDevice central, BLECharacteristic characteristic);
void overrideWritten(BLEDevice central, BLECharacteristic characteristic);

uint16_t ppmInput[8]; // Channels set here after decode the BT Data

using namespace mbed;

bool scanning = false;
BLEDevice peripheral;
BLECharacteristic fff6;
BLECharacteristic butpress;
BLECharacteristic overridech;
Timer watchdog;
Timer sbusupdate;
Ticker ioTick;

// Create the SBUS in/out UARTS
//UART SerialSbusTx(digitalPinToPinName(23u),digitalPinToPinName(21u),NC,NC); //
//UART SerialSbusRx(NC,D8,NC,NC); //
//bfs::SbusRx sbus_rx(&SerialSbusRx);
#define UARTE0_BASE_ADDR            0x40002000  // As per nRF52840 Product spec - UARTE
#define UART_CONFIG_REG_OFFSET      0x56C
#define UART_BAUDRATE_REG_OFFSET    0x524 // As per nRF52840 Product spec - UARTE
#define UART0_BAUDRATE_REGISTER     (*(( unsigned int *)(UARTE0_BASE_ADDR + UART_BAUDRATE_REG_OFFSET)))
#define UART0_CONFIG_REGISTER       (*(( unsigned int *)(UARTE0_BASE_ADDR + UART_CONFIG_REG_OFFSET)))
#define BAUD100000                   0x0198EF80
#define CONF8E2                      0x0000001E

bfs::SbusTx sbus_tx(&Serial1);
std::array<uint16_t, 16> sbus_data;

void setup()
{
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(LED_BLUE, OUTPUT);
    pinMode(D2, INPUT_PULLUP); // Default
    pinMode(D3, INPUT_PULLUP); // Optional for mating to custom case
    digitalWrite(LED_BLUE,HIGH); // Connected If Blue

    Serial.begin(115200);

    // Start SBUSOut on Pin D8
    sbus_tx.Begin();

    // Issue with Arduino, Cannot set 8E2 without crashing mbed.
    // this manually sets the config regs to 100000 baud 8E2
    UART0_BAUDRATE_REGISTER = BAUD100000;
    UART0_CONFIG_REGISTER = CONF8E2;

    if(!BLE.begin()) {
        Serial.println("Could not start BLE");
        while(1);
    }

    Serial.print("Local BLE Address: ");
    Serial.println(BLE.address());

    PpmOut_setChnCount(CHANNEL_COUNT);
    PpmOut_setPin(D10);

    PpmIn_setPin(D9);

    // Default all CH's at center
    for(int i=0;i < CHANNEL_COUNT; i++) {
        ppmChannels[i] = PPM_CENTER;
        sbus_data[i] = SBUS_CENTER;
    }

    // Start the IO task at 100hz interrupt
    ioTick.attach(callback(io_Task),std::chrono::milliseconds(IO_PERIOD));

    // Start watchdog timer
    watchdog.start();
    sbusupdate.start();
}

// Connected and data being sent
bool bleconnected = false;

void loop()
{
    // Read the PPM input data
    uint16_t ppmin[16];
    PpmIn_execute();
    ppminchcnt = PpmIn_getChannels(ppmin);

    // PPM Data Received
    if(ppminchcnt >= 4 && ppminchcnt <= CHANNEL_COUNT) {
        // If not connected push all PPM's to output
        if(!bleconnected) {
            for(int i=0;i < ppminchcnt; i++)
                ppmChannels[i] = ppmin[i];

        // If connected only push the PPM's that aren't overriden
        } else {
            for(int i=0;i < ppminchcnt; i++) {
                if(!(chanoverrides & (1<<i))) {
                    ppmChannels[i] = ppmin[i];
                }
            }
        }

    // No PPM Input Data
    } else {
        // If not connected set all ch's to zero
        if(!bleconnected) {
            for(int i=0;i < CHANNEL_COUNT; i++)
                ppmChannels[i] = PPM_CENTER;

        // If connected, set all non-overrides to center
        } else {
            for(int i=0;i < CHANNEL_COUNT; i++) {
                if(!(chanoverrides & (1<<i))) {
                    ppmChannels[i] = PPM_CENTER;
                }
            }
        }
    }

    // Set all the PPM Output Channels
    for(int i=0;i < CHANNEL_COUNT; i++) {
        ppmChannels[i] = MAX(MIN(ppmChannels[i],2000),1000);
        PpmOut_setChannel(i,ppmChannels[i]);
    }

    // Set and send the SBUS data
    for(int i=0;i < CHANNEL_COUNT; i++) {
        sbus_data[i] = ppmChannels[i] - (PPM_CENTER - SBUS_CENTER);
    }

    sbus_tx.tx_channels(sbus_data);
    sbus_tx.failsafe(false);
    sbus_tx.lost_frame(false);
    sbus_tx.ch17(false);
    sbus_tx.ch18(false);
    if(sbusupdate.read_high_resolution_us() > SBUS_UPDATE_RATE) {
        sbusupdate.reset();
        sbus_tx.Write();
    }

    // Reset to center from this slave board
    if(bleconnected && butpress) {
        if(digitalRead(D2) == 0 ||
           digitalRead(D3) == 0) {
            butpress.writeValue((uint8_t)'R');
        } else {
            butpress.writeValue((uint8_t)0);
        }
    }

    // Start Scan for PARA Slaves
    if(!BLE.connected() && !scanning) {
        Serial.println("Starting Scan");
        BLE.scan();
        scanning = true;
        bleconnected = false;
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
                                Serial.println("Subscribed to data!");
                            } else {
                                Serial.println("Subscribe tp data failed");
                                fault = true;
                            }
                        } else  {
                            Serial.println("Couldn't find characteristic");
                            fault = true;
                        }
                        // If this is a Headtracker it may have a reset center option
                        butpress = peripheral.service("fff1").characteristic("fff2");
                        if(butpress) {
                            Serial.println("Tracker has ability to remote reset center");
                        }
                        overridech = peripheral.service("fff1").characteristic("fff1");
                        if(overridech) {
                            overridech.setEventHandler(BLEWritten, overrideWritten);  // Call this function on data received
                            // Initial read of overridden channels
                            overridech.readValue(chanoverrides);
                            Serial.println("Tracker has the channels it wants overriden");
                            delay(100); // Doesn't always send data if done right away
                            if(overridech.subscribe()) {
                                Serial.println("Subscribed to channel overrides!");
                            } else {
                                Serial.println("Subscribe to override Failed");
                                fault = true;
                            }
                        } else
                            chanoverrides = 0xFFFF;
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

    // Connected
    if(BLE.connected()) {
        // Check how long we have been connected but haven't received any data
        // if longer than timeout, disconnect.
        uint32_t wdtime = std::chrono::duration_cast<std::chrono::milliseconds>(watchdog.elapsed_time()).count();
        if(wdtime > WATCHDOG_TIMEOUT) {
            Serial.println("***WATCHDOG.. Forcing disconnect. No data received");
            BLE.disconnect();
            bleconnected = false;
        }

    // Not Connected
    } else {
        digitalWrite(LED_BLUE,HIGH);
        bleconnected = false;
        watchdog.reset();
    }

    BLE.poll();
}

void printHex(uint16_t val) {
    Serial.print("0x");
    Serial.print(val,HEX);
}

void overrideWritten(BLEDevice central, BLECharacteristic characteristic)
{
    characteristic.readValue(chanoverrides);
}

// Called when Radio Outputs new Data
void fff6Written(BLEDevice central, BLECharacteristic characteristic) {
    // Got Data Must Be Connected
    digitalWrite(LED_BLUE,LOW);
    bleconnected = true;

    // Got some data clear the watchdog timer
    watchdog.reset();

    uint8_t buffer1[BLUETOOTH_LINE_LENGTH+1];
    int len = characteristic.readValue(buffer1,32);

    // Simulate sending byte by byte like opentx uses
    for(int i=0;i<len;i++) {
        processTrainerByte(buffer1[i]);
    }

    // Got the Channel Data, Set PPM Output if override bits set
    for(int i=0;i<CHANNEL_COUNT;i++) {
        // Only set the data on channels that are allowed to be overriden
        if(chanoverrides & (1<<i)) {
            ppmChannels[i] = ppmInput[i];
        }
    }

#ifdef DEBUG
    Serial.print("OR: ");
    printHex(chanoverrides);
    Serial.print("|");
    for(int i=0;i<CHANNEL_COUNT;i++) {
        Serial.print("Ch");Serial.print(i+1);Serial.print(":");Serial.print(ppmChannels[i]);Serial.print(" ");
    }
    Serial.println("");
#endif
}

// Any IO Related Tasks, buttons, etc.. ISR. Run at 1Khz
void io_Task()
{
  static int i =0;
  // Fast Blink to know it's running
  if(i==10) {
    digitalWrite(LED_BUILTIN, HIGH);
  }
  if(i==20) {
    digitalWrite(LED_BUILTIN, LOW);
    i=0;
  }
  i++;
}