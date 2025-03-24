#include "LoRaWan_APP.h"
#include "Arduino.h"
#include <Wire.h>
#include <Seeed_BME280.h>
#include "softSerial.h"
#include "CubeCell_NeoPixel.h"
CubeCell_NeoPixel pixelsRGB(1, RGB, NEO_GRB + NEO_KHZ800);
bool statusRGB = false;

#define LONGDELAY 600000
#define SHORTDELAY 10000

#define SW GPIO4
#define LEDGREEN GPIO5
#define LEDYELLOW GPIO0
bool status = false, modeStatus = false;

/* Wind */
#define DEPIN GPIO3
softSerial softwareSerial(GPIO1 /*TX pin*/, GPIO2 /*RX pin*/);
byte readWindSpeed[8] = { 0x01, 0x03, 0x00, 0x00, 0x00, 0x01, 0x84, 0x0A };
byte readWindDirect[8] = { 0x0A, 0x03, 0x00, 0x00, 0x00, 0x02, 0xC5, 0x70 };

unsigned long millisDirection = 0, millisSpeed = 0, millisSW = 0, millisLED = 0;
byte windSpeed[2] = { 0x00, 0x00 };
byte windDirection = 0;
/* Wind */

BME280 bme280;

/* OTAA para*/
uint8_t devEui[] = { 0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x06, 0xDE, 0xF7 };
uint8_t appEui[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
uint8_t appKey[] = { 0x64, 0x6C, 0x72, 0x9A, 0x8E, 0xE8, 0x88, 0x95, 0x25, 0xD9, 0x3D, 0x60, 0x2E, 0xCB, 0xE5, 0xE1 };

/* ABP para*/
uint8_t nwkSKey[] = { 0x15, 0xb1, 0xd0, 0xef, 0xa4, 0x63, 0xdf, 0xbe, 0x3d, 0x11, 0x18, 0x1e, 0x1e, 0xc7, 0xda, 0x85 };
uint8_t appSKey[] = { 0xd7, 0x2c, 0x78, 0x75, 0x8c, 0xdc, 0xca, 0xbf, 0x55, 0xee, 0x4a, 0x77, 0x8d, 0x16, 0xef, 0x67 };
uint32_t devAddr = (uint32_t)0x007e6ae1;

/*LoraWan channelsmask, default channels 0-7*/
uint16_t userChannelsMask[6] = { 0x00FF, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 };


LoRaMacRegion_t loraWanRegion = ACTIVE_REGION;
DeviceClass_t loraWanClass = LORAWAN_CLASS;
uint32_t appTxDutyCycle = 10000;

/*OTAA or ABP*/
bool overTheAirActivation = LORAWAN_NETMODE;

/*ADR enable*/
bool loraWanAdr = LORAWAN_ADR;

/* set LORAWAN_Net_Reserve ON, the node could save the network info to flash, when node reset not need to join again */
bool keepNet = LORAWAN_NET_RESERVE;

/* Indicates if the node is sending confirmed or unconfirmed messages */
bool isTxConfirmed = LORAWAN_UPLINKMODE;

/* Application port */
uint8_t appPort = 2;

uint8_t confirmedNbTrials = 4;

/* Prepares the payload of the frame */
static void prepareTxFrame(uint8_t port) {
  
  appDataSize = 12;
  //appData[0] = 0x02;
  int temperature = (int)bme280.getTemperature();
  int humidity = (int)bme280.getHumidity();
  uint32_t pressure = bme280.getPressure();
  Serial.println("sending");
  appData[0] = 0x01;
  appData[1] = windSpeed[0];
  appData[2] = windSpeed[1];
  appData[3] = windDirection;

  appData[4] = (temperature >> 8) & 0xFF;
  appData[5] = temperature & 0xFF;

  appData[6] = (humidity >> 8) & 0xFF;
  appData[7] = humidity & 0xFF;

  appData[8] = (pressure >> 24) & 0xFF;
  appData[9] = (pressure >> 16) & 0xFF;
  appData[10] = (pressure >> 8) & 0xFF;
  appData[11] = pressure & 0xFF;
}


void setup() {
  pinMode(Vext,OUTPUT);
  digitalWrite(Vext,LOW);
  pixelsRGB.begin();
  pixelsRGB.clear();

  pinMode(LEDGREEN,OUTPUT);
  pinMode(LEDYELLOW,OUTPUT);
  txDutyCycleTime = SHORTDELAY;
  pinMode(SW, INPUT);
  Serial.begin(115200);
  Serial.println("---- init ----");
  init();
#if (AT_SUPPORT)
  enableAt();
#endif
  deviceState = DEVICE_STATE_INIT;
  LoRaWAN.ifskipjoin();
  Serial.println("---- Start ----");
}

void loop() {
  //switchFunction(); // ==================================================================
  switch (deviceState) {
    case DEVICE_STATE_INIT:
      {
#if (LORAWAN_DEVEUI_AUTO)
        LoRaWAN.generateDeveuiByChipID();
#endif
#if (AT_SUPPORT)
        //getDevParam();
#endif
        //printDevParam();
        LoRaWAN.init(loraWanClass, loraWanRegion);
        deviceState = DEVICE_STATE_JOIN;
        break;
      }
    case DEVICE_STATE_JOIN:
      {
        LoRaWAN.join();
        break;
      }
    case DEVICE_STATE_SEND:
      {
        prepareTxFrame(appPort);
        LoRaWAN.send();
        deviceState = DEVICE_STATE_CYCLE;
        break;
      }
    case DEVICE_STATE_CYCLE:
      {
        LoRaWAN.cycle(txDutyCycleTime);
        deviceState = DEVICE_STATE_SLEEP;
        break;
      }
    case DEVICE_STATE_SLEEP:
      {
        LoRaWAN.sleep();
        break;
      }
    default:
      {
        deviceState = DEVICE_STATE_INIT;
        break;
      }
  }
  windLoop();
}

void init() {
  softwareSerial.begin(9600);
  pinMode(DEPIN, OUTPUT);

  if (!bme280.init()) {
    Serial.println("BME280 initialization failed!");
    while (1)
      ;
  }
  Serial.println("BME280 initialized successfully");
}

void sendByteRS485(byte data[8], byte size) {
  digitalWrite(DEPIN, 1);
  delayMicroseconds(1);
  for (int i = 0; i < size; i++) {
    softwareSerial.write(data[i]);
  }
  digitalWrite(DEPIN, 0);
}

void windLoop() {
  if (softwareSerial.available() > 0) {
    byte i = 0;
    char data[256];
    while (softwareSerial.available()) {
      data[i] = softwareSerial.read();
      i++;
    }
    if (data[0] == 0x0A) {
      Serial.println("--- Direction ---");
      Serial.println((data[6] / 2) + 1);
      windDirection = (data[6] / 2) + 1;
    } else if (data[0] == 0x01) {
      Serial.println("--- Speed ---");
      Serial.println(((data[3] << 8) + data[4]) / 10);
      windSpeed[0] = data[3];
      windSpeed[1] = data[4];
    }
  }
  if (millis() - millisSpeed > 5000) {
    Serial.println("Reading speed");
    sendByteRS485(readWindSpeed, sizeof(readWindSpeed));
    millisSpeed = millis();
  }
  if (millis() - millisDirection > 6000) {
    Serial.println("Reading direct");
    sendByteRS485(readWindDirect, sizeof(readWindDirect));
    millisDirection = millis();
  }
}

void switchFunction(){
  if(!digitalRead(SW) && !status){
    while(!digitalRead(SW));
    Serial.println(status);
    status = true;
    millisLED = millis();
    millisSW = millis();
  }
  if(status){
    if(millis() - millisLED > 200){
      digitalWrite(LEDGREEN, !digitalRead(LEDGREEN));
      statusRGB = !statusRGB;
      if(statusRGB){
        if(modeStatus)
          pixelsRGB.setPixelColor(0, pixelsRGB.Color(0, 255, 0));
        else
          pixelsRGB.setPixelColor(0, pixelsRGB.Color(0, 255, 255));
      }
      else
        pixelsRGB.setPixelColor(0, pixelsRGB.Color(0, 0, 0));
      pixelsRGB.show();
      millisLED = millis();
    }
    if(millis() - millisSW < 3000){
      if(!digitalRead(SW)){
        while(!digitalRead(SW));
        Serial.println("Change");
        modeStatus = !modeStatus;
        if(modeStatus)
          txDutyCycleTime = LONGDELAY;
        else
          txDutyCycleTime = SHORTDELAY;
        LoRaWAN.cycle(txDutyCycleTime);
      }
    }
    else{
      pixelsRGB.setPixelColor(0, pixelsRGB.Color(0, 0, 0));
      pixelsRGB.show();
      digitalWrite(LEDGREEN, 0);
      digitalWrite(LEDYELLOW, 0);
      status = false;
    }
  }
}