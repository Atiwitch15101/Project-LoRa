#include "LoRaWan_APP.h"
#include "Arduino.h"
#include "softSerial.h"
#include "CubeCell_NeoPixel.h"
CubeCell_NeoPixel pixelsRGB(1, RGB, NEO_GRB + NEO_KHZ800);
bool statusRGB = false;

softSerial softwareSerial(GPIO2 /*TX pin*/, GPIO3 /*RX pin*/);

#define SW GPIO4
#define SHORTDELAY 10000
#define LONGDELAY 600000
byte swStatus = 0;

unsigned int pm1 = 0;
unsigned int pm2_5 = 0;
unsigned int pm10 = 0;

byte cmdToPassive[7] = {0x42, 0x4d, 0xe1, 0x00, 0x00, 0x01, 0x70};
byte cmdToRead[7] = {0x42, 0x4d, 0xe2, 0x00, 0x00, 0x01, 0x71};

char data[64];
byte dataIndex = 0;

unsigned long millisTime = 0, millisSW = 0, millisLed = 0, millisTest = 0;

/* OTAA para*/
uint8_t devEui[] = { 0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x06, 0xE1, 0x65 };
uint8_t appEui[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
uint8_t appKey[] = { 0xF7, 0x5E, 0x43, 0x69, 0xE2, 0x21, 0x9A, 0x9E, 0x82, 0xA3, 0x58, 0xD0, 0x3F, 0x47, 0x47, 0x75 };

/* ABP para*/
uint8_t nwkSKey[] = { 0x15, 0xb1, 0xd0, 0xef, 0xa4, 0x63, 0xdf, 0xbe, 0x3d, 0x11, 0x18, 0x1e, 0x1e, 0xc7, 0xda,0x85 };
uint8_t appSKey[] = { 0xd7, 0x2c, 0x78, 0x75, 0x8c, 0xdc, 0xca, 0xbf, 0x55, 0xee, 0x4a, 0x77, 0x8d, 0x16, 0xef,0x67 };
uint32_t devAddr =  ( uint32_t )0x007e6ae1;

/*LoraWan channelsmask, default channels 0-7*/ 
uint16_t userChannelsMask[6]={ 0x00FF,0x0000,0x0000,0x0000,0x0000,0x0000 };

/*LoraWan region, select in arduino IDE tools*/
LoRaMacRegion_t loraWanRegion = ACTIVE_REGION;
DeviceClass_t  loraWanClass = LORAWAN_CLASS;
uint32_t appTxDutyCycle = 15000;

/*OTAA or ABP*/
bool overTheAirActivation = LORAWAN_NETMODE;

/*ADR enable*/
bool loraWanAdr = LORAWAN_ADR;
bool keepNet = LORAWAN_NET_RESERVE;
bool isTxConfirmed = LORAWAN_UPLINKMODE;
uint8_t appPort = 2;
uint8_t confirmedNbTrials = 4;
static void prepareTxFrame( uint8_t port )
{

    Serial.print("\"pm1\": ");
    Serial.print(pm1);
    Serial.print(" ug/m3");
    Serial.print(", ");

    Serial.print("\"pm2_5\": ");
    Serial.print(pm2_5);
    Serial.print(" ug/m3");
    Serial.print(", ");

    Serial.print("\"pm10\": ");
    Serial.print(pm10);
    Serial.println(" ug/m3");

    uint16_t level = analogReadmV(ADC);

    appDataSize = 10;
    appData[0] = 0x03;
    appData[1] = txDutyCycleTime != SHORTDELAY ? 'F':'L';
    appData[2] = (pm1 >> 8) & 0xFF;
    appData[3] = pm1 & 0xFF;
    appData[4] = (pm2_5 >> 8) & 0xFF;
    appData[5] = pm2_5 & 0xFF;
    appData[6] = (pm10 >> 8) & 0xFF;
    appData[7] = pm10 & 0xFF;

    appData[8] = (level >> 8) & 0xFF;
    appData[9] = level & 0xFF;
}


void setup() {
	Serial.begin(115200);
#if(AT_SUPPORT)
	enableAt();
#endif
	deviceState = DEVICE_STATE_INIT;
	LoRaWAN.ifskipjoin();

  txDutyCycleTime = SHORTDELAY;

  pinMode(Vext,OUTPUT);
  digitalWrite(Vext,LOW);
  pixelsRGB.begin();
  pixelsRGB.clear();   

  softwareSerial.begin(9600);
  pinMode(SW, INPUT_PULLUP);
  //pinMode(LED_BUILTIN, OUTPUT);
  sendByte(cmdToPassive);
  //Mcu.begin(HELTEC_BOARD,SLOW_CLK_TPYE);
  Serial.println("---- Start ----");
}

void loop()
{
	readPM();
  lora();
  //sw();
}

void sw(){
  if(!digitalRead(SW)){
    while(!digitalRead(SW));
    if(swStatus == 0){
      swStatus = 1;
      millisSW = millis();
      millisLed = millis();
    }
    else if(swStatus == 1){
      if(txDutyCycleTime == SHORTDELAY)
        txDutyCycleTime = LONGDELAY;
      else
        txDutyCycleTime = SHORTDELAY;
    }
  }
  if(swStatus == 1 && millis() - millisLed > 200){
    //digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    statusRGB = !statusRGB;
      if(statusRGB){
        if(swStatus)
          pixelsRGB.setPixelColor(0, pixelsRGB.Color(0, 255, 0));
        else
          pixelsRGB.setPixelColor(0, pixelsRGB.Color(0, 255, 255));
      }
      else
        pixelsRGB.setPixelColor(0, pixelsRGB.Color(0, 0, 0));
      pixelsRGB.show();
    millisLed = millis();
  }
  if(millis() - millisSW > 3000){
    //digitalWrite(LED_BUILTIN, txDutyCycleTime == SHORTDELAY ? 1 : 0);
    pixelsRGB.setPixelColor(0, pixelsRGB.Color(0, 0, 0));
    pixelsRGB.show();
    swStatus = 0;
  }
}

void readPM(){
  if(softwareSerial.available()){
    while(softwareSerial.available()){
      char temp = softwareSerial.read();
      data[dataIndex++] = temp;
    }
    if(dataIndex >= 32){
      dataIndex = 0;
      if(data[0] == 0x42 && data[1] == 0x4D){
        pm1 = 256 * data[4] + data[5];
        pm2_5 = 256 * data[6] + data[7];
        pm10 = 256 * data[8] + data[9];

      }
    }
  }
  if(millis() - millisTime > 5000){
    dataIndex = 0;
    Serial.read();
    sendByte(cmdToRead);
    Serial.println("Read");
    millisTime = millis();
  }
}

void sendByte(byte data[7]){
  for(byte i = 0; i < 7; i++){
    softwareSerial.write(data[i]);
    delayMicroseconds(1);
  }
}

void lora(){
  switch( deviceState )
	{
		case DEVICE_STATE_INIT:
		{
#if(LORAWAN_DEVEUI_AUTO)
			LoRaWAN.generateDeveuiByChipID();
#endif
#if(AT_SUPPORT)
			getDevParam();
#endif
			printDevParam();
			LoRaWAN.init(loraWanClass,loraWanRegion);
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
			prepareTxFrame( appPort );
			LoRaWAN.send();
			deviceState = DEVICE_STATE_CYCLE;
			break;
		}
		case DEVICE_STATE_CYCLE:
		{
			// Schedule next packet transmission
			txDutyCycleTime = appTxDutyCycle + randr( 0, APP_TX_DUTYCYCLE_RND );
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
}