#include "LoRaWan_APP.h"
#include "Arduino.h"
#include "softSerial.h"
#include "CubeCell_NeoPixel.h"

#define LONGDELAY 600000
#define SHORTDELAY 10000

#define SW GPIO3
#define LEDGREEN GPIO5
#define LEDYELLOW GPIO0

CubeCell_NeoPixel pixelsRGB(1, RGB, NEO_GRB + NEO_KHZ800);
bool statusRGB = false;

bool status = false, modeStatus = false, isBlink = false;

/* Rain */

softSerial softwareSerial(GPIO1 /*TX pin*/, GPIO2 /*RX pin*/);
int rainRate = 0;
unsigned long millisTime = 0, millisSW = 0, millisLED = 0;

/* Rain */

/*
 * set LoraWan_RGB to Active,the RGB active in loraWan
 * RGB red means sending;
 * RGB purple means joined done;
 * RGB blue means RxWindow1;
 * RGB yellow means RxWindow2;
 * RGB green means received done;
 */

/* OTAA para*/
uint8_t devEui[] = { 0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x06, 0xD7, 0xC4 };
uint8_t appEui[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
uint8_t appKey[] = { 0x22, 0xCF, 0x0A, 0x36, 0x7E, 0x71, 0x8A, 0xE7, 0x27, 0xCA, 0xCD, 0xF4, 0xCE, 0x06, 0x1B, 0xA5 };

/* ABP para*/
uint8_t nwkSKey[] = { 0x15, 0xb1, 0xd0, 0xef, 0xa4, 0x63, 0xdf, 0xbe, 0x3d, 0x11, 0x18, 0x1e, 0x1e, 0xc7, 0xda,0x85 };
uint8_t appSKey[] = { 0xd7, 0x2c, 0x78, 0x75, 0x8c, 0xdc, 0xca, 0xbf, 0x55, 0xee, 0x4a, 0x77, 0x8d, 0x16, 0xef,0x67 };
uint32_t devAddr =  ( uint32_t )0x007e6ae1;

/*LoraWan channelsmask, default channels 0-7*/ 
uint16_t userChannelsMask[6]={ 0x00FF,0x0000,0x0000,0x0000,0x0000,0x0000 };

/*LoraWan region, select in arduino IDE tools*/
LoRaMacRegion_t loraWanRegion = ACTIVE_REGION;

/*LoraWan Class, Class A and Class C are supported*/
DeviceClass_t  loraWanClass = LORAWAN_CLASS;

/*the application data transmission duty cycle.  value in [ms].*/
uint32_t appTxDutyCycle = 15000;

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
/*!
* Number of trials to transmit the frame, if the LoRaMAC layer did not
* receive an acknowledgment. The MAC performs a datarate adaptation,
* according to the LoRaWAN Specification V1.0.2, chapter 18.4, according
* to the following table:
*
* Transmission nb | Data Rate
* ----------------|-----------
* 1 (first)       | DR
* 2               | DR
* 3               | max(DR-1,0)
* 4               | max(DR-1,0)
* 5               | max(DR-2,0)
* 6               | max(DR-2,0)
* 7               | max(DR-3,0)
* 8               | max(DR-3,0)
*
* Note, that if NbTrials is set to 1 or 2, the MAC will not decrease
* the datarate, in case the LoRaMAC layer did not receive an acknowledgment
*/
uint8_t confirmedNbTrials = 4;

/* Prepares the payload of the frame */
static void prepareTxFrame( uint8_t port )
{
	/*appData size is LORAWAN_APP_DATA_MAX_SIZE which is defined in "commissioning.h".
	*appDataSize max value is LORAWAN_APP_DATA_MAX_SIZE.
	*if enabled AT, don't modify LORAWAN_APP_DATA_MAX_SIZE, it may cause system hanging or failure.
	*if disabled AT, LORAWAN_APP_DATA_MAX_SIZE can be modified, the max value is reference to lorawan region and SF.
	*for example, if use REGION_CN470, 
	*the max value for different DR can be found in MaxPayloadOfDatarateCN470 refer to DataratesCN470 and BandwidthsCN470 in "RegionCN470.h".
	*/
    appDataSize = 4;
    

    Serial.println((float)rainRate/100);
    appData[0] = 0x02;
    appData[1] = modeStatus ? 'L' : 'S';
    appData[2] = (rainRate >> 8 ) & 0xFF;
    appData[3] = rainRate & 0xFF;
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
  init();
#if(AT_SUPPORT)
	enableAt();
#endif
	deviceState = DEVICE_STATE_INIT;
	LoRaWAN.ifskipjoin();
}

void loop()
{
	switch( deviceState )
	{
		case DEVICE_STATE_INIT:
		{
#if(LORAWAN_DEVEUI_AUTO)
			LoRaWAN.generateDeveuiByChipID();
#endif
#if(AT_SUPPORT)
			//getDevParam();
#endif
			//printDevParam();
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
			//txDutyCycleTime = appTxDutyCycle + randr( 0, APP_TX_DUTYCYCLE_RND );
			LoRaWAN.cycle(10000);
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
  //switchFunction(); // ============================================== Comment for test
  rainLoop();
}

void init(){
  softwareSerial.begin(9600);
}

void rainLoop(){
  if(softwareSerial.available() > 0){
    byte i = 0;
    String data = "";
		while (softwareSerial.available()){
      char d = softwareSerial.read();
      data += d;
		}
    float temp = data.substring(1, data.lastIndexOf(" ")).toFloat();
    rainRate = (int)(temp*100);
	}
  if(millis() - millisTime > 5000){
    softwareSerial.write("A\n");
    millisTime = millis();
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