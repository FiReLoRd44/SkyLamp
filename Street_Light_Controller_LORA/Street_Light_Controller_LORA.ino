/*
 *  Fully operational temperature sensor on analog A0 to test the LoRa gateway
 *
 *  Copyright (C) 2016-2020 Congduc Pham, University of Pau, France
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.

 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with the program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *****************************************************************************
 * last update: November 12th, 2020 by C. Pham
 * 
 * NEW: LoRa communicain library moved from Libelium's lib to StuartProject's lib
 * https://github.com/StuartsProjects/SX12XX-LoRa
 * to support SX126X, SX127X and SX128X chips (SX128X is LoRa in 2.4GHz band)
 * 
 * This version uses the same structure than the Arduino_LoRa_Demo_Sensor where
 * the sensor-related code is in a separate file
 */

#include <SPI.h> 
//this is the standard behaviour of library, use SPI Transaction switching
#define USE_SPI_TRANSACTION 
///////////////////////////////////////////////////////////////////////////////////////////////////////////
// please uncomment only 1 choice 
//#define SX126X
#define SX127X
//#define SX128X
///////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef SX126X
#include <SX126XLT.h>                                          
#include "SX126X_RadioSettings.h"
#endif

#ifdef SX127X
#include <SX127XLT.h>                                          
#include "SX127X_RadioSettings.h"
#endif

#ifdef SX128X
#include <SX128XLT.h>                                          
#include "SX128X_RadioSettings.h"
#endif

#include <NewPing.h>
#include "DHT.h"

const byte dhtPin = 3;
#define DHTTYPE DHT11  
#define TRIGGER_PIN  7
#define ECHO_PIN     6
#define LED 2
#define MAX_DISTANCE 200
float duration, distance, c;
int iterations = 5;
int light;
float counts, mv, perc;
DHT dht(dhtPin, DHTTYPE);

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

#include "my_temp_sensor_code.h"

// http://patorjk.com/software/taag
/********************************************************************
 _____              __ _                       _   _             
/  __ \            / _(_)                     | | (_)            
| /  \/ ___  _ __ | |_ _  __ _ _   _ _ __ __ _| |_ _  ___  _ __  
| |    / _ \| '_ \|  _| |/ _` | | | | '__/ _` | __| |/ _ \| '_ \ 
| \__/\ (_) | | | | | | | (_| | |_| | | | (_| | |_| | (_) | | | |
 \____/\___/|_| |_|_| |_|\__, |\__,_|_|  \__,_|\__|_|\___/|_| |_|
                          __/ |                                  
                         |___/                                   
********************************************************************/

#if defined ARDUINO_ESP8266_ESP01 || defined ARDUINO_ESP8266_NODEMCU || defined ESP8266
//uncomment if you want to disable WiFi on ESP8266 boards
//#include <ESP8266WiFi.h>
#endif

///////////////////////////////////////////////////////////////////
// COMMENT OR UNCOMMENT TO CHANGE FEATURES. 
// ONLY IF YOU KNOW WHAT YOU ARE DOING!!! OTHERWISE LEAVE AS IT IS
//#define WITH_EEPROM
//if you are low on program memory, comment STRING_LIB to save about 2K
//#define STRING_LIB
//#define LOW_POWER
//#define LOW_POWER_HIBERNATE
//#define WITH_ACK
//#define LOW_POWER_TEST
//uncomment to use a customized frequency. TTN plan includes 868.1/868.3/868.5/867.1/867.3/867.5/867.7/867.9 for LoRa
//#define MY_FREQUENCY 868100000
//when sending to a LoRaWAN gateway (e.g. running util_pkt_logger) but with no native LoRaWAN format, just to set the correct sync word
//#define PUBLIC_SYNCWORD
///////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////
// ADD HERE OTHER PLATFORMS THAT DO NOT SUPPORT EEPROM
#if defined ARDUINO_SAM_DUE || defined __SAMD21G18A__
#undef WITH_EEPROM
#endif

///////////////////////////////////////////////////////////////////
// ADD HERE OTHER PLATFORMS THAT DO NOT SUPPORT LOW POWER LIB
#if defined ARDUINO_SAM_DUE || defined _VARIANT_ARDUINO_DUE_X_
#undef LOW_POWER
#endif
///////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////
// CHANGE HERE THE NODE ADDRESS 
uint8_t node_addr=8;
//////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////
// CHANGE HERE THE TIME IN MINUTES BETWEEN 2 READING & TRANSMISSION
unsigned int idlePeriodInMin = 1;
///////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////
// IF YOU SEND A LONG STRING, INCREASE THE SIZE OF MESSAGE
uint8_t message[100];
///////////////////////////////////////////////////////////////////

//create a library class instance called LT
//to handle LoRa radio communications

#ifdef SX126X
SX126XLT LT;
#endif

#ifdef SX127X
SX127XLT LT;
#endif

#ifdef SX128X
SX128XLT LT;
#endif

//keep track of the number of successful transmissions
uint32_t TXPacketCount=0;

/*****************************
 _____           _      
/  __ \         | |     
| /  \/ ___   __| | ___ 
| |    / _ \ / _` |/ _ \
| \__/\ (_) | (_| |  __/
 \____/\___/ \__,_|\___|
*****************************/

// we wrapped Serial.println to support the Arduino Zero or M0
#if defined __SAMD21G18A__ && not defined ARDUINO_SAMD_FEATHER_M0
#define PRINTLN                   SerialUSB.println("")              
#define PRINT_CSTSTR(param)       SerialUSB.print(F(param))
#define PRINTLN_CSTSTR(param)     SerialUSB.println(F(param))
#define PRINT_STR(fmt,param)      SerialUSB.print(param)
#define PRINTLN_STR(fmt,param)    SerialUSB.println(param)
#define PRINT_VALUE(fmt,param)    SerialUSB.print(param)
#define PRINTLN_VALUE(fmt,param)  SerialUSB.println(param)
#define PRINT_HEX(fmt,param)      SerialUSB.print(param,HEX)
#define PRINTLN_HEX(fmt,param)    SerialUSB.println(param,HEX)
#define FLUSHOUTPUT               SerialUSB.flush()
#else
#define PRINTLN                   Serial.println("")
#define PRINT_CSTSTR(param)       Serial.print(F(param))
#define PRINTLN_CSTSTR(param)     Serial.println(F(param))
#define PRINT_STR(fmt,param)      Serial.print(param)
#define PRINTLN_STR(fmt,param)    Serial.println(param)
#define PRINT_VALUE(fmt,param)    Serial.print(param)
#define PRINTLN_VALUE(fmt,param)  Serial.println(param)
#define PRINT_HEX(fmt,param)      Serial.print(param,HEX)
#define PRINTLN_HEX(fmt,param)    Serial.println(param,HEX)
#define FLUSHOUTPUT               Serial.flush()
#endif

#ifdef WITH_EEPROM
#include <EEPROM.h>
#endif

#ifdef WITH_ACK
#define NB_RETRIES 2
#endif

#ifdef LOW_POWER
// this is for the Teensy36, Teensy35, Teensy31/32 & TeensyLC
// need v6 of Snooze library
#if defined __MK20DX256__ || defined __MKL26Z64__ || defined __MK64FX512__ || defined __MK66FX1M0__
#define LOW_POWER_PERIOD 1
#include <Snooze.h>
SnoozeTimer timer;
SnoozeBlock sleep_config(timer);
#elif defined ARDUINO_ESP8266_ESP01 || defined ARDUINO_ESP8266_NODEMCU || defined ESP8266
#define LOW_POWER_PERIOD 60
//we will use the deepSleep feature, so no additional library
#else // for all other boards based on ATMega168, ATMega328P, ATMega32U4, ATMega2560, ATMega256RFR2, ATSAMD21G18A
#define LOW_POWER_PERIOD 8
// you need the LowPower library from RocketScream
// https://github.com/rocketscream/Low-Power
#include "LowPower.h"

#ifdef __SAMD21G18A__
// use the RTC library
#include "RTCZero.h"
/* Create an rtc object */
RTCZero rtc;
#endif
#endif
unsigned int nCycle = idlePeriodInMin*60/LOW_POWER_PERIOD;
#endif

unsigned long nextTransmissionTime=0L;

#ifdef WITH_EEPROM
struct sx1272config {

  uint8_t flag1;
  uint8_t flag2;
  uint8_t seq;
  // can add other fields such as LoRa mode,...
};

sx1272config my_sx1272config;
#endif

#ifndef STRING_LIB

char *ftoa(char *a, double f, int precision)
{
 long p[] = {0,10,100,1000,10000,100000,1000000,10000000,100000000};
 
 char *ret = a;
 long heiltal = (long)f;
 itoa(heiltal, a, 10);
 while (*a != '\0') a++;
 *a++ = '.';
 long desimal = abs((long)((f - heiltal) * p[precision]));
 if (desimal < p[precision-1]) {
  *a++ = '0';
 } 
 itoa(desimal, a, 10);
 return ret;
}

#endif

/*****************************
 _____      _               
/  ___|    | |              
\ `--.  ___| |_ _   _ _ __  
 `--. \/ _ \ __| | | | '_ \ 
/\__/ /  __/ |_| |_| | |_) |
\____/ \___|\__|\__,_| .__/ 
                     | |    
                     |_|    
******************************/

void setup()
{

pinMode(LED,OUTPUT);
dht.begin();
#if defined ARDUINO_ESP8266_ESP01 || defined ARDUINO_ESP8266_NODEMCU || defined ESP8266
  //uncomment to disable WiFi on the ESP8266 boards
  //will save about 50mA
  //WiFi.disconnect();
  //WiFi.mode(WIFI_OFF);
  //WiFi.forceSleepBegin();
  //delay(1);
#endif
  
  // initialization of the temperature sensor
  sensor_Init();

#ifdef LOW_POWER
#ifdef __SAMD21G18A__
  rtc.begin();
#endif  
#else
  digitalWrite(PIN_POWER,HIGH);
#endif

  delay(3000);
  // Open serial communications and wait for port to open:
#if defined __SAMD21G18A__ && not defined ARDUINO_SAMD_FEATHER_M0 
  SerialUSB.begin(38400);
#else
  Serial.begin(38400);  
#endif
  
  randomSeed(analogRead(0)+analogRead(0)+analogRead(0));
      
  // Print a start message
  PRINT_CSTSTR("Simple LoRa temperature sensor\n");

#ifdef ARDUINO_AVR_PRO
  PRINT_CSTSTR("Arduino Pro Mini detected\n");  
#endif
#ifdef ARDUINO_AVR_NANO
  PRINT_CSTSTR("Arduino Nano detected\n");   
#endif
#ifdef ARDUINO_AVR_MINI
  PRINT_CSTSTR("Arduino Mini/Nexus detected\n");  
#endif
#ifdef ARDUINO_AVR_MEGA2560
  PRINT_CSTSTR("Arduino Mega2560 detected\n");  
#endif
#ifdef ARDUINO_SAM_DUE
  PRINT_CSTSTR("Arduino Due detected\n");  
#endif
#ifdef __MK66FX1M0__
  PRINT_CSTSTR("Teensy36 MK66FX1M0 detected\n");
#endif
#ifdef __MK64FX512__
  PRINT_CSTSTR("Teensy35 MK64FX512 detected\n");
#endif
#ifdef __MK20DX256__
  PRINT_CSTSTR("Teensy31/32 MK20DX256 detected\n");
#endif
#ifdef __MKL26Z64__
  PRINT_CSTSTR("TeensyLC MKL26Z64 detected\n");
#endif
#if defined ARDUINO_SAMD_ZERO && not defined ARDUINO_SAMD_FEATHER_M0
  PRINT_CSTSTR("Arduino M0/Zero detected\n");
#endif
#ifdef ARDUINO_AVR_FEATHER32U4 
  PRINT_CSTSTR("Adafruit Feather32U4 detected\n"); 
#endif
#ifdef ARDUINO_SAMD_FEATHER_M0
  PRINT_CSTSTR("Adafruit FeatherM0 detected\n");
#endif
#if defined ARDUINO_ESP8266_ESP01 || defined ARDUINO_ESP8266_NODEMCU || defined ESP8266
  PRINT_CSTSTR("Expressif ESP8266 detected\n");
  //uncomment if you want to disable the WiFi, this will reset your board
  //but maybe it is preferable to use the WiFi.mode(WIFI_OFF), see above
  //ESP.deepSleep(1, WAKE_RF_DISABLED);  
#endif
#if defined ARDUINO_Heltec_WIFI_LoRa_32 || defined ARDUINO_WIFI_LoRa_32  || defined HELTEC_LORA
  PRINT_CSTSTR("Heltec WiFi LoRa 32 detected\n");
#endif

// See http://www.nongnu.org/avr-libc/user-manual/using_tools.html
// for the list of define from the AVR compiler

#ifdef __AVR_ATmega328P__
  PRINT_CSTSTR("ATmega328P detected\n");
#endif 
#ifdef __AVR_ATmega32U4__
  PRINT_CSTSTR("ATmega32U4 detected\n");
#endif 
#ifdef __AVR_ATmega2560__
  PRINT_CSTSTR("ATmega2560 detected\n");
#endif 
#ifdef __SAMD21G18A__ 
  PRINT_CSTSTR("SAMD21G18A ARM Cortex-M0+ detected\n");
#endif
#ifdef __SAM3X8E__ 
  PRINT_CSTSTR("SAM3X8E ARM Cortex-M3 detected\n");
#endif
#ifdef ESP32 
  PRINT_CSTSTR("ESP32 detected\n");
#endif
      
  //start SPI bus communication
  SPI.begin();

  //setup hardware pins used by device, then check if device is found
#ifdef SX126X
  if (LT.begin(NSS, NRESET, RFBUSY, DIO1, DIO2, DIO3, RX_EN, TX_EN, SW, LORA_DEVICE))
#endif

#ifdef SX127X
  if (LT.begin(NSS, NRESET, DIO0, DIO1, DIO2, LORA_DEVICE))
#endif

#ifdef SX128X
  if (LT.begin(NSS, NRESET, RFBUSY, DIO1, DIO2, DIO3, RX_EN, TX_EN, LORA_DEVICE))
#endif
  {
    PRINT_CSTSTR("LoRa Device found\n");                                  
    delay(1000);
  }
  else
  {
    PRINT_CSTSTR("No device responding\n");
    while (1){ }
  }

/*******************************************************************************************************
  Based from SX12XX example - Stuart Robinson 
*******************************************************************************************************/

  //The function call list below shows the complete setup for the LoRa device using the
  // information defined in the Settings.h file.
  //The 'Setup LoRa device' list below can be replaced with a single function call;
  //LT.setupLoRa(Frequency, Offset, SpreadingFactor, Bandwidth, CodeRate, Optimisation);

  //***************************************************************************************************
  //Setup LoRa device
  //***************************************************************************************************
  //got to standby mode to configure device
  LT.setMode(MODE_STDBY_RC);
#ifdef SX126X
  LT.setRegulatorMode(USE_DCDC);
  LT.setPaConfig(0x04, PAAUTO, LORA_DEVICE);
  LT.setDIO3AsTCXOCtrl(TCXO_CTRL_3_3V);
  LT.calibrateDevice(ALLDevices);                //is required after setting TCXO
  LT.calibrateImage(DEFAULT_CHANNEL);
  LT.setDIO2AsRfSwitchCtrl();
#endif
#ifdef SX128X
  LT.setRegulatorMode(USE_LDO);
#endif
  //set for LoRa transmissions                              
  LT.setPacketType(PACKET_TYPE_LORA);
  //set the operating frequency                 
#ifdef MY_FREQUENCY
  LT.setRfFrequency(MY_FREQUENCY, Offset);
#else  
  LT.setRfFrequency(DEFAULT_CHANNEL, Offset);                   
#endif
//run calibration after setting frequency
#ifdef SX127X
  LT.calibrateImage(0);
#endif
  //set LoRa modem parameters
#if defined SX126X || defined SX127X
  LT.setModulationParams(SpreadingFactor, Bandwidth, CodeRate, Optimisation);
#endif
#ifdef SX128X
  LT.setModulationParams(SpreadingFactor, Bandwidth, CodeRate);
#endif                                     
  //where in the SX buffer packets start, TX and RX
  LT.setBufferBaseAddress(0x00, 0x00);
  //set packet parameters
#if defined SX126X || defined SX127X                     
  LT.setPacketParams(8, LORA_PACKET_VARIABLE_LENGTH, 255, LORA_CRC_ON, LORA_IQ_NORMAL);
#endif
#ifdef SX128X
  LT.setPacketParams(12, LORA_PACKET_VARIABLE_LENGTH, 255, LORA_CRC_ON, LORA_IQ_NORMAL, 0, 0);
#endif
  //syncword, LORA_MAC_PRIVATE_SYNCWORD = 0x12, or LORA_MAC_PUBLIC_SYNCWORD = 0x34 
#if defined SX126X || defined SX127X
#ifdef PUBLIC_SYNCWORD
  LT.setSyncWord(LORA_MAC_PUBLIC_SYNCWORD);              
#else
  LT.setSyncWord(LORA_MAC_PRIVATE_SYNCWORD);
#endif
  //set for highest sensitivity at expense of slightly higher LNA current
  LT.setHighSensitivity();  //set for maximum gain
#endif
#ifdef SX126X
  //set for IRQ on TX done and timeout on DIO1
  LT.setDioIrqParams(IRQ_RADIO_ALL, (IRQ_TX_DONE + IRQ_RX_TX_TIMEOUT), 0, 0);
#endif
#ifdef SX127X
  //set for IRQ on TX done
  LT.setDioIrqParams(IRQ_RADIO_ALL, IRQ_TX_DONE, 0, 0);
  LT.setPA_BOOST(PA_BOOST);
#endif
#ifdef SX128X
  LT.setDioIrqParams(IRQ_RADIO_ALL, (IRQ_TX_DONE + IRQ_RX_TX_TIMEOUT), 0, 0);
#endif                                  
  
  //***************************************************************************************************

  PRINTLN;
  //reads and prints the configured LoRa settings, useful check
  LT.printModemSettings();                               
  PRINTLN;
  //reads and prints the configured operating settings, useful check
  LT.printOperatingSettings();                           
  PRINTLN;
  PRINTLN;
#if defined SX126X || defined SX127X  
  //print contents of device registers, normally 0x00 to 0x4F
  LT.printRegisters(0x00, 0x4F);
#endif                       
#ifdef SX128X
  //print contents of device registers, normally 0x900 to 0x9FF 
  LT.printRegisters(0x900, 0x9FF);
#endif  
    
/*******************************************************************************************************
  End from SX12XX example - Stuart Robinson 
*******************************************************************************************************/

#ifdef WITH_EEPROM
#if defined ARDUINO_ESP8266_ESP01 || defined ARDUINO_ESP8266_NODEMCU
  EEPROM.begin(512);
#endif
  // get config from EEPROM
  EEPROM.get(0, my_sx1272config);

  // found a valid config?
  if (my_sx1272config.flag1==0x12 && my_sx1272config.flag2==0x34) {
    PRINT_CSTSTR("Get back previous sx1272 config\n");
    // set sequence number for SX1272 library
    LT.setTXSeqNo(my_sx1272config.seq);
    PRINT_CSTSTR("Using packet sequence number of ");
    PRINT_VALUE("%d", LT.readTXSeqNo());
    PRINTLN;  
  }
  else {
    // otherwise, write config and start over
    my_sx1272config.flag1=0x12;
    my_sx1272config.flag2=0x34;
    my_sx1272config.seq=LT.readTXSeqNo();  
  }
#endif

  PRINT_CSTSTR("Setting Power: ");
  PRINTLN_VALUE("%d", MAX_DBM); 

  LT.setDevAddr(node_addr);
  PRINT_CSTSTR("node addr: ");
  PRINT_VALUE("%d", node_addr);
  PRINTLN;  

#ifdef SX126X
  PRINT_CSTSTR("SX126X");
#endif
#ifdef SX127X
  PRINT_CSTSTR("SX127X");
#endif
#ifdef SX128X
  PRINT_CSTSTR("SX128X");
#endif 
    
  // Print a success message
  PRINT_CSTSTR(" successfully configured\n");

  delay(500);
}

/*****************************
 _                       
| |                      
| |     ___   ___  _ __  
| |    / _ \ / _ \| '_ \ 
| |___| (_) | (_) | |_) |
\_____/\___/ \___/| .__/ 
                  | |    
                  |_|    
*****************************/

void loop(void)
{
  long startSend;
  long endSend;
  int e;
  float temp;
  float humidity;

  counts = analogRead(A1);
  Serial.print("\n");
  Serial.println(counts);
  //Calculating the accurate voltage
  mv = counts * 4.63 / 1023;
  //Mapping the voltage to accurate battery percentage
  perc = mapb(mv, 3.6, 4.2, 0, 100);
  Serial.print("Voltage= ");
  Serial.println(mv);
  Serial.print("Battery level= ");
  Serial.print(perc);
  Serial.println(" %");
  
  //get the temp in celcius (default)
  temp = dht.readTemperature();
  humidity = dht.readHumidity();
  //reading the analog value from pin A0
  light = analogRead(A0);
  //printing the analog values on the serial monitor
  Serial.println(light);
  //Checking day and night
  if (light > 150)
  {
    Serial.println("Day");
  }
  else
  {
    Serial.println("Night");
    Serial.print("Distance is:");
    Serial.println(sonar.ping_cm()); 

    //duration = pulseIn(11, HIGH);
    duration = sonar.ping();
    Serial.print("Duration = ");
    Serial.println(duration);
   
  
    //calculate the speed of sound based on value from temp sensor
    c = 331.4 + temp * 0.606 + humidity * 0.0124;
    c = c * 100 * 0.000001;
    //calculate the distance from duration and speed of sound
    distance = c * (duration/2) ;
  
    Serial.print("Temperature = ");
    Serial.print(temp);
    Serial.println(" Deg C  ");

    Serial.print("Humidity = ");
    Serial.println(humidity);
    
    Serial.print("Speed of sound = ");
    Serial.println(c);
    
    Serial.print("Distance = ");
    Serial.println(distance);

    //Turn on the light if distance is less than 4 cm and greater than 0 cm, if greater than that keep it off. 
      if (distance > 0 && distance < 4)
      { 
        digitalWrite(LED,HIGH);
        Serial.println("LED on");
      }
  
      else 
      {
        digitalWrite(LED,LOW);
        Serial.println("LED off");
      }
  
  }
  delay(2000);

//#ifndef LOW_POWER
  // 600000+random(15,60)*1000
  if (millis() > nextTransmissionTime) {
//#endif

/*#ifdef LOW_POWER
      digitalWrite(PIN_POWER,HIGH);
      // security?
      delay(200);   
#endif*/
//Reading the voltage of battery 
  

//      temp = 0.0;
//      
//      for (int i=0; i<5; i++) {
//          temp += sensor_getValue();  
//          delay(100);
//      }


      
//#ifdef LOW_POWER
//      digitalWrite(PIN_POWER,LOW);
//#endif

//      PRINT_CSTSTR("Mean temp is ");
//      temp = temp/5;
//      PRINT_VALUE("%f", temp);
//      PRINTLN;

      // for testing, uncomment if you just want to test, without a real temp sensor plugged
      //temp = 20.5;
      
      uint8_t r_size;
      
      // the recommended format if now \!TC/22.5
#ifdef STRING_LIB
      r_size=sprintf((char*)message,"\\!%s/%s",nomenclature_str,String(temp).c_str());
#else
      char float_temp[10];
      char float_hum[10];
      char float_battery[10];
      //ftoa: 12146 Bytes, dtostrf: 13442 Bytes, STRING_LIB: 14504 Bytes 
      //dtostrf takes about 1300 byte of program space more than our ftoa() procedure
      //dtostrf((float)temp, 4, 2, float_str);
      ftoa(float_temp,temp,2);
      ftoa(float_hum,humidity,2);
      ftoa(float_battery,perc,2);
      r_size=sprintf((char*)message,"\\!%s/%s/%s/%s/%s/%s","TC",float_temp,"HUM",float_hum,"BP",float_battery);
#endif

      PRINT_CSTSTR("Sending ");
      PRINT_STR("%s",(char*)(message));
      PRINTLN;
      
      PRINT_CSTSTR("Real payload size is ");
      PRINT_VALUE("%d", r_size);
      PRINTLN;

      LT.printASCIIPacket(message, r_size);
      PRINTLN;
      
      LT.CarrierSense();
        
      uint8_t p_type=PKT_TYPE_DATA;
    
#ifdef WITH_ACK
      p_type=PKT_TYPE_DATA | PKT_FLAG_ACK_REQ;
      PRINTLN_CSTSTR("%s","Will request an ACK");         
#endif
      startSend=millis();
      
      //will return packet length sent if OK, otherwise 0 if transmit error      
      if (LT.transmitAddressed(message, r_size, p_type, DEFAULT_DEST_ADDR, LT.readDevAddr(), 10000, MAX_DBM, WAIT_TX))
      {
        endSend = millis();                                          
        TXPacketCount++;
        uint16_t localCRC = LT.CRCCCITT(message, r_size, 0xFFFF);
        PRINT_CSTSTR("CRC,");
        PRINT_HEX("%d", localCRC);

        if (LT.readAckStatus()) {
          PRINTLN;
          PRINT_CSTSTR("Received ACK from ");
          PRINTLN_VALUE("%d", LT.readRXSource());
          PRINT_CSTSTR("SNR of transmitted pkt is ");
          PRINTLN_VALUE("%d", LT.readPacketSNRinACK());          
        }
      }
      else
      {
        endSend=millis();
        //if here there was an error transmitting packet
        uint16_t IRQStatus;
        IRQStatus = LT.readIrqStatus();                      
        PRINT_CSTSTR("SendError,");
        PRINT_CSTSTR(",IRQreg,");
        PRINT_HEX("%d", IRQStatus);
        LT.printIrqStatus(); 
      }
  
#ifdef WITH_EEPROM
      // save packet number for next packet in case of reboot    
      my_sx1272config.seq=LT.readTXSeqNo();    
      EEPROM.put(0, my_sx1272config);
#if defined ARDUINO_ESP8266_ESP01 || defined ARDUINO_ESP8266_NODEMCU || defined ESP8266
      EEPROM.commit();
#endif
#endif
      PRINTLN;
      PRINT_CSTSTR("LoRa pkt size ");
      PRINT_VALUE("%d", r_size);
      PRINTLN;
      
      PRINT_CSTSTR("LoRa pkt seq ");   
      PRINT_VALUE("%d", LT.readTXSeqNo()-1);    
      PRINTLN;
    
      PRINT_CSTSTR("LoRa Sent in ");
      PRINT_VALUE("%ld", endSend-startSend);
      PRINTLN;
/*  
#if defined LOW_POWER && not defined ARDUINO_SAM_DUE
      PRINT_CSTSTR("Switch to power saving mode\n");

      //CONFIGURATION_RETENTION=RETAIN_DATA_RAM on SX128X
      //parameter is ignored on SX127X
      LT.setSleep(CONFIGURATION_RETENTION);
        
      FLUSHOUTPUT;    
#ifdef LOW_POWER_TEST
      delay(10000);
#else            
      delay(10);
#endif

#ifdef __SAMD21G18A__
      // For Arduino M0 or Zero we use the built-in RTC
      rtc.setTime(17, 0, 0);
      rtc.setDate(1, 1, 2000);
      rtc.setAlarmTime(17, idlePeriodInMin, 0);
      // for testing with 20s
      //rtc.setAlarmTime(17, 0, 20);
      rtc.enableAlarm(rtc.MATCH_HHMMSS);
      //rtc.attachInterrupt(alarmMatch);
      rtc.standbyMode();
      
      LowPower.standby();
      
      PRINT_CSTSTR("SAMD21G18A wakes up from standby\n");      
      FLUSHOUTPUT;
#else

#if defined __MK20DX256__ || defined __MKL26Z64__ || defined __MK64FX512__ || defined __MK66FX1M0__
      // warning, setTimer accepts value from 1ms to 65535ms max
      // milliseconds      
      // by default, LOW_POWER_PERIOD is 60s for those microcontrollers      
      timer.setTimer(LOW_POWER_PERIOD*1000);
#endif

      nCycle = idlePeriodInMin*60/LOW_POWER_PERIOD;
                
      for (uint8_t i=0; i<nCycle; i++) {  

#if defined ARDUINO_AVR_MEGA2560 || defined ARDUINO_AVR_PRO || defined ARDUINO_AVR_NANO || defined ARDUINO_AVR_UNO || defined ARDUINO_AVR_MINI || defined __AVR_ATmega32U4__ 
          // ATmega2560, ATmega328P, ATmega168, ATmega32U4
          LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
          
#elif defined __MK20DX256__ || defined __MKL26Z64__ || defined __MK64FX512__ || defined __MK66FX1M0__
          // Teensy31/32 & TeensyLC
#ifdef LOW_POWER_HIBERNATE
          Snooze.hibernate(sleep_config);
#else            
          Snooze.deepSleep(sleep_config);
#endif
#elif defined ARDUINO_ESP8266_ESP01 || defined ARDUINO_ESP8266_NODEMCU || defined ESP8266
          //in microseconds
          //it is reported that RST pin should be connected to pin 16 to actually reset the board when deepsleep
          //timer is triggered
          ESP.deepSleep(LOW_POWER_PERIOD*1000*1000);
#else
          // use the delay function
          delay(LOW_POWER_PERIOD*1000);
#endif                        
          PRINT_CSTSTR(".");
          FLUSHOUTPUT;
          delay(1);                        
      }
#endif*/      
      
//#else
      PRINT_VALUE("%ld", nextTransmissionTime);
      PRINTLN;
      PRINT_CSTSTR("Will send next value at\n");
      // can use a random part also to avoid collision
      nextTransmissionTime=millis()+(unsigned long)idlePeriodInMin*60*1000; //+(unsigned long)random(15,60)*1000;
      PRINT_VALUE("%ld", nextTransmissionTime);
      PRINTLN;
}
//#endif

  LT.wake();
}
float mapb(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
