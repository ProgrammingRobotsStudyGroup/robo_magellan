/* Safety radio transmitter for RoboMagellan

*/
/* RFM69 library and code by Felix Rusu - felix@lowpowerlab.com
// Get libraries at: https://github.com/LowPowerLab/
// Make sure you adjust the settings in the configuration section below !!!
// **********************************************************************************
// Copyright Felix Rusu, LowPowerLab.com
// Library and code by Felix Rusu - felix@lowpowerlab.com
// **********************************************************************************
// License
// **********************************************************************************
// This program is free software; you can redistribute it 
// and/or modify it under the terms of the GNU General    
// Public License as published by the Free Software       
// Foundation; either version 3 of the License, or        
// (at your option) any later version.                    
//                                                        
// This program is distributed in the hope that it will   
// be useful, but WITHOUT ANY WARRANTY; without even the  
// implied warranty of MERCHANTABILITY or FITNESS FOR A   
// PARTICULAR PURPOSE. See the GNU General Public        
// License for more details.                              
//                                                        
// You should have received a copy of the GNU General    
// Public License along with this program.
// If not, see <http://www.gnu.org/licenses></http:>.
//                                                        
// Licence can be viewed at                               
// http://www.gnu.org/licenses/gpl-3.0.txt
//
// Please maintain this license information along with authorship
// and copyright notices in any redistribution of this code
// **********************************************************************************/

#include <RFM69.h>    //get it here: https://www.github.com/lowpowerlab/rfm69
#include <RFM69registers.h>  // added this so I could change bit rate
#include <SPI.h>
#include <elapsedMillis.h>

//*********************************************************************************************
// *********** IMPORTANT CONFIGURATION SETTINGS *************
//*********************************************************************************************
#define NETWORKID     100  // The same on all nodes that talk to each other
#define NODEID        2    // The unique identifier of this node
#define RECEIVER      1    // The recipient of packets

//Match frequency to frequency of receiver
#define FREQUENCY     RF69_915MHZ
#define ENCRYPTKEY    "RGRoboMagellan16" //exactly the same 16 characters/bytes on all nodes!
#define IS_RFM69HCW   true // set to 'true' if you are using an RFM69HCW module

//*********************************************************************************************
#define SERIAL_BAUD   115200
#define DEBUG         0  // 1 for serial debug, 0 for not

/* for Feather 32u4 Radio */
#define RFM69_CS      8
#define RFM69_IRQ     7
#define RFM69_IRQN    4  // Pin 7 is IRQ 4!
#define RFM69_RST     4

#define LED_BUILTIN   13  // onboard blinky
#define BUTTON        23  // Enable pushbutton
#define POWER_OFF     5   // signal to turn off power via Enable
#define V_BAT_PIN     A9  // v divider on battery

const int V_BAT_LOW_THRESHOLD = 3500;  // below this mV level, signal low battery

RFM69 radio = RFM69(RFM69_CS, RFM69_IRQ, IS_RFM69HCW, RFM69_IRQN);

void setup() {
    
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(BUTTON, INPUT_PULLUP);
  pinMode(POWER_OFF, OUTPUT);
  
  digitalWrite(LED_BUILTIN, LOW);
  digitalWrite(POWER_OFF, LOW);
  
  // Hard Reset the RFM module
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, HIGH);
  delay(100);
  digitalWrite(RFM69_RST, LOW);
  delay(100);

  // Initialize radio
  radio.initialize(FREQUENCY,NODEID,NETWORKID);
  if (IS_RFM69HCW) {
    radio.setHighPower();    // Only for RFM69HCW & HW!
  }
  
  radio.setPowerLevel(31); // power output ranges from 0 (5dBm) to 31 (20dBm)
  radio.encrypt(ENCRYPTKEY);
  
  radio.setFrequency(902000000);  // set frequency; must match on all nodes
  
  // packet mode, FSK, Gaussian shaping BT=1.0 (same as Radiohead test)
  radio.writeReg(REG_DATAMODUL, RF_DATAMODUL_DATAMODE_PACKET | RF_DATAMODUL_MODULATIONTYPE_FSK | RF_DATAMODUL_MODULATIONSHAPING_01);
  
  // bit rate
  radio.writeReg(REG_BITRATEMSB, RF_BITRATEMSB_25000);  // set bitrate = 25 kbps
  radio.writeReg(REG_BITRATELSB, RF_BITRATELSB_25000);
  
  // frequency deviation
  radio.writeReg(REG_FDEVMSB, RF_FDEVMSB_50000);  // set FDEV = 50 kHz
  radio.writeReg(REG_FDEVLSB, RF_FDEVLSB_50000);
  
  // Rx BW
  radio.writeReg(REG_RXBW, RF_RXBW_DCCFREQ_110 | RF_RXBW_MANT_16 | RF_RXBW_EXP_2);  // set RxBw = 125 kHz
  radio.writeReg(REG_AFCBW, RF_AFCBW_DCCFREQAFC_110 | RF_AFCBW_MANTAFC_16 | RF_AFCBW_EXPAFC_1);  // set RxAfcBw = 250 kHz
  
  // AFC
  radio.writeReg(REG_AFCCTRL, RF_AFCCTRL_LOWBETA_OFF);  // low beta off
  // below defaults to off
  //radio.writeReg(REG_AFCFEI, RF_AFCFEI_AFCAUTOCLEAR_ON | RF_AFCFEI_AFCAUTO_ON);  // afc auto every time Rx mode entered and autoclear every time
  
  // RSSI thresh
  //radio.writeReg(REG_RSSITHRESH, 0xA0);  // RSSI thresh -80 dBm; this is what Jeelabs used
  //radio.writeReg(REG_RSSITHRESH, 0xC8);  // RSSI thresh -100 dBm; default is -114 dBm
  //radio.writeReg(REG_RSSITHRESH, 0xD6);  // RSSI thresh -107 dBm;
  
  // preamble length
  radio.writeReg(REG_PREAMBLELSB, 0x05);  // preamble size 5 bytes
  
  // sync words
  radio.writeReg(REG_SYNCCONFIG, RF_SYNC_ON | RF_SYNC_SIZE_2);  // sync on, size 2
  radio.writeReg(REG_SYNCVALUE1, 0x2D);  // same as RFM69
  radio.writeReg(REG_SYNCVALUE2, 0xD4);  // same as RFM69
    
  // packet config: variable length, whitening, CRC, autoclear, no address filtering
  radio.writeReg(REG_PACKETCONFIG1, RF_PACKET1_FORMAT_VARIABLE | RF_PACKET1_DCFREE_WHITENING | RF_PACKET1_CRC_ON | RF_PACKET1_CRCAUTOCLEAR_ON | RF_PACKET1_ADRSFILTERING_OFF);
  radio.writeReg(REG_PACKETCONFIG2, 0x02);
  
  // go with default for payload length
  //radio.writeReg(REG_PAYLOADLENGTH, 0x42);  // payload length;  confusing, 0x42 is jeelabs value
  
  // FIFO threshold
  radio.writeReg(REG_FIFOTHRESH, RF_FIFOTHRESH_TXSTART_FIFONOTEMPTY | RF_FIFOTHRESH_VALUE);  
    
  // DAGC
  radio.writeReg(REG_TESTDAGC, RF_DAGC_IMPROVED_LOWBETA0);  // DAGC for fading improvement
  
  timer4_init();  // initialize timer4 for LED status blinking
   
  if (DEBUG) {
    while (!Serial); // wait until serial console is open, remove if not tethered to computer. Delete this line on ESP8266
    Serial.begin(SERIAL_BAUD);
    //Serial.println("Robot Garden RoboMagellan Safety Transmitter");
    Serial.print("\nTransmitting at ");
    Serial.print(radio.getFrequency());
    Serial.println(" MHz");
  }
}

void loop() { 
  int ack_count = 0;  // number of ACKs in ackCountWindow tries
  int try_count = 0;  // number of messages tried
  int try_max = 10;  // how many tries to use for link quality
  elapsedMillis since_tx = 0;  // ms since transmit
  
  while (!digitalRead(BUTTON)) {  // as long as button is down, send heartbeat
    unsigned long tx_interval = 200;  // ms interval between transmissions
    while (since_tx < tx_interval - 15) {
      ;  // should we consider sleeping here to save battery?
    }
  
    char radiopacket[30] = "RoboMagellan Heartbeat"; 
    try_count += 1;
    since_tx = 0;
    if (radio.sendWithRetry(RECEIVER, radiopacket, strlen(radiopacket), 1, 35))  //target node Id, message as string or byte array, message length, # retries, timeout ms for ACK
    {
      ack_count += 1;  // received an ACK
    }
    
    if (try_count >= try_max)  // time to assess link quality
    {
      if (DEBUG)
      {
        Serial.print("try_count: "); Serial.print(try_count);
        Serial.print(";      ack_count: "); Serial.println(ack_count);
      }
      int link_qual = 0;
      if (ack_count >= 8)  // TODO: magic numbers
      {
        link_qual = 0; // great link quality
      } else if (ack_count >= 5)
      {
        link_qual = 1;  // medium link quality
      } else
      {
        link_qual = 2;  // poor link quality
      }
      
      long measured_vbat = analogRead(V_BAT_PIN);  // read battery voltage
      measured_vbat = measured_vbat * 2 * 3300 / 1024;  // mV
      bool batt_low = false;
      if (measured_vbat < V_BAT_LOW_THRESHOLD) 
      {
        batt_low = true;
      }
            
      Blink(link_qual, batt_low);  // set the LED blink pattern
      try_count = 0;  // reset
      ack_count = 0;
    }
    
    radio.receiveDone(); //put radio in RX mode (don't know why...)
    
    if (DEBUG) {
      Serial.flush(); //make sure all serial data is clocked out before sleeping the MCU      
    }
  }
}
  
void Blink(int link, bool batt)
{
  unsigned int period_slow = 511;  // must be less than 1024
  unsigned int period_fast = 190;
  unsigned int width_narrow_on = 7;  // pulse width for short ON flashes
  unsigned int width_narrow_off = 14;  // off-width for short OFF dropouts 
  int pattern = 0;
  
  if (! batt)
  {
    pattern = link;  // 0, 1, 2
  } else
  {
    pattern = link + 3;  // 3, 4, 5
  }
  
  switch (pattern)
    {
      case 0:  // on solid
        TC4H = period_slow >> 8;  // bits 9, 10 for period
        OCR4C = period_slow & 0xFF;  // bits 0-7 for period (TOP)
        TC4H = period_slow >> 8;  // bits 9, 10 for pulse width
        OCR4A = period_slow & 0xFF;  // bits 0-7 for pulse width
        break;
      case 1:  // 50% on-off at slow rate
        TC4H = period_slow >> 8;
        OCR4C = period_slow & 0xFF;
        TC4H = (period_slow / 2) >> 8;       
        OCR4A = (period_slow / 2) & 0xFF;
        break;
      case 2:  // off with brief on flash at slow rate
        TC4H = period_slow >> 8;  // bits 9, 10 for period
        OCR4C = period_slow & 0xFF;  // bits 0-7 for period (TOP)
        TC4H = width_narrow_on >> 8;  // bits 9, 10 again
        OCR4A = width_narrow_on & 0xFF;  // sets pulse width
        break;
      case 3:  // on with brief off dropout at fast rate
        TC4H = period_fast >> 8; 
        OCR4C = period_fast & 0xFF;
        TC4H = (period_fast - width_narrow_off) >> 8;
        OCR4A = (period_fast - width_narrow_off) & 0xFF;
        break;
      case 4:  // 50% on-off at fast rate
        TC4H = period_fast >> 8;  // for bits 9, 10
        OCR4C = period_fast & 0xFF;  // TOP - sets period
        TC4H = (period_fast / 2) >> 8;  // bits 9, 10 again
        OCR4A = (period_fast / 2) & 0xFF;  // sets pulse width
        break;
      case 5:  // off with brief on flash at fast rate
        TC4H = period_fast >> 8;  // for bits 9, 10
        OCR4C = period_fast & 0xFF;  // TOP - sets period
        TC4H = width_narrow_on >> 8;  // bits 9, 10 again
        OCR4A = width_narrow_on & 0xFF;  // sets pulse width
        break;
    }
}

void timer4_init(void)
{
  TCCR4B = 0;
  TCCR4B = bit(CS43) | bit(CS42) | bit(CS41) | bit(CS40);  // pre-scaler 16384
  TCCR4D = 0; 
  TCCR4D = bit(WGM40);  // phase and freq correct PWM
  TC4H = 1;  // for bits 9, 10
  OCR4C = 255;  // TOP - sets period
  TC4H = 0;  // bits 9, 10 again
  OCR4A = 255;  // sets pulse width
  TCCR4A = 0;  // weird - no output at all when these two lines were first in the initialization
  TCCR4A = bit(COM4A1) | bit(PWM4A);  // non-inverted; PWM OCR4A enabled 
}