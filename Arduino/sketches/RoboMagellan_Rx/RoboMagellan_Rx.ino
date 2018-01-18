/* Safety radio receiver for RoboMagellan

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

#include <Servo.h>
#include <elapsedMillis.h>

// for Feather OLED display
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

//*********************************************************************************************
// *********** IMPORTANT CONFIGURATION SETTINGS *************
//*********************************************************************************************
#define NETWORKID     100  // The same on all nodes that talk to each other
#define NODEID        1    // The unique identifier of this node
#define RECEIVER      2    // The recipient of packets; TODO not needed?

//Match frequency to frequency of transmitter
#define FREQUENCY     RF69_915MHZ
#define ENCRYPTKEY    "RGRoboMagellan16" //exactly the same 16 characters/bytes on all nodes!
#define IS_RFM69HCW   true // set to 'true' if you are using an RFM69HCW module

//*********************************************************************************************
#define SERIAL_BAUD   115200
#define DEBUG         0  // 1 for debug, 0 for not
#define DISPLAY_ON    0  // 1 for display, 0 for off

/* for Feather 32u4 Radio */
#define RFM69_CS      8
#define RFM69_IRQ     7
#define RFM69_IRQN    4  // Pin 7 is IRQ 4!
#define RFM69_RST     4

#define LED_BUILTIN   13            // onboard blinky
#define RED_LED_PIN   A5            // sim debug only
#define GRN_LED_PIN   A3            // sim debug only
#define BLU_LED_PIN   A2            // sim debug only
#define STOP_SERVO_PIN          12   // generates stopping RC sequence if heartbeat lost
#define SELECT_SERVO_PIN        11  // controls RC MUX
#define SIM_THROTTLE_SERVO_PIN  10  // sim only
#define RELAY_PIN  6                // drives relay for status output

// for Feather OLED display
#define BUTTON_A 9
#define BUTTON_B 6  // TODO CONFLICTS with RELAY_PIN
#define BUTTON_C 5
#if (SSD1306_LCDHEIGHT != 32)
 #error("Height incorrect, please fix Adafruit_SSD1306.h!");
#endif

RFM69 radio = RFM69(RFM69_CS, RFM69_IRQ, IS_RFM69HCW, RFM69_IRQN);

// for Feather OLED display
Adafruit_SSD1306 display = Adafruit_SSD1306();

int sim_throttle_val = 2000;  // val for simulated PH throttle

enum class State {IDLE, ENABLED, STOPPING};
State state = State::IDLE;  // state of the safety Rx
enum class HB_state {INVALID, VALIDATING, VALID};
HB_state hb_state = HB_state::INVALID;  // state of the heartbeat

Servo stop_servo;
Servo select_servo;
Servo sim_throttle_servo;

int SELECT_THROTTLE = 2000;  // usec pulse width for RC mux to use throttle input (slave)
int SELECT_STOP = 1000;  // usec pulse width for RC mux to use stop signal (master)
int SIM_THROTTLE_SERVO_NEUTRAL = 1500;  // usec pulse width for throttle off
int STOP_SERVO_THROTTLE_NEUTRAL = 1520;  // usec pw for ESC neutral
int STOP_SERVO_THROTTLE_SLOW_FWD = 1600;  // usec pw for ESC slow fwd during stop sequence
int STOP_SERVO_THROTTLE_BRAKE = 1000;  // usec pw for ESC braking

elapsedMillis ms_since_last_msg;  // timer for incoming heartbeat messages
unsigned long ms_max_msg_gap = 500;  // gap between messages > this will timeout heartbeat
int msg_count = 0;  // number of msgs counted toward validating signal
int msgs_to_validate = 3;  // must get this many msgs to declare signal valid
elapsedMillis msg_rate_timer;  // time the msg_rate_period

unsigned long msg_rate_period = 1000;  // count msgs in this time period (ms)
int msg_rate_count = 0;  // counter for msgs recd

void setup() {
  // for Feather OLED display
  pinMode(BUTTON_A, INPUT_PULLUP);
  pinMode(BUTTON_B, INPUT_PULLUP);
  pinMode(BUTTON_C, INPUT_PULLUP);
  
  // set pin modes
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(GRN_LED_PIN, OUTPUT);
  pinMode(BLU_LED_PIN, OUTPUT);
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);  // ensure status output starts NOK
      
  stop_servo.attach(STOP_SERVO_PIN);
  stop_servo.writeMicroseconds(SIM_THROTTLE_SERVO_NEUTRAL);  // the OFF val for throttle
  select_servo.attach(SELECT_SERVO_PIN);
  select_servo.writeMicroseconds(SELECT_STOP);  // start with MUX on stop channel
  sim_throttle_servo.attach(SIM_THROTTLE_SERVO_PIN);
  sim_throttle_servo.writeMicroseconds(SIM_THROTTLE_SERVO_NEUTRAL);  // start in OFF
  
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
  
  if (DISPLAY_ON)  // for Feather OLED display
  {   
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the 128x32)
    // Show image buffer on the display hardware.
    // Since the buffer is intialized with an Adafruit splashscreen
    // internally, this will display the splashscreen.
    display.display();
    delay(1000);
    // Clear the buffer.
    display.clearDisplay();
    display.display();
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.setCursor(0,0);
    display.println("IDLE");
    display.display();
  }
  
  radio.setFrequency(902000000);
  
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
  //radio.writeReg(REG_RSSITHRESH, 0xA0);  // RSSI thresh -80 dBm; this is what Jeelabs used; makes much worse
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
  //radio.writeReg(REG_PAYLOADLENGTH, 0x42);  // payload length;  confusing, using jeelabs value
  
  // FIFO threshold
  radio.writeReg(REG_FIFOTHRESH, RF_FIFOTHRESH_TXSTART_FIFONOTEMPTY | RF_FIFOTHRESH_VALUE);  //   
  
  // DAGC
  radio.writeReg(REG_TESTDAGC, RF_DAGC_IMPROVED_LOWBETA0);  // DAGC for fading improvement
    
  if (DEBUG) 
  {  
  while (!Serial); // wait until serial console is open, remove if not tethered to computer. Delete this line on ESP8266
    Serial.begin(SERIAL_BAUD);
    //Serial.println("Robot Garden RoboMagellan Safety Receiver");
    Serial.print("\nListening at ");
    Serial.print(radio.getFrequency());
    Serial.println(" MHz");
  }
}

void loop() {
  switch (hb_state){
    case HB_state::INVALID:
      if (is_new_msg()) {
        hb_state = HB_state::VALIDATING;
        msg_count = 1;  // this must be first msg
        ms_since_last_msg = 0;  // start timing
      }
      break;
    case HB_state::VALIDATING:
      if (is_new_msg()) {
        ms_since_last_msg = 0;  // reset timer
        msg_count += 1;  // one more step toward confirming valid signal
        if (msg_count >= msgs_to_validate) {  // enough confirming msgs
            hb_state = HB_state::VALID;  
        }
      }
      if (ms_since_last_msg > ms_max_msg_gap) {  // timer expired
        hb_state = HB_state::INVALID;  
        msg_count = 0;
      }
      break;
    case HB_state::VALID:
      if (is_new_msg()) {
        ms_since_last_msg = 0;  // reset timer
      }
      if (ms_since_last_msg > ms_max_msg_gap) {  // timer expired
        hb_state = HB_state::INVALID;  
        msg_count = 0;
      }
      break;
    default:
      hb_state = HB_state::INVALID;
      msg_count = 0;
      break;
  }
  
  switch (state){
    case State::IDLE:
      stop_servo.writeMicroseconds(SIM_THROTTLE_SERVO_NEUTRAL);  // need to discover what's right here for Xmaxx ESC
      select_servo.writeMicroseconds(SELECT_STOP);  // choose special stop channel (master)
      digitalWrite(RELAY_PIN, LOW);  // open the status relay
      if (hb_state == HB_state::VALID) {  // heartbeat is valid
        state = State::ENABLED;  
        if (DISPLAY_ON)
        {
          display.clearDisplay();
          display.display();
          display.setCursor(0,0);
          display.println("ENABLED");
          display.display();
        }
      }
      // following only for debug
      digitalWrite(BLU_LED_PIN, HIGH);   
      digitalWrite(GRN_LED_PIN, LOW);
      digitalWrite(RED_LED_PIN, LOW);
      //delay(500);              
      // end debug
      break;
    case State::ENABLED:
      // following only for debug
      digitalWrite(GRN_LED_PIN, HIGH);   
      digitalWrite(BLU_LED_PIN, LOW);
      digitalWrite(RED_LED_PIN, LOW);        
      // end debug
      digitalWrite(RELAY_PIN, HIGH);  // close the status relay
      select_servo.writeMicroseconds(SELECT_THROTTLE);  // choose Pixhawk channel (slave)
      sim_throttle_servo.writeMicroseconds(sim_throttle_val);
      //delay(100);  // only for demo
      
      if (hb_state != HB_state::VALID) {  // lost heartbeat
        state = State::STOPPING;
        if (DISPLAY_ON)
        {
          display.clearDisplay();
          display.display();
          display.setCursor(0,0);
          display.println("STOPPING");
          display.display();
        }
      }
      break;
    case State::STOPPING:
      // following only for debug
      digitalWrite(RED_LED_PIN, HIGH);   
      digitalWrite(GRN_LED_PIN, LOW);
      digitalWrite(BLU_LED_PIN, LOW);
      // end debug
      digitalWrite(RELAY_PIN, LOW);  // open the status relay
      select_servo.writeMicroseconds(SELECT_STOP);  // choose special stop channel (master)
      // TODO: magic numbers
      stop_servo.writeMicroseconds(STOP_SERVO_THROTTLE_SLOW_FWD);  // slow forward so we get brake rather than reverse next
      delay(100);  // verify what delay is needed to avoid reverse
      stop_servo.writeMicroseconds(STOP_SERVO_THROTTLE_BRAKE);  // brake
      delay(500);  // how long should this be?
      // following statement maybe should write neutral rather than zero?
      stop_servo.writeMicroseconds(STOP_SERVO_THROTTLE_NEUTRAL);  // need to discover what's right here for Xmaxx ESC
      
      delay(1000);
      state = State::IDLE;
      if (DISPLAY_ON)
      {
        display.clearDisplay();
        display.display();
        display.setCursor(0,0);
        display.println("IDLE");
        display.display();
      }
      break;
    default:
      state = State::IDLE;
      break;
  }
  
  if (DEBUG) {
    Serial.flush(); //make sure all serial data is clocked out before sleeping the MCU      
  }
}

bool is_new_msg() {;  // returns true if new valid message was rec'd
  //check if something was received (could be an interrupt from the radio)
  if (radio.receiveDone())
  {
    //print message received to serial
    if (DEBUG) {
      //Serial.print('[');Serial.print(radio.SENDERID);Serial.print("] ");
      //Serial.print((char*)radio.DATA);
      //Serial.print("   [RX_RSSI:");Serial.print(radio.RSSI);Serial.print("]");
    }
    //check if received message contains expected text
    if (strstr((char *)radio.DATA, "RoboMagellan Heartbeat"))
    {
      //check if sender wanted an ACK
      if (radio.ACKRequested())
      {
        radio.sendACK();
        if (DEBUG) 
        {
          //Serial.println(" - ACK sent");
        }
      }
      if (msg_rate_timer < msg_rate_period) 
      {
        msg_rate_count += 1;
      } else  // display and reset
      {  
        if (DEBUG)
        {
          Serial.print("Msg/s: "); Serial.println(msg_rate_count);  // assuming 1 sec count
        }
        if (DISPLAY_ON)  // for Feather OLED display
        {
          display.setCursor(0, 18);
          display.fillRect(0, 18, 128, 14, BLACK);  // erasing line :(
          display.display();
          display.setCursor(0, 18);
          display.print("Msg/s: ");
          display.println(msg_rate_count);  // assuming 1 sec count
          display.display();
          display.setCursor(0,0);
        }
        msg_rate_count = 0;  
        msg_rate_timer = 0;
      }     
      // Blink(LED_BUILTIN, 40, 1); //blink LED 3 times, 40ms between blinks
      return(true);  // valid message was received
    }  
  }
  radio.receiveDone(); //put radio in RX mode; from Adafruit, but I don't understand need for this call...
  return(false);  // a valid message was not received
}

void Blink(byte PIN, byte DELAY_MS, byte loops)
{
  elapsedMillis led_dwell = 0;  // time since last LED switch
  for (byte i=0; i<loops; i++)
  {
    digitalWrite(PIN,HIGH);
    while (led_dwell < DELAY_MS);  // wait
    led_dwell = 0;
    digitalWrite(PIN,LOW);
    while (led_dwell < DELAY_MS);  // wait
    led_dwell = 0;
  }
}

