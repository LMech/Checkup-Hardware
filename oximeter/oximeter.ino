#include "MAX30102.h"
#include "Pulse.h"
#include "heartRate.h"
#include "Wire.h"
#include <avr/pgmspace.h>
#include <EEPROM.h>
#include <avr/sleep.h>
#include <SoftwareSerial.h>

// Routines to clear and set bits
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif


MAX30102 sensor;
Pulse pulseIR;
Pulse pulseRed;

SoftwareSerial bluetooth(9, 10);

#define LED LED_BUILTIN
#define BUTTON 3
#define OPTIONS 7

static const uint8_t heart_bits[] PROGMEM = { 0x00, 0x00, 0x38, 0x38, 0x7c, 0x7c, 0xfe, 0xfe, 0xfe, 0xff,
                                              0xfe, 0xff, 0xfc, 0x7f, 0xf8, 0x3f, 0xf0, 0x1f, 0xe0, 0x0f,
                                              0xc0, 0x07, 0x80, 0x03, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00,
                                              0x00, 0x00
                                            };

//spo2_table is approximated as  -45.060*ratioAverage* ratioAverage + 30.354 *ratioAverage + 94.845 ;
const uint8_t spo2_table[184] PROGMEM =
{ 95, 95, 95, 96, 96, 96, 97, 97, 97, 97, 97, 98, 98, 98, 98, 98, 99, 99, 99, 99,
  99, 99, 99, 99, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100,
  100, 100, 100, 100, 99, 99, 99, 99, 99, 99, 99, 99, 98, 98, 98, 98, 98, 98, 97, 97,
  97, 97, 96, 96, 96, 96, 95, 95, 95, 94, 94, 94, 93, 93, 93, 92, 92, 92, 91, 91,
  90, 90, 89, 89, 89, 88, 88, 87, 87, 86, 86, 85, 85, 84, 84, 83, 82, 82, 81, 81,
  80, 80, 79, 78, 78, 77, 76, 76, 75, 74, 74, 73, 72, 72, 71, 70, 69, 69, 68, 67,
  66, 66, 65, 64, 63, 62, 62, 61, 60, 59, 58, 57, 56, 56, 55, 54, 53, 52, 51, 50,
  49, 48, 47, 46, 45, 44, 43, 42, 41, 40, 39, 38, 37, 36, 35, 34, 33, 31, 30, 29,
  28, 27, 26, 25, 23, 22, 21, 20, 19, 17, 16, 15, 14, 12, 11, 10, 9, 7, 6, 5,
  3, 2, 1
} ;


int getVCC() {
  //reads internal 1V1 reference against VCC
#if defined(__AVR_ATmega1284P__)
  ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);  // For ATmega1284
#else
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);  // For ATmega328
#endif
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA, ADSC));
  uint8_t low = ADCL;
  unsigned int val = (ADCH << 8) | low;
  //discard previous result
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA, ADSC));
  low = ADCL;
  val = (ADCH << 8) | low;

  return (((long)1024 * 1100) / val) / 100;
}


int  beatAvg;
int  SPO2, SPO2f;
int  voltage;  
void setup(void) {
  Serial.begin(9600);
  bluetooth.begin(9600);
  pinMode(LED, OUTPUT);
  pinMode(BUTTON, INPUT_PULLUP);
  Wire.begin();
  Wire.setClock(400000);
  delay(300);
  if (!sensor.begin())  {
    Serial.println("Sensor not connected");
    while (1);
  }
  sensor.setup();
}

long lastBeat = 0;    //Time of the last beat
long previous = 0;
bool placed = 0;
const byte RATE_SIZE = 4; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;



void loop()  {
  sensor.check();
  long now = millis();   //start time of this cycle

  // Check if sensor connected
  if (!sensor.available()) return;
  uint32_t irValue = sensor.getIR();
  uint32_t redValue = sensor.getRed();
  sensor.nextSample();

  // Check if placed
  if (irValue < 5000) {
    placed = 0;
    voltage = getVCC();
    delay(100);
  } else {
    placed = 1;
    
    // remove DC element
    int16_t IR_signal, Red_signal;
    bool beatRed, beatIR;

    IR_signal =  pulseIR.ma_filter(pulseIR.dc_filter(irValue)) ;
    Red_signal = pulseRed.ma_filter(pulseRed.dc_filter(redValue));
    beatRed = pulseRed.isBeat(Red_signal);
    beatIR =  pulseIR.isBeat(IR_signal);

    if (checkForBeat(irValue) == true) {
      long btpm = 60000 / (millis() - lastBeat);
      lastBeat = millis();
      if (btpm > 20 && btpm < 255)
      {
        rates[rateSpot++] = (byte)btpm; //Store this reading in the array
        rateSpot %= RATE_SIZE; //Wrap variable
        beatAvg = 0;

        //Take average of readings
        for (byte x = 0 ; x < RATE_SIZE ; x++)
          beatAvg += rates[x];
        beatAvg /= RATE_SIZE;
      }
      digitalWrite(LED, HIGH);
      
      // compute SpO2 ratio
      long numerator   = (pulseRed.avgAC() * pulseIR.avgDC()) / 256;
      long denominator = (pulseRed.avgDC() * pulseIR.avgAC()) / 256;
      int RX100 = (denominator > 0) ? (numerator * 100) / denominator : 999;
      // using formula
      SPO2f = (10400 - RX100 * 17 + 50) / 100;
      
      // from table
      if ((RX100 >= 0) && (RX100 < 184))
        SPO2 = pgm_read_byte_near(&spo2_table[RX100]);
    }
  }
  if ((now - previous > 6000) && placed) {
    previous = now;
//    Serial.print(beatAvg);
//    Serial.print("  ");
//    Serial.println(SPO2);
    String cBeatAvg = (String)(char)beatAvg;
    String cSPO2 = (String)(char)SPO2;
    String s = cBeatAvg + cSPO2;
//    Serial.println(s);
    Serial.println(bluetooth.available());
    bluetooth.write(-1);
    bluetooth.print(s);
  }
}
