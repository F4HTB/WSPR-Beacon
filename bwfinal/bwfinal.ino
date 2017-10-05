#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include "si5351.h"
#include "Wire.h"
#include <WSPRSI.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x3F, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);

extern Si5351 si5351;

SoftwareSerial ss(7, 6);
TinyGPSPlus gps;


#define ppsPin                   2
unsigned int tcount = 2;
unsigned long mult = 0;

int32_t measdif, measdif_old = 2147483647, calfact = 0, calfact_old;
uint64_t meas_freqC;
volatile boolean CorectFlag = false;
volatile boolean CorectFlagCycle = false;

volatile boolean sendTX = false;
volatile bool s_GpsOneSecTick = false;
volatile boolean LCDpoint = false;

char txCall[] = "F4HTB";
char txLoc[] = "JJ00";//"JJ00";
char txPow[] = "30";

unsigned long freq = 0;
byte sec = 0;
volatile bool synctime = false;
byte cycle = 0;


//******************************************************************
// LCD Function
//******************************************************************
void lcdmess(String mess, int line) {
  while (mess.length() < 16)mess = mess + " ";
  lcd.home ();
  lcd.setCursor(0, line);
  lcd.print(mess);
}

//******************************************************************
// Clock correction functions
//******************************************************************
void PPSinterrupt()
{
  if (!CorectFlagCycle)tcount++;
  if (tcount == 4)                               // Start counting the 2.5 MHz signal from Si5351A CLK0
  {
    TCCR1B = 7;                                  //Clock on rising edge of pin 5
  }
  else if (tcount == 44)                              //The 40 second gate time elapsed - stop counting
  {
    TCCR1B = 0;                                  //Turn off counter
    meas_freqC = mult * 0x10000 + TCNT1;
    CorectFlag = true;
    TCNT1 = 0;                                   //Reset count to zero
    mult = 0;
    tcount = 0;                                  //Reset the seconds counter
  }
  sec++;
  if (sec >= 120)sec = 0;
  s_GpsOneSecTick = true;
}

void correctFreqSI() {
  measdif = (int32_t)(meas_freqC - (uint64_t)(100000000)); // Error E calculation
  calfact = (measdif * 5) + calfact_old; // compute the new calfact
  si5351.set_correction(calfact);
  si5351.set_freq(250000000ULL, SI5351_CLK2);
  if (abs(measdif_old) <= abs(measdif)) {
    calfact = calfact_old;
    CorectFlagCycle = true;
    si5351.set_correction(calfact);
    si5351.set_freq(250000000ULL, SI5351_CLK2);
	measdif_old = 2147483647;
  }
  else {
    CorectFlagCycle = false;
    calfact_old = calfact;
  }
  measdif_old = measdif;
  lcdmess("CalFact " +  String(calfact) + " " + String(CorectFlagCycle), 1);
  CorectFlag = false;
}

// Timer 1 overflow intrrupt vector.
ISR(TIMER1_OVF_vect)
{
  mult++;                                          //Increment multiplier
  TIFR1 = (1 << TOV1);                             //Clear overlow flag
}

//******************************************************************
// Locator calculation function
//******************************************************************

void calcLocator(char *dst, double lat, double lon) {
  int o1, o2, o3;
  int a1, a2, a3;
  double remainder;
  // longitude
  remainder = lon + 180.0;
  o1 = (int)(remainder / 20.0);
  remainder = remainder - (double)o1 * 20.0;
  o2 = (int)(remainder / 2.0);
  remainder = remainder - 2.0 * (double)o2;
  o3 = (int)(12.0 * remainder);

  // latitude
  remainder = lat + 90.0;
  a1 = (int)(remainder / 10.0);
  remainder = remainder - (double)a1 * 10.0;
  a2 = (int)(remainder);
  remainder = remainder - (double)a2;
  a3 = (int)(24.0 * remainder);
  dst[0] = (char)o1 + 'A';
  dst[1] = (char)a1 + 'A';
  dst[2] = (char)o2 + '0';
  dst[3] = (char)a2 + '0';
  //dst[4] = (char)o3 + 'A';
  //dst[5] = (char)a3 + 'A';
  //dst[6] = (char)0;
}

//******************************************************************
// Function for wait time to send
//******************************************************************

void gpsgatetime ()
{
  if (CorectFlagCycle)cycle++;
  if (cycle == 4)cycle = 0;
  switch (cycle) {
    case 0:
      CorectFlagCycle = false;
      synctime = false;
      freq = 0;
      break;
    case 1:
      freq = 14095600L + (long)(random(145, 155) * 10);
      break;
    case 2:
      freq = 14095600L + (long)(random(145, 155) * 10);
      break;
    case 3:
      freq = 14095600L + (long)(random(145, 155) * 10);
      break;
    default:
      freq = 0;
      break;
  }
  if (freq != 0)TX();
}

//******************************************************************
// TX Function
//******************************************************************



void TX() {
  sendTX = true;
  lcdmess("TX " + String(freq) + "hz C" + String(cycle), 0);
  lcdmess(String((char*)txCall) + " " + String((char*)txLoc) + " " + String((char*)txPow) + "dBm", 1);
  while (!s_GpsOneSecTick) {
    delayMicroseconds(100);
  }
  Beacon.WSPRSITx(freq, txCall, txLoc, txPow);
  sendTX = false;
  lcdmess("Wait for TX", 0);
}

//******************************************************************
// Mains
//******************************************************************

void setup() {
  ////Serial init
  pinMode(A7, INPUT);
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);

  //Serial.begin(57600);
  ss.begin(9600);//9600

  //SI5351 init -17463
  si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0, 0);
  si5351.set_correction(0);
  si5351.set_pll(SI5351_PLL_FIXED, SI5351_PLLA);
  si5351.drive_strength(SI5351_CLK2, SI5351_DRIVE_8MA);
  si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_8MA);
  si5351.set_freq(250000000ULL, SI5351_CLK2);// 250000000ULL 259547697
  si5351.set_freq(1000000ULL, SI5351_CLK1);
  si5351.set_freq(1409700000ULL, SI5351_CLK0);
  si5351.output_enable(SI5351_CLK2, 1);
  si5351.output_enable(SI5351_CLK1, 0);
  si5351.output_enable(SI5351_CLK0, 1);
  si5351.update_status();
  delay(500);


  //PPS correction init
  //Set up Timer1 as a frequency counter - input at pin 5
  TCCR1B = 0;                                    //Disable Timer5 during setup
  TCCR1A = 0;                                    //Reset
  TCNT1  = 0;                                    //Reset counter to zero
  TIFR1  = 1;                                    //Reset overflow
  TIMSK1 = 1;                                    //Turn on overflow flag
  // Inititalize GPS 1pps input
  pinMode(ppsPin, INPUT);
  // Set 1PPS pin 2 for external interrupt input
  attachInterrupt(digitalPinToInterrupt(ppsPin), PPSinterrupt, RISING);

  lcd.begin (16, 2);
  lcd.setBacklight(HIGH);
  lcd.setCursor(0, 0);
  lcd.print("WSPR Beacon");
  lcd.setCursor(0, 1);
  lcd.print("F4HTB");
  delay(5000);
}

void LCDprintpoint() {
  lcd.setCursor(15, 1);
  if (LCDpoint) {
    lcd.print(".");
  }
  else {
    lcd.print(" ");
  }
  LCDpoint = ! LCDpoint;
}

void loop() {
  while (synctime) {
    while (!s_GpsOneSecTick) {
      delay(100);
    }
    if (!sendTX) {
      if (CorectFlag && !cycle)correctFreqSI();
      if (sec == 0)gpsgatetime();
      lcdmess("Sleep C" + String(cycle) + " " + String(sec) + " s" , 0);
      LCDprintpoint();
    }
    s_GpsOneSecTick = false;
  }
  while (!synctime) {
    while (!s_GpsOneSecTick) {
      delayMicroseconds(1000);
    }
    if (ss.available()) {
      if (gps.encode(ss.read())) {
        if (gps.location.isUpdated()) {
          if (strcmp(txLoc, "JJ00") == 0) {
            calcLocator(txLoc, gps.location.lat(), gps.location.lng());
            if (Beacon.WSPRSISymbGen == 0)Beacon.WSPRSIEncode(txCall, txLoc, txPow);
            s_GpsOneSecTick = false;
          } else if (gps.date.isValid() && gps.time.isValid() && (int(gps.time.second()) > 5)) {
            sec = int(gps.time.second()) + (int((gps.time.minute() % 2 )) * 60);
            s_GpsOneSecTick = false;
            while (!s_GpsOneSecTick) {
              delayMicroseconds(100);
            }
            synctime = true;
            ss.end();
            lcdmess("Time is sync.", 1);
          }
        } else if (s_GpsOneSecTick) {
          s_GpsOneSecTick = false;
          lcdmess("GPS Sync...", 0);
          LCDprintpoint();
        }
      }
    } else {
      if (!ss.isListening())ss.begin(9600);
      ss.flush();
    }
  }
}

