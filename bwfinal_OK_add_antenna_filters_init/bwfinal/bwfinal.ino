#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include "si5351.h"
#include "Wire.h"
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>

LiquidCrystal_I2C lcd(0x3F, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);

Si5351 si5351;

SoftwareSerial ss(7, 6);
TinyGPSPlus gps;

String inputStringUSBSerial = "";
boolean stringCompleteUSBSerial = false;

#define ppsPin  2
#define PAPin 3

// Broches d'adresse filtres antennes
const byte PIN_ADDR_A = 8;
const byte PIN_ADDR_B = 9;
const byte PIN_ADDR_C = 10;
const byte PIN_ADDR_D = 11;
// Broche de signal filtres antennes
const byte PIN_SIG = 12;
#define resetantenna  6

unsigned int tcount = 2;
unsigned long mult = 0;

int32_t measdif = 0, measdif_old = 2147483647, calfact = 0, calfact_old = 0;
uint64_t meas_freqC = 0;
volatile boolean CorectFlag = false;
volatile boolean CorectFlagCycle = false;

volatile boolean sendTX = false;
volatile bool s_GpsOneSecTick = false;
volatile boolean LCDpoint = false;

char txCall[] = "F4HTB";
char txLoc[] = "JJ00";//"JJ00";
char txPow[] = "30";

byte sec = 0;
volatile bool synctime = false;
byte cycle = 0;

byte WSPRSISymb[162];
bool WSPRSISymbGen = 0;

unsigned long  Frequencys[20]   = {};//{0,14095600L, 14095600L, 0, 14095600L, 14095600L};//Frequency by cycle
byte   FtoAnt[20]       = {};//{1,1, 2, 1, 2, 1}; //antenna by cycle
#define ncycles 19 //0 to n ° cycles

//******************************************************************
// Others functions
//******************************************************************
void waitticks() {
  while (!s_GpsOneSecTick) {
    delayMicroseconds(100);
  }
}

/*****************************************************************
  write for filters and antenna
*****************************************************************/
void writeAnalogMux(byte channel) {

  // On disable la valeur courante
  digitalWrite(PIN_SIG, LOW);
  delayMicroseconds(100);

  // On selectionne la voie
  digitalWrite(PIN_ADDR_A, bitRead(channel, 0));
  digitalWrite(PIN_ADDR_B, bitRead(channel, 1));
  digitalWrite(PIN_ADDR_C, bitRead(channel, 2));
  digitalWrite(PIN_ADDR_D, bitRead(channel, 3));
  delayMicroseconds(100);

  // On ecrit la valeur courante
  digitalWrite(PIN_SIG, HIGH);
  delay(10);
}

void selectfilter(long freqf = 0, int filter = 1) {
  if (freqf != 0) {
    if (freqf > 30000000) {
      filter = 1;
    }
    else if (freqf < 2000000) {
      filter = 2;
    }
    else if (freqf < 4000000) {
      filter = 3;
    }
    else if (freqf < 10500000) {
      filter = 4;
    }
    else if (freqf < 21000000) {
      filter = 5;
    }
    else if (freqf < 30000000) {
      filter = 6;
    }
  }
  writeAnalogMux((filter - 1));
}

void selectantenna(int antenna) {
  antenna += 6;
  writeAnalogMux(resetantenna);
  delayMicroseconds(100);
  writeAnalogMux(antenna);
}

//******************************************************************
// LCD Function
//******************************************************************
void lcdmess(String mess, int line) {
  while (mess.length() < 16)mess = mess + " ";
  lcd.home ();
  lcd.setCursor(0, line);
  lcd.print(mess);
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
  if ((abs(measdif_old) <= abs(measdif))) {
    calfact = calfact_old;
    if (calfact != 0)CorectFlagCycle = true;
    measdif_old = 2147483647;
  }
  else {
    CorectFlagCycle = false;
    calfact_old = calfact;
  }
  si5351.set_correction(calfact, SI5351_PLL_INPUT_XO);
  si5351.set_freq(250000000ULL, SI5351_CLK2);
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
}

//******************************************************************
// Function for wait time to send
//******************************************************************

void gpsgatetime ()
{
  if (CorectFlagCycle) {
    cycle++;
    while ((Frequencys[cycle] == -1) && (cycle != ncycles))cycle++;
    if (cycle == ncycles)cycle = 0;
    if (Frequencys[cycle] == 0) {
      selectfilter();
      CorectFlagCycle = false;
      synctime = false;
    }
  }
  if (Frequencys[cycle] != 0)TX();
}

//******************************************************************
// TX Function
//******************************************************************

void TX() {
  sendTX = true;
  selectantenna(FtoAnt[cycle]);
  selectfilter(Frequencys[cycle]);
  digitalWrite(PAPin, HIGH);
  long freqWSPRSI = (Frequencys[cycle] + (long)(random(145, 155) * 10)) * 100;
  lcdmess(String(freqWSPRSI) + " C" + String(cycle) + " A" + String(FtoAnt[cycle]), 0);
  dtxinfo();
  waitticks();
  si5351.output_enable(SI5351_CLK0, 1);
  for (int element = 0; element < 162; element++) {
    si5351.set_freq((freqWSPRSI + (WSPRSISymb[element] * 14648) / 100), SI5351_CLK0);
    delay(680); delayMicroseconds(2666);
  }
  si5351.output_enable(SI5351_CLK0, 0);
  digitalWrite(PAPin, LOW);
  sendTX = false;
}

//******************************************************************
// sequence de programmation operateur
//******************************************************************

void initcyclesepprom() {
  for (byte i = 0; i <= ncycles; i++) {
    Frequencys[i] = EEPROMReadlong(i * 4);
  }
  for (byte i = 0; i <= ncycles; i++) {
    FtoAnt[i] = EEPROMReadlong((i * 4) + 100);
    delay(1);//bug epprom
  }
  for (byte i = 0; i <= 6; i++) {
    txCall[i] = char(int(EEPROMReadlong(((i * 4) + 400))));
    if (txCall[i] == '\0')break;
    delay(1);//bug epprom
  }
}

void savcyclesepprom() {
  for (byte i = 0; i <= ncycles; i++) {
    EEPROMWritelong(i * 4, Frequencys[i]);
  }
  for (byte i = 0; i <= ncycles; i++) {
    EEPROMWritelong(((i * 4) + 100), FtoAnt[i]);
  }
  for (byte i = 0; i <= sizeof(txCall); i++) {
    EEPROMWritelong(((i * 4) + 400), int(txCall[i]));
  }
}

void showprog() {
  for (int i = 0; i <= ncycles; i++) {
    if (Frequencys[i] != -1 && FtoAnt[i] != -1) {
      Serial.println("C" + String(i) + " freq:" + String(Frequencys[i]) + "hz");
      Serial.println("C" + String(i) + " ant:" + String(FtoAnt[i]));
    }
  }
  Serial.println("QRZ " + String(txCall));
}

void initprog() {
  lcdmess(F("Init..."), 0);

  Serial.println(F("s show infos"));
  Serial.println(F("d 01 del cycle 1"));
  Serial.println(F("a cycle/freq/ant add,max 20, (a 01/014095600/3)"));
  Serial.println(F("q set qrz"));
  Serial.println(F("r save"));

  while (inputStringUSBSerial[0] != 'e') {
    while (Serial.available()) {
      char inChar = (char)Serial.read();
      if (inChar != '\n') {
        inputStringUSBSerial += inChar;
      } else {
        stringCompleteUSBSerial = true;
      }
    }
    if (stringCompleteUSBSerial) {
      if (inputStringUSBSerial[0] == 'd') {
        Frequencys[inputStringUSBSerial.substring(2, 4).toInt()] = -1;
        FtoAnt[inputStringUSBSerial.substring(2, 4).toInt()] = -1;
      }
      if (inputStringUSBSerial[0] == 'a') {
        char tarray[10];
        inputStringUSBSerial.substring(5, 14).toCharArray(tarray, sizeof(tarray));
        Frequencys[inputStringUSBSerial.substring(2, 4).toInt()] = atol(tarray);
        FtoAnt[inputStringUSBSerial.substring(2, 4).toInt()] = inputStringUSBSerial.substring(15, 16).toInt();
      }
      if (inputStringUSBSerial[0] == 'r') {
        savcyclesepprom();
        initcyclesepprom();
        showprog();
        Serial.println(F("You can reset"));
      }
      if (inputStringUSBSerial[0] == 's') {
        showprog();
      }
      if (inputStringUSBSerial[0] == 'q') {
        inputStringUSBSerial.substring(2, 7).toCharArray(txCall, (inputStringUSBSerial.length() - 1)); 
      }
      inputStringUSBSerial = "";
      stringCompleteUSBSerial = false;
    }
  }
}


void EEPROMWritelong(int address, long value)
{
  //Decomposition from a long to 4 bytes by using bitshift.
  //One = Most significant -> Four = Least significant byte
  byte four = (value & 0xFF);
  byte three = ((value >> 8) & 0xFF);
  byte two = ((value >> 16) & 0xFF);
  byte one = ((value >> 24) & 0xFF);

  //Write the 4 bytes into the eeprom memory.
  EEPROM.write(address, four);
  EEPROM.write(address + 1, three);
  EEPROM.write(address + 2, two);
  EEPROM.write(address + 3, one);
}

long EEPROMReadlong(long address)
{
  //Read the 4 bytes from the eeprom memory.
  long four = EEPROM.read(address);
  long three = EEPROM.read(address + 1);
  long two = EEPROM.read(address + 2);
  long one = EEPROM.read(address + 3);

  //Return the recomposed long by using bitshift.
  return ((four << 0) & 0xFF) + ((three << 8) & 0xFFFF) + ((two << 16) & 0xFFFFFF) + ((one << 24) & 0xFFFFFFFF);
}


//******************************************************************
// Mains
//******************************************************************

void setup() {
  Serial.begin(9600);

  ////Serial init
  pinMode(A7, INPUT);
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(PAPin, OUTPUT);
  digitalWrite(PAPin, LOW);
  ss.begin(9600);

  //SI5351 init -17463
  si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0, 0);
  si5351.set_correction(0, SI5351_PLL_INPUT_XO);
  si5351.set_pll(SI5351_PLL_FIXED, SI5351_PLLA);
  si5351.drive_strength(SI5351_CLK2, SI5351_DRIVE_8MA);
  si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_8MA);
  si5351.set_freq(250000000ULL, SI5351_CLK2);// 250000000ULL 259547697
  si5351.set_freq(1000000ULL, SI5351_CLK1);
  si5351.set_freq(1000000ULL, SI5351_CLK0);
  si5351.output_enable(SI5351_CLK2, 1);
  si5351.output_enable(SI5351_CLK1, 0);
  si5351.output_enable(SI5351_CLK0, 0);
  si5351.update_status();
  delay(500);


  //PPS correction init
  //Set up Timer1 as a frequency counter - input at pin 5
  TCCR1B = 0;                                    //Disable Timer5 during setup
  TCCR1A = 0;                                    //Reset
  TCNT1  = 0;                                    //Reset counter to zero
  TIFR1  = 1;                                    //Reset overflow
  TIMSK1 = 1;                                    //Turn on overflow flag
  pinMode(ppsPin, INPUT);// Inititalize GPS 1pps input
  attachInterrupt(digitalPinToInterrupt(ppsPin), PPSinterrupt, RISING);// Set 1PPS pin 2 for external interrupt input

  // Place les broches d'adresse en sortie et à LOW pour filtres et antennes
  pinMode(PIN_SIG, OUTPUT);
  pinMode(PIN_ADDR_A, OUTPUT);
  pinMode(PIN_ADDR_B, OUTPUT);
  pinMode(PIN_ADDR_C, OUTPUT);
  pinMode(PIN_ADDR_D, OUTPUT);
  digitalWrite(PIN_SIG, LOW);
  digitalWrite(PIN_ADDR_A, LOW);
  digitalWrite(PIN_ADDR_B, LOW);
  digitalWrite(PIN_ADDR_C, LOW);
  digitalWrite(PIN_ADDR_D, LOW);

  lcd.begin (16, 2);
  lcd.setBacklight(HIGH);
  lcdmess(F("WSPR Beacon"), 0);
  lcdmess(F("Tests"), 1);
  delay(2000);

  //test antennas last on 1
  for (int i = 6; i >= 1; i--) {
    selectantenna(i);
    lcdmess("Ant " + String(i), 1);
    delay(200);
  }

  //test filters last on bypass
  for (int i = 6; i >= 1; i--) {
    selectfilter(0, i);
    lcdmess("Fil " + String(i), 1);
    delay(200);
  }

  lcdmess(txCall, 1);
  Serial.println(F("Send key go init"));
  initcyclesepprom();
  delay(5000);
  if (Serial.available()) {
    initprog();
  }
}



void loop() {
  while (synctime) {
    waitticks();
    if (!sendTX) {
      if (CorectFlag && (Frequencys[cycle] == 0))correctFreqSI();
      if (sec == 0)gpsgatetime();
      lcdmess("Sleep C" + String(cycle) + " " + String(sec) + " s" , 0);
      LCDprintpoint();
    }
    s_GpsOneSecTick = false;
  }
  while (!synctime) {
    synctimegps();
  }
  dtxinfo();
}


void synctimegps() {
  waitticks();
  waitticks();
  if (ss.available()) {
    if (gps.encode(ss.read())) {
      if (gps.location.isUpdated()) {
        if (strcmp(txLoc, "JJ00") == 0) {
          calcLocator(txLoc, gps.location.lat(), gps.location.lng());
          if (WSPRSISymbGen == 0)WSPRSIEncode(txCall, txLoc, txPow);
          s_GpsOneSecTick = false;
        } else if (gps.date.isValid() && gps.time.isValid() && (int(gps.time.second()) > 2)) {
          sec = int(gps.time.second()) + (int((gps.time.minute() % 2 )) * 60);
          s_GpsOneSecTick = false;
          waitticks();
          synctime = true;
          ss.end();
        }
      } else if (s_GpsOneSecTick) {
        s_GpsOneSecTick = false;
        lcdmess(F("GPS Sync."), 0);
        LCDprintpoint();
      }
    }
  } else {
    if (!ss.isListening())ss.begin(9600);
    ss.flush();
  }
}

void dtxinfo(){
  lcdmess(String((char*)txCall) + " " + String((char*)txLoc) + " " + String((char*)txPow) + "d", 1);
}

/*************************************************************************
  WSPRSI message encoder
  http://www.g4jnt.com/Coding%5CWSPRSI_Coding_Process.pdf
**************************************************************************/
void WSPRSIEncode(char * callWSPRSIProc, char * locWSPRSIProc, char * powWSPRSIProc)
{
  char callsign[] = "      ";
  char encode_call[] = "       ";
  char locator[] = "    ";
  char power[] = "00";
  byte pwr_lvl;
  char ch;
  int counter = 0;

  ch = *callWSPRSIProc++;
  while (ch != '\0')
  {
    callsign[counter] = ch;
    ch = *callWSPRSIProc++;
    counter++;
  }
  counter = 0;
  ch = *locWSPRSIProc++;
  while (ch != '\0')
  {
    locator[counter] = ch;
    ch = *locWSPRSIProc++;
    counter++;
  }

  counter = 0;
  ch = *powWSPRSIProc++;
  while (ch != '\0')
  {
    power[counter] = ch;
    ch = *powWSPRSIProc++;
    counter++;
  }

  for (int i = 0; i < 6; i++) {
    if (callsign[i] == ' ')
    {
      callsign[i] = callsign[i] + 4;
    }
    else if (callsign[i] > 47 && callsign[i] < 58)
    {
      callsign[i] = callsign[i] - 48;
    }
    else if (callsign[i] > 64 && callsign[i] < 91)
    {
      callsign[i] = callsign[i] - 55;
    }
    else if (callsign[i] > 96 && callsign[i] < 123)
    {
      callsign[i] = callsign[i] - 87;
    }
  }

  if (callsign[2] > 9) {
    encode_call[0] = 36;
    for (int i = 0; i < 5; i++)
    {
      encode_call[i + 1] = callsign [i];
    }
  }

  /*****************************************************************
    Callsign encoding
  *****************************************************************/
  unsigned long N;

  N = encode_call[0];
  N = 36 * N + encode_call[1];
  N = 10 * N + encode_call[2];
  N = 27 * N + encode_call[3] - 10;
  N = 27 * N + encode_call[4] - 10;
  N = 27 * N + encode_call[5] - 10;

  /*****************************************************************
    Locator encoding
  *****************************************************************/
  unsigned long M1;

  for (int i = 0; i < 4; i++) {
    if (locator[i] > 64 && locator[i] < 83) {
      locator[i] = locator[i] - 65;
    }
    else if (locator[i] > 96 && locator[i] < 123)
    {
      locator[i] = locator[i] - 97;
    }
    else if (locator[i] > 47 && locator[i] < 58) {
      locator[i] = locator[i] - 48;
    }
  }

  M1 = 180 * (179 - 10 * locator[0] - locator[2]) + 10 * locator[1] + locator[3];

  /*****************************************************************
    Power encoding
  *****************************************************************/
  pwr_lvl = 10 * (power[0] - '0') + power[1] - '0';

  /*****************************************************************
    Locator + power packing
  *****************************************************************/
  unsigned long M;
  M = 128 * M1 + pwr_lvl + 64;

  /*****************************************************************
    Bit packing
  *****************************************************************/
  unsigned char c[11];

  c[0] = N >> 20; // 8 MSB
  c[1] = N >> 12; // 8 next
  c[2] = N >> 4;  // 8 next
  c[3] = N << 4;  // 4 LSB -> 4 MSB c[3]
  c[3] = c[3] + (0x0f & M >> 18);
  c[4] = M >> 10;
  c[5] = M >> 2;
  c[6] = M << 6;
  c[7] = 0;
  c[8] = 0;
  c[9] = 0;
  c[10] = 0;

  /*****************************************************************
    Convolutional encoding
  *****************************************************************/
  unsigned long reg = 0;
  int conv;
  int conv_byte = 0;
  int compt_bits = 0;
  byte S[162];

  conv = c[0];
  int m = 0;

  for (int i = 0; i < 81; i++) {
    if (compt_bits % 8 == 0) {
      conv = c[conv_byte];
      conv_byte++;
    }
    reg = reg | bitRead(conv, 7);

    S[m++] = WSPRSIparity(reg & 0xf2d05351);
    S[m++] = WSPRSIparity(reg & 0xe4613c47);

    compt_bits++;
    conv = conv << 1;
    reg = reg << 1;
  }


  /*****************************************************************
    Interleaving
  *****************************************************************/

  byte D[162];
  int p = 0;
  byte temp = 0;
  int j = 0;

  for (int i = 0; i < 256; i++)
  {
    j = i;
    temp = j;
    for (int k = 0; k < 8; k++) {
      bitWrite(j, k , bitRead(temp, 7 - k));
    }
    if (j < 162) {
      D[j] = S[p];
      p++;
    }
  }

  /*****************************************************************
    Merge with sync vector
  *****************************************************************/
  const static byte Sync[] PROGMEM = {1, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 1, 1, 0, 0, 0, 1, 0, 0, 1, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 1, 1,
                                      0, 0, 1, 1, 0, 1, 0, 0, 0, 1, 1, 0, 1, 0, 0, 0, 0, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 1, 0, 0, 1, 0, 1, 1, 0, 0, 0, 1, 1, 0, 1, 0, 1, 0, 0, 0, 1, 0,
                                      0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 1, 1, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0,
                                      1, 0, 1, 1, 0, 0, 0, 1, 1, 0, 0, 0
                                     };

  for (int i = 0; i < 162; i++) {
    WSPRSISymb[i] = pgm_read_word(&Sync[i]) + 2 * D[i];
  }
  WSPRSISymbGen = 1;
}

/*****************************************************************
  Parity calculator
*****************************************************************/
byte WSPRSIparity(unsigned long tempo) {
  byte par = 0;
  for (int k = 0; k < 32; k++) {
    par = par ^ (tempo & 0x01);
    tempo = tempo >> 1;
  }
  return par;
}



