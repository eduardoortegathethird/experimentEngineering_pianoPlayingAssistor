// A fun sketch to demonstrate the use of the tone() function written by Brett Hagman.

// This plays RTTTL (RingTone Text Transfer Language) songs using the
// now built-in tone() command in Wiring and Arduino.
// Written by Brett Hagman
// http://www.roguerobotics.com/

// To play the output on a small speaker (i.e. 8 Ohms or higher), simply use
// a 1K Ohm resistor from the output pin to the speaker, and connect the other
// side of the speaker to ground.

// You can get more RTTTL songs from
// http://code.google.com/p/rogue-code/wiki/ToneLibraryDocumentation

//const int tonePin = 27;  // for rEDI board

#include "I2Cdev.h"
#include "MPU6050.h"
#include "arduinoFFT.h"
#include <Adafruit_NeoPixel.h>

#ifdef __AVR__
  #include <avr/power.h>
#endif

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  #include "Wire.h"
#endif

MPU6050 accelgyro;

int16_t ax, ay, az;



const int tonePin = 13;  // arbitrary for arduino boards, set this to whatever you want

#define OCTAVE_OFFSET 0
#define SONGLENGTH    36
#define PIN 6
#define NUMPIXELS 300
#define OUTPUT_READABLE_ACCELGYRO

#define SAMPLES 64              //Must be a power of 2
#define SAMPLING_FREQUENCY 1000 //Hz, must be less than 10000 due to ADC
arduinoFFT FFT = arduinoFFT();

unsigned int sampling_period_us;
unsigned long microseconds;

double vReal[SAMPLES];
double vImag[SAMPLES];

#define LED_PIN 12
bool blinkState = false;

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_RGBW + NEO_KHZ800);
int DelayVal = 500;



// These values can also be found as constants in the Tone library (Tone.h)
int notes[] = { 0,
262, 277, 294, 311, 330, 349, 370, 392, 415, 440, 466, 494,
523, 554, 587, 622, 659, 698, 740, 784, 831, 880, 932, 988,
1047, 1109, 1175, 1245, 1319, 1397, 1480, 1568, 1661, 1760, 1865, 1976,
2093, 2217, 2349, 2489, 2637, 2794, 2960, 3136, 3322, 3520, 3729, 3951
};
int playNote[SONGLENGTH];
int i=0;


//char *song = "The Simpsons:d=4,o=5,b=160:c.6,e6,f#6,8a6,g.6,e6,c6,8a,8f#,8f#,8f#,2g,8p,8p,8f#,8f#,8f#,8g,a#.,8c6,8c6,8c6,c6";
//char *song = "Indiana:d=4,o=5,b=250:e,8p,8f,8g,8p,1c6,8p.,d,8p,8e,1f,p.,g,8p,8a,8b,8p,1f6,p,a,8p,8b,2c6,2d6,2e6,e,8p,8f,8g,8p,1c6,p,d6,8p,8e6,1f.6,g,8p,8g,e.6,8p,d6,8p,8g,e.6,8p,d6,8p,8g,f.6,8p,e6,8p,8d6,2c6";
//char *song = "TakeOnMe:d=4,o=4,b=160:8f#5,8f#5,8f#5,8d5,8p,8b,8p,8e5,8p,8e5,8p,8e5,8g#5,8g#5,8a5,8b5,8a5,8a5,8a5,8e5,8p,8d5,8p,8f#5,8p,8f#5,8p,8f#5,8e5,8e5,8f#5,8e5,8f#5,8f#5,8f#5,8d5,8p,8b,8p,8e5,8p,8e5,8p,8e5,8g#5,8g#5,8a5,8b5,8a5,8a5,8a5,8e5,8p,8d5,8p,8f#5,8p,8f#5,8p,8f#5,8e5,8e5";
//char *song = "Entertainer:d=4,o=5,b=140:8d,8d#,8e,c6,8e,c6,8e,2c.6,8c6,8d6,8d#6,8e6,8c6,8d6,e6,8b,d6,2c6,p,8d,8d#,8e,c6,8e,c6,8e,2c.6,8p,8a,8g,8f#,8a,8c6,e6,8d6,8c6,8a,2d6";
//char *song = "Muppets:d=4,o=5,b=250:c6,c6,a,b,8a,b,g,p,c6,c6,a,8b,8a,8p,g.,p,e,e,g,f,8e,f,8c6,8c,8d,e,8e,8e,8p,8e,g,2p,c6,c6,a,b,8a,b,g,p,c6,c6,a,8b,a,g.,p,e,e,g,f,8e,f,8c6,8c,8d,e,8e,d,8d,c";
//char *song = "Xfiles:d=4,o=5,b=125:e,b,a,b,d6,2b.,1p,e,b,a,b,e6,2b.,1p,g6,f#6,e6,d6,e6,2b.,1p,g6,f#6,e6,d6,f#6,2b.,1p,e,b,a,b,d6,2b.,1p,e,b,a,b,e6,2b.,1p,e6,2b.";
//char *song = "Looney:d=4,o=5,b=140:32p,c6,8f6,8e6,8d6,8c6,a.,8c6,8f6,8e6,8d6,8d#6,e.6,8e6,8e6,8c6,8d6,8c6,8e6,8c6,8d6,8a,8c6,8g,8a#,8a,8f";
//char *song = "20thCenFox:d=16,o=5,b=140:b,8p,b,b,2b,p,c6,32p,b,32p,c6,32p,b,32p,c6,32p,b,8p,b,b,b,32p,b,32p,b,32p,b,32p,b,32p,b,32p,b,32p,g#,32p,a,32p,b,8p,b,b,2b,4p,8e,8g#,8b,1c#6,8f#,8a,8c#6,1e6,8a,8c#6,8e6,1e6,8b,8g#,8a,2b";
//char *song = "Bond:d=4,o=5,b=80:32p,16c#6,32d#6,32d#6,16d#6,8d#6,16c#6,16c#6,16c#6,16c#6,32e6,32e6,16e6,8e6,16d#6,16d#6,16d#6,16c#6,32d#6,32d#6,16d#6,8d#6,16c#6,16c#6,16c#6,16c#6,32e6,32e6,16e6,8e6,16d#6,16d6,16c#6,16c#7,c.7,16g#6,16f#6,g#.6";
//char *song = "MASH:d=8,o=5,b=140:4a,4g,f#,g,p,f#,p,g,p,f#,p,2e.,p,f#,e,4f#,e,f#,p,e,p,4d.,p,f#,4e,d,e,p,d,p,e,p,d,p,2c#.,p,d,c#,4d,c#,d,p,e,p,4f#,p,a,p,4b,a,b,p,a,p,b,p,2a.,4p,a,b,a,4b,a,b,p,2a.,a,4f#,a,b,p,d6,p,4e.6,d6,b,p,a,p,2b";
//char *song = "StarWars:d=4,o=5,b=45:32p,32f#,32f#,32f#,8b.,8f#.6,32e6,32d#6,32c#6,8b.6,16f#.6,32e6,32d#6,32c#6,8b.6,16f#.6,32e6,32d#6,32e6,8c#.6,32f#,32f#,32f#,8b.,8f#.6,32e6,32d#6,32c#6,8b.6,16f#.6,32e6,32d#6,32c#6,8b.6,16f#.6,32e6,32d#6,32e6,8c#6";
//char *song = "GoodBad:d=4,o=5,b=56:32p,32a#,32d#6,32a#,32d#6,8a#.,16f#.,16g#.,d#,32a#,32d#6,32a#,32d#6,8a#.,16f#.,16g#.,c#6,32a#,32d#6,32a#,32d#6,8a#.,16f#.,32f.,32d#.,c#,32a#,32d#6,32a#,32d#6,8a#.,16g#.,d#";
//char *song = "TopGun:d=4,o=4,b=31:32p,16c#,16g#,16g#,32f#,32f,32f#,32f,16d#,16d#,32c#,32d#,16f,32d#,32f,16f#,32f,32c#,16f,d#,16c#,16g#,16g#,32f#,32f,32f#,32f,16d#,16d#,32c#,32d#,16f,32d#,32f,16f#,32f,32c#,g#";
//char *song = "A-Team:d=8,o=5,b=125:4d#6,a#,2d#6,16p,g#,4a#,4d#.,p,16g,16a#,d#6,a#,f6,2d#6,16p,c#.6,16c6,16a#,g#.,2a#";
//char *song = "Flinstones:d=4,o=5,b=40:32p,16f6,16a#,16a#6,32g6,16f6,16a#.,16f6,32d#6,32d6,32d6,32d#6,32f6,16a#,16c6,d6,16f6,16a#.,16a#6,32g6,16f6,16a#.,32f6,32f6,32d#6,32d6,32d6,32d#6,32f6,16a#,16c6,a#,16a6,16d.6,16a#6,32a6,32a6,32g6,32f#6,32a6,8g6,16g6,16c.6,32a6,32a6,32g6,32g6,32f6,32e6,32g6,8f6,16f6,16a#.,16a#6,32g6,16f6,16a#.,16f6,32d#6,32d6,32d6,32d#6,32f6,16a#,16c.6,32d6,32d#6,32f6,16a#,16c.6,32d6,32d#6,32f6,16a#6,16c7,8a#.6";
//char *song = "Jeopardy:d=4,o=6,b=125:c,f,c,f5,c,f,2c,c,f,c,f,a.,8g,8f,8e,8d,8c#,c,f,c,f5,c,f,2c,f.,8d,c,a#5,a5,g5,f5,p,d#,g#,d#,g#5,d#,g#,2d#,d#,g#,d#,g#,c.7,8a#,8g#,8g,8f,8e,d#,g#,d#,g#5,d#,g#,2d#,g#.,8f,d#,c#,c,p,a#5,p,g#.5,d#,g#";
//char *song = "Gadget:d=16,o=5,b=50:32d#,32f,32f#,32g#,a#,f#,a,f,g#,f#,32d#,32f,32f#,32g#,a#,d#6,4d6,32d#,32f,32f#,32g#,a#,f#,a,f,g#,f#,8d#";
//char *song = "Smurfs:d=32,o=5,b=200:4c#6,16p,4f#6,p,16c#6,p,8d#6,p,8b,p,4g#,16p,4c#6,p,16a#,p,8f#,p,8a#,p,4g#,4p,g#,p,a#,p,b,p,c6,p,4c#6,16p,4f#6,p,16c#6,p,8d#6,p,8b,p,4g#,16p,4c#6,p,16a#,p,8b,p,8f,p,4f#";
//char *song = "MahnaMahna:d=16,o=6,b=125:c#,c.,b5,8a#.5,8f.,4g#,a#,g.,4d#,8p,c#,c.,b5,8a#.5,8f.,g#.,8a#.,4g,8p,c#,c.,b5,8a#.5,8f.,4g#,f,g.,8d#.,f,g.,8d#.,f,8g,8d#.,f,8g,d#,8c,a#5,8d#.,8d#.,4d#,8d#.";
//char *song = "LeisureSuit:d=16,o=6,b=56:f.5,f#.5,g.5,g#5,32a#5,f5,g#.5,a#.5,32f5,g#5,32a#5,g#5,8c#.,a#5,32c#,a5,a#.5,c#.,32a5,a#5,32c#,d#,8e,c#.,f.,f.,f.,f.,f,32e,d#,8d,a#.5,e,32f,e,32f,c#,d#.,c#";
//char *song = "MissionImp:d=16,o=6,b=95:32d,32d#,32d,32d#,32d,32d#,32d,32d#,32d,32d,32d#,32e,32f,32f#,32g,g,8p,g,8p,a#,p,c7,p,g,8p,g,8p,f,p,f#,p,g,8p,g,8p,a#,p,c7,p,g,8p,g,8p,f,p,f#,p,a#,g,2d,32p,a#,g,2c#,32p,a#,g,2c,a#5,8c,2p,32p,a#5,g5,2f#,32p,a#5,g5,2f,32p,a#5,g5,2e,d#,8d";
//char *song = "SweetChildOMine:d=4,o=6,b=225:d5,d,a5,g5,g,a5,f_,d5,d,a5,g5,g,a5,f_,a5,e5,d,a5,g,a5,f_,a5,e5,d,a5,g5,g,a5,f_,a5,f_,a5,e5,d,a5,g5,g,a5,f_,a5";
//char *song = "LilJon:d=4,o=6,b=100:16b5,16c,p,16b5,16c,p,16c,16c_,8p,16c_,16d_,p,16c_,16d_,p,16d_,8e.,16b5,16c,p,16b5,16c,p,16c,16c_,8p,16c_,16d_,p,16c_,16d_,p,16d_,8e.,8c.,8c.,8c,p,c_,8d_.,8d_.,8d_,p,c_";
//char *song = "TotoAfr:d=4,o=6,b=90:16c5,16p,16d_,16d_,8d_,16d_,16d_,16g_5,8d_,16d_,16g_5,8d_,16d_,16d_5,16p,8d_,8d_,16d,d,16p,16a_5,16p,8p,16c5,16p,16d_,16d_,8d_,16d_,16d_,16g_5,8d_,16d_,16g_5,8d_,16d_,8d_,8d_,8d_,16d,d,16p,16a_5,16p,8p,16c5,16p,16d_,16d_,8d_,16d_,16d_,16g_5,8d_.,16g_5,16p,8d_,d_,16d_5,16p,16d_,2d,a_5,16c5,16p,16d_,16d_,16d_,8d_,16d_,16g_5,8d_,16d_,16g_5,8d_,16d_,16d_5,8d_,16d_,16d_5,8d_,d,8p,16c,8a_5,d,16a_5,16d_,8d,2c";
//char *song = "TwinkleTwinkle:d=4,o=4,b=120:c,c,g,g,a,a,2g,f,f,e,e,d,d,2c,g,g,f,f,e,e,2d,g,g,f,f,e,e,2d,c,c,g,g,a,a,2g,f,f,e,e,d,d,1c,q";
char *song = "marrylamb:d=4,o=4,b=250:a#,g#,f#,g#,a#,a#,2a#,f#,f#,2f#,a#,a#,2a#,a#,g#,f#,g#,a#,a#,a#,a#,g#,g#,a#,g#,f#";

int get_freq(void)
{
  long int tstart = micros();

  
  for(int j = 0; j<SAMPLES; j++)
  {
    // read raw accel/gyro measurements from device
    //accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // these methods (and a few others) are also available

    accelgyro.getAcceleration(&ax, &ay, &az);
    //accelgyro.getRotation(&gx, &gy, &gz);

    #ifdef OUTPUT_READABLE_ACCELGYRO
        // display tab-separated accel/gyro x/y/z values

        /*Serial.print("a/g:\t");
        Serial.print(ax); Serial.print("\t");
        Serial.print(ay); Serial.print("\t");
        Serial.println(az); */

       /*Serial.print("\t");
       Serial.print(gx); Serial.print("\t");
       Serial.print(gy); Serial.print("\t");
       Serial.println(gz);*/

        vReal[j] = az;
        vImag[j] = 0;

        //at = ((ax*ax) + (ay*ay) + (az*az));
        //Serial.println(at);

    #endif

    #ifdef OUTPUT_BINARY_ACCELGYRO
        Serial.write((uint8_t)(ax >> 8)); Serial.write((uint8_t)(ax & 0xFF));
        Serial.write((uint8_t)(ay >> 8)); Serial.write((uint8_t)(ay & 0xFF));
        Serial.write((uint8_t)(az >> 8)); Serial.write((uint8_t)(az & 0xFF));
        //Serial.write((uint8_t)(gx >> 8)); Serial.write((uint8_t)(gx & 0xFF));
        //Serial.write((uint8_t)(gy >> 8)); Serial.write((uint8_t)(gy & 0xFF));
        //Serial.write((uint8_t)(gz >> 8)); Serial.write((uint8_t)(gz & 0xFF));
    #endif

    // blink LED to indicate activity
    //blinkState = !blinkState;
    //digitalWrite(LED_PIN, blinkState);

  }

   long int tend = micros();
   //long int time = tend - tstart;
   long int freq = round(1000000 / ((tend - tstart) / SAMPLES));  // and then we freq it!!

   FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
   FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);
   FFT.ComplexToMagnitude(vReal, vImag, SAMPLES);

   double peak = FFT.MajorPeak(vReal, SAMPLES, freq);
  Serial.print("Peak = "); Serial.println(peak);
//  Serial.print("Freq = ");Serial.println(freq);
  return (int) peak;
}



void play_rtttl(char *p)
{
  // Absolutely no error checking in here

  byte default_dur = 4;
  byte default_oct = 6;
  int bpm = 63;
  int num;
  long wholenote;
  long duration;
  byte note;
  byte scale;

  // format: d=N,o=N,b=NNN:
  // find the start (skip name, etc)

  while(*p != ':') p++;    // ignore name
  p++;                     // skip ':'

  // get default duration
  if(*p == 'd')
  {
    p++; p++;              // skip "d="
    num = 0;
    while(isdigit(*p))
    {
      num = (num * 10) + (*p++ - '0');
    }
    if(num > 0) default_dur = num;
    p++;                   // skip comma
  }

//  Serial.print("ddur: "); Serial.println(default_dur, 10);

  // get default octave
  if(*p == 'o')
  {
    p++; p++;              // skip "o="
    num = *p++ - '0';
    if(num >= 3 && num <=7) default_oct = num;
    p++;                   // skip comma
  }

  //Serial.print("doct: "); Serial.println(default_oct, 10);

  // get BPM
  if(*p == 'b')
  {
    p++; p++;              // skip "b="
    num = 0;
    while(isdigit(*p))
    {
      num = (num * 10) + (*p++ - '0');
    }
    bpm = num;
    p++;                   // skip colon
  }

  //Serial.print("bpm: "); Serial.println(bpm, 10);

  // BPM usually expresses the number of quarter notes per minute
  wholenote = (60 * 1000L / bpm) * 4;  // this is the time for whole note (in milliseconds)

  //Serial.print("wn: "); Serial.println(wholenote, 10);


  // now begin note loop
  while(*p)
  {
    // first, get note duration, if available
    num = 0;
    while(isdigit(*p))
    {
      num = (num * 10) + (*p++ - '0');
    }

    if(num) duration = wholenote / num;
    else duration = wholenote / default_dur;  // we will need to check if we are a dotted note after

    // now get the note
    note = 0;

    switch(*p)
    {
      case 'c':
        note = 1;
        break;
      case 'd':
        note = 3;
        break;
      case 'e':
        note = 5;
        break;
      case 'f':
        note = 6;
        break;
      case 'g':
        note = 8;
        break;
      case 'a':
        note = 10;
        break;
      case 'b':
        note = 12;
        break;
      case 'p':
      default:
        note = 0;
    }
    p++;

    // now, get optional '#' sharp
    if(*p == '#')
    {
      note++;
      p++;
    }

    // now, get optional '.' dotted note
    if(*p == '.')
    {
      duration += duration/2;
      p++;
    }

    // now, get scale
    if(isdigit(*p))
    {
      scale = *p - '0';
      p++;
    }
    else
    {
      scale = default_oct;
    }

    scale += OCTAVE_OFFSET;

    if(*p == ',')
      p++;       // skip comma for next note (or we may be at the end)

    // now play the note

    if(note)
    {
    //  Serial.print("Playing: ");
    //  Serial.print(scale, 10); Serial.print(' ');
    //  Serial.print(note, 10); Serial.print(" (");
      Serial.println(notes[(scale - 4) * 12 + note], 10);
      playNote[i]=(notes[(scale - 4) * 12 + note]);
    //  Serial.print(") ");
    //  Serial.println(duration, 10);
     i++;
      tone(tonePin, notes[(scale - 4) * 12 + note]);
      delay(duration);
      noTone(tonePin);
    }
    else
    {
    //  Serial.print("Pausing: ");
    //  Serial.println(duration, 10);
      delay(duration);
    }
  }

}

void setup(void)
{
  // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // This is for Trinket 5V 16MHz, you can remove these three lines if you are not using a Trinket
  #if defined (__AVR_ATtiny85__)
    if (F_CPU == 16000000) clock_prescale_set(clock_div_1);
  #endif
    // End of trinket special code

    pixels.begin(); // This initializes the NeoPixel library.
    

    // initialize serial communication
    // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
    // it's really up to you depending on your project)
    Serial.begin(38400);

    // initialize device
    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

    // configure Arduino LED for
    pinMode(LED_PIN, OUTPUT);

    sampling_period_us = round(1000000*(1.0/SAMPLING_FREQUENCY));

  play_rtttl(song);
  Serial.println("Done.");

  for (i=0; i < 36; i++)
  {
    Serial.println(playNote[i]);
  }

}

#define isdigit(n) (n >= '0' && n <= '9')

int ALLoctaveFreq[] = {220,  233,   246,  261,  277,  293,  311,  329,  349,  369,  391,   415,  
                       440,  466,   493,  523,  554,  587,  622,  659,  698,  739,  783,   830};  //fifth octave

int TheSong[] = {466,415,369,415,466,466,466,369,369,369,466,466,466,466,415,369,415,466,466,466,466,415,415,466,415,369};

int SongTest[] = {587,622,659,1047,659,1047,659,1047,1047,1175,1245,1319,1047,1175,1319,988,1175,1047,587,622,659,1047,659,1047,659,1047,880,784,740,880,1047,1319,1175,1047,880,1175};

//an array of readings that will be replaced by the acceleramtor data
int readings[] = {0,0,0,0,466,0,0,415,370,0,415,0,466,0,0,0,466,466,0,0,370,0,370,0,370,0,466,0,466,0,466,0,466,0,415,0,370,0,415,0,466,0,466,0,466,0,466,0,415,0,415,0,466,0,415,0,370,0,0,0,0,0,0,0,0};  

//struct of map the frequency to the pixel of the strip
struct Map
{
  int frequency;
  int pixel;
};

//struct to map the queue of note frequencies to 
struct Song
{
  int note_freq;
  int associated_pixel;
};

void loop(void)
{

  

int NumbNotesLength = (sizeof(ALLoctaveFreq)/sizeof(int));

int i = 0;
struct Map TheMap[NumbNotesLength];

while(i < NumbNotesLength)
{
  TheMap[i].frequency = ALLoctaveFreq[i];
  TheMap[i].pixel = i;
  i++;
}

  i = 0;

  int z = 0;

int song_length = (sizeof(TheSong)/sizeof(int));

struct Song SongNotes[song_length];

for(z = 0; z < song_length; z++)
{
  //inserts the frequency assoicated to the first note (increments through the list
  SongNotes[z].note_freq = TheSong[z];

  //traverses through all of the frequency to find the pixel associated with it (brute force method)
  for(i = 0; i < NumbNotesLength; i++)
  {
    
    if(SongNotes[z].note_freq == TheMap[i].frequency)
    {
      //inserts to the pixel that is associated to the frequency
      SongNotes[z].associated_pixel = TheMap[i].pixel;
    }
  }
}

z = 0;
i = 0;

int pixel_location = SongNotes[z].associated_pixel;

//sets the the frequency we will need based on the first note
int freq_need = SongNotes[z].note_freq;

//setst the beginning of the values we will read in (replace with accelleremator)
//int freq_read = readings[i];
int freq_read = get_freq();

//shows the first pixel via red
pixels.setPixelColor(pixel_location,pixels.Color(0,100,0));
pixels.show();
delay(DelayVal);

//variables used for the upper and lower boundary
int UpperBoundary;
int LowerBoundary;
int BoundaryRadii = 30;

int f = 0;
//traverses through the song - infinte
while(1)
{
  
  //traverses through song based on the note associated with the z variable
  while(f <= song_length)
  {
    UpperBoundary = freq_need + BoundaryRadii;
    LowerBoundary = freq_need - BoundaryRadii;
  //  Serial.print("freq_need = "); Serial.println(freq_need);
  //  Serial.print("UpperBoundary = "); Serial.println(UpperBoundary);
  //  Serial.print("LowerBoundary = "); Serial.println(LowerBoundary);
  //  Serial.print("freq_read = "); Serial.println(freq_read);
  //  Serial.print("f = "); Serial.println(f);
  //  f--;
    
    //read in variable and compare to the needed 
    if((LowerBoundary <= freq_read) && (freq_read <= UpperBoundary))
    {
      delay(50);
      //recheck the freq read
      freq_read = get_freq();
      if((LowerBoundary <= freq_read) && (freq_read <= UpperBoundary))
      {
        //sets the pixel currently active to clear
        pixels.setPixelColor(pixel_location, pixels.Color(0,0,0));
        //pixels.show();
       // delay(200);
      
        //increments to the next note
        f++;
      
        //ensures that we do not go over the length of the song
        if(f < song_length)
        {
          pixel_location = SongNotes[f].associated_pixel;
          pixels.setPixelColor(pixel_location,pixels.Color(0,0,100));
          pixels.show();
          freq_need = SongNotes[f].note_freq;
        }
        //ensures that the looping variable does not satisfy the nested while parameter
        else
        {
         f += 2;
        }  
      }
    }
    
    //increments the variable that will read our value
    delay(DelayVal);
    freq_read = get_freq();
  }
  
  //clears the strip of lights
  for(int n = 0; n < NUMPIXELS; n++)
  {
    pixels.setPixelColor(n,pixels.Color(0,0,0));
  }
  pixels.show();
  delay(5000);
  //resets the looping variable associated with z
  f = 0;
  
  //resets the freq needed to read in to the first note of the song
  //signify the first note by teh color red
  pixel_location = SongNotes[z].associated_pixel;
  pixels.setPixelColor(pixel_location,pixels.Color(0,100,0));
  freq_need = SongNotes[f].note_freq;
  
  //resets the variable to read in our "readings" will chnage once accelleremator is in place
  //i = 0;
  freq_read = get_freq();
  pixels.show();
  delay(DelayVal);
  //up above sets up the beginning of the song and we are able to play it again
}



}

