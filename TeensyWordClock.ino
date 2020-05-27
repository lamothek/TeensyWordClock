/*
 * Word Clock - WS2812B
 * 
 * A word clock powered by Teensy 3.2 using WS2812B RGB leds
 * 
 * Hardware:
 *  Teensy 3.2 with crystal for RTC
 *  WS2812B LEDs
 *  WS2812B LEDs
 *  
 * Created: 26/01/2020
 * Kevin Lamothe
 * 
 * kevinlamothe.ca
 * github.com/lamothek
 * 
 */

#include <TimeLib.h>
#include <FastLED.h>

#define LED_PIN 15          //
#define LED_COUNT 130        //Will actually be 99
#define TIME_HEADER  "T"    //Header tag for serial time sync message

CRGB leds[LED_COUNT];

const byte timeSetPin = 1;            //Pin for setting run/set mode
const byte setPinStatus = 13;         //Pin (LED) for displaying set status of each press           
const byte interruptPinMinute = 11;   //Pin name for minute adjust
const byte interruptPinHour = 12;     //Pin name for hour adjust
const byte brightnessSetPin = A0;     //Pin for reading voltage to set brightness A0

volatile int _Year = 0;               //Global int for Year
volatile int _Month = 0;               //Global int for Month
volatile int _Day = 0;                //Global int for Day
volatile int _Minute = 0;             //Global int for Minute
volatile int _Hour = 0;               //Global int for Hour
volatile int _Brightness = 10;        //Global int for LED brigthness 0 - 255
volatile int DELAY_MS = 125;          //Global int for delay in service routines *Might add hardware debounce
volatile bool MODE = false;           //System mode for selecting RUN/SET modes


/*
 * Setup section for defining pin types, serial port, ect.
 */
void setup() 
{
    //Setup serial port
    Serial.begin(9600);
    delay(250);
    Serial.println("Teensy Word Clock");

    //Try and get timestatus from RTC and spit out a serial message
    if (timeStatus()!= timeSet) 
    {
      Serial.println("Unable to sync with the RTC");
    } 
    else 
    {
      Serial.println("RTC has set the system time");
    }

    setSyncProvider(getTeensy3Time);  //Get time
    
    //Setup minute interrupt on Pin 11 as a digital falling type
    pinMode(interruptPinMinute, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(interruptPinMinute), isrMinute, FALLING);
    
    //Setup hour interrupt on Pin 12 as a digital falling type
    pinMode(interruptPinHour, INPUT_PULLDOWN);
    attachInterrupt(digitalPinToInterrupt(interruptPinHour), isrHour, FALLING);
    
    pinMode(timeSetPin, INPUT);         //Setup timeSetPin as digital input
    pinMode(setPinStatus, OUTPUT);      //LED on pin 13 set as output

    //LED setup
    FastLED.addLeds<NEOPIXEL, LED_PIN>(leds, LED_COUNT);

    PingPong();
}


/*
 * Code to loop through continually after setup.
 */
void loop() 
{
     MODE = digitalRead(timeSetPin);                            //Check which mode is slected RUN/SET
     _Brightness = systemBrightness();                          //Check system brightness setting - TODO - Maybe put in setup and run once?
    _SerialOutput(MODE, _Hour, _Minute, 1250, _Brightness);     //Spit out some important info on the serial port
    
    //PingPong();
    _Hour = hour();
    _Minute = minute();
    DisplayTime();
    //
    if (Serial.available())
    {
        time_t t = processSyncMessage();
        
        if (t != 0)
        {
            Teensy3Clock.set(t);
            setTime(t);
        }
    }

    Serial.print(hour());
    printDigits(minute());
    printDigits(second());
    Serial.print(" ");
    Serial.print(day());
    Serial.print(" ");
    Serial.print(month());
    Serial.print(" ");
    Serial.print(year()); 
    Serial.println(); 
}

/*
 * Service Routine for manual hour adjustment on button press.
 *  Checks if system MODE is in SET position.
 *  MODE = True - flash LED 3 times, showing an error.
 *  MODE = False = Increment hour by 1 or roll over to 1 if > 12, flash LED once.
 *  Delay added for debounce, may be removed with hardware debounce.
 */
void isrHour() 
{
    if (MODE)
    {
        flashLED(3);
    }
    else
    {
       _Hour++;             //Increment hour global
    
        //Reset hour to 1 if above 12
        if (_Hour > 12)
        {
            _Hour = 1;
        }
        
        delay(DELAY_MS);    //Delay for debounce  
        flashLED(1);
    }
}

/*   
 *  Service Routine for manual minute adjustment on button press. 
 *  Checks if system MODE is in SET position.
 *  MODE = True - flash LED 3 times, showing an error.
 *  MODE = False = Increment minute by 1 or roll over to 0 if > 59, flash LED once.
 *  Delay added for debounce, may be removed with hardware debounce.
 */ 
void isrMinute() 
{
    if (MODE)
    {
        flashLED(3);
    }
    else
    {
        _Minute++;          //Increment minute global
    
        //Reset minute to 0 if above59
        if (_Minute > 59)
        {
            _Minute = 0;
        }
        
        delay(DELAY_MS);    //Delay for debounce 
        flashLED(1);
    }
}

/*
 * Function to get time_t value of Teensy RTC.
 */
time_t getTeensy3Time()
{
  return Teensy3Clock.get();
}


/*
 * 
 */
unsigned long processSyncMessage() 
{
  unsigned long pctime = 0L;
  const unsigned long DEFAULT_TIME = 1357041600; //Jan 1 2013 

  if(Serial.find(TIME_HEADER)) 
  {
     pctime = Serial.parseInt();
     return pctime;
     
     //Check the value is a valid time (greater than Jan 1 2013)
     if( pctime < DEFAULT_TIME) 
     {
       pctime = 0L; // return 0 to indicate that the time is not valid
     }
  }
  
  return pctime;
}

/*
 * Utility function to for printing time better.
 * Prints colon and leading 0
 */
void printDigits(int digits)
{
  Serial.print(":");
  
  if(digits < 10)
  {
     Serial.print('0');
  }
   
  Serial.print(digits);
}

/* 
 *  Serial output method.
 *  Mostly used for debugging purposes.
 *  _mode - boolean value of mode select to display which setting type we are in.
 *  _hour - current hour.
 *  _minute - current minute.
 *  _delay - delay time in ms.
 */ 
void _SerialOutput(bool _mode, int _hour, int _minute, int _delay, int _brightness)
{
    String modeString = "";
    
    if (_mode)
        modeString = "Run"; 
    else
        modeString = "Set";

    Serial.println("--------------------");
    Serial.println("Mode: " + modeString);
    Serial.println((String)"Time: " + _hour + ":" + _minute);
    Serial.println((String)"Brigthness: " + _brightness);
    delay(_delay);
}

/*
 *  Method to flash the built in LED when required. 
 *  Delays 100ms between each flash.
 *  numFlashes - number of times to flash the LED.
 */
void flashLED(int _numFlashes)
{
    for (int i = 0; i < _numFlashes; i++)
    {
        digitalWrite(setPinStatus, HIGH);
        delay(100);
        digitalWrite(setPinStatus, LOW);
        delay(100);
    }
}

/*
 * Function for setting system brightness
 * Reads an analog pin and maps the counts to a voltage from a voltage division.
 * Converts to a mapped value of 0-255 for system brightness in LED library.
 * Set system brightness using FastLED.setBrightness() function.
 * Returns value of brightness as a percentage
 */
double systemBrightness()
{
    float voltage = analogRead(brightnessSetPin) * (3.3 / 1023.0);
    int _LEDBrightness = ceil((voltage / 3.3) * 255);
    
    //Force minimum brightness to be 10
    if (_LEDBrightness < 10)
    {
        _LEDBrightness = 10;
    }
    
    FastLED.setBrightness(_LEDBrightness);
    FastLED.show();
    
    double brightnessPercent = (_LEDBrightness / 255.0) * 100.0;
    
    //return brightnessPercent;
    return 5;
}

void PingPong()
{
    //leds[0] = CRGB::Red;
    //FastLED.show();
    /*
    for (int i = 0; i < LED_COUNT; i++)
    {
        leds[i] = CRGB::Red;
        FastLED.show();
        FastLED.delay(5);
    }
    for (int j = LED_COUNT-1; j > 0; j--)
    {
        leds[j] = CRGB::Green;
        FastLED.show();
        FastLED.delay(5);
    }
     for (int k = 0; k < LED_COUNT; k++)
    {
        leds[k] = CRGB::Blue;
        FastLED.show();
        FastLED.delay(5);
    }
      for (int k = 0; k < LED_COUNT; k++)
    {
        leds[k] = CRGB::White;
        FastLED.show();
        FastLED.delay(5);
    }
    */
    for (int l = LED_COUNT-1; l > 0; l--)
    {
        leds[l] = CRGB::Black;
        FastLED.show();
        FastLED.delay(5);
    }
}

void ResetLeds()
{
    for (int l = LED_COUNT-1; l > 0; l--)
    {
        leds[l] = CRGB::Black;
        //FastLED.show();
    }
           
}

void DisplayTime()
{

    ResetLeds();
    
    leds[12] = CRGB::White;
    leds[11] = CRGB::White;
    leds[9] = CRGB::White;
    leds[8] = CRGB::White;

    if (minute() >= 0 && minute() < 5) || (minute() ) //Oclock
    {
        leds[122] = CRGB::White;
        leds[121] = CRGB::White;
        leds[120] = CRGB::White;
        leds[119] = CRGB::White;
        leds[118] = CRGB::White;
        leds[117] = CRGB::White;
    }
    else if (minute() >= 5 && < 10)
    {
        
    }
    else if ()
    {
        
    }
    else if ()
    {
        
    }
    else if ()
    {
        
    }

    switch (hour())
    {
        case 1:
            leds[64] = CRGB::White;
            leds[63] = CRGB::White;
            leds[62] = CRGB::White;
            break;
        case 2:
            leds[90] = CRGB::White;
            leds[89] = CRGB::White;
            leds[88] = CRGB::White;
            break;
        case 3:
        case 15:
            leds[56] = CRGB::White;
            leds[55] = CRGB::White;
            leds[54] = CRGB::White;
            leds[53] = CRGB::White;
            leds[52] = CRGB::White;
            FastLED.show();
            break;
        case 4:
        case 16:
            leds[72] = CRGB::White;
            leds[71] = CRGB::White;
            leds[70] = CRGB::White;
            leds[69] = CRGB::White;
            break;
        case 5:
            leds[94] = CRGB::White;
            leds[93] = CRGB::White;
            leds[92] = CRGB::White;
            leds[91] = CRGB::White;
            break;
        case 6:
            leds[116] = CRGB::White;
            leds[115] = CRGB::White;
            leds[114] = CRGB::White;
            break;
        case 7:
            leds[99] = CRGB::White;
            leds[98] = CRGB::White;
            leds[97] = CRGB::White;
            leds[96] = CRGB::White;
            leds[95] = CRGB::White;
            break;
        case 8:
            leds[83] = CRGB::White;
            leds[82] = CRGB::White;
            leds[81] = CRGB::White;
            leds[80] = CRGB::White;
            leds[79] = CRGB::White;
            break;
        case 9:
            leds[103] = CRGB::White;
            leds[102] = CRGB::White;
            leds[101] = CRGB::White;
            leds[100] = CRGB::White;
            break;
        case 10:
            leds[106] = CRGB::White;
            leds[105] = CRGB::White;
            leds[104] = CRGB::White;
            break;
        case 11:
            leds[77] = CRGB::White;
            leds[76] = CRGB::White;
            leds[75] = CRGB::White;
            leds[74] = CRGB::White;
            leds[73] = CRGB::White;
            leds[72] = CRGB::White;
            break;
        case 12:
            leds[129] = CRGB::White;
            leds[128] = CRGB::White;
            leds[127] = CRGB::White;
            leds[126] = CRGB::White;
            leds[125] = CRGB::White;
            leds[124] = CRGB::White;
            break;
    }


    FastLED.show();

    
}
