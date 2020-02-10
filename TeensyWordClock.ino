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

#define LED_PIN 15
#define LED_COUNT 15    //will actually be 99
CRGB leds[LED_COUNT];

const byte timeSetPin = 1;            //Pin for setting run/set mode
const byte setPinStatus = 13;         //Pin (LED) for displaying set status of each press           
const byte interruptPinMinute = 11;   //Pin name for minute adjust
const byte interruptPinHour = 12;     //Pin name for hour adjust
const byte brightnessSetPin = A0;     //Pin for reading voltage to set brightness A0

volatile int _Minute = 0;             //Global int for minutes **Placeholder for now
volatile int _Hour = 1;               //Global int for hours *Placeholder for now
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
    Serial.println("Teensy Word Clock");
    if (timeStatus()!= timeSet) {
    Serial.println("Unable to sync with the RTC");
  } else {
    Serial.println("RTC has set the system time");
  }

    //Setup minute interrupt on Pin 11 as a digital falling type
    pinMode(interruptPinMinute, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(interruptPinMinute), isrMinute, FALLING);
    
    //Setup hour interrupt on Pin 12 as a digital falling type
    pinMode(interruptPinHour, INPUT_PULLDOWN);
    attachInterrupt(digitalPinToInterrupt(interruptPinHour), isrHour, FALLING);
    
    pinMode(timeSetPin, INPUT);         //Setup timeSetPin as digital input
    pinMode(setPinStatus, OUTPUT);      //LED on pin 13 set as output

    FastLED.addLeds<NEOPIXEL, LED_PIN>(leds, LED_COUNT);

    setSyncProvider(getTeensy3Time);
}


/*
 * Code to loop through continually after setup.
 */
void loop() 
{
     MODE = digitalRead(timeSetPin);                            //Check which mode is slected RUN/SET
     _Brightness = systemBrightness();                           //Check system brightness setting - TODO - Maybe put in setup and run once?
    _SerialOutput(MODE, _Hour, _Minute, 1250, _Brightness);     //Spit out some important info on the serial port
    
    PingPong();

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

/*  code to process time sync messages from the serial port   */
#define TIME_HEADER  "T"   // Header tag for serial time sync message

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
     
     if( pctime < DEFAULT_TIME) 
     { // check the value is a valid time (greater than Jan 1 2013)
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
    
    return brightnessPercent;
}

void PingPong()
{
    leds[0] = CRGB::Red;
    FastLED.show();
    /*
    for (int i = 0; i < LED_COUNT; i++)
    {
        leds[i] = CRGB::Red;
        FastLED.show();
        FastLED.delay(25);
    }
    for (int j = LED_COUNT-1; j > 0; j--)
    {
        leds[j] = CRGB::Green;
        FastLED.show();
        FastLED.delay(50);
    }
     for (int k = 0; k < LED_COUNT; k++)
    {
        leds[k] = CRGB::Blue;
        FastLED.show();
        FastLED.delay(50);
    }
    for (int l = LED_COUNT-1; l > 0; l--)
    {
        leds[l] = CRGB::Black;
        FastLED.show();
        FastLED.delay(50);
    }
    */
}
