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
#define FASTLED_ALLOW_INTERRUPTS 0  //Disable interrubles for WS2812B timing requirements

#include <TimeLib.h>
#include <FastLED.h>

#define LED_PIN 15          //Pin on Teensy to write LED data to
#define LED_COUNT 130       //Number of LEDs in word clock
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
volatile int _MinuteState = -1;       //Global int for Minute logic, avoids repeating if, else if in functions
volatile int DELAY_MS = 125;          //Global int for delay in service routines *Might add hardware debounce
volatile bool MODE = false;           //System mode for selecting RUN/SET modes


/*
 * Setup section for defining pin types, serial port, ect.
 */
void setup() 
{
    //Setup serial port
    Serial.begin(9600);
    delay(500);
    
    Serial.println("Teensy Word Clock");

    if (Serial.available())
    {
        time_t t = processSyncMessage();
        
        if (t != 0)
        {
            Teensy3Clock.set(t);
            setTime(t);
        }
    }

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
}

/*
 * Code to loop through continually after setup.
 */
void loop() 
{
     MODE = digitalRead(timeSetPin);                            //Check which mode is slected RUN/SET
    
     _Brightness = systemBrightness();                          //Check system brightness setting - TODO - Maybe put in setup and run once?
    _SerialOutput(MODE, _Hour, _Minute, 2000, _Brightness);     //Spit out some important info on the serial port
    
    _Hour = hour();         //Set hour global for function use
    _Minute = minute();     //Set minute globacl for function use
    
    DisplayTime();          //Display the current time with LEDs
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
 * Function for processing sync messages with PC Time.
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
    {
        modeString = "Run"; 
    }
    else
    {
        modeString = "Set";
    }

    Serial.println("--------------------");
    Serial.println("Mode: " + modeString);
    Serial.println((String)"Brightness: " + _brightness);
    Serial.print((String)"Date/Time: ");
    Serial.print(_Hour);
    printDigits(_Minute);
    printDigits(second());
    Serial.print(" ");
    Serial.print(day());
    Serial.print("/");
    Serial.print(month());
    Serial.print("/");
    Serial.print(year()); 
    Serial.println(); 
    delay(_delay);
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

/*
 * Function for setting system brightness
 * Reads an analog pin and maps the counts to a voltage from a voltage division.
 * Converts to a mapped value of 0-255 for system brightness in LED library.
 * Set system brightness using FastLED.setBrightness() function.
 * Returns value of brightness as a percentage
 */
void DisplayTime()
{
    

    FastLED.clear();            //Reset all LEDs to black (off)

    DisplayMinutes();       //Set LEDs for minutes
    DisplayHour();          //Set LEDs for hour
    DisplayStatusWords();   //Set non-time words
    
    FastLED.show();         //Write data to LEDs
}

/*
 * Function for displaying current hour LEDs.
 * Uses _Hour global variable to set hour based on current minute reading.
 */
void DisplayHour()
{
    //Increment hour if we are past 30 minutes, reset in each loop
    if (minute() >= 30)
    {
        _Hour++;
    }

    //Set the hour LEDs baed on _Hour reading
    switch (_Hour)
    {
        case 1:
        case 13:
            leds[64] = CRGB::White;
            leds[63] = CRGB::White;
            leds[62] = CRGB::White;
            break;
        case 2:
        case 14:
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
            break;
        case 4:
        case 16:
            leds[71] = CRGB::White;
            leds[70] = CRGB::White;
            leds[69] = CRGB::White;
            leds[68] = CRGB::White;
            break;
        case 5:
        case 17:
            leds[94] = CRGB::White;
            leds[93] = CRGB::White;
            leds[92] = CRGB::White;
            leds[91] = CRGB::White;
            break;
        case 6:
        case 18:
            leds[116] = CRGB::White;
            leds[115] = CRGB::White;
            leds[114] = CRGB::White;
            break;
        case 7:
        case 19:
            leds[99] = CRGB::White;
            leds[98] = CRGB::White;
            leds[97] = CRGB::White;
            leds[96] = CRGB::White;
            leds[95] = CRGB::White;
            break;
        case 8:
        case 20:
            leds[83] = CRGB::White;
            leds[82] = CRGB::White;
            leds[81] = CRGB::White;
            leds[80] = CRGB::White;
            leds[79] = CRGB::White;
            break;
        case 9:
        case 21:
            leds[103] = CRGB::White;
            leds[102] = CRGB::White;
            leds[101] = CRGB::White;
            leds[100] = CRGB::White;
            break;
        case 10:
        case 22:
            leds[106] = CRGB::White;
            leds[105] = CRGB::White;
            leds[104] = CRGB::White;
            break;
        case 11:
        case 23:
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
}

/*
 * Fucntion for setting LEDs to display minute range.
 * Reads minute() function from time library and sets appropriate LEDs.
 * _MinuteState used to avoid duplicating similar if, else if statements in DisplayStatusWords().
 */
void DisplayMinutes()
{
    if ((minute() >= 0 && minute() < 5)) //Oclock
    {
        leds[122] = CRGB::White;
        leds[121] = CRGB::White;
        leds[120] = CRGB::White;
        leds[119] = CRGB::White;
        leds[118] = CRGB::White;
        leds[117] = CRGB::White;
        _MinuteState = 0;
    }
    else if ((minute() >= 5 && minute() < 10) || (minute() >= 55 && minute() < 60)) //Five
    {
        leds[35] = CRGB::White;
        leds[36] = CRGB::White;
        leds[37] = CRGB::White;
        leds[38] = CRGB::White;
        _MinuteState = 1;
    }
    else if ((minute() >= 10 && minute() < 15) || (minute() >= 50 && minute() < 55)) //Ten
    {
        leds[4] = CRGB::White;
        leds[5] = CRGB::White;
        leds[6] = CRGB::White;
        _MinuteState = 2;
    }
    else if ((minute() >= 15 && minute() < 20) || (minute() >= 45 && minute() < 50)) //Fifteen
    {
        leds[20] = CRGB::White;
        leds[21] = CRGB::White;
        leds[22] = CRGB::White;
        leds[23] = CRGB::White;
        leds[24] = CRGB::White;
        leds[25] = CRGB::White;
        _MinuteState = 3;
    }
    else if ((minute() >= 20 && minute() < 25) || (minute() >= 40 && minute() < 45)) //Twenty
    {
        leds[13] = CRGB::White;
        leds[14] = CRGB::White;
        leds[15] = CRGB::White;
        leds[16] = CRGB::White;
        leds[17] = CRGB::White;
        leds[18] = CRGB::White;
        _MinuteState = 4;
    }
     else if ((minute() >= 25 && minute() < 30) || (minute() >= 35 && minute() < 40)) //Twenty Five
    {
        leds[13] = CRGB::White;
        leds[14] = CRGB::White;
        leds[15] = CRGB::White;
        leds[16] = CRGB::White;
        leds[17] = CRGB::White;
        leds[18] = CRGB::White;
        leds[35] = CRGB::White;
        leds[36] = CRGB::White;
        leds[37] = CRGB::White;
        leds[38] = CRGB::White;
        _MinuteState = 5;
    }
    else if (minute() >= 30 || minute() < 35) //Half
    {
        leds[0] = CRGB::White;
        leds[1] = CRGB::White;
        leds[2] = CRGB::White;
        leds[3] = CRGB::White;
        _MinuteState = 6;
    }
}

/*
 * Function for setting state of LEDs for words other than minuts or hours.
 * IT, IS - Always on.
 * MINUTES - Set with _MinuteState variable set in DisplayMinutes() function.
 * PAST, TO - Set based on minute reading.
 */
void DisplayStatusWords()
{
    //Default LEDS, always on
    leds[12] = CRGB::White; //IT
    leds[11] = CRGB::White;
    leds[9] = CRGB::White;  //IS
    leds[8] = CRGB::White;

    //Used to set word MINUTES to ON. 
    //Five, Ten, Twenty, Twenty Five
    if (_MinuteState == 1 || _MinuteState == 2 || _MinuteState == 4 || _MinuteState == 5)
    {
        leds[27] = CRGB::White;
        leds[28] = CRGB::White;
        leds[29] = CRGB::White;
        leds[30] = CRGB::White;
        leds[31] = CRGB::White;
        leds[32] = CRGB::White;
        leds[33] = CRGB::White;
    }

    if (minute() < 35)              //Past
    {
        leds[42] = CRGB::White;
        leds[43] = CRGB::White;
        leds[44] = CRGB::White;
        leds[45] = CRGB::White;
    }
    else if (minute() >= 35)        //TO
    {
        leds[40] = CRGB::White;
        leds[41] = CRGB::White;
    }
}
