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
 */

const byte timeSetPin = 1;            //Pin for setting run/set mode
const byte setPinStatus = 13;         //Pin (LED) for displaying set status of each press           
const byte interruptPinMinute = 11;   //Pin name for minute adjust
const byte interruptPinHour = 12;     //Pin name for hour adjust

volatile int _Minute = 0;             //Global int for minutes **Placeholder for now
volatile int _Hour = 1;               //Global int for hours *Placeholder for now

volatile int DELAY_MS = 200;          //Global int for delay in service routines *Might add hardware debounce

/*
 * Setup section for defining pin types, serial port, ect.
 */
void setup() 
{
  flashLED(2);
  
  Serial.begin(9600); //Open the serial port at 9600 baud

  //Setup minute interrupt on Pin 11 as a digital falling type
  pinMode(interruptPinMinute, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPinMinute), isrMinute, FALLING);

  //Setup hour interrupt on Pin 12 as a digital falling type
  pinMode(interruptPinHour, INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(interruptPinHour), isrHour, FALLING);

  pinMode(timeSetPin, INPUT);     //Setup timeSetPin as digital input

  pinMode(setPinStatus, OUTPUT);  //LED on pin 13 set as output
}

/*
 * Code to loop through continually after setup.
 */
void loop() 
{
  _SerialOutput(_Hour, _Minute, 2000);
}

/*
 * Service Routine for manual hour adjustment on button press.
 */
void isrHour() 
{
  _Hour++;          //Increment hour global

  //Reset hour to 1 if above 12
  if (_Hour > 12)
  {
    _Hour = 1;
  }
  
  delay(DELAY_MS);    //Delay for debounce
}

/*   
 *  Service Routine for manual minute adjustment on button press. 
 */ 
void isrMinute() 
{
  _Minute++;          //Increment minute global
  
  //Reset minute to 0 if above59
  if (_Minute > 59)
  {
    _Minute = 0;
  }
  
  delay(DELAY_MS);    //Delay for debounce
}

/* 
 *  Serial output method.
 *  Mostly used for debugging purposes.
 *  _hour - current hour.
 *  _minute - current minute.
 *  _delay - delay time in ms.
 */ 
void _SerialOutput(int _hour, int _minute, int _delay)
{
  Serial.print(_hour);
  Serial.print(":");
  Serial.print(_minute);
  Serial.println();
  delay(_delay);
}

/*
 *  Method to flash the built in LED when required. 
 *  Delays 250ms between each flash.
 *  numFlashes - number of times to flash the LED.
 */
void flashLED(int _numFlashes)
{
  for (int i = 0; i < _numFlahes; i++)
  {
    digitalWrite(setPinStatus, HIGH);
    delay(250);
    digitalWrite(setPinStatus, LOW);
    delay(250);
  }
}
