/*
  Button

 Turns on and off a light emitting diode(LED) connected to digital
 pin 13, when pressing a pushbutton attached to pin 2.


 The circuit:
 * LED attached from pin 13 to ground
 * pushbutton attached to pin 2 from +5V
 * 10K resistor attached to pin 2 from ground

 * Note: on most Arduinos there is already an LED on the board
 attached to pin 13.


 created 2005
 by DojoDave <http://www.0j0.org>
 modified 30 Aug 2011
 by Tom Igoe

 This example code is in the public domain.

 http://www.arduino.cc/en/Tutorial/Button

  Atmel SAMD21/W25 Xplained pros have a buitin LED connected to D13: LED0
  and a user available push button: SW0.
  updated 15 Dec 2015 by T. VIARD
 */

// constants won't change. They're used here to
// set pin numbers:
const int buttonPin = PIN_SW0;    // the number of the pushbutton pin
const int ledPin    = PIN_LED_13; // the number of the LED pin
unsigned long prevTime,curTime;
bool toggle=false;
bool ledState=true,pinReset=true;

// variables will change:
int buttonState = 0;         // variable for reading the pushbutton status

void setup(void)
{
  Serial.begin(19200);
  // initialize the LED pin as an output:
  pinMode(ledPin, OUTPUT);
  // initialize the pushbutton pin as an input:
  pinMode(buttonPin, INPUT_PULLUP);
  digitalWrite(ledPin,HIGH);
  Serial.println("Ready");
}

void loop(void)
{ 
  if(!toggle){
    if(!digitalRead(buttonPin)){
        toggle=true;
    }
    else{
      pinReset=true;
      
    }
  }
  
  else{
    curTime=millis();
    if((curTime-prevTime)>10){
      if(!digitalRead(buttonPin)){
        prevTime=curTime;
        if(pinReset){
          ledState=!ledState;
        }
        pinReset=false;
        digitalWrite(ledPin,ledState);
        toggle=false;
        
      }
      else{
        toggle=false;
      }
    }
  }
}
