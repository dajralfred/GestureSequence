
unsigned long bouncePrevTime=0,bounceCurTime,idlePrevTime=0,idleCurTime;
bool toggle=false;
bool ledState=true,pinReset=true;

void setup(void)
{
  Serial.begin(19200);
  // initialize the LED pin as an output:
  pinMode(PIN_LED_13, OUTPUT);
  // initialize the pushbutton pin as an input:
  pinMode(PIN_SW0, INPUT_PULLUP);
  digitalWrite(PIN_LED_13,HIGH);
  Serial.println("Ready");
}

void loop(void)
{ 
  idleCurTime=millis();
  if((idleCurTime-idlePrevTime)>=1000){
    ledState=!ledState;
    digitalWrite(PIN_LED_13,ledState);
    idlePrevTime=idleCurTime;
  }
  if(!toggle){
    if(!digitalRead(PIN_SW0)){
        toggle=true;
        bouncePrevTime=millis();
    }
    else{
      pinReset=true;
      
    }
  }
  
  else{
    bounceCurTime=millis();
    if((bounceCurTime-bouncePrevTime)>10){
      if(!digitalRead(PIN_SW0)){
        bouncePrevTime=bounceCurTime;
        if(pinReset){
          Serial.println("Switching to state 1");
          //state=1;
          digitalWrite(PIN_LED_13,HIGH);
        }
        pinReset=false;
        toggle=false;
      }
      
      else{
        toggle=false;
      }
    }
  }
}
