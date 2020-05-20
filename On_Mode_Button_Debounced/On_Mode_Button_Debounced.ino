
unsigned long onBouncePrevTime=0,onBounceCurTime,onPrevTime=0,onCurTime;
bool onToggle=false, startCount=false, onPinReset=true;

void setup(void)
{
  Serial.begin(19200);
  // initialize the LED pin as an output:
  // initialize the pushbutton pin as an input:
  pinMode(PIN_SW0, INPUT_PULLUP);
  Serial.println("Ready");
}

void loop(void)
{ 
  onPrevTime=millis();
  while((!digitalRead(PIN_SW0)&&startCount)){
    onCurTime=millis();
    if((onCurTime-onPrevTime)>=5000){
      //resetKey=true;
      //state=1;
      //calculateOffset();
      Serial.println("Key Reset triggered");
      startCount=false;
    }
  }
  startCount=false;
  if(!onToggle){
    if(!digitalRead(PIN_SW0)){
        onToggle=true;
        onBouncePrevTime=millis();
    }
    else{
      onPinReset=true;      
    }
  }
  
  else{
    onBounceCurTime=millis();
    if((onBounceCurTime-onBouncePrevTime)>10){
      if(!digitalRead(PIN_SW0)){
        onBouncePrevTime=onBounceCurTime;
        if(onPinReset){
          startCount=true;
        }
        onPinReset=false;
        onToggle=false;
      }
      
      else{
        onToggle=false;
      }
    }
  }
}
