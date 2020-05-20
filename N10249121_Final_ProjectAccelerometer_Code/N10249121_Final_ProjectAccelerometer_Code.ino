/*
 * The on-board LED will be used as a status indicator
 * Initially, a default recordKey is used as defined below. 
 *  recordKeys are of a length, 6.
 * A 1 Hz blinking LED inidicates that Idle mode is active
 *  Pressing the SW0, button will turn the LED off and Gesture mode will be activated. 
 * A non-blinking LED inidcates gesture mode is active.
 *  If no gestures are detected, gesture-mode will remain active until timeout of 5 secs.
 *  During calibration, user should hold their hand steady and Accelerometer should remain relatively flat(the largest portion of graviational acceleration should be in Z direction)
 *  Large movements during calibration will lead to Gesture mode being exited due to a "Force Mismatch" and the MCU will return to an idle state
 * A 2 Hz sequence of 2 LED blinks indicates when a single gesture is recorded and also functions as a delay during which the user may move their hand. 
 *  A gesture consists of a movement away from the origin + a return movement toward the origin+ stop. Use serial output and practice to get a feel of it.
 *  Movement is permitted during blinking event after each gesture described above.
 *  After each Gesture-Recorded Indication Sequence, the accelerometer is recalibrated. The user should again hold the accelerometer relatively flat and stationary.
 *  Recalibration is complete when the LED returns to a consistent off state.
 * A solid light indicates that the sequence of gestures is correct/ verified and the On-state has been entered.
 * Holding SW0 for 5 secs in On-State(while LED light is solid)  will enter Gesture mode to reset the recordKey
 * A 4 Hz sequence of 5 LED blinks indicates an error
 * If the LED does not return to the idle state after an error sequence, simply re-enter the last gesture. There was a Force-Mismatch
 * A return of the LED to the idle state from gesture mode after an error is due to a Timeout or Orientation Error
 * A return of the LED to the 1 Hz blinking state inidicates that the Idle-state has been reactivated after an error.
 */

//NB delay() functions eratically when millis() in use and attachInterrupt is not working or not mapped correctly. Therefore, neither of these two functions are used.

#include <Wire.h>
const int MPU = 0x68; // MPU6050 I2C address
signed short initialX,initialY,initialZ;
float AccX, AccY, AccZ;
float offsetX,offsetY,offsetZ;
float AccErrorX, AccErrorY;
float elapsedTime, currentTime, previousTime;
float mvtThresholdAX=0.525, mvtThresholdAY=0.6, mvtThresholdAZ=0.775; 
float mvtThresholdBX=0.15, mvtThresholdBY=0.15, mvtThresholdBZ=0.255;
//mvtThresholdA sets the minimum acceleration to be recorded as a gesture. Seting this value too high/low leads to issues.
//mvtThresholdB sets the maximum acceleration values that should be observed along the other axes during a gesture event
float gestThreshold=1.5; //This sets the amount of acceptable error between acceleration in initital and opposite directions.
unsigned long returnTimeout=5000;
unsigned char state=0; //0:Idle/Polling , 1:Gesture mode , 2:On/Reset
char recordKey[][3]={"X+","X-","Y+","Y-","Z+","Z-"};//X+:Right, X-:Left, Y+:Forward, Y-:Backward, Z+:Up, Z-: Down
char inputKey[][3]={"  ","  ","  ","  ","  ","  "};
char blankKey[][3]={"  ","  ","  ","  ","  ","  "};
uint8_t keyCount=sizeof(recordKey)/sizeof(recordKey[0]);
uint8_t entryCount=0;
unsigned long bouncePrevTime=0,bounceCurTime,idlePrevTime=0,idleCurTime;
bool toggle=false;
bool ledState=true,pinReset=true;
bool resetKey=false;
unsigned long onBouncePrevTime=0,onBounceCurTime,onPrevTime=0,onCurTime;
bool onToggle=false, startCount=false, onPinReset=true;

void setup() {
  pinMode(PIN_LED_13,OUTPUT);
  pinMode(PIN_SW0, INPUT_PULLUP);
  digitalWrite(PIN_LED_13,HIGH);
  Serial.begin(19200);
  Wire.begin();                      // Initialize comunication
  Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);        //end the transmission
  /*
  // Configure Accelerometer Sensitivity - Full Scale Range (default +/- 2g)
  Wire.beginTransmission(MPU);
  Wire.write(0x1C);                  //Talk to the ACCEL_CONFIG register (1C hex)
  Wire.write(0x10);                  //Set the register bits as 00010000 (+/- 8g full scale range)
  Wire.endTransmission(true);
  */

  //calculateOffset();
  Serial.println("Ready");
}
void loop() {
  // === Read acceleromter data === //
  switch(state){
    case 0:{//Idle State: 1Hz LED blink. Button debounced
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
              Serial.println("Switching to state 1...");
              state=1;
              blinkLED(1000,2);
              digitalWrite(PIN_LED_13,LOW);//Turn LED before calibration
              calculateOffset();
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
      break;

    case 1:{
      digitalWrite(PIN_LED_13,HIGH);//Ensure LED is OFF
      if(offsetX==0&&offsetY==0&&offsetZ==0){//Check if calibration was successful. Fails if accelerometer not held relativelt flat.
        blinkLED(250,5);
        Serial.println("Orientation Error!");
        state=0;
        break;
      }
       /*
        * Recall that this is an Inertial Measurement Unit (IMU). The algorithm used to read these gestures stem from the law of conservation of momentum.
        * The average momentum in the original direction should be followed by roughly equal magnitude momentum in the opposite direction with some error.
        * Thresholds are used to distinguish different gesture events.
        */
      entryCount=0;
      while(entryCount<keyCount){
        readAcc(false);//Read accelerometer

        //Code here for gestures on X axis
        if((abs(AccX)>mvtThresholdAX)&&(abs(AccY)<mvtThresholdBX)&&(abs(AccZ)<mvtThresholdBX)){//Detect linear movement along X-axis
          unsigned long int countOver=0;
          float cumsumX=0;
          //readAcc(false);
          signed char sign=signOfBit(AccX);
          Serial.print("Sign of Movement: ");
          Serial.println((int)sign);
          unsigned long int mvtPrevTime=millis();
          while((AccX*(float)sign)>0){
            unsigned long int mvtCurTime=millis();
            if((mvtCurTime-mvtPrevTime)>=1){
              cumsumX+=AccX;
              countOver++;
              readAcc(false);
              mvtPrevTime=mvtCurTime;
            }          
          }
          float avgX=0;
          avgX=cumsumX/(float)countOver;
          
          if(sign>0){
            bool timeoutFlag=false;
            unsigned long int gesPrevTime=millis();
            while(AccX>(-1.0*mvtThresholdAX)){
              unsigned long int gesCurTime=millis();
              if((gesCurTime-gesPrevTime)>returnTimeout){
                blinkLED(250,5);
                state=0;
                Serial.println("Error: Gesture Return Timeout!");
                timeoutFlag=true;
                break;
              }
              readAcc(false);
            }
            if(timeoutFlag){
              break;
            }
          }
          
          else{
            bool timeoutFlag=false;
            unsigned long int gesPrevTime=millis();
            while(AccX<mvtThresholdAX){
              unsigned long int gesCurTime=millis();
              if((gesCurTime-gesPrevTime)>returnTimeout){
                blinkLED(250,5);
                state=0;
                Serial.println("Error: Gesture Return Timeout!");
                timeoutFlag=true;
                break;
              }
              readAcc(false);
            }
            if(timeoutFlag){
              break;
            }
          }
  
          unsigned long int countOver2=0;
          float cumsumX2=0;
          //readAcc(false);
          sign=signOfBit(AccX);
          mvtPrevTime=millis();
          while((AccX*(float)sign)>0){
            unsigned long int mvtCurTime=millis();
            if((mvtCurTime-mvtPrevTime)>=1){
              cumsumX2+=AccX;
              countOver2++;
              readAcc(false);
              mvtPrevTime=mvtCurTime;
            }          
          }
          float avgX2=0;
          avgX2=cumsumX2/(float)countOver2;
          float cumErrorX=(abs(cumsumX2)-abs(cumsumX))/abs(cumsumX);
          float avgErrorX=(abs(avgX2)-abs(avgX))/abs(avgX);
          Serial.print("CumsumX error: ");
          Serial.println(cumErrorX);
          Serial.print("AvgX error: ");
          Serial.println(avgErrorX);
          if(abs(avgErrorX)>gestThreshold){
            blinkLED(100,5);
            //state=0;
            Serial.println("Error: Gesture Force Mismatch!");
            //break;
          }
          else{
            Serial.print("X");
            Serial.print((char)(44+sign));
            Serial.println(" Gesture Recorded");
            inputKey[entryCount][0]='X';
            inputKey[entryCount][1]=((char)(44+sign));
            blinkLED(500,2);
            digitalWrite(PIN_LED_13,LOW);//Turn LED On before calibration
            entryCount++;
          }
          calculateOffset();
          digitalWrite(PIN_LED_13,HIGH);//Turn LED off after recalibration
        }

        //Code here for gestures on Y axis
        else if((abs(AccY)>mvtThresholdAY)&&(abs(AccX)<mvtThresholdBY)&&(abs(AccZ)<mvtThresholdBY)){//Detect linear movement along Y-axis
          unsigned long int countOver=0;
          float cumsumY=0;
          //readAcc(false);
          signed char sign=signOfBit(AccY);
          Serial.print("Sign of Movement: ");
          Serial.println((int)sign);
          unsigned long int mvtPrevTime=millis();
          while((AccY*(float)sign)>0){
            unsigned long int mvtCurTime=millis();
            if((mvtCurTime-mvtPrevTime)>=1){
              cumsumY+=AccY;
              countOver++;
              readAcc(false);
              mvtPrevTime=mvtCurTime;
            }          
          }
          float avgY=0;
          avgY=cumsumY/(float)countOver;
          
          if(sign>0){
            bool timeoutFlag=false;
            unsigned long int gesPrevTime=millis();
            while(AccY>(-1.0*mvtThresholdAY)){
              unsigned long int gesCurTime=millis();
              if((gesCurTime-gesPrevTime)>returnTimeout){
                blinkLED(250,5);
                state=0;
                Serial.println("Error: Gesture Return Timeout!");
                timeoutFlag=true;
                break;
              }
              readAcc(false);
            }
            if(timeoutFlag){
              break;
            }
          }
          
          else{
            bool timeoutFlag=false;
            unsigned long int gesPrevTime=millis();
            while(AccY<mvtThresholdAY){
              unsigned long int gesCurTime=millis();
              if((gesCurTime-gesPrevTime)>returnTimeout){
                blinkLED(250,5);
                state=0;
                Serial.println("Error: Gesture Return Timeout!");
                timeoutFlag=true;
                break;
              }
              readAcc(false);
            }
            if(timeoutFlag){
              break;
            }
          }
  
          unsigned long int countOver2=0;
          float cumsumY2=0;
          //readAcc(false);
          sign=signOfBit(AccY);
          mvtPrevTime=millis();
          while((AccY*(float)sign)>0){
            unsigned long int mvtCurTime=millis();
            if((mvtCurTime-mvtPrevTime)>=1){
              cumsumY2+=AccY;
              countOver2++;
              readAcc(false);
              mvtPrevTime=mvtCurTime;
            }          
          }
          float avgY2=0;
          avgY2=cumsumY2/(float)countOver2;
          float cumErrorY=(abs(cumsumY2)-abs(cumsumY))/abs(cumsumY);
          float avgErrorY=(abs(avgY2)-abs(avgY))/abs(avgY);
          Serial.print("CumsumY error: ");
          Serial.println(cumErrorY);
          Serial.print("AvgY error: ");
          Serial.println(avgErrorY);
          if(abs(avgErrorY)>gestThreshold){
            blinkLED(100,5);
            //state=0;
            Serial.println("Error: Gesture Force Mismatch!");
            //break;
          }
          else{
            Serial.print("Y");
            Serial.print((char)(44+sign));
            Serial.println(" Gesture Recorded");
            inputKey[entryCount][0]='Y';
            inputKey[entryCount][1]=((char)(44+sign));
            blinkLED(500,2);
            digitalWrite(PIN_LED_13,LOW);//Turn LED On before calibration
            entryCount++;
          }
          calculateOffset();
          digitalWrite(PIN_LED_13,HIGH);//Turn LED off after recalibration
        }
        
        //Code here for gestures on Z axis
        else if((abs(AccZ)>mvtThresholdAZ)&&(abs(AccX)<mvtThresholdBZ)&&(abs(AccY)<mvtThresholdBZ)){//Detect linear movement along Z-axis
          unsigned long int countOver=0;
          float cumsumZ=0;
          //readAcc(false);
          signed char sign=signOfBit(AccZ);
          Serial.print("Sign of Movement: ");
          Serial.println((int)sign);
          unsigned long int mvtPrevTime=millis();
          while((AccZ*(float)sign)>0){
            unsigned long int mvtCurTime=millis();
            if((mvtCurTime-mvtPrevTime)>=1){
              cumsumZ+=AccZ;
              countOver++;
              readAcc(false);
              mvtPrevTime=mvtCurTime;
            }          
          }
          float avgZ=0;
          avgZ=cumsumZ/(float)countOver;
          
          if(sign>0){
            bool timeoutFlag=false;
            unsigned long int gesPrevTime=millis();
            while(AccZ>(-1.0*mvtThresholdAZ)){
              unsigned long int gesCurTime=millis();
              if((gesCurTime-gesPrevTime)>returnTimeout){
                blinkLED(250,5);
                state=0;
                Serial.println("Error: Gesture Return Timeout!");
                timeoutFlag=true;
                break;
              }
              readAcc(false);
            }
            if(timeoutFlag){
              break;
            }
          }
          
          else{
            bool timeoutFlag=false;
            unsigned long int gesPrevTime=millis();
            while(AccZ<mvtThresholdAZ){
              unsigned long int gesCurTime=millis();
              if((gesCurTime-gesPrevTime)>returnTimeout){
                blinkLED(250,5);
                state=0;
                Serial.println("Error: Gesture Return Timeout!");
                timeoutFlag=true;
                break;
              }
              readAcc(false);
            }
            if(timeoutFlag){
              break;
            }
          }
  
          unsigned long int countOver2=0;
          float cumsumZ2=0;
          //readAcc(false);
          sign=signOfBit(AccZ);
          mvtPrevTime=millis();
          while((AccZ*(float)sign)>0){
            unsigned long int mvtCurTime=millis();
            if((mvtCurTime-mvtPrevTime)>=1){
              cumsumZ2+=AccZ;
              countOver2++;
              readAcc(false);
              mvtPrevTime=mvtCurTime;
            }          
          }
          float avgZ2=0;
          avgZ2=cumsumZ2/(float)countOver2;
          float cumErrorZ=(abs(cumsumZ2)-abs(cumsumZ))/abs(cumsumZ);
          float avgErrorZ=(abs(avgZ2)-abs(avgZ))/abs(avgZ);
          Serial.print("CumsumZ error: ");
          Serial.println(cumErrorZ);
          Serial.print("AvgZ error: ");
          Serial.println(avgErrorZ);
          if(abs(avgErrorZ)>gestThreshold){
            blinkLED(100,5);
            //state=0;
            Serial.println("Error: Gesture Force Mismatch!");
            //break;
          }
          else{
            Serial.print("Z");
            Serial.print((char)(44+sign));
            Serial.println(" Gesture Recorded");
            inputKey[entryCount][0]='Z';
            inputKey[entryCount][1]=((char)(44+sign));
            blinkLED(500,2);
            digitalWrite(PIN_LED_13,LOW);//Turn LED On before calibration
            entryCount++;
          }
          calculateOffset();
          digitalWrite(PIN_LED_13,HIGH);//Turn LED off after recalibration
        }
        
      }
      
      //Code here for after inputKey is filled
      Serial.print("The entered key is: ");//First print the inputKey to Serial
      for(int i=0;i<keyCount;i++){
        if(i<(keyCount-1)){
          Serial.print(inputKey[i]);
          Serial.print(", ");
        }
        else{
          Serial.println(inputKey[i]);
        }
      }

      uint8_t blankMatch=0;
      for(int i=0;i<keyCount;i++){
        for(int j=0;j<2;j++){
          if(inputKey[i][j]==' '){
            blankMatch++;
          }
        }
      }
      if(blankMatch>0){
        resetKey=false;
        Serial.println("Invalid Key Entered!");
      }
      
      if(resetKey){//Check whether to reset the existing Key or compare inputKey against existing key
        for(int i=0;i<keyCount;i++){
          for(int j=0;j<2;j++){
            recordKey[i][j]=inputKey[i][j];
          }
        }
        state=0;
        resetKey=false;
        Serial.println("Key change successful!");
        Serial.print("The new key is: ");
        for(int i=0;i<keyCount;i++){
          if(i<(keyCount-1)){
            Serial.print(recordKey[i]);
            Serial.print(", ");
          }
          else{
            Serial.println(recordKey[i]);
          }
        }
      }
      else{
        uint8_t keyMatch=0;
        for(int i=0;i<keyCount;i++){
          for(int j=0;j<2;j++){
            if(recordKey[i][j]==inputKey[i][j]){
              keyMatch++;
            }
          }
        }
        if(keyMatch==(2*keyCount)){
          state=2;
          Serial.println("Success: Key Match!");
        }
        else{
          blinkLED(100,5);
          state=0;
          Serial.println("Error: Input Key incorrect!");
        }
      }
      for(int i=0;i<keyCount;i++){
          for(int j=0;j<2;j++){
            inputKey[i][j]=blankKey[i][j];
          }
        }  
    }
      break;

    case 2:{//On State: Resource is unlocked. Holding down switch SW0 for 5 seconds will trigger a Key Reset
      digitalWrite(PIN_LED_13,LOW);
      onPrevTime=millis();
      while((!digitalRead(PIN_SW0)&&startCount)){
        onCurTime=millis();
        if((onCurTime-onPrevTime)>=5000){
          resetKey=true;
          state=1;
          Serial.println("Key Reset triggered");
          onPrevTime=onCurTime;
          while((onCurTime-onPrevTime)<3000){
            onCurTime=millis();
          }
          calculateOffset();
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
      break;

    default:
      state=0;
      break;
      
  }
}
void calculateOffset() {
  // We can call this funtion in the setup section to calculate the accelerometer offset. From here we will get the offset values used in the calculations printed on the Serial Monitor.
  // Note that we should place the IMU relatively flat in order to get the proper values
  // Read accelerometer values ("numOfSamples"<-See below) times
  unsigned char c=0;
  offsetX=0;
  offsetY=0;
  offsetZ=0;
  char numOfSamples=100;
  while (c < numOfSamples) {
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    initialX = (Wire.read() << 8 | Wire.read());
    AccX = (float)initialX / 16384.0*9.81; // X-axis value
    initialY = (Wire.read() << 8 | Wire.read()); 
    AccY = (float)initialY / 16384.0*9.81; // Y-axis value
    initialZ = (Wire.read() << 8 | Wire.read()); 
    AccZ = (float)initialZ / 16384.0*9.81; // Z-axis value
    // Sum all readings
    offsetX = offsetX + AccX;
    offsetY = offsetY + AccY;
    offsetZ = offsetZ + AccZ;
    c++;
  }
  //Divide the sum by 100 to get the error value
  offsetX = offsetX / (float)numOfSamples;
  offsetY = offsetY / (float)numOfSamples;
  offsetZ = offsetZ / (float)numOfSamples;
  if((abs(offsetZ)<abs(offsetX))||(abs(offsetZ)<abs(offsetY))){
    offsetX=0;
    offsetY=0;
    offsetZ=0;
  }

  Serial.print("offsetX: ");
  Serial.println(offsetX);
  Serial.print("offsetY: ");
  Serial.println(offsetY);
  Serial.print("offsetZ: ");
  Serial.println(offsetZ);

}

void blinkLED(unsigned short rate,unsigned char blinks){//Blinking function used as an indicator and delay
  unsigned char c=0;
  while(c<blinks){
    digitalWrite(PIN_LED_13,LOW);//Turn LED On
    previousTime=currentTime;
    while((currentTime-previousTime)<rate)currentTime=millis();

    
    digitalWrite(PIN_LED_13,HIGH);//Turn LED Off
    previousTime=currentTime;
    while((currentTime-previousTime)<rate)currentTime=millis();
    c++;
  }
  return;
}

void readAcc(bool recalibrate){//This function reads the accelerometer and uses the offset values to convert from actual acceleration to delta acceleration
  if(recalibrate==true){
    calculateOffset();
  }
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
  //For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet and multiple by 9.81 to convert to m/s2
  initialX = (Wire.read() << 8 | Wire.read());
  AccX = (float)initialX / 16384.0*9.81; // X-axis value
  initialY = (Wire.read() << 8 | Wire.read()); 
  AccY = (float)initialY / 16384.0*9.81; // Y-axis value
  initialZ = (Wire.read() << 8 | Wire.read()); 
  AccZ = (float)initialZ / 16384.0*9.81; // Z-axis value
  AccX=AccX-offsetX;
  AccY=AccY-offsetY;
  AccZ=AccZ-offsetZ;
}

signed char signOfBit(float num){//Helper function used to convert change the range of signbit() function from [0,1] to [1,-1] respectively
  signed char ret=1;
  Serial.println(num);
  if(signbit(num)){
    ret=-1;
  }
  return ret;
}
