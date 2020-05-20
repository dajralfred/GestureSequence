/*
 * The on-board LED will be used as a status indicator
 * Initially, if no recordKey is saved, pressing the SW0 will set the LED high immediately. 
 * A 1 Hz blinking light inidcates gesture mode is active.It will remain active until timeout of 5 secs after calibration is complete
 *  User should hold their hand steady and Accelerometer should remain relatively flat(the largest portion of graviational force should be in Z direction)
 *  Large movements during calibration will lead to Gesture mode being exited and the MCU will return to an idle state
 * A 5 Hz sequence of 2 LED blinks indicates when a single gesture is recorded. 
 *  A gesture consists of a movement away from the origin and a return movement to the origin with roughly the same force.
 * A solid light indicates that the sequence of gestures is correct/ verified
 * Holding SW0 for 5 secs after verification(while LED light is solid)  will enter Gesture mode to reset the recordKey
 * A 4 Hz sequence of 5 LED blinks and a return of the LED to the off state inidicates that the entered sequence is wrong.
 * This code will read take an accelerometer reading when the botton is pressed
 */
#include <Wire.h>
const int MPU = 0x68; // MPU6050 I2C address
signed short initialX,initialY,initialZ;
float AccX, AccY, AccZ;
float offsetX,offsetY,offsetZ;
float AccErrorX, AccErrorY;
float elapsedTime, currentTime, previousTime;
float mvtThresholdA=2.5, mvtThresholdB=1.5;
float gestThreshold=0.5;
unsigned long returnTimeout=5000;
unsigned char state=1; //0:Idle/Polling , 1:Gesture mode , 2:On/Reset

void setup() {
  pinMode(PIN_LED_13,OUTPUT);
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
  // Configure Gyro Sensitivity - Full Scale Range (default +/- 250deg/s)
  Wire.beginTransmission(MPU);
  Wire.write(0x1B);                   // Talk to the GYRO_CONFIG register (1B hex)
  Wire.write(0x10);                   // Set the register bits as 00010000 (1000deg/s full scale)
  Wire.endTransmission(true);
  delay(20);
  */

  calculateOffset();
}
void loop() {
  // === Read acceleromter data === //
  switch(state){
    case 0:
      //state=1;
      break;

    case 1:
      digitalWrite(PIN_LED_13,HIGH);
      if(offsetX==0&&offsetY==0&&offsetZ==0){//Check if calibration was successful. Fails if accelerometer not held relativelt flat.
        blinkLED(250,5);
        Serial.println("Here!");
        state=0;
        break;
      }
      
      readAcc(false);//Read accelerometer
      if((abs(AccX)>mvtThresholdA)&&(abs(AccY)<mvtThresholdB)&&(abs(AccZ)<mvtThresholdB)){//Detect linear movement along X-axis
        unsigned long int countOver=0;
        float cumsumX=0;
        //readAcc(false);
        signed char sign=signOfBit(AccX);
        Serial.print("Sign of Movement: ");
        Serial.println((int)sign);
        if(sign>0){
          unsigned long int mvtPrevTime=millis();
          while(AccX>0){
            unsigned long int mvtCurTime=millis();
            if((mvtCurTime-mvtPrevTime)>=1){
              cumsumX+=AccX;
              countOver++;
              readAcc(false);
              mvtPrevTime=mvtCurTime;
            }          
          }
        }
        else{
          unsigned long int mvtPrevTime=millis();
          while(AccX<0){
            unsigned long int mvtCurTime=millis();
            if((mvtCurTime-mvtPrevTime)>=1){
              cumsumX+=AccX;
              countOver++;
              readAcc(false);
              mvtPrevTime=mvtCurTime;
            }          
          }
        }
        
        float avgX=0;
        avgX=cumsumX/(float)countOver;
        
        if(sign>0){
          bool timeoutFlag=false;
          unsigned long int gesPrevTime=millis();
          while(AccX>(-1.0*mvtThresholdA)){
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
          while(AccX<mvtThresholdA){
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
        if(sign>0){
          unsigned long int mvtPrevTime=millis();
          while(AccX>0){
            unsigned long int mvtCurTime=millis();
            if((mvtCurTime-mvtPrevTime)>=1){
              cumsumX2+=AccX;
              countOver2++;
              readAcc(false);
              mvtPrevTime=mvtCurTime;
            }          
          }
        }
        else{
          unsigned long int mvtPrevTime=millis();
          while(AccX<0){
            unsigned long int mvtCurTime=millis();
            if((mvtCurTime-mvtPrevTime)>=1){
              cumsumX2+=AccX;
              countOver2++;
              readAcc(false);
              mvtPrevTime=mvtCurTime;
            }          
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
          break;
        }
        else{
          Serial.print("X");
          Serial.print((char)(44+sign));
          Serial.println(" Gesture Recorded");
          blinkLED(500,3);
          //state=2;
        }
        calculateOffset();
      }
      /*currentTime = millis();            // Current time actual time read
      //elapsedTime = (currentTime - previousTime) / 1000; // Divide by 1000 to get seconds
      if(elapsedTime>=0.25){
        Serial.print(AccX);
        Serial.print("/");
        Serial.print(AccZ);
        Serial.print("/");
        Serial.println(AccY);
        previousTime = currentTime; 
      }*/
      break;

    case 2:
      digitalWrite(PIN_LED_13,LOW);
  }
}
void calculateOffset() {
  // We can call this funtion in the setup section to calculate the accelerometer and gyro data error. From here we will get the error values used in the above equations printed on the Serial Monitor.
  // Note that we should place the IMU flat in order to get the proper values, so that we then can the correct values
  // Read accelerometer values 200 times
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
  //Divide the sum by 200 to get the error value
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

void blinkLED(unsigned short rate,unsigned char blinks){
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

void readAcc(bool recalibrate){
  if(recalibrate==true){
    calculateOffset();
  }
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
  //For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
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

signed char signOfBit(float num){
  signed char ret=1;
  Serial.println(num);
  if(signbit(num)){
    ret=-1;
  }
  return ret;
}
