/*
 * The on-board LED will be used as a status indicator
 * Initially, if no recordKey is saved, pressing the SW0 will set the LED high immediately. 
 * A 1 Hz blinking light inidcates gesture mode is active.It will remain active until timeout of 5 secs after calibration is complete
 * A 2 Hz blinking light means that gesture mode is calibrating. 
 *  User should hold their hand steady and Accelerometer should remain relatively flat(the largest portion of graviational force should be in Z direction)
 *  Large movements during calibration will lead to Gesture mode being exited and the MCU will return to an idle state
 * A 5 Hz sequence of 2 LED blinks indicates when a single gesture is recorded. 
 *  A gesture consists of a movement away from the origin and a return movement to the origin with roughly the same force.
 * A solid light indicates that the sequence of gestures is correct/ verified
 * Holding SW0 for 5 secs after verification(while LED light is solid)  will enter Gesture mode to reset the recordKey
 * A 5 Hz sequence of 5 LED blinks and a return of the LED to the off state inidicates that the entered sequence is wrong.
 * This code will read take an accelerometer reading when the botton is pressed
 */
#include <Wire.h>
const int MPU = 0x68; // MPU6050 I2C address
signed short initialX,initialY,initialZ;
float AccX, AccY, AccZ;
float AccErrorX, AccErrorY;
float elapsedTime, currentTime, previousTime;
int c = 0;
void setup() {
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
  // Call this function if you need to get the IMU error values for your module
  //calculate_IMU_error();
  delay(20);
}
void loop() {
  // === Read acceleromter data === //
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

  currentTime = millis();            // Current time actual time read
  elapsedTime = (currentTime - previousTime) / 1000; // Divide by 1000 to get seconds
  if(elapsedTime>=0.25){
    Serial.print(AccX);
    Serial.print("/");
    Serial.print(AccZ);
    Serial.print("/");
    Serial.println(AccY);
    previousTime = currentTime; 
  }
}
void calculate_IMU_error() {
  // We can call this funtion in the setup section to calculate the accelerometer and gyro data error. From here we will get the error values used in the above equations printed on the Serial Monitor.
  // Note that we should place the IMU flat in order to get the proper values, so that we then can the correct values
  // Read accelerometer values 200 times
  while (c < 200) {
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    AccX = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccY = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    // Sum all readings
    AccErrorX = AccErrorX + ((atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI));
    AccErrorY = AccErrorY + ((atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / PI));
    c++;
  }
  //Divide the sum by 200 to get the error value
  AccErrorX = AccErrorX / 200;
  AccErrorY = AccErrorY / 200;
  c = 0;

  Serial.print("AccErrorX: ");
  Serial.println(AccErrorX);
  Serial.print("AccErrorY: ");
  Serial.println(AccErrorY);

}
