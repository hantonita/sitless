/**************************************************************************/
/*!
 Hanna Senior Project 2015
 Coding for SitLess device -  a wearable device that records how long a person 
 sits throughout a day and, when necessary, reminds a person to stand up. By using 
 an acceleration sensor, a microcontroller and an SD storage card, the device 
 records changes between sitting and standing and, if desired, provides discreet 
 feedback to the user through a small vibration motor.
 
 Device is based on Arduino platform using a 3V Adafruit Trinket Pro with a LiPo 
 backpack. Acceleration is measured through an MMA8451Q sensor connected on the I2C 
 bus pins A4 and A5. For time keeping, a Real Time Clock chip DS 3231 is used
 which is also connected to the I2C bus. Data is logged to an SD card, connected 
 to the SPI bus, pins D10 through D13. User feedback is provided through two LEDs 
 and a vibration motor driven by MOSFETs connected to D4, D8, and D6 respectively. 
 Battery voltage is measured through analog pin A0. 
 
 This sketch is using two libraries from Adafruit to read the acceleration sensor 
 Adafruit_MMA8451.h 
 Adafruit_Sensor.h found at https://github.com/adafruit/Adafruit_MMA8451_Library
 and a library for reading the RTC chip from Jack Christensen
 DS3232RTC.h found at https://github.com/JChristensen/DS3232RTC
 */
/**************************************************************************/
//New value for theta threshold

#include <Wire.h>                  //required library for I2C
#include <Adafruit_MMA8451.h>      //required library for accelerometer
#include <Adafruit_Sensor.h>       //required library for accelerometer
#include <SPI.h>                   //required library for SPI
#include <SD.h>                    //required library for SD card
#include <DS3232RTC.h>             //required library for Real Time Clock
#include <Time.h>

//declare sensor objects
Adafruit_MMA8451 mma = Adafruit_MMA8451();

//define hardware pins
const int chipSelect = 10;          
const int voltagePin = A0;
const int orangeLed = 8;
const int greenLed = 4;
const int motor = 6;

//Define battery voltage thresholds
// -------------------------------------------  Voltage is approx. analogRead * 6.416 mV
const int lowBattery = 585;                    // 3.75V
const int minimumVoltage = 580;                // 3.72V
const int chargingVoltage = 450;               // 2.89V
const int hysteresisVoltage = 2;
int checkVoltage;

//Motion data calculation variables
//Theta = angle of device relative to the gravitational force of the earth
const double thetaThreshold = -0.6;            //threshold determining when person is sitting or standing (angle at which switch occurs)
double thetaMovingAvg;                         //exponential moving average of theta
const double movingAvgRatio = 0.1;             //weight of current measurement
int intervalCount = 0;                             
const int averagingInterval = 10;              //
double accelTotal = 0.0;
double priorAccelTotal = 0.0;
double accelIntegral = 0.0;
const double thetaMin = -2.2;                   // threshold for "impossible angles"
const double thetaMax = 0.5;                    // threshold for "impossible angles"


const unsigned long intLength = 100; 
unsigned long nextInt = 0;
const int flashInterval = 5;
int countFlash = 0;
boolean wasSitting = false;
boolean isSitting = false;

// warning interval variables
int creditMax;
int creditDec;
int creditInc;
int creditSec;
boolean isBuzzing = false;
unsigned long lastBuzz = 0;
unsigned long intervalBuzz;
boolean useBuzzer;
boolean secondBuzz;

File dataFile;





void setup(void) {

  //initialize hardware pins and turn off LEDs and motor
  pinMode(greenLed, OUTPUT); 
  digitalWrite(greenLed, LOW);
  pinMode(orangeLed, OUTPUT);
  digitalWrite(orangeLed, LOW);
  pinMode (motor, OUTPUT);
  digitalWrite(motor, LOW);

  //initialize serial communications (for debugging) and I2C
  Serial.begin(57600);
  Serial.println(__FILE__);
  Serial.println("Hanna Senior Project");
  Wire.begin();

  //make sure battery is full and device is not just charging and initialize accelerometer
  //Note: arduino can still be running when accelerometer is not supplied with power (see circuit diagram). This would hang the program.
  checkVoltage = checkBattery();

  //initialize rtc communication and check whether clock had power loss
  setSyncProvider(RTC.get);
  if(timeStatus() != timeSet || year() < 2015)  {
    Serial.println("Time not set!");
    abortProg(2);
  }


  // start SD card communication and open data file and diagnostics file
  if (!SD.begin(chipSelect)) {
    Serial.println("SD card failed.");
    abortProg(1);
  }

  dataFile = SD.open("config.txt", FILE_READ);
  if(!dataFile){
    Serial.println("config file error.");
    abortProg(4);
  }
  else{
    useBuzzer = dataFile.parseInt() > 0;
    intervalBuzz = long( dataFile.parseInt() ) * 1000;
    creditMax = dataFile.parseInt();
    creditInc = dataFile.parseInt();
    creditDec = dataFile.parseInt();
    dataFile.close();
    blinking(1);
  }
  creditSec = creditMax;

  dataFile = SD.open(timeFileName(), FILE_WRITE);
  if( ! dataFile ){
    Serial.println("File open failed.");
    abortProg(5);
  }

  initializeAccelerometer();  

  diagMsg("Program started");
}




void loop() {
  double theta = 0;

  //check battery voltage while waiting for next measurement interval  
  while(millis() < nextInt){
    checkVoltage = checkBattery();
  }
  nextInt = millis() + intLength;
  intervalCount++; 

  if(checkVoltage < 0){
    digitalWrite(orangeLed, HIGH);
    digitalWrite(greenLed, HIGH);
    delay(2);
  }
  
  //turn LEDs back off
  digitalWrite(orangeLed, LOW);
  digitalWrite(greenLed, LOW);

  //determine orientation of device and measure movement activity
    mma.read();
    theta = atan2(double(mma.z), double(mma.y));
    thetaMovingAvg = movingAvgRatio * theta  + (1-movingAvgRatio) * thetaMovingAvg;
    wasSitting = isSitting;
    isSitting = thetaMovingAvg < thetaThreshold;
    priorAccelTotal = accelTotal;
    accelTotal = sqrt(double(mma.x) * double(mma.x) + double(mma.y) * double(mma.y) + double(mma.z) * double(mma.z));
    accelIntegral += abs(accelTotal - priorAccelTotal);
  

  if( thetaMovingAvg > thetaMax || thetaMovingAvg < thetaMin ) {           // stop to buzz if an impossible angle is detected
    digitalWrite(motor, HIGH);
    delay(250);
    digitalWrite(motor, LOW);
    delay(250);
  }


  // flash LED when changing positions
  if(wasSitting != isSitting){
    if(isSitting){
      digitalWrite (orangeLed, HIGH);
    }
    else{
      digitalWrite (greenLed, HIGH);
    }
    countFlash = 0;                      //resetting flash interval to make sure next flash comes only after full interval
  }

  //write data to SD card and serial monitor at certain interval 
  if(intervalCount >= averagingInterval){ 
    digitalWrite(motor, LOW);
    isBuzzing = false;
    //adjust credit seconds user has depending on whether they are sitting or standing
    if (useBuzzer){
      if (isSitting){
        creditSec -= creditDec;
        if (creditSec < 0){
          creditSec = 0;
          if (millis() >= lastBuzz + intervalBuzz){
            isBuzzing = true;
            lastBuzz = millis();
            secondBuzz = true;
          }
        }
      }
      else{
        creditSec += creditInc;
        secondBuzz = false;
        if(creditSec > creditMax){
          creditSec = creditMax;
        }
      }
    }

    Serial.print("data");
    Serial.print("\t");
    Serial.print(theta);
    Serial.print("\t");
    Serial.print(thetaMovingAvg);
    Serial.print("\t");
    Serial.print(accelIntegral);
    Serial.print("\t");
    Serial.print(int(isSitting));
    Serial.print("\t");    
    Serial.print(checkVoltage);
    Serial.print("\t");        
    Serial.print(creditSec);
    Serial.println();

    dataFile.print("data");
    dataFile.print("\t");
    dataFile.print(timeStamp());
    dataFile.print("\t");
    dataFile.print("\t"); 
    dataFile.print(mma.x); 
    dataFile.print("\t"); 
    dataFile.print(mma.y); 
    dataFile.print("\t"); 
    dataFile.print(mma.z);
    dataFile.print("\t"); 
    dataFile.print(thetaMovingAvg);
    dataFile.print("\t");
    dataFile.print(accelIntegral);
    dataFile.print("\t");
    dataFile.print(int(isSitting));
    dataFile.print("\t");    
    dataFile.print(checkVoltage);
    dataFile.print("\t");        
    dataFile.print(creditSec);
    dataFile.print("\t");        
    dataFile.print(int(isBuzzing));    
    dataFile.println();
    dataFile.flush();
  
  if(isBuzzing){
  for(int i = 0; i < 5; i++){
    digitalWrite(motor, HIGH);
    delay(100);               
    digitalWrite(motor, LOW);
    delay(100);
  }
  isBuzzing = false;
  } 
    //signal device's assumed user position 
    if(countFlash >= flashInterval){
      if(isSitting){
        digitalWrite (orangeLed, HIGH);
      }
      else{
        digitalWrite (greenLed, HIGH);
      }
      countFlash = 0;
    }

    delay(1);
    digitalWrite(orangeLed, LOW);
    digitalWrite(greenLed, LOW);

    intervalCount = 0;
    accelIntegral = 0;
    countFlash++;
  }




}

// ------------------------------------------------------------------------------------------------------------------------------------------
// pad an integer with a "0" if smaller than 10
String twoDigit(int number) {
  if( number > 9 ) return String(number);
  return "0" + String(number);
}
// ------------------------------------------------------------------------------------------------------------------------------------------
// create a string made from today's data and the time right now
char* timeFileName() {
  char fileName[13];
  (twoDigit(year()) + twoDigit(month()) + twoDigit(day()) + ".txt").toCharArray(fileName,13);
  return fileName;
}
// ------------------------------------------------------------------------------------------------------------------------------------------
// create a neat time stamp using rtc data
String timeStamp() {
  return(twoDigit(month()) + '/' + twoDigit(day()) + ' ' + twoDigit(hour()) + ':' + twoDigit(minute()) + ':' + twoDigit(second()));
}

//-------------------------------------------------------------------------------------------------------------------------------------------
// write a diagnostics message to file
void diagMsg(String message){
  Serial.print("DIAG");
  Serial.print("\t");
  Serial.print(timeStamp());
  Serial.print("\t");
  Serial.println(message);

  dataFile.print("DIAG");
  dataFile.print("\t");
  dataFile.print(timeStamp());
  dataFile.print("\t");
  dataFile.println(message);
  dataFile.flush();
}
//-------------------------------------------------------------------------------------------------------------------------------------------
// catch situations where program should not continue, and signal issue to user
void abortProg(int error){
  while (true){
    blinking(error);
  }
}
//--------------------------------------------------------------------------------------------------------------------------------------------
// check battery voltage and hold program if voltage is too low or device is turned off but being charged (arduino runs but accelerometer does not)
int checkBattery(){
  int currentVoltage = analogRead(voltagePin);

  if (currentVoltage <= minimumVoltage && currentVoltage > chargingVoltage){
    diagMsg("Getting tired...");
    delay(1000);
    currentVoltage = analogRead(voltagePin);
    if (currentVoltage > chargingVoltage){
      diagMsg("Going to sleep...");
      diagMsg(String(currentVoltage));
      digitalWrite(orangeLed, LOW);
      digitalWrite(greenLed, LOW);
      digitalWrite(motor, LOW);
      while (analogRead(voltagePin) <= minimumVoltage + hysteresisVoltage){
      } 
      currentVoltage = analogRead(voltagePin);
      initializeAccelerometer();
      diagMsg("Waking up."); 
    }
  }

  // checking for charging condition
  if (currentVoltage <= chargingVoltage){
    diagMsg("Charging started...");
    diagMsg(String(currentVoltage));
    digitalWrite(orangeLed, HIGH);
    digitalWrite(greenLed, LOW);
    digitalWrite(motor, LOW);
    while (analogRead(voltagePin) <= chargingVoltage){
    }
    currentVoltage = analogRead(voltagePin);
    initializeAccelerometer();
    diagMsg("Charging stopped.");
  }    


  if (currentVoltage <= lowBattery){
    return currentVoltage * (-1);
  }
  return currentVoltage;
}
//------------------------------------------------------------------------------------------------------------------------------------------------
// initialize accelerometer chip 
void initializeAccelerometer(){
  if (! mma.begin()) {
    diagMsg("Couldnt start accelerometer");
    abortProg(3);
  }
  mma.setRange(MMA8451_RANGE_2_G);
}
//------------------------------------------------------------------------------------------------------------------------------------------------
//flash LEDs "blinks" times
void blinking(int blinks){
  for (int i = 0; i < blinks; i++){
    digitalWrite(orangeLed, HIGH);
    digitalWrite(greenLed, HIGH);
    delay(100);
    digitalWrite(orangeLed, LOW);
    digitalWrite(greenLed, LOW);
    delay (200);
  }
  delay(1000);

}
