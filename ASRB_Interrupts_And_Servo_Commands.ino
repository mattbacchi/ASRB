/* Code to Read all of the sensors, store all their values, and print the data.

Created on August 28th, 2012

Edited on October 8th, 2012

*/


/* All of the "include" statements*/

#include "pitches.h"
#include <DistanceGP2Y0A21YK.h>
#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include <Wire.h>
#include <Servo.h>

/* Include Interrupt */

#include <avr/interrupt.h>
#include <avr/io.h>

/* Interrupt Integers */

unsigned int toggle = 0;  //used to keep the state of the LED

/* IR integers  */

DistanceGP2Y0A21YK Dist01;  //defines Dist01 as the sharp GP2Y0A21YK
DistanceGP2Y0A21YK Dist02;  //defines Dist02 as the sharp GP2Y0A21YK
DistanceGP2Y0A21YK Dist03;  //defines Dist03 as the sharp GP2Y0A21YK
DistanceGP2Y0A21YK Dist04;  //defines Dist04 as the sharp GP2Y0A21YK
DistanceGP2Y0A21YK Dist05;  //defines Dist05 as the sharp GP2Y0A21YK

int Prox01;  //creates the integer for the first IR sensor
int Prox02;  //creates the integer for the second IR sensor
int Prox03;  //creates the integer for the third IR sensor
int Prox04;  //creates the integer for the fourth IR sensor
int Prox05;  //creates the integer for the fifth IR sensor

/* Button boolean */

boolean button = 7;

/* Piezo Speaker */

int melody[] = {
  NOTE_F6, 0, NOTE_F6, 0, NOTE_F6, 0, NOTE_F6, 0};  //melody to be played when backing the car up
int noteDurations[] = {
   2,6,2,6,2,6,2,6 };

/* LED Integers */

int ResetLed = 12;         //Reset LED
int ReverseLed = 11;       //Reverse LED
int AlertLed = 13;         //Alert (Backwards) LED
int InterruptLed = 53;     //Interrupt LED (temporary)

/* Motor Integers*/

Servo Steering;  //Creates the servo object named "Steering"
Servo Motor;     //Creates the servo object for the DC motor
int spd = 100;   //Sets the speed of the DC motor
int pos = 120;     //Stores the position of the Servo

/* Compass Integers */

int HMC6352SlaveAddress = 0x42;
int HMC6352ReadAddress = 0x41; //"A" in hex, A command is:
int headingValue;
float GOAL = 90; //Target of the compass (where to orient to)

/* GPS Integers */

TinyGPS gps;
#define nss Serial1

static void gpsdump(TinyGPS &gps);
static bool feedgps();
static void print_float(float val, float invalid, int len, int prec);
static void print_int(unsigned long val, unsigned long invalid, int len);
static void print_date(TinyGPS &gps);
static void print_str(const char *str, int len);

//Timer2 Overflow Interrupt Vector, called every 1ms

ISR(TIMER4_OVF_vect) {
    toggle = !toggle;    //toggles the LED state
    Serial.println("Interrupted!!");
    Motor.write(spd);
    
    if ((Prox01 <= 15) || (Prox02 <= 15))        //If the distance is less than 15 inches
  Steering.write(125);    //turn right
else if ((Prox04 <= 15) || (Prox05 <= 15))   //If the distance is less then 15 inches
  Steering.write(65);                        //turn left
else if ((Prox01 <= 20) || (Prox02 <= 20))
  Steering.write(115);
else if ((Prox04 <= 20) || (Prox05 <= 20))
  Steering.write(75);
else if ((Prox01 <= 25) || (Prox02 <= 25))
  Steering.write(105);
else if ((Prox04 <= 25) || (Prox05 <= 25))
  Steering.write(85);
else if ((Prox01 >15) && (Prox02 >15) && (Prox03 >15) && (Prox04 >15) && (Prox05 >15))
  Steering.write(95);
 else if (Prox03 <= 20)
  FrontAlert();
        

  digitalWrite(InterruptLed,toggle);
  TCNT4 = 40536;           //Reset Timer to 40536 out of 65536
  TIFR4 = 0x00;            //Timer4 INT Flag Reg: Clear Timer Overflow Flag
};


/* SETUP */


void setup()

{
  
  Serial.begin(9600);
  Serial.println("Begin ASRB code with Interrupts");
  
  /* LED's & button */
  
  pinMode(button, INPUT);
  pinMode(InterruptLed, OUTPUT);
  digitalWrite(InterruptLed, HIGH);
  delay(500);
  pinMode(ReverseLed, OUTPUT);
  digitalWrite(ReverseLed, HIGH);
  delay(500);
  pinMode(AlertLed,OUTPUT);
  digitalWrite(AlertLed, HIGH);
  delay(500);
  pinMode(ResetLed, OUTPUT);
  digitalWrite(ResetLed, HIGH);
  delay(500);
  digitalWrite(InterruptLed, LOW);
  digitalWrite(ReverseLed, LOW);
  digitalWrite(AlertLed, LOW);
  digitalWrite(ResetLed, LOW);
  

  //Setup Timer4 to fire every 1ms
  TCCR4B = 0x00;        //Disbale Timer4 while we set it up
  TCNT4  = 40536;       //Reset Timer Count to 40,536 out of 65,536
  TIFR4  = 0x00;        //Timer4 INT Flag Reg: Clear Timer Overflow Flag
  TIMSK4 = 0x01;        //Timer4 INT Reg: Timer2 Overflow Interrupt Enable
  TCCR4A = 0x00;        //Timer4 Control Reg A: Normal port operation, Wave Gen Mode normal
  TCCR4B = 0x03;        //Timer4 Control Reg B: Timer Prescaler set to 64
  
  /* Attaching the Motor */
  
  Steering.attach(10);   //Attaches the Steering servo to pin 10
  Motor.attach(9);       //Attaches the motor to PWM pin 9
  
  /* IR Sensors */
  
  Dist01.begin(A0);    //tells program "Dist01" will be used by the serial monitor
  Dist02.begin(A1);    //tells program "Dist02" will be used by the serial monitor
  Dist03.begin(A2);    //tells program "Dist03" will be used by the serial monitor
  Dist04.begin(A3);    //tells program "Dist04" will be used by the serial monitor
  Dist05.begin(A4);    //tells program "Dist05" will be used by the serial monitor
  
  /* Compass */
  
  // "The Wire library uses 7 bit addresses throughout. 
  //If you have a datasheet or sample code that uses 8 bit address, 
  //you'll want to drop the low bit (i.e. shift the value one bit to the right), 
  //yielding an address between 0 and 127."
  HMC6352SlaveAddress = HMC6352SlaveAddress >> 1; // I know 0x42 is less than 127, but this is still required
  Wire.begin();
  
  /* GPS */
  
  nss.begin(57600);

/*  
  Serial.print("Testing TinyGPS library v. "); Serial.println(TinyGPS::library_version());
  Serial.println("by Mikal Hart");
  Serial.println();
  Serial.print("Sizeof(gpsobject) = "); Serial.println(sizeof(TinyGPS));
  Serial.println();
  Serial.println("Sats HDOP Latitude Longitude Fix  Date       Time       Date Alt     Course Speed Card  Distance Course Card  Chars Sentences Checksum");
  Serial.println("          (deg)    (deg)     Age                        Age  (m)     --- from GPS ----  ---- to London  ----  RX    RX        Fail");
  Serial.println("--------------------------------------------------------------------------------------------------------------------------------------");
*/

}

void loop()
{
  /* Check status of the button */
  
  if (button = true)
    BackUpNoise();
  
  /* Get Data From The IR Sensors */
  
  Prox01 = Dist01.getDistanceCentimeter();  //Obtains and converts the voltage from the sensor to Inches (Dist01)
  Prox02 = Dist02.getDistanceCentimeter();  //Obtains and converts the voltage from the sensor to Inches (Dist02)
  Prox03 = Dist03.getDistanceCentimeter();  //Obtains and converts the voltage from the sensor to Inches (Dist03)
  Prox04 = Dist04.getDistanceCentimeter();  //Obtains and converts the voltage from the sensor to Inches (Dist04)
  Prox05 = Dist05.getDistanceCentimeter();  //Obtains and converts the voltage from the sensor to Inches (Dist05)
  
  /* Get Data From The Compass */
  
  Wire.beginTransmission(HMC6352SlaveAddress);
  Wire.write(HMC6352ReadAddress);              // The "Get Data" command
  Wire.endTransmission();

  //time delays required by HMC6352 upon receipt of the command
  //Get Data. Compensate and Calculate New Heading : 6ms

  delay(6);
  Wire.requestFrom(HMC6352SlaveAddress, 2); //get the two data bytes, MSB and LSB

  //"The heading output data will be the value in tenths of degrees
  //from zero to 3599 and provided in binary format over the two bytes."
  
  byte MSB = Wire.read();
  byte LSB = Wire.read();
  float headingSum = (MSB << 8) + LSB; //(MSB / LSB sum)
  float headingInt = headingSum / 10; 
  float OrientationDiff = GOAL - headingInt;
  //creating the "float" integer OrientationDiff as the target minus the current orientaion

  /* Get GPS Data */

  bool newdata = false;
  unsigned long start = millis();
  
  // Every second we print an update
  while (millis() - start < 1000)
  {
    if (feedgps())
      newdata = true;
  }
  
  gpsdump(gps);
  
  /* Printing Data */

/*
  Serial.print(" ------");
  Serial.print(Prox01);
  Serial.print("--------");
  Serial.print(Prox02);
  Serial.print("--------");
  Serial.print(Prox03);
  Serial.print("--------");
  Serial.print(Prox04);
  Serial.print("--------");
  Serial.print(Prox05);
  Serial.println("------");
  Serial.print(OrientationDiff);
  Serial.println(" degrees");
*/

}


/* Back Up Sound Method */

void BackUpNoise() {                                   //Makes the "Backing Up" noise
  for (int thisNote = 0; thisNote < 9; thisNote++) {
    int noteDuration = 1000/noteDurations[thisNote];
    tone(8, melody[thisNote], noteDuration);
    int pauseBetweenNotes = noteDuration * 1.30;
    delay(pauseBetweenNotes);
    noTone(8);
  }
}

/* Reverse Method */

void REVERSE() {                                        //Throws the car in reverse and turns left
  Motor.write(140);
  digitalWrite(ReverseLed, HIGH);
  delay(500);
  Steering.write(95);
  Motor.write(130);
  delay(500);
  Motor.write(110);
  delay(500);
  Motor.write(115);
  BackUpNoise();
  Motor.write(110);
  Steering.write(120);
  delay(1500);
  Steering.write(60);
  Motor.write(100);
  delay(500);
  Motor.write(spd);
  digitalWrite(ReverseLed, LOW);
  
}

/* LED Alert in Front Of Car */

void FrontAlert() {                                     //Warns that the car senses something in front of it
  Steering.write(95);
  digitalWrite(AlertLed, HIGH);
  delay(100);
  digitalWrite(AlertLed, LOW);
  delay(100);
  digitalWrite(AlertLed, HIGH);
  delay(100);
  digitalWrite(AlertLed, LOW);
  delay(100);
  digitalWrite(AlertLed, HIGH);
  delay(100);
  digitalWrite(AlertLed, LOW);
  delay(100);
  REVERSE();
}


/* GPS Extras */

static void gpsdump(TinyGPS &gps)
{
  float flat, flon;
  unsigned long age, date, time, chars = 0;
  unsigned short sentences = 0, failed = 0;
  static const float LONDON_LAT = 51.508131, LONDON_LON = -0.128002;
  
  print_int(gps.satellites(), TinyGPS::GPS_INVALID_SATELLITES, 5);
  print_int(gps.hdop(), TinyGPS::GPS_INVALID_HDOP, 5);
  gps.f_get_position(&flat, &flon, &age);
  print_float(flat, TinyGPS::GPS_INVALID_F_ANGLE, 9, 5);
  print_float(flon, TinyGPS::GPS_INVALID_F_ANGLE, 10, 5);
  print_int(age, TinyGPS::GPS_INVALID_AGE, 5);

  print_date(gps);

  print_float(gps.f_altitude(), TinyGPS::GPS_INVALID_F_ALTITUDE, 8, 2);
  print_float(gps.f_course(), TinyGPS::GPS_INVALID_F_ANGLE, 7, 2);
  print_float(gps.f_speed_kmph(), TinyGPS::GPS_INVALID_F_SPEED, 6, 2);
  print_str(gps.f_course() == TinyGPS::GPS_INVALID_F_ANGLE ? "*** " : TinyGPS::cardinal(gps.f_course()), 6);
  print_int(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0UL : (unsigned long)TinyGPS::distance_between(flat, flon, LONDON_LAT, LONDON_LON) / 1000, 0xFFFFFFFF, 9);
  print_float(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : TinyGPS::course_to(flat, flon, 51.508131, -0.128002), TinyGPS::GPS_INVALID_F_ANGLE, 7, 2);
  print_str(flat == TinyGPS::GPS_INVALID_F_ANGLE ? "*** " : TinyGPS::cardinal(TinyGPS::course_to(flat, flon, LONDON_LAT, LONDON_LON)), 6);

  gps.stats(&chars, &sentences, &failed);
  print_int(chars, 0xFFFFFFFF, 6);
  print_int(sentences, 0xFFFFFFFF, 10);
  print_int(failed, 0xFFFFFFFF, 9);
//  Serial.println();
}

static void print_int(unsigned long val, unsigned long invalid, int len)
{
  char sz[32];
  if (val == invalid)
    strcpy(sz, "*******");
  else
    sprintf(sz, "%ld", val);
  sz[len] = 0;
  for (int i=strlen(sz); i<len; ++i)
    sz[i] = ' ';
  if (len > 0) 
    sz[len-1] = ' ';
  //Serial.print(sz);
  feedgps();
}

static void print_float(float val, float invalid, int len, int prec)
{
  char sz[32];
  if (val == invalid)
  {
    strcpy(sz, "*******");
    sz[len] = 0;
        if (len > 0) 
          sz[len-1] = ' ';
    for (int i=7; i<len; ++i)
        sz[i] = ' ';
    //Serial.print(sz);
  }
  else
  {
    //Serial.print(val, prec);
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1);
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
    //for (int i=flen; i<len; ++i)
    //  Serial.print(" ");
  }
  feedgps();
}

static void print_date(TinyGPS &gps)
{
  int year;
  byte month, day, hour, minute, second, hundredths;
  unsigned long age;
  gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &age);
/*  if (age == TinyGPS::GPS_INVALID_AGE)
    Serial.print("*******    *******    ");
  else
  {
    char sz[32];
    sprintf(sz, "%02d/%02d/%02d %02d:%02d:%02d   ",
        month, day, year, hour, minute, second);
    Serial.print(sz);
  }*/
  print_int(age, TinyGPS::GPS_INVALID_AGE, 5);
  feedgps();
}

static void print_str(const char *str, int len)
{
  int slen = strlen(str);
  for (int i=0; i<len; ++i)
    Serial.print(i<slen ? str[i] : ' ');
  feedgps();
}

static bool feedgps()
{
  while (nss.available())
  {
    if (gps.encode(nss.read()))
      return true;
  }
  return false;
}
