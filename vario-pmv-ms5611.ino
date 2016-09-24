// this is the "poor mans vario" version -- http://www.modelbouwforum.nl/threads/poor-mans-vario.199904/
//**********************************
//*   MS5611 sensor version    *
//**********************************
// This is the Poor Mans Vario code for the MS5611 sensor
// All code by Rolf R Bakke, Oct 2012
// Modified by Hans Meijdam, June 2013. added  altitude feature
// Modified by Hans Meijdam, November 2013.
//   Led on Arduino is in sync with audio
//   Bug fixed that now "Vario Mode" is default mode if no servo channel PWM data is present on pin D3
//   Improved documentation here and there

#include <Wire.h>
const byte led = 13;
unsigned int calibrationData[7];
unsigned long time = 0;
float toneFreq, toneFreqLowpass, pressure, lowpassFast, lowpassSlow;
float p0; // this will be used to store the airfield elevation pressure
int altitude;
int ch1; // Here's where we'll keep our channel values
int ddsAcc;


void setup()
{
  Wire.begin();
  Serial.begin(9600); // in case you want to use the included serial.print debugging commands that are currently commented out
  delay(200);
  setupSensor();
  for (int p0=0; p0 <= 100; p0++){
    pressure = getPressure(); // warming up the sensor for ground level setting
  }
  p0 = getPressure(); // Setting the ground level pressure
  lowpassFast = lowpassSlow = pressure;
  pinMode(3, INPUT); // Set our input pins as such for altitude command input from receiver via pin D3
}


void loop()
{
  pressure = getPressure();
  //  Serial.print(p0);
  //  Serial.print(" ");
  //  Serial.print(pressure);
  altitude = (float)44330 * (1 - pow(((float) pressure/p0), 0.190295));
  //  Serial.print(" ");
  //  Serial.println(altitude);
  lowpassFast = lowpassFast + (pressure - lowpassFast) * 0.1;
  lowpassSlow = lowpassSlow + (pressure - lowpassSlow) * 0.05;
  toneFreq = (lowpassSlow - lowpassFast) * 50;
  toneFreqLowpass = toneFreqLowpass + (toneFreq - toneFreqLowpass) * 0.1;
  toneFreq = constrain(toneFreqLowpass, -500, 500);
  ddsAcc += toneFreq * 100 + 2000;
  if (toneFreq < 0 || ddsAcc > 0)
  {
    tone(2, toneFreq + 510);
    ledOn();  // the Arduino led will blink if the Vario plays a tone, so you can test without having audio connected
  }
  else
  {
    noTone(2);
    ledOff();
  }
  int ones = altitude%10;
  int tens = (altitude/10)%10;
  int hundreds = (altitude/100)%10;
  int thousands = (altitude/1000)%10;
  //  Serial.print ("thousands: ");
  //  Serial.println   (thousands);
  //  Serial.print ("hundreds:  ");
  //  Serial.println   (hundreds);
  //  Serial.print ("tens:      ");
  //  Serial.println   (tens);
  //  Serial.print ("ones:      ");
  //  Serial.println   (ones);
  while (millis() < time);        //loop frequency timer
  time += 20;

  ch1 = pulseIn(3, HIGH, 25000); // Read the pulse width of servo signal connected to pin D3
  //  if(ch1>1000){
  //    Serial.println("Left Switch: Engaged");
  //  }
  //  if(ch1<1000){
  //    Serial.println("Left Switch: Disengaged");
  //  }
  if((map(ch1, 1000,2000,-500,500)) > 0) // interpret the servo channel pulse, if the Vario should beep altitude or send vario sound
  {
    noTone(2); // create 750 ms of silence, or you won't hear the first altitude beep
    ledOff();
    delay(750);
    if(hundreds == 0)
    {
      tone(2,900);                //long duration tone if the number is zero
      ledOn();
      delay(600);
      noTone(2);
      ledOff();
    }
    else
      for(char a = 0; a < hundreds; a++)          //this loop makes a beep for each hundred meters altitude
      {
        tone(2,900); // 900 Hz tone frequency for the hundreds
        ledOn();
        delay(200);
        noTone(2);
        ledOff();
        delay(200);
      }
    delay(750);                            //longer delay between hundreds and tens

    if(tens == 0)
    {
      tone(2,1100);                //long pulse if the number is zero
      ledOn();
      delay(600);
      noTone(2);
      ledOff();
    }
    else
      for(char a = 0; a < tens; a++)          //this loop makes a beep for each ten meters altitude
      {
        tone(2,1100); //1100 Hz tone frequency for the tens
        ledOn();
        delay(200);
        noTone(2);
        ledOff();
        delay(200);
      }

    for (int p0=0; p0 <= 40; p0++)
    {
      pressure = getPressure(); // warming up the sensor again, by reading it 40 times
    }
  }
}


long getPressure()
{
  long D1, D2, dT, P;
  float TEMP;
  int64_t OFF, SENS;

  D1 = getData(0x48, 10);
  D2 = getData(0x50, 1);

  dT = D2 - ((long)calibrationData[5] << 8);
  TEMP = (2000 + (((int64_t)dT * (int64_t)calibrationData[6]) >> 23)) / (float)100;
  OFF = ((unsigned long)calibrationData[2] << 16) + (((int64_t)calibrationData[4] * dT) >> 7);
  SENS = ((unsigned long)calibrationData[1] << 15) + (((int64_t)calibrationData[3] * dT) >> 8);
  P = (((D1 * SENS) >> 21) - OFF) >> 15;

  //  Serial.println(TEMP);
  //Serial.println(P);

  return P;
}


long getData(byte command, byte del)
{
  long result = 0;
  twiSendCommand(0x77, command);
  delay(del);
  twiSendCommand(0x77, 0x00);
  Wire.requestFrom(0x77, 3);
  if(Wire.available()!=3) Serial.println("Error: raw data not available");
  for (int i = 0; i <= 2; i++)
  {
    result = (result<<8) | Wire.read();
  }
  return result;
}


void setupSensor()
{
  twiSendCommand(0x77, 0x1e);
  delay(100);

  for (byte i = 1; i <=6; i++)
  {
    unsigned int low, high;

    twiSendCommand(0x77, 0xa0 + i * 2);
    Wire.requestFrom(0x77, 2);
    if(Wire.available()!=2) Serial.println("Error: calibration data not available");
    high = Wire.read();
    low = Wire.read();
    calibrationData[i] = high<<8 | low;
    Serial.print("calibration data #");
    Serial.print(i);
    Serial.print(" = ");
    Serial.println( calibrationData[i] );
  }
}


void twiSendCommand(byte address, byte command)
{
  Wire.beginTransmission(address);
  if (!Wire.write(command)) Serial.println("Error: write()");
  if (Wire.endTransmission())
  {
    Serial.print("Error when sending command: ");
    Serial.println(command, HEX);
  }
}


void ledOn()
{
  digitalWrite(led,1);
}


void ledOff()
{
  digitalWrite(led,0);
}
