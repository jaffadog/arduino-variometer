//**********************************
//*   BMP085 and BMP180 version    *
//**********************************
// This is the Poor Mans Vario code for the cheaper BMP085 and the BMP180 sensor by BOSCH Sensortec
// Arduino <> Sensor: Connect VCC to VCC and GND to GND, SCL goes to analogue pin 5, SDA to analogue pin4.
// Servo signal input, connect to Arduino pin D3
// Audio output to transmitter on pin D2
// All code by Rolf R Bakke, Oct 2012
// Modified by Hans Meijdam, June 2013: added altitude feature
// Modified by Hans Meijdam, November 2013: Sensor routine created for BMP085 and BMP180
//

#include "Wire.h"
const byte led = 13;
unsigned long time = 0;
float toneFreq, toneFreqLowpass, flpressure, lowpassFast, lowpassSlow ;
float p0; // this will be used to store the airfield elevation pressure
int altitude;
int ch1; // Here's where we'll keep our channel values
int ddsAcc;
#define I2C_ADDRESS 0x77
const unsigned char oversampling_setting = 3; //oversamplig for measurement
const unsigned char pressure_waittime[4] = {
  5, 8, 14, 26 };
//Sensor parameters taken from the BMP085 datasheet
int ac1, ac2, ac3;
unsigned int ac4, ac5, ac6;
int b1, b2, mb, mc, md;
int temperature;
long pressure;

void setup()
{
  Serial.begin(9600); // start serial for output
  Serial.println("Setting up BMP085");
  Wire.begin();
  bmp085_get_cal_data();
  bmp085_read_temperature_and_pressure(&temperature,&pressure);
  flpressure=pressure;// move long type pressure into float type flpressure
  p0 = lowpassFast = lowpassSlow = flpressure;
  Serial.print(" p0 = ");
  Serial.println(p0);
  pinMode(3, INPUT); // Set our input pins as such for altitude command input from receiver via pin D3
}

void loop()
{
  bmp085_read_temperature_and_pressure(&temperature,&pressure);
  // Serial.print(temperature,DEC);
  // Serial.print(" ");
  // Serial.print(pressure,DEC);
  // Serial.print(" ");
  flpressure = pressure;// move long type pressure into float type flpressure
  altitude = (float)44330 * (1 - pow(((float) flpressure/p0), 0.190295));
  // Serial.print(" ");
  // Serial.println(altitude);
  lowpassFast = lowpassFast + (flpressure - lowpassFast) * 0.3;
  lowpassSlow = lowpassSlow + (flpressure - lowpassSlow) * 0.15;
  toneFreq = (lowpassSlow - lowpassFast) * 50;
  toneFreqLowpass = toneFreqLowpass + (toneFreq - toneFreqLowpass) * 0.1;
  toneFreq = constrain(toneFreqLowpass, -1000, 1000);
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
  while (millis() < time);        //loop frequency timer
  time += 20;
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

  ch1 = pulseIn(3, HIGH, 25000); // Read the pulse width of servo signal connected to pin D3
  //  Serial.print (ch1);
  //
  //   if(ch1>1000){
  //    Serial.println("Left Switch: Engaged");
  //   }
  //   if(ch1<1000){
  //     Serial.println("Left Switch: Disengaged");
  //   }
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

    for (int p0=0; p0 <= 20; p0++)
    {
      // warming up the sensor again, by reading it 20 times
      bmp085_read_temperature_and_pressure(&temperature,&pressure);
    }
  }
}

void bmp085_read_temperature_and_pressure(int* temperature, long* pressure) {
  int ut= bmp085_read_ut();
  long up = bmp085_read_up();
  long x1, x2, x3, b3, b5, b6, p;
  unsigned long b4, b7;

  //calculate the temperature
  x1 = ((long)ut - ac6) * ac5 >> 15;
  x2 = ((long) mc << 11) / (x1 + md);
  b5 = x1 + x2;
  *temperature = (b5 + 8) >> 4;

  //calculate the pressure
  b6 = b5 - 4000;
  x1 = (b2 * (b6 * b6 >> 12)) >> 11;
  x2 = ac2 * b6 >> 11;
  x3 = x1 + x2;

  if (oversampling_setting == 3) b3 = ((int32_t) ac1 * 4 + x3 + 2) << 1;
  if (oversampling_setting == 2) b3 = ((int32_t) ac1 * 4 + x3 + 2);
  if (oversampling_setting == 1) b3 = ((int32_t) ac1 * 4 + x3 + 2) >> 1;
  if (oversampling_setting == 0) b3 = ((int32_t) ac1 * 4 + x3 + 2) >> 2;

  x1 = ac3 * b6 >> 13;
  x2 = (b1 * (b6 * b6 >> 12)) >> 16;
  x3 = ((x1 + x2) + 2) >> 2;
  b4 = (ac4 * (uint32_t) (x3 + 32768)) >> 15;
  b7 = ((uint32_t) up - b3) * (50000 >> oversampling_setting);
  p = b7 < 0x80000000 ? (b7 * 2) / b4 : (b7 / b4) * 2;

  x1 = (p >> 8) * (p >> 8);
  x1 = (x1 * 3038) >> 16;
  x2 = (-7357 * p) >> 16;
  *pressure = p + ((x1 + x2 + 3791) >> 4);
}

unsigned int bmp085_read_ut() {
  write_register(0xf4,0x2e);
  delay(5); //longer than 4.5 ms
  return read_int_register(0xf6);
}

void bmp085_get_cal_data() {
  Serial.println("Reading Calibration Data");
  ac1 = read_int_register(0xAA);
  Serial.print("AC1: ");
  Serial.println(ac1,DEC);
  ac2 = read_int_register(0xAC);
  Serial.print("AC2: ");
  Serial.println(ac2,DEC);
  ac3 = read_int_register(0xAE);
  Serial.print("AC3: ");
  Serial.println(ac3,DEC);
  ac4 = read_int_register(0xB0);
  Serial.print("AC4: ");
  Serial.println(ac4,DEC);
  ac5 = read_int_register(0xB2);
  Serial.print("AC5: ");
  Serial.println(ac5,DEC);
  ac6 = read_int_register(0xB4);
  Serial.print("AC6: ");
  Serial.println(ac6,DEC);
  b1 = read_int_register(0xB6);
  Serial.print("B1: ");
  Serial.println(b1,DEC);
  b2 = read_int_register(0xB8);
  Serial.print("B2: ");
  Serial.println(b2,DEC);
  mb = read_int_register(0xBA);
  Serial.print("MB: ");
  Serial.println(mb,DEC);
  mc = read_int_register(0xBC);
  Serial.print("MC: ");
  Serial.println(mc,DEC);
  md = read_int_register(0xBE);
  Serial.print("MD: ");
  Serial.println(md,DEC);
}

long bmp085_read_up() {
  write_register(0xf4,0x34+(oversampling_setting<<6));
  delay(pressure_waittime[oversampling_setting]);

  unsigned char msb, lsb, xlsb;
  Wire.beginTransmission(I2C_ADDRESS);
  Wire.write(0xf6); // register to read
  Wire.endTransmission();

  Wire.requestFrom(I2C_ADDRESS, 3); // read a byte
  while(!Wire.available()) {
    // waiting
  }
  msb = Wire.read();
  while(!Wire.available()) {
    // waiting
  }
  lsb |= Wire.read();
  while(!Wire.available()) {
    // waiting
  }
  xlsb |= Wire.read();
  return (((long)msb<<16) | ((long)lsb<<8) | ((long)xlsb)) >>(8-oversampling_setting);
}

void write_register(unsigned char r, unsigned char v)
{
  Wire.beginTransmission(I2C_ADDRESS);
  Wire.write(r);
  Wire.write(v);
  Wire.endTransmission();
}

char read_register(unsigned char r)
{
  unsigned char v;
  Wire.beginTransmission(I2C_ADDRESS);
  Wire.write(r); // register to read
  Wire.endTransmission();

  Wire.requestFrom(I2C_ADDRESS, 1); // read a byte
  while(!Wire.available()) {
    // waiting
  }
  v = Wire.read();
  return v;
}

int read_int_register(unsigned char r)
{
  unsigned char msb, lsb;
  Wire.beginTransmission(I2C_ADDRESS);
  Wire.write(r); // register to read
  Wire.endTransmission();

  Wire.requestFrom(I2C_ADDRESS, 2); // read a byte
  while(!Wire.available()) {
    // waiting
  }
  msb = Wire.read();
  while(!Wire.available()) {
    // waiting
  }
  lsb = Wire.read();
  return (((int)msb<<8) | ((int)lsb));
}


void ledOn()
{
  digitalWrite(led,1);
}


void ledOff()
{
  digitalWrite(led,0);
}
