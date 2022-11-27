/***************************************************************************

  _   _           ____                                
 | \ | |  _   _  / ___|   _ __     __ _    ___    ___ 
 |  \| | | | | | \___ \  | '_ \   / _` |  / __|  / _ \
 | |\  | | |_| |  ___) | | |_) | | (_| | | (__  |  __/
 |_| \_|  \__,_| |____/  | .__/   \__,_|  \___|  \___|
                         |_|                          
  (C)2022 NuSpace Pte Ltd
Description:
    Balloon Satellite Example Code without SD card integration. 
Author: Hubert Khoo Hui Bo
Date Written: 24th November 2022

***************************************************************************/

/* include all important libraries */
#include <Wire.h>
#include <SPI.h>

#include <SoftwareSerial.h>

union temperature {
    uint8_t     bytes[sizeof( float )];
    float       temp;
} ;

temperature uni;

// All required includes, define for BMP 280, taken from bmp280test example code
#include <Adafruit_BMP280.h>

#define BMP_SCK  (52)
#define BMP_MISO (50)
#define BMP_MOSI (51)
#define BMP_CS   (8)

//Adafruit_BMP280 bmp; // I2C
Adafruit_BMP280 bmp(BMP_CS); // hardware SPI
//Adafruit_BMP280 bmp(BMP_CS, BMP_MOSI, BMP_MISO,  BMP_SCK);

// all required includes for MPU-6500 reading. Taken from example code MPU_9600_SPI_all data

#include <MPU6500_WE.h>
const int csPin = 9;  // Chip Select Pin < -- change to whatever you want to use! Don't overlap hardware SPI pins.
bool useSPI = true;    // SPI use flag

MPU6500_WE myMPU6500 = MPU6500_WE(&SPI, csPin, useSPI); // Uses hardware SPI Pins.

/*
Not all pins on the Mega and Mega 2560 boards support change interrupts,
so only the following can be used for RX: 10, 11, 12, 13, 14, 15, 50,
51, 52, 53, A8 (62), A9 (63), A10 (64), A11 (65), A12 (66), A13 (67),
A14 (68), A15 (69).
*/

SoftwareSerial HC12(10, 11); // HC-12 TX Pin, HC-12 RX Pin

void setup() {
  //Wire.begin();
  Serial.begin(9600); // start up the local serial monitor.

  HC12.begin(9600); // start up the serial pins connected to the HC-12

  while ( !Serial ) delay(100);   // wait for native usb

  unsigned status;

  //status = bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);

  status = bmp.begin(0x68); // The GY-91 board used had such an address.

  if (!status) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                      "try a different address!"));
    Serial.print("SensorID was: 0x"); Serial.println(bmp.sensorID(),16);
    Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
    Serial.print("        ID of 0x60 represents a BME 280.\n");
    Serial.print("        ID of 0x61 represents a BME 680.\n");
    while (1) delay(10);
  }


  while(!myMPU6500.init()){
    Serial.println("MPU6500 does not respond");
    delay(1000);
  }


  Serial.println("MPU6500 is connected");


  Serial.println("Position you MPU6500 flat and don't move it - calibrating...");
  delay(1000);
  myMPU6500.autoOffsets();
  Serial.println("Done!");
    
  //myMPU6500.setAccOffsets(-14240.0, 18220.0, -17280.0, 15590.0, -20930.0, 12080.0);
  //myMPU6500.setGyrOffsets(45.0, 145.0, -105.0);
  myMPU6500.enableGyrDLPF();
  //myMPU6500.disableGyrDLPF(MPU6500_BW_WO_DLPF_8800); // bandwdith without DLPF
  myMPU6500.setGyrDLPF(MPU6500_DLPF_6);
  myMPU6500.setSampleRateDivider(5);
  myMPU6500.setGyrRange(MPU6500_GYRO_RANGE_250);
  myMPU6500.setAccRange(MPU6500_ACC_RANGE_2G);
  myMPU6500.enableAccDLPF(true);
  myMPU6500.setAccDLPF(MPU6500_DLPF_6);


  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
}

void loop() {

  Serial.print(F("Temperature = "));
  Serial.print(bmp.readTemperature());
  Serial.println(" *C");

  Serial.print(F("Pressure = "));
  Serial.print(bmp.readPressure());
  Serial.println(" Pa");

  Serial.print(F("Approx altitude = "));
  Serial.print(bmp.readAltitude(1013.25)); /* Adjusted to local forecast! */
  Serial.println(" m");

  Serial.println();

  xyzFloat gValue = myMPU6500.getGValues();
  xyzFloat gyr = myMPU6500.getGyrValues();
  float temp = myMPU6500.getTemperature();
  float resultantG = myMPU6500.getResultantG(gValue);

  // assign float value to union.
  uni.temp = temp;

  Serial.println("Acceleration in g (x,y,z):");
  Serial.print(gValue.x);
  Serial.print("   ");
  Serial.print(gValue.y);
  Serial.print("   ");
  Serial.println(gValue.z);
  Serial.print("Resultant g: ");
  Serial.println(resultantG);

  Serial.println("Gyroscope data in degrees/s: ");
  Serial.print(gyr.x);
  Serial.print("   ");
  Serial.print(gyr.y);
  Serial.print("   ");
  Serial.println(gyr.z);

  Serial.print("Temperature in °C: ");
  Serial.println(temp);

  Serial.println("********************************************");
  
  // output the temperature data to the HC12.
  HC12.write( '>' );
  HC12.write(uni.bytes, sizeof( float ) );
  HC12.write( '<' );

  delay(1000);
}
