// NeoPixels: GRBW, 35 pixels


// Pins
// * = exposed on header
// 2* / P0.10
// 3 / P1.11 - Gyro + Accel IRQ
// 4 / P1.10 - Blue LED ("Conn")
// 5* / P1.08 - NeoPixel Connector (Propmaker)
// 6* / P0.07 - IRQ (Propmaker)
// 7 / P1.02 - Button
// 8 / P0.16 - NeoPixel
// 9* / P0.26 - Button (Propmaker)
// 10* / P0.27 - Power Enable (Propmaker)
// 11* / P0.06 - Red LED (Propmaker)
// 12* / P0.08 - Green LED (Propmaker)
// 13* / P1.09 - Blue LED (Propmaker)
// 13* / P1.09 - Red LED
// 34 / P0.00 - PDM microphone data
// 35 / P0.01 - PDM microphone clock
// 36 / P1.00 - APDS9960 Light Gesture Proximity IRQ
// A0* / P0.04 - Audio Amp Input (Propmaker)
// A1* / P0.05
// A2* / P0.30
// A3* / P0.28
// A4* / P0.02
// A5* / P0.03
// A6 / P0.29 - VOLTAGE_MONITOR (100K+100K voltage divider)
// SCK* / P0.14
// MO* / P0.13
// MI* / P0.15
// RX* / P0.24
// TX* / P0.25
// I2C devices
// 0x18 - LIS3DH Accelerometer (on propmaker)
// 0x1C - LIS3MDL Magnetometer
// 0x39 - APDS9960 Light Gesture Proximity
// 0x3C - SSD1315 OLED Display
// 0x40 - INA219 Current sensor (on INA)
// 0x44 - SHT30 Humidity
// 0x6A - LSM6DS33 Gyro + Accel
// 0x77 - BMP280 Temperature + Pressure

#define PIN_UNDERLIGHT 5
#define PIN_POWER_ENABLE 10
#define PIN_TAIL_C 11
#define PIN_TAIL_L 12
#define PIN_TAIL_R 13
#define PIN_HEADLIGHT_DIM A5
#define PIN_VBAT A6

#define I2C_LIS3DH 0x18
#define I2C_LIS3MDL 0x1C
#define I2C_APDS9960 0x39
#define I2C_SSD1315 0x3C
#define I2C_INA219 0x40
#define I2C_SHT30 0x44
#define I2C_LSM6DS33 0x6A
#define I2C_BMP280 0x77

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <bluefruit.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include <Adafruit_APDS9960.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_LSM6DS33.h>
#include <Adafruit_SHT31.h>

#include <Adafruit_SPIFlash.h>
#include <Adafruit_AHRS.h>
#include <Adafruit_Sensor_Calibration.h>

// Peripherals
Adafruit_SSD1306 oled(128, 64);
Adafruit_NeoPixel underlight(35, PIN_UNDERLIGHT, NEO_GRBW | NEO_KHZ800);
Adafruit_LSM6DS33 lsm6ds33; // Gyro and Accel
Adafruit_LIS3MDL  lis3mdl;  // Magnetometer
Adafruit_APDS9960 apds9960; // Proximity, Light, Gesture, Color
Adafruit_BMP280   bmp280;   // Temperature, Barometric
Adafruit_SHT31    sht30;    // Humid

// BLE Services
BLEDfu  bledfu;
BLEDis  bledis;
//BLEUart bleuart;

void setup() {
  Serial.begin(115200);

  oled.begin(SSD1306_SWITCHCAPVCC, I2C_SSD1315);
  oled.ssd1306_command(SSD1306_SEGREMAP);
  oled.ssd1306_command(SSD1306_COMSCANINC);
  oled.display();
  oled.setTextSize(1);
  oled.setTextColor(WHITE);

  pinMode(PIN_POWER_ENABLE, OUTPUT);
  digitalWrite(PIN_POWER_ENABLE, HIGH); // FIXME

  underlight.begin();

  Bluefruit.begin();
  Bluefruit.setTxPower(4);
  Bluefruit.setName("Discobike");
  //Bluefruit.Periph.setConnectCallback(connect_callback);

  bledfu.begin();

  bledis.setManufacturer("Quentin Smith");
  bledis.setModel("Discobike v3");
  bledis.begin();

  //bleuart.begin();

  startAdv();
}

void startAdv(void)
{  
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  
  // Include bleuart 128-bit uuid
  //Bluefruit.Advertising.addService(bleuart);

  // Secondary Scan Response packet (optional)
  // Since there is no room for 'Name' in Advertising packet
  Bluefruit.ScanResponse.addName();
  
  /* Start Advertising
   * - Enable auto advertising if disconnected
   * - Interval:  fast mode = 20 ms, slow mode = 318.75 ms
   * - Timeout for fast mode is 30 seconds
   * - Start(timeout) with timeout = 0 will advertise forever (until connected)
   * 
   * For recommended advertising interval
   * https://developer.apple.com/library/content/qa/qa1931/_index.html   
   */
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 510);    // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
  Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds
}

void loop() {
  // put your main code here, to run repeatedly:

}
