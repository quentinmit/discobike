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
#include <BLEAdafruitService.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include <Adafruit_INA219.h>

#include <Adafruit_APDS9960.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_LSM6DS33.h>
#include <Adafruit_SHT31.h>

#include <Adafruit_SPIFlash.h>
#include <Adafruit_AHRS.h>
#include <Adafruit_Sensor_Calibration.h>

#ifndef UUID16_CHR_VOLTAGE
#define UUID16_CHR_VOLTAGE                                    0x2B18
#endif

// Peripherals
Adafruit_SSD1306 oled(128, 64);
Adafruit_INA219 ina219;
Adafruit_NeoPixel underlight(35, PIN_UNDERLIGHT, NEO_GRBW | NEO_KHZ800);
Adafruit_LSM6DS33 lsm6ds33; // Gyro and Accel
Adafruit_LIS3MDL  lis3mdl;  // Magnetometer
Adafruit_APDS9960 apds9960; // Proximity, Light, Gesture, Color
Adafruit_BMP280   bmp280;   // Temperature, Barometric
Adafruit_SHT31    sht30;    // Humid

const uint16_t maxPWM = 625;

// BLE Services
BLEDfu  bledfu;
BLEDis  bledis;
//BLEUart bleuart;
BLEBas  blebas;  // battery
//BLEAdafruitQuaternion bleQuater;
BLEService blevolts = BLEService(UUID16_CHR_VOLTAGE);
BLECharacteristic blevoltc = BLECharacteristic(UUID16_CHR_VOLTAGE);

SoftwareTimer notify_timer;

// Sensor calibration
#define FILE_SENSOR_CALIB       "sensor_calib.json"
Adafruit_Sensor_Calibration_SDFat cal;

Adafruit_FlashTransport_QSPI flashTransport;
Adafruit_SPIFlash flash(&flashTransport);
FatFileSystem fatfs;

// Sensor fusion
SoftwareTimer filter_timer;
Adafruit_Madgwick filter;
//Adafruit_NXPSensorFusion filter;
#define FILTER_UPDATE_RATE_HZ 10

SemaphoreHandle_t xWireSemaphore = NULL;
StaticSemaphore_t xWireMutexBuffer;

#define STACK_SIZE 200
StaticTask_t xImuTaskBuffer;
StackType_t xImuTaskStack[ STACK_SIZE ];

// Data
typedef enum {
  OFF = 0,
  AUTO,
  DAY,
  NIGHT,
  BLINK,
} headlight_mode_t;
headlight_mode_t desired_mode = AUTO;
headlight_mode_t actual_mode = OFF;
float brightness = 0;
float vext = 0;

void setup() {
  xWireSemaphore = xSemaphoreCreateMutexStatic( &xWireMutexBuffer );

  Serial.begin(115200);
  //while ( !Serial ) delay(10); // XXX
  //Serial.println("Connected");

  oled.cp437();
  oled.begin(SSD1306_SWITCHCAPVCC, I2C_SSD1315);
  // H- and V-flip the display
  oled.ssd1306_command(SSD1306_SEGREMAP);
  oled.ssd1306_command(SSD1306_COMSCANINC);
  oled.display();
  oled.setTextSize(1);
  oled.setTextColor(WHITE);

  pinMode(PIN_POWER_ENABLE, OUTPUT);
  digitalWrite(PIN_POWER_ENABLE, HIGH); // FIXME

  pinMode(PIN_HEADLIGHT_DIM, OUTPUT);
  digitalWrite(PIN_HEADLIGHT_DIM, HIGH);

  HwPWM3.takeOwnership(1);
  HwPWM3.setClockDiv(NRF_PWM_CLK_125kHz);
  HwPWM3.setMaxValue(maxPWM);
  HwPWM3.addPin(PIN_HEADLIGHT_DIM);
  HwPWM3.writeChannel(0, maxPWM/2);
  HwPWM3.begin();

  analogReadResolution(14);

  underlight.begin();

  ina219.begin();

  // Init flash, filesystem and calibration & load calib json
  flash.begin();
  fatfs.begin(&flash);
  cal.begin(FILE_SENSOR_CALIB, &fatfs);
  cal.loadCalibration();

  // Light
  apds9960.begin();
  apds9960.enableColor(true);

  // IMU
  lsm6ds33.begin_I2C();
  lsm6ds33.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
  lsm6ds33.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);
  lsm6ds33.setAccelDataRate(LSM6DS_RATE_104_HZ);
  lsm6ds33.setGyroDataRate(LSM6DS_RATE_104_HZ);

  // Magnetometer
  lis3mdl.begin_I2C();
  lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);
  lis3mdl.setDataRate(LIS3MDL_DATARATE_1000_HZ);
  lis3mdl.setPerformanceMode(LIS3MDL_MEDIUMMODE);
  lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);

  // Temperature, Barometer
  bmp280.begin();

  // Increase I2C speed to 400 Khz
  Wire.setClock(400000);

  Bluefruit.begin();
  Bluefruit.setTxPower(4);
  Bluefruit.setName("Discobike");
  //Bluefruit.Periph.setConnectCallback(connect_callback);

  bledfu.begin();

  bledis.setManufacturer("Quentin Smith");
  bledis.setModel("Discobike v3");
  bledis.begin();

  //bleuart.begin();

  blebas.begin();
  blebas.write(100);

  blevolts.begin();
  blevoltc.setProperties(CHR_PROPS_READ|CHR_PROPS_NOTIFY);
  blevoltc.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  blevoltc.setFixedLen(2);
  // UUID docs say units are 1/64 volt, but exponent must be power of 10...
  blevoltc.setPresentationFormatDescriptor(6, -2, UUID16_UNIT_ELECTRIC_POTENTIAL_DIFFERENCE_VOLT);
  blevoltc.begin();
  blevoltc.write16(0);

  Serial.println("bluetooth services started");

  // Quaternion with sensor calibration
  //bleQuater.begin(&filter, lsm6ds33.getAccelerometerSensor(), lsm6ds33.getGyroSensor(), &lis3mdl);
  //bleQuater.setCalibration(&cal);

  filter.begin(FILTER_UPDATE_RATE_HZ);
  // Ignore the returned task handle
  xTaskCreateStatic(
                    imu_update_task,       /* Function that implements the task. */
                    "imu",           /* Text name for the task. */
                    STACK_SIZE,      /* Number of indexes in the xStack array. */
                    ( void * ) 1,    /* Parameter passed into the task. */
                    3,               /* Priority at which the task is created. */
                    xImuTaskStack,          /* Array to use as the task's stack. */
                    &xImuTaskBuffer );  /* Variable to hold the task's data structure. */

  startAdv();

  notify_timer.begin(500, notify_timer_cb, NULL, true);
  notify_timer.start();

  Serial.println("begin finished");
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

void notify_timer_cb(TimerHandle_t xTimer) {
  uint16_t volt = vext * 100;
  if (blevoltc.read16() != volt) {
    blevoltc.notify16(volt);
  }
}

long imu_skipped = 0;

void imu_update_task(void *pvParameters) {
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(1000.0/FILTER_UPDATE_RATE_HZ);

  // Initialise the xLastWakeTime variable with the current time.
  xLastWakeTime = xTaskGetTickCount();

  for( ;; ) {
    // Wait for the next cycle.
    vTaskDelayUntil( &xLastWakeTime, xFrequency );
    _imu_update();
    // If we've gotten behind, reset the last wake time.
    if (xLastWakeTime + xFrequency < xTaskGetTickCount()) {
      xLastWakeTime = xTaskGetTickCount();
      imu_skipped++;
    }
  }
}

sensors_event_t accel_evt, gyro_evt, mag_evt;
void _imu_update() {
  //digitalWrite(LED_RED, HIGH);
  xSemaphoreTake(xWireSemaphore, portMAX_DELAY);

  // get sensor events
  lsm6ds33.getAccelerometerSensor()->getEvent(&accel_evt);
  lsm6ds33.getGyroSensor()->getEvent(&gyro_evt);
  lis3mdl.getEvent(&mag_evt);

  // calibrate sensor if available
  cal.calibrate(accel_evt);
  cal.calibrate(gyro_evt);
  cal.calibrate(mag_evt);

  // Convert gyro from Rad/s to Degree/s
  gyro_evt.gyro.x *= SENSORS_RADS_TO_DPS;
  gyro_evt.gyro.y *= SENSORS_RADS_TO_DPS;
  gyro_evt.gyro.z *= SENSORS_RADS_TO_DPS;

  // apply filter
  filter.update(gyro_evt.gyro.x , gyro_evt.gyro.y, gyro_evt.gyro.z,
                  accel_evt.acceleration.x, accel_evt.acceleration.y, accel_evt.acceleration.z,
                  mag_evt.magnetic.x, mag_evt.magnetic.y, mag_evt.magnetic.z);

  xSemaphoreGive(xWireSemaphore);
  //digitalWrite(LED_RED, LOW);
}

float getRawHeading(){
  // Hold the module so that Z is pointing 'up' and you can measure the heading with x&y
  // Calculate heading when the magnetometer is level, then correct for signs of axis.
  float heading = atan2(mag_evt.magnetic.y, mag_evt.magnetic.x);
  heading += PI;
  // Correct for when signs are reversed.
  if(heading < 0)
    heading += 2*PI;
  // Check for wrap due to addition of declination.
  if(heading > 2*PI)
    heading -= 2*PI;
  // Convert radians to degrees for readability.
  heading = heading * 180/M_PI; 
  return heading;
}

size_t printFixed(Print& print, signed long n, uint8_t width, uint8_t base, uint8_t fill = '0')
{
  char buf[8 * sizeof(long) + 1]; // Assumes 8-bit chars plus zero byte.
  char *str = &buf[sizeof(buf) - 1];

  *str = '\0';

  // prevent crash if called with base == 1
  if (base < 2) base = 10;
  bool negative = (n < 0);
  if (negative) {
    n = -n;
  }

  do {
    char c = n % base;
    n /= base;

    *--str = c < 10 ? c + '0' : c + 'A' - 10;
  } while(n);
  if (negative) {
    *--str = '-';
  }
  char *start = &buf[sizeof(buf) - 1 - width];
  while (str > start) {
    *--str = fill;
  }

  return print.write(str);
}

const uint8_t DEG = 248;

void printAngle(Print& print, float angle, uint8_t symbol = DEG)
{
  printFixed(print, angle, 3, DEC, ' ');
  print.write('.');
  printFixed(print, (int)(abs(angle*100)) % 100, 2, DEC, '0');
  print.write(symbol);
}

float invSqrt(float x) {
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long *)&y;
  i = 0x5f3759df - (i >> 1);
  y = *(float *)&i;
  y = y * (1.5f - (halfx * y * y));
  y = y * (1.5f - (halfx * y * y));
  return y;
}

float vecMag(sensors_vec_t v) {
  return 1/invSqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

uint32_t timestamp;
TickType_t last_move_time;
const float ONE_G = 9.80665;

void loop() {
  TickType_t now = xTaskGetTickCount();

  //while ( !Serial ) delay(10); // XXX
  // print the heading, pitch and roll
  float roll, pitch, heading;
  roll = filter.getRoll();
  pitch = filter.getPitch();
  heading = 360.0f-filter.getYaw();

  if (Serial) {
    Serial.print("Orientation: ");
    Serial.print(heading);
    Serial.print(", ");
    Serial.print(pitch);
    Serial.print(", ");
    Serial.println(roll);
  }
  
  //lis3mdl.read();
  //float heading = filter.getRoll();
  if (xSemaphoreTake(xWireSemaphore, 10) != pdTRUE) {
    return;
  }
  float temperature = bmp280.readTemperature() * 1.8 + 32.0;
  vext = ina219.getBusVoltage_V();
  float current_mA = ina219.getCurrent_mA();
  uint16_t r,g,b,c;
  apds9960.getColorData(&r, &g, &b, &c);
  // 3.5 counts/lux in the c channel according to datasheet
  xSemaphoreGive(xWireSemaphore);
  uint16_t lux = c/3.5; //apds9960.calculateLux(r, g, b);

  float vbat = analogRead(PIN_VBAT) * 3.6f * 2.0f / 16384.0f;

  // Update mode
  float accel_mag = vecMag(accel_evt.acceleration);
  if (accel_mag > 12) {
    last_move_time = now;
  }
  // TODO: Use IMU angle for mode too?
  if (vext < 11.2) {
    brightness = 0;
    actual_mode = OFF;
  } else {
    switch (desired_mode) {
      case AUTO:
        if (lux > 50) {
          actual_mode = DAY;
        } else {
          actual_mode = NIGHT;
        }
        if ((now - last_move_time) > pdMS_TO_TICKS(10000)) {
          actual_mode = OFF;
        }
        break;
      default:
        actual_mode = desired_mode;
    }
  }

  float target_brightness = 0;
  switch (actual_mode) {
  case NIGHT:
    target_brightness = 1;
    break;
  case DAY:
    // TODO: Blink
    target_brightness = 0.5;
    break;
  }
  if (target_brightness > brightness) {
    brightness += 0.01;
    brightness = min(brightness, target_brightness);
  } else {
    brightness -= 0.01;
    brightness = max(brightness, target_brightness);
  }
  HwPWM3.writeChannel(0, maxPWM * brightness);

  // Update BLE characteristics
  // TODO: Notify
  //blevoltc.write16(vext*64);

  // Display
  oled.clearDisplay();

  oled.setCursor(0, 0);
  printFixed(oled, (int)vext, 2, DEC, '0');
  oled.print(F("."));
  printFixed(oled, (int)(vext * 10) % 10, 1, DEC);
  oled.print(F("Vext "));
  if (current_mA < 0) {
    current_mA = 0;
  }
  printFixed(oled, (int)current_mA / 1000, 1, DEC, '0');
  oled.print(F("."));
  printFixed(oled, (int)current_mA % 1000, 3, DEC, '0');
  oled.print(F("A "));

  printFixed(oled, (int)vbat, 1, DEC, '0');
  oled.print(F("."));
  printFixed(oled, (int)(vbat * 100) % 100, 2, DEC);
  oled.print(F("Vbat"));
  
  oled.write('\n');
  /*oled.print(magX);
  oled.print(" ");
  oled.print(magY);
  oled.print(" ");
  oled.print(magZ);
  oled.println();*/
  printFixed(oled, heading, 3, DEC, ' ');
  oled.write(DEG);
  printFixed(oled, roll, 3, DEC, ' ');
  oled.write(DEG);
  
  oled.setCursor(10*8, oled.getCursorY());
  printFixed(oled, (int)temperature, 3, DEC, ' ');
  oled.print(F("."));
  printFixed(oled, (int)(temperature * 10) % 10, 1, DEC);
  oled.write(DEG);
  //oled.print((double)heading, 0);
  //oled.print(F("o"));

  oled.write('\n');
  //oled.print(accel);
  //printFixed(oled, getRawHeading(), 3, DEC, ' ');
  //oled.write(DEG);
  printAngle(oled, accel_mag / ONE_G, 'G');
  oled.print(F("G  "));
  printFixed(oled, lux, 5, DEC, ' ');
  oled.print(F(" lux "));
  oled.write('\n');

  oled.print(F("IMU late: "));
  printFixed(oled, imu_skipped, 3, DEC, '0');
  oled.write('\n');
  
  oled.print(F("Mode: "));
  switch (actual_mode) {
    case DAY:
      oled.print(F("Day"));
      break;
    case NIGHT:
      oled.print(F("Night"));
      break;
    case OFF:
      oled.print(F("OFF"));
      break;
  }
  oled.write(' ');
  printFixed(oled, brightness * 100, 3, DEC, ' ');
  oled.write('%');
  
  /*
  oled.print(now.month());
  oled.print(F("/"));
  oled.print(now.day());
  oled.setCursor(8*8, 6);
  oled.print(now.hour() > 10 ? '1' : '0');
  oled.print(now.hour() % 10);
  oled.print(F(":"));
  oled.print((now.minute() / 10)%10);
  oled.print(now.minute() % 10);
  oled.print(F(":"));
  oled.print(now.second() / 10);
  oled.print(now.second() % 10);
  */
  if (xSemaphoreTake(xWireSemaphore, 10) != pdTRUE) {
    return;
  }
  oled.display();
  xSemaphoreGive(xWireSemaphore);
}
