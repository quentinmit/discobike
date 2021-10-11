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
#define PIN_RST A1
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

#include <limits>

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <bluefruit.h>
#include <BLEAdafruitService.h>
#include "BLERunTimeStats.h"
#include "BLERemoteVariable.h"
#include <hal/nrf_wdt.h>

#include "Task.h"
#include "effects.h"

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_INA219.h>
#include <INA219_WE.h>
#include <Adafruit_APDS9960.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_LSM6DS33.h>
#include <Adafruit_SHT31.h>

#include <Adafruit_SPIFlash.h>
#include <Adafruit_AHRS.h>
#include <Adafruit_Sensor_Calibration.h>
#include <Adafruit_BusIO_Register.h>

#ifndef UUID16_CHR_VOLTAGE
#define UUID16_CHR_VOLTAGE                                    0x2B18
#endif

#define LSM6DS_CTRL6_C 0x15       ///< Accelerometer mode
#define LSM6DS_CTRL7_G 0x16       ///< Gyro mode

class LSM6DS33 : public Adafruit_LSM6DS33 {
public:
  void setLowPowerMode(bool enabled) {
    Adafruit_BusIO_Register ctrl6 = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DS_CTRL6_C);
    Adafruit_BusIO_RegisterBits xl_hm_mode =
      Adafruit_BusIO_RegisterBits(&ctrl6, 1, 4);
    xl_hm_mode.write(enabled);
    Adafruit_BusIO_Register ctrl7 = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DS_CTRL7_G);
    Adafruit_BusIO_RegisterBits g_hm_mode =
      Adafruit_BusIO_RegisterBits(&ctrl7, 1, 7);
    g_hm_mode.write(enabled);
  }
};

// Peripherals
Adafruit_NeoPixel neopixel = Adafruit_NeoPixel(NEOPIXEL_NUM, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);
Adafruit_SSD1306 oled(128, 64, &Wire, -1, 400000UL, 400000UL);
INA219_WE ina219;
Adafruit_NeoPixel underlight(35, PIN_UNDERLIGHT, NEO_GRBW | NEO_KHZ800);
LSM6DS33 lsm6ds33; // Gyro and Accel
Adafruit_LIS3MDL  lis3mdl;  // Magnetometer
Adafruit_APDS9960 apds9960; // Proximity, Light, Gesture, Color
Adafruit_BMP280   bmp280;   // Temperature, Barometric
Adafruit_SHT31    sht30;    // Humid

const uint16_t PWM_MAX_HEADLIGHT = 625;
const uint16_t PWM_MAX_TAILLIGHT = 255;

const TickType_t TAIL_PERIOD = pdMS_TO_TICKS(2000);
const TickType_t LAST_MOVE_TIMEOUT = pdMS_TO_TICKS(20000);
const TickType_t LAST_MOVE_TAIL_TIMEOUT = pdMS_TO_TICKS(60000);
const TickType_t LAST_MOVE_UNDER_TIMEOUT = pdMS_TO_TICKS(60000);
const TickType_t DISPLAY_TIMEOUT = pdMS_TO_TICKS(60000);
// Check for Vext every 1s to save power
const TickType_t VEXT_POLL_PERIOD = pdMS_TO_TICKS(1000);
const float ONE_G = 9.80665;

const bool DEBUG_IMU = false;

// BLE Services
BLEDfu  bledfu;
BLEDis  bledis;
//BLEUart bleuart;
BLEBas  blebas;  // battery
#if configGENERATE_RUN_TIME_STATS
BLERunTimeStats bleruntimestats;
#endif
//BLEAdafruitQuaternion bleQuater;
BLEService blevolts = BLEService(UUID16_CHR_VOLTAGE);
BLECharacteristic blevoltc = BLECharacteristic(UUID16_CHR_VOLTAGE);
const uint8_t UUID128_BASE[16] =
{
  0x87, 0x8d, 0x53, 0x29, 0x5f, 0x2e, 0x43, 0x08,
  0x85, 0xc9, 0xbd, 0x1f, 0x00, 0x00, 0x00, 0x00
};
BLEUuid UUID(uint8_t one, uint8_t two) {
  uint8_t* u128 = (uint8_t*) rtos_malloc(16);
  memcpy(u128, UUID128_BASE, 16);
  u128[13] = one;
  u128[12] = two;
  uint8_t ret[16] =
  {
    0x87, 0x8d, 0x53, 0x29, 0x5f, 0x2e, 0x43, 0x08,
    0x85, 0xc9, 0xbd, 0x1f, two, one, 0x00, 0x00
  };
  return BLEUuid(u128);
}

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

// Screen refresh takes ~28ms, so max rate without dropped updates is 35 Hz.
#define FILTER_UPDATE_RATE_HZ 50

SemaphoreHandle_t xWireSemaphore = NULL;
StaticSemaphore_t xWireMutexBuffer;

void _imu_update();
PeriodicTask imu_task(pdMS_TO_TICKS(1000.0/FILTER_UPDATE_RATE_HZ), "imu", 3, _imu_update, NRF_WDT);
void _output_update();
PeriodicTask output_task(pdMS_TO_TICKS(1000.0/30), "output", 3, _output_update, NRF_WDT);
void _display_update();
PeriodicTask display_task(pdMS_TO_TICKS(1000.0/5), "display", 2, _display_update, NRF_WDT);

// Data
typedef enum {
  OFF = 0,
  AUTO,
  DAY,
  NIGHT,
  BLINK,
} headlight_mode_t;
headlight_mode_t desired_mode = AUTO;
BLEService blehl(UUID(0, 0));
BLERemoteVariable<headlight_mode_t> blehlmode(&desired_mode, UUID(0, 1));

typedef enum {
  UL_OFF = 0,
  UL_AUTO,
  UL_ON,
  UL_FORCE,
} underlight_mode_t;
underlight_mode_t underlight_mode = UL_OFF;
typedef enum {
  SOLID = 0,
  COLOR_WIPE,
  THEATER_CHASE,
  RAINBOW,
  THEATER_CHASE_RAINBOW,
  CYLON_BOUNCE,
  FIRE,
} underlight_effect_t;
underlight_effect_t underlight_effect = SOLID;
typedef struct {
  uint8_t red, green, blue, white;
} rgbw_t;
rgbw_t underlight_color = {0, 0, 0, 255};
int16_t underlight_speed = 256;
uint8_t underlight_brightness = 0;
BLEService bleul(UUID(1, 0));
BLERemoteVariable<underlight_mode_t> bleulmode(&underlight_mode, UUID(1, 1));
BLERemoteVariable<underlight_effect_t> bleuleffect(&underlight_effect, UUID(1, 2));
BLERemoteVariable<rgbw_t> bleulcolor(&underlight_color, UUID(1, 3));
BLERemoteVariable<int16_t> bleulspeed(&underlight_speed, UUID(1, 5));


headlight_mode_t actual_mode = OFF;
float brightness = 0;
float taillight_brightness = 0;

bool display_on = true;
bool vbus_detected = false;

TickType_t last_move_time = 0;
TickType_t last_vbus_time = 0;
TickType_t last_vext_time = 0;
TickType_t last_vext_poll_time = 0;

// Measured data
float vbat = 0;
float vext = 0;
float accel_mag = 0;
uint16_t lux = 0;
//uint16_t lux2 = 0;

void setup() {
  xWireSemaphore = xSemaphoreCreateMutexStatic( &xWireMutexBuffer );

  Serial.begin(115200);
  //while ( !Serial ) delay(10); // XXX
  //Serial.println("Connected");

  pinMode(PIN_RST, INPUT_PULLUP);

  if (nrf_wdt_started(NRF_WDT)) {
    // WDT is already enabled.
    Serial.println("WDT already enabled, first boot after DFU?");
    Serial.flush();
    neopixel.fill(neopixel.Color(0x11, 0, 0x11, 0));
    neopixel.show();
    // Wait for WDT reset; attempt to force a hardware reset
    pinMode(PIN_RST, OUTPUT);
    digitalWrite(PIN_RST, LOW);
    while (1);
    // Things that don't work:
    // Set D0.18 low using GPIO registers
    // Trigger a reset using system off + PIN_VBAT or PIN_D22 (SDA)
    // Trigger a reset using system off + external SDA pullup manually
    //nrf_gpio_cfg_sense_input(12, NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_HIGH);
    //sd_power_system_off();
    // Trigger a reset using system off via Arduino
    //systemOff(PIN_D22, true);
  }
  nrf_wdt_behaviour_set(NRF_WDT, NRF_WDT_BEHAVIOUR_RUN_SLEEP);
  nrf_wdt_reload_value_set(NRF_WDT, 10*32768);
  NRF_WDT->RREN = 0; // n.b. RR0 is enabled on reset

  oled.cp437();
  oled.begin(SSD1306_SWITCHCAPVCC, I2C_SSD1315);
  // H- and V-flip the display
  oled.ssd1306_command(SSD1306_SEGREMAP);
  oled.ssd1306_command(SSD1306_COMSCANINC);
  oled.clearDisplay();
  oled.display();
  oled.setTextSize(1);
  oled.setTextColor(WHITE);

  pinMode(PIN_POWER_ENABLE, OUTPUT);
  digitalWrite(PIN_POWER_ENABLE, HIGH); // FIXME

  // Headlight is at 100% if PWM is not driving
  pinMode(PIN_HEADLIGHT_DIM, OUTPUT);
  digitalWrite(PIN_HEADLIGHT_DIM, LOW);

  HwPWM3.takeOwnership(1);
  HwPWM3.setClockDiv(NRF_PWM_CLK_125kHz);
  HwPWM3.setMaxValue(PWM_MAX_HEADLIGHT);
  HwPWM3.addPin(PIN_HEADLIGHT_DIM);
  HwPWM3.writeChannel(0, PWM_MAX_HEADLIGHT/2);
  HwPWM3.begin();

  pinMode(PIN_TAIL_C, OUTPUT);
  pinMode(PIN_TAIL_L, OUTPUT);
  pinMode(PIN_TAIL_R, OUTPUT);

  HwPWM2.takeOwnership(2);
  HwPWM2.setClockDiv(NRF_PWM_CLK_1MHz);
  HwPWM2.setMaxValue(255);
  HwPWM2.addPin(PIN_TAIL_C);
  HwPWM2.addPin(PIN_TAIL_L);
  HwPWM2.writeChannel(0, 255);
  HwPWM2.writeChannel(1, 255);
  if (!DEBUG_IMU) {
    HwPWM2.addPin(PIN_TAIL_R);
    HwPWM2.writeChannel(2, 255);
  }

  analogReadResolution(14);

  underlight.begin();

  // Current
  // ~700 uA on
  // 6 uA powered down
  ina219.init();
  ina219.setADCMode(SAMPLE_MODE_128);

  // Init flash, filesystem and calibration & load calib json
  flash.begin();
  fatfs.begin(&flash);
  cal.begin(FILE_SENSOR_CALIB, &fatfs);
  cal.loadCalibration();

  // Light
  // 200 uA
  apds9960.begin();
  apds9960.enableColor(true);

  // IMU
  // 1250 uA high performance
  // 900 uA normal mode
  // 420 uA low power mode
  // 6 uA powered down
  lsm6ds33.begin_I2C();
  lsm6ds33.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
  lsm6ds33.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);
  lsm6ds33.setAccelDataRate(LSM6DS_RATE_104_HZ);
  lsm6ds33.setGyroDataRate(LSM6DS_RATE_104_HZ);
  lsm6ds33.setLowPowerMode(true);

  // Magnetometer
  // +1700 uA for 80 -> 1000 Hz (!)
  // 270 uA high res at 20 Hz
  // 40 uA low power at 20 Hz
  // 1 uA powered down
  lis3mdl.begin_I2C();
  lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);
  lis3mdl.setDataRate(LIS3MDL_DATARATE_80_HZ);
  lis3mdl.setPerformanceMode(LIS3MDL_MEDIUMMODE);
  lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);

  // Temperature, Barometer
  // 247 uA at 10 Hz
  bmp280.begin();
  enableTemperature(true);

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

#if configGENERATE_RUN_TIME_STATS
  bleruntimestats.begin();
#endif

  blevolts.begin();
  blevoltc.setProperties(CHR_PROPS_READ|CHR_PROPS_NOTIFY);
  blevoltc.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  blevoltc.setFixedLen(2);
  // UUID docs say units are 1/64 volt, but exponent must be power of 10...
  blevoltc.setPresentationFormatDescriptor(6, -2, UUID16_UNIT_ELECTRIC_POTENTIAL_DIFFERENCE_VOLT);
  blevoltc.begin();
  blevoltc.write16(0);
  blehl.begin();
  blehlmode.begin();
  bleul.begin();
  bleulmode.begin();
  bleuleffect.begin();
  bleulcolor.begin();
  bleulspeed.begin();

  Serial.println("bluetooth services started");

  // Quaternion with sensor calibration
  //bleQuater.begin(&filter, lsm6ds33.getAccelerometerSensor(), lsm6ds33.getGyroSensor(), &lis3mdl);
  //bleQuater.setCalibration(&cal);

  filter.begin(FILTER_UPDATE_RATE_HZ);
  imu_task.create();

  output_task.create();
  display_task.create();

  startAdv();

  notify_timer.begin(500, notify_timer_cb, NULL, true);
  notify_timer.start();

  Serial.println("begin finished");
  nrf_wdt_task_trigger(NRF_WDT, NRF_WDT_TASK_START);
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

void enableTemperature(bool enabled) {
  bmp280.setSampling(
    enabled ? Adafruit_BMP280::MODE_NORMAL : Adafruit_BMP280::MODE_SLEEP,
    Adafruit_BMP280::SAMPLING_X2, // temperature
    Adafruit_BMP280::SAMPLING_X16, // pressure
    Adafruit_BMP280::FILTER_X4, // filter
    Adafruit_BMP280::STANDBY_MS_63 // = 10 Hz ODR
  );
}

void notify_timer_cb(TimerHandle_t xTimer) {
  uint16_t volt = vext * 100;
  if (blevoltc.read16() != volt) {
    blevoltc.notify16(volt);
  }

  // Formula from https://electronics.stackexchange.com/a/551667
  // Matches table at https://blog.ampow.com/lipo-voltage-chart/
  float vbat_percent = 123 - (123 / pow(1+pow(vbat/3.7, 80), 0.165));

  blebas.notify(vbat_percent);
}

sensors_event_t accel_evt, gyro_evt, temp_evt, mag_evt;
void _imu_update() {
  if (DEBUG_IMU) {
    digitalWrite(LED_RED, HIGH);
  }
  xSemaphoreTake(xWireSemaphore, portMAX_DELAY);

  // get sensor events
  lsm6ds33.getEvent(&accel_evt, &gyro_evt, &temp_evt);
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
  if (DEBUG_IMU) {
    digitalWrite(LED_RED, LOW);
  }
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

void printAngle(Print& print, float angle, uint8_t symbol = DEG, uint8_t digits1 = 3)
{
  printFixed(print, angle, digits1, DEC, ' ');
  print.write('.');
  printFixed(print, (int)(abs(angle*100)) % 100, 2, DEC, '0');
  if (symbol) {
    print.write(symbol);
  }
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

const uint8_t PROGMEM gamma8[] = {
    0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
    0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  1,  1,  1,
    1,  1,  1,  1,  1,  1,  1,  1,  1,  2,  2,  2,  2,  2,  2,  2,
    2,  3,  3,  3,  3,  3,  3,  3,  4,  4,  4,  4,  4,  5,  5,  5,
    5,  6,  6,  6,  6,  7,  7,  7,  7,  8,  8,  8,  9,  9,  9, 10,
   10, 10, 11, 11, 11, 12, 12, 13, 13, 13, 14, 14, 15, 15, 16, 16,
   17, 17, 18, 18, 19, 19, 20, 20, 21, 21, 22, 22, 23, 24, 24, 25,
   25, 26, 27, 27, 28, 29, 29, 30, 31, 32, 32, 33, 34, 35, 35, 36,
   37, 38, 39, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 50,
   51, 52, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 66, 67, 68,
   69, 70, 72, 73, 74, 75, 77, 78, 79, 81, 82, 83, 85, 86, 87, 89,
   90, 92, 93, 95, 96, 98, 99,101,102,104,105,107,109,110,112,114,
  115,117,119,120,122,124,126,127,129,131,133,135,137,138,140,142,
  144,146,148,150,152,154,156,158,160,162,164,167,169,171,173,175,
  177,180,182,184,186,189,191,193,196,198,200,203,205,208,210,213,
  215,218,220,223,225,228,231,233,236,239,241,244,247,249,252,255 };

template<class T> constexpr const T addclamp(const T a, const T b) {
  return ((std::numeric_limits<T>::max() - a) < b) ? std::numeric_limits<T>::max() : (a + b);
}
template<class T> constexpr const T subclamp(const T a, const T b) {
  return (a < b) ? 0 : a - b;
}
template<class T> constexpr const T stepclamp(const T current, const T target, const T step) {
  return (target < current) ? (
    ((current - target) < step) ? target : subclamp(current, step)
  ) : (
    ((target - current) < step) ? target : addclamp(current, step)
  );
}

void _output_update() {
  TickType_t now = xTaskGetTickCount();

  //while ( !Serial ) delay(10); // XXX
  
  //lis3mdl.read();
  //float heading = filter.getRoll();
  if (xSemaphoreTake(xWireSemaphore, 10) != pdTRUE) {
    return;
  }
  if (display_on) {
    vext = ina219.getBusVoltage_V();
  } else {
    if ((now-last_vext_poll_time) > VEXT_POLL_PERIOD) {
      // Wake up INA219 and trigger a voltage read
      ina219.writeRegister(
        INA219_REG_CONFIG,
        INA219_CONFIG_BVOLTAGERANGE_32V |
        INA219_CONFIG_GAIN_8_320MV | INA219_CONFIG_BADCRES_12BIT |
        INA219_CONFIG_SADCRES_12BIT_1S_532US |
        INA219_CONFIG_MODE_BVOLT_TRIGGERED
      );
      while(!(ina219.readRegister(INA219_REG_BUSVOLTAGE) & 0x0002)); // checks if sampling is completed
      vext = ina219.getBusVoltage_V();
      ina219.setMeasureMode(POWER_DOWN);
      last_vext_poll_time = now;
    }
  }
  uint16_t r,g,b,c;
  apds9960.getColorData(&r, &g, &b, &c);
  // 3.5 counts/lux in the c channel according to datasheet
  xSemaphoreGive(xWireSemaphore);
  lux = c/3.5;
  lux = max(lux, apds9960.calculateLux(r, g, b)); // If C is overloaded, use RGB

  vbus_detected = NRF_POWER->USBREGSTATUS & POWER_USBREGSTATUS_VBUSDETECT_Msk;

  // Update mode
  accel_mag = vecMag(accel_evt.acceleration);
  if (accel_mag > 12) {
    last_move_time = now;
  }
  // TODO: Use IMU angle for mode too?
  if (vbus_detected) {
    last_vbus_time = now;
  }

  // Update underlight
  bool underlight_on = vbus_detected;
  uint8_t underlight_target_brightness = 0;
  switch (underlight_mode) {
    case UL_FORCE:
    underlight_on = true;
    case UL_ON:
    underlight_target_brightness = 255;
    break;
    case UL_AUTO:
    if (vbus_detected && (now - last_move_time) < LAST_MOVE_UNDER_TIMEOUT) {
      underlight_target_brightness = 255;
    }
    break;
  }
  underlight_brightness = stepclamp(underlight_brightness, underlight_target_brightness, (uint8_t)3);
  if (underlight_brightness == 0) {
    // Power down completely when we reach 0 brightness
    underlight_on = false;
  }
  //Serial.printf("On: %d brightness: %02x target: %02x\n", underlight_on, underlight_brightness, underlight_target_brightness);
  underlight.setBrightness(underlight.gamma8(underlight_brightness));
  // TODO: Make sure it doesn't cause flashing if we enable power before drive is set
  digitalWrite(PIN_POWER_ENABLE, underlight_on);

  bool taillight_on = false;
  if (vext < 11.2 || !vbus_detected) {
    brightness = 0;
    taillight_brightness = 0;
    actual_mode = OFF;
  } else {
    last_vext_time = now;
    taillight_on = (now - last_move_time) < LAST_MOVE_TAIL_TIMEOUT;
    switch (desired_mode) {
      case AUTO:
        if (lux > 10) {
          actual_mode = DAY;
        } else {
          actual_mode = NIGHT;
        }
        if ((now - last_move_time) > LAST_MOVE_TIMEOUT) {
          actual_mode = OFF;
        }
        break;
      default:
        actual_mode = desired_mode;
    }
  }

  // Update headlight
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
  brightness = stepclamp(brightness, target_brightness, (float)0.01);
  HwPWM3.writeChannel(0, PWM_MAX_HEADLIGHT * brightness, true); // Invert (high = off)

  // Update taillight
  float taillight_target_brightness = taillight_on ? 1 : 0;
  taillight_brightness = stepclamp(taillight_brightness, taillight_target_brightness, (float)0.05);
  if (taillight_brightness == 0) {
    // Onboard red LED (on channel 2) uses up to 1 mA!
    for (int i = 0; i < 3; i++) {
      HwPWM2.writeChannel(i, 0);
    }
  } else {
    float tail_phase = (now % TAIL_PERIOD) / (float)TAIL_PERIOD;
    float tail_intensity = (tail_phase*2);
    if (tail_intensity > 1) {
      tail_intensity = 2-tail_intensity;
    }
    //tail_intensity *= tail_intensity*tail_intensity; // Apply gamma of 3
    uint16_t tail_pwm_1 = gamma8[(uint8_t)(255*taillight_brightness*tail_intensity)];
    uint16_t tail_pwm_2 = gamma8[(uint8_t)(255*taillight_brightness*(1-tail_intensity))];
    HwPWM2.writeChannel(0, tail_pwm_1);
    HwPWM2.writeChannel(1, tail_pwm_2);
    if (!DEBUG_IMU) {
      HwPWM2.writeChannel(2, tail_pwm_2);
    }
  }
  if (underlight_on) {
    _underlight_update();
  }

  // Update BLE characteristics
  // TODO: Notify
  //blevoltc.write16(vext*64);
}

uint32_t underlight_frame = 0;
void _underlight_update() {
  uint32_t color1 = underlight.Color(underlight_color.red, underlight_color.green, underlight_color.blue, underlight_color.white);
  switch (underlight_effect) {
    case SOLID:
    underlight.fill(color1);
    break;
    case COLOR_WIPE:
    colorWipe(underlight, underlight_frame, underlight_speed, color1);
    break;
    case THEATER_CHASE:
    theaterChase(underlight, underlight_frame, underlight_speed, color1);
    break;
    case RAINBOW:
    rainbow(underlight, underlight_frame, underlight_speed);
    break;
    case THEATER_CHASE_RAINBOW:
    theaterChaseRainbow(underlight, underlight_frame, underlight_speed);
    break;
    case CYLON_BOUNCE:
    cylonBounce(underlight, underlight_frame, underlight_speed, color1);
    break;
    case FIRE:
    fire(underlight, underlight_frame, underlight_speed);
    break;
  }
  underlight.show();
  underlight_frame++;
}

void i2cyield() {
  xSemaphoreGive(xWireSemaphore);
  yield();
  xSemaphoreTake(xWireSemaphore, portMAX_DELAY);
}

void _display_update() {
  // Display-only data
  
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

  vbat = analogRead(PIN_VBAT) * 3.6f * 2.0f / 16384.0f;

  //return; // XXX: Test to see what it does to power consumption

  TickType_t now = xTaskGetTickCount();

  bool new_display_on = (now-max(last_move_time, last_vbus_time)) < DISPLAY_TIMEOUT;

  if (new_display_on != display_on) {
    xSemaphoreTake(xWireSemaphore, portMAX_DELAY);
    // Turning off the temperature sensor saves about 247 uA
    enableTemperature(new_display_on);
    // Turning off the OLED with oled.dim(true) but continuing to draw saves about 4 mA
    // Turning off the OLED with oled.dim(true) saves about 4.5 mA
    // Turning off the driver and charge pump saves about 5 mA
    // Turning off the INA219 saves about an additional 0.7 mA
    if (new_display_on) {
      oled.ssd1306_command(SSD1306_CHARGEPUMP);
      oled.ssd1306_command(0x14);
      oled.ssd1306_command(SSD1306_DISPLAYON);
      Bluefruit.autoConnLed(true);
      if (Bluefruit.Advertising.isRunning()) {
        Bluefruit._startConnLed();
      } else {
        Bluefruit._stopConnLed();
      }
      ina219.powerUp();
      //ina219.init();
      // TODO: Refactor to call common code
      //ina219.setADCMode(SAMPLE_MODE_128);
    } else {
      oled.ssd1306_command(SSD1306_DISPLAYOFF);
      oled.ssd1306_command(SSD1306_CHARGEPUMP);
      oled.ssd1306_command(0x10);
      Bluefruit._stopConnLed();
      Bluefruit._setConnLed(false);
      Bluefruit.autoConnLed(false);
      ina219.powerDown();
    }
    display_on = new_display_on;
    xSemaphoreGive(xWireSemaphore);
  }

  if (!display_on) {
    return;
  }
  
  if (xSemaphoreTake(xWireSemaphore, 10) != pdTRUE) {
    return;
  }
  float temperature = bmp280.readTemperature() * 1.8 + 32.0; // degF
  float pressure = bmp280.readPressure() / 100; // hPa
  float temperature2 = temp_evt.temperature * 1.8 + 32.0; // degF
  float current_mA = ina219.getCurrent_mA();
  int mag_ok = lis3mdl.magneticFieldAvailable();
  xSemaphoreGive(xWireSemaphore);

  float altitude = 44330 * (1.0 - pow(pressure / 1013, 0.1903));
  
  // Display
  oled.clearDisplay();

  // Line 0
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
  oled.write(vbus_detected ? 'V' : 'v');

  // Line 1
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

  // Line 2
  oled.write('\n');
  oled.printf("%7.2fhPa ", pressure);
  oled.printf("%6.2f'", altitude*3.28084);

  // Line 3 (debug)
  oled.write('\n');
  //oled.print(accel);
  printFixed(oled, getRawHeading(), 3, DEC, ' ');
  oled.write(DEG);
  oled.print(mag_ok);

  printAngle(oled, temperature2, DEG);

  //oled.printf("%4d lux", lux2);

  // Line 4
  oled.write('\n');
  printAngle(oled, accel_mag / ONE_G, 'G');
  oled.print(F("    "));
  printFixed(oled, lux, 5, DEC, ' ');
  oled.print(F(" lux "));
  oled.write('\n');

  /*
  oled.print(F("IMU late: "));
  printFixed(oled, imu_task.skipped(), 3, DEC, '0');
  oled.write('\n');
  */
  
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
  if (actual_mode != OFF) {
    oled.write(' ');
    printFixed(oled, (LAST_MOVE_TIMEOUT - (now-last_move_time))/configTICK_RATE_HZ, 2, DEC, ' ');
    oled.write('s');
  }
  
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
  oled.display(i2cyield);
  xSemaphoreGive(xWireSemaphore);
}

void debug_wdt(Print& print) {
  if (nrf_wdt_started(NRF_WDT)) {
    print.print("WDT:");
  } else {
    print.print("wdt:");
  }
  for (int i = 0; i <= 7; i++) {
    nrf_wdt_rr_register_t x = (nrf_wdt_rr_register_t)i;
    if (nrf_wdt_reload_request_is_enabled(NRF_WDT, x)) {
      if (nrf_wdt_request_status(NRF_WDT, x)) {
        print.write('+');
      } else {
        print.write('-');
      }
    } else {
      print.write('x');
    }
  }
}

void loop() {
  // Everything is done by periodic tasks
  suspendLoop();
  yield();
}
