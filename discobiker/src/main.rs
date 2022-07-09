#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use core::cell::RefCell;
use core::mem;
use defmt::*;
use embassy::blocking_mutex::ThreadModeMutex;
use embassy::executor::Spawner;
use embassy::time::{Duration, Timer};
use embassy::util::Forever;
use embassy_nrf as _;
use embassy_nrf::gpio::{Input, Level, Output, OutputDrive, Pull};
use embassy_nrf::interrupt;
use embassy_nrf::interrupt::Priority;
use embassy_nrf::peripherals::{SAADC, TWISPI0};
use embassy_nrf::twim::{self, Twim};
use embassy_nrf::wdt;
use embassy_nrf::{pac, saadc};
use embassy_nrf::{peripherals, Peripherals};

use embassy::blocking_mutex::raw::ThreadModeRawMutex;
use embassy::mutex::Mutex;
use embassy_embedded_hal::shared_bus::i2c::I2cBusDevice;

use drogue_device::drivers::led::neopixel::rgbw::{NeoPixelRgbw, GREEN};

use nrf_softdevice::ble::{gatt_server, peripheral};
use nrf_softdevice::{raw, Softdevice};
use nrf_softdevice_defmt_rtt as _;
use panic_probe as _;

use apds9960::Apds9960;

use paste::paste;

mod ina219;
use crate::ina219::{INA219, INA219_ADDR};
mod output;

use num_traits::float::Float;

macro_rules! define_pin {
    ($name:ident, $pin:ident) => {
        paste! {
                type [<Pin $name>] = peripherals::$pin;
            macro_rules! [<use_pin_ $name:snake>] {
                ($pstruct:ident) => {
                    $pstruct.$pin
                }
            }
        }
    };
}

// Pins
// * = exposed on header
// 2* / P0.10
// 3 / P1.11 - Gyro + Accel IRQ
// 4 / P1.10 - Blue LED ("Conn")
// 5* / P1.08 - NeoPixel Connector (Propmaker)
define_pin!(Underlight, P1_08);
// 6* / P0.07 - IRQ (Propmaker)
// 7 / P1.02 - Button
// 8 / P0.16 - NeoPixel
define_pin!(NeoPixel, P0_16);
// 9* / P0.26 - Button (Propmaker)
// 10* / P0.27 - Power Enable (Propmaker)
define_pin!(PowerEnable, P0_27);
// 11* / P0.06 - Red LED (Propmaker)
define_pin!(TailC, P0_06);
// 12* / P0.08 - Green LED (Propmaker)
define_pin!(TailL, P0_08);
// 13* / P1.09 - Blue LED (Propmaker)
// 13* / P1.09 - Red LED
define_pin!(TailR, P1_09);
// 34 / P0.00 - PDM microphone data
// 35 / P0.01 - PDM microphone clock
// 36 / P1.00 - APDS9960 Light Gesture Proximity IRQ
// 14 / A0* / P0.04 - Audio Amp Input (Propmaker)
// 15 / A1* / P0.05
define_pin!(Rst, P0_05);
// 16 / A2* / P0.30
// 17 / A3* / P0.28
// 18 / A4* / P0.02
// 19 / A5* / P0.03 - Headlight dim
define_pin!(HeadlightDim, P0_03);
// 20 / A6 / P0.29 - VOLTAGE_MONITOR (100K+100K voltage divider)
define_pin!(Vbat, P0_29);
// 21 / A7 / ???
// 22 / SDA* / P0.12 - I2C data
define_pin!(Sda, P0_12);
// 23 / SCL* / P0.11 - I2C clock
define_pin!(Scl, P0_11);
// 24 / MI* / P0.15
// 25 / MO* / P0.13
#[cfg(feature = "mdbt50q")]
define_pin!(Led, P0_13);
// 26 / SCK* / P0.14
// 0 / TX* / P0.25
// 1 / RX* / P0.24
// I2C devices
// 0x18 - LIS3DH Accelerometer (on propmaker)
// 0x1C - LIS3MDL Magnetometer
// 0x39 - APDS9960 Light Gesture Proximity
// 0x3C - SSD1315 OLED Display
// 0x40 - INA219 Current sensor (on INA)
// 0x44 - SHT30 Humidity
// 0x6A - LSM6DS33 Gyro + Accel
// 0x77 - BMP280 Temperature + Pressure

#[nrf_softdevice::gatt_service(uuid = "180f")]
struct BatteryService {
    #[characteristic(uuid = "2a19", read, notify)]
    battery_level: u8,
}

#[nrf_softdevice::gatt_server]
struct Server {
    bas: BatteryService,
}

#[embassy::task]
async fn softdevice_task(sd: &'static Softdevice) {
    sd.run().await;
}

static SERVER: ThreadModeMutex<RefCell<Option<Server>>> = ThreadModeMutex::new(RefCell::new(None));

#[embassy::task]
async fn bluetooth_task(sd: &'static Softdevice) {
    let server: Server = unwrap!(gatt_server::register(sd));

    let v = unwrap!(server.bas.battery_level_get());
    info!("Initial battery level: {=u8}", v);

    SERVER.borrow().replace(Some(server));

    #[rustfmt::skip]
    let adv_data = &[
        // 0x01 = flags
        0x02, 0x01, raw::BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE as u8,
        // 0x03 = service class UUIDs
        0x03, 0x03, 0x09, 0x18,
        // 0x09 = complete local name
        0x0b, 0x09, b'D', b'i', b's', b'c', b'o', b'b', b'i', b'k', b'e', b'R',
    ];
    #[rustfmt::skip]
    let scan_data = &[
        0x03, 0x03, 0x09, 0x18,
    ];

    loop {
        let config = peripheral::Config::default();
        let adv = peripheral::ConnectableAdvertisement::ScannableUndirected {
            adv_data,
            scan_data,
        };
        let conn = unwrap!(peripheral::advertise_connectable(sd, adv, &config).await);

        info!("advertising done!");
        if let Some(server) = SERVER.borrow().borrow().as_ref() {
            // Run the GATT server on the connection. This returns when the connection gets disconnected.
            let res = gatt_server::run(&conn, server, |e| match e {
                ServerEvent::Bas(_) => {}
            })
            .await;

            if let Err(e) = res {
                info!("gatt_server run exited with error: {:?}", e);
            }
        }
    }
}

#[embassy::task]
async fn adc_task(psaadc: SAADC, pin_vbat: PinVbat, interval: Duration) {
    let config = saadc::Config::default();
    let channel_config = saadc::ChannelConfig::single_ended(pin_vbat);
    let mut saadc = saadc::Saadc::new(psaadc, interrupt::take!(SAADC), config, [channel_config]);

    loop {
        let mut buf = [0; 1];
        saadc.sample(&mut buf).await;
        let vbat = (buf[0] as f32) * 3.6 * 2.0 / 16384.0;
        let vbat_percent = if vbat <= 0.0 {
            0
        } else {
            123 - (123.0 / ((1f32 + (vbat / 3.7).powi(80)).powf(0.165))) as u8
        };
        info!(
            "vbat: {=i16} = {=f32} V = {=u8} %",
            &buf[0], vbat, vbat_percent
        );
        if let Some(server) = SERVER.borrow().borrow().as_ref() {
            if let Err(e) = server.bas.battery_level_set(vbat_percent) {
                error!("battery_level_set had error: {:?}", e);
            }
        }
        Timer::after(interval).await;
    }
}

#[cfg(feature = "mdbt50q")]
#[embassy::task]
async fn blinker(mut led: Output<'static, PinLed>, interval: Duration) {
    loop {
        led.set_high();
        Timer::after(interval).await;
        led.set_low();
        Timer::after(interval).await;
    }
}

type I2cDevice = I2cBusDevice<'static, ThreadModeRawMutex, Twim<'static, TWISPI0>>;

fn start_wdt(p_wdt: peripherals::WDT) -> wdt::WatchdogHandle {
    let mut config = wdt::Config::default();
    config.timeout_ticks = 32768 * 5; // 5 seconds

    // This is needed for `probe-run` to be able to catch the panic message
    // in the WDT interrupt. The core resets 2 ticks after firing the interrupt.
    config.run_during_debug_halt = false;

    let (_wdt, [handle]) = match wdt::Watchdog::try_new(p_wdt, config) {
        Ok(x) => x,
        Err(_) => {
            info!("Watchdog already enabled; first boot after DFU? Waiting for it to timeout...");
            // TODO: neopixel.fill(neopixel.Color(0x11, 0, 0x11, 0));
            // TODO: neopixel.show();
            loop {}
        }
    };
    handle
}

fn config() -> embassy_nrf::config::Config {
    let mut config = embassy_nrf::config::Config::default();
    config.gpiote_interrupt_priority = Priority::P2;
    config.time_interrupt_priority = Priority::P2;
    config
}

#[embassy::main(config = "config()")]
async fn main(spawner: Spawner, p: Peripherals) {
    info!("Hello World!");
    let mut neopixel = unwrap!(NeoPixelRgbw::<'_, _, 1>::new(p.PWM0, use_pin_neo_pixel!(p)));
    if let Err(e) = neopixel.set(&[GREEN]).await {
        error!("failed to set neopixel on boot: {:?}", e);
    }

    let wdt_handle = start_wdt(p.WDT);

    let sdconfig = nrf_softdevice::Config {
        clock: Some(raw::nrf_clock_lf_cfg_t {
            source: raw::NRF_CLOCK_LF_SRC_RC as u8,
            rc_ctiv: 4,
            rc_temp_ctiv: 2,
            accuracy: raw::NRF_CLOCK_LF_ACCURACY_20_PPM as u8,
        }),
        conn_gap: Some(raw::ble_gap_conn_cfg_t {
            conn_count: 6,
            event_length: 24,
        }),
        conn_gatt: Some(raw::ble_gatt_conn_cfg_t { att_mtu: 256 }),
        gatts_attr_tab_size: Some(raw::ble_gatts_cfg_attr_tab_size_t {
            attr_tab_size: 32768,
        }),
        gap_role_count: Some(raw::ble_gap_cfg_role_count_t {
            adv_set_count: 1,
            periph_role_count: 3,
            central_role_count: 3,
            central_sec_count: 0,
            _bitfield_1: raw::ble_gap_cfg_role_count_t::new_bitfield_1(0),
        }),
        gap_device_name: Some(raw::ble_gap_cfg_device_name_t {
            p_value: b"DiscobikeR" as *const u8 as _,
            current_len: 9,
            max_len: 9,
            write_perm: unsafe { mem::zeroed() },
            _bitfield_1: raw::ble_gap_cfg_device_name_t::new_bitfield_1(
                raw::BLE_GATTS_VLOC_STACK as u8,
            ),
        }),
        ..Default::default()
    };

    let sd = Softdevice::enable(&sdconfig);

    #[cfg(feature = "mdbt50q")]
    let led = Output::new(use_pin_led!(p), Level::Low, OutputDrive::Standard);

    let saadc = p.SAADC;
    let pin_vbat = use_pin_vbat!(p);

    // Requires embassy-nrf/unstable-pac
    // TODO: Replace with safe API when one exists.
    let power: pac::POWER = unsafe { mem::transmute(()) };

    let mut twimconfig = twim::Config::default();
    twimconfig.frequency = twim::Frequency::K400;
    twimconfig.sda_pullup = true;
    twimconfig.scl_pullup = true;
    let irq = interrupt::take!(SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0);
    let i2c = Twim::new(p.TWISPI0, irq, use_pin_sda!(p), use_pin_scl!(p), twimconfig);

    static I2C_BUS: Forever<Mutex<ThreadModeRawMutex, Twim<TWISPI0>>> = Forever::new();
    let i2c_bus = Mutex::<ThreadModeRawMutex, _>::new(i2c);
    let i2c_bus = I2C_BUS.put(i2c_bus);

    let mut ina219_dev = INA219::new(I2cBusDevice::new(i2c_bus), INA219_ADDR);

    let apds9960 = Apds9960::new(I2cBusDevice::new(i2c_bus));

    unwrap!(spawner.spawn(softdevice_task(sd)));
    unwrap!(spawner.spawn(bluetooth_task(sd)));
    unwrap!(spawner.spawn(output::output_task(wdt_handle, power, ina219_dev, apds9960)));
    unwrap!(spawner.spawn(adc_task(saadc, pin_vbat, Duration::from_millis(500))));
    #[cfg(feature = "mdbt50q")]
    unwrap!(spawner.spawn(blinker(led, Duration::from_millis(300))));
}
