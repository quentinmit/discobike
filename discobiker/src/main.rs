#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(generic_associated_types)]
#![feature(cell_update)]
#![feature(result_option_inspect)]
#![feature(int_log)]
#![feature(trace_macros)]
#![feature(generic_arg_infer)]
#![feature(mixed_integer_ops)]

mod log;

use core::cell::{Cell, RefCell};
use core::convert::TryInto;
use core::fmt;
use core::mem;
use embassy_executor::Spawner;
use embassy_nrf as _;
use embassy_nrf::executor::InterruptExecutor;
use embassy_nrf::gpio::{Level, Output, OutputDrive};
use embassy_nrf::interrupt::{self, InterruptExt, Priority};
use embassy_nrf::peripherals::{SAADC, TWISPI0};
use embassy_nrf::twim::{self, Twim};
use embassy_nrf::wdt;
use embassy_nrf::{pac, saadc};
use embassy_nrf::{peripherals, Peripherals};
use embassy_nrf::rng::Rng;
use embassy_sync::blocking_mutex::{ThreadModeMutex, CriticalSectionMutex};
use embassy_time::{Duration, Instant, Timer};
use static_cell::StaticCell;

use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice as I2cBusDevice;
use embassy_sync::blocking_mutex::{
    raw::{CriticalSectionRawMutex, ThreadModeRawMutex},
    Mutex as BlockingMutex,
};
use embassy_sync::mutex::Mutex;

use drogue_device::drivers::led::neopixel::{rgb, rgb::NeoPixelRgb};

#[cfg(feature = "defmt")]
use defmt::Debug2Format;
#[cfg(feature = "defmt")]
use defmt_rtt as _;
#[allow(non_snake_case)]
#[cfg(not(feature = "defmt"))]
fn Debug2Format<'a, T: fmt::Debug>(item: &'a T) -> &'a T {
    item
}

use nrf_softdevice::ble::{gatt_server, peripheral};
use nrf_softdevice::{ble::Connection, raw, Softdevice};
use panic_probe as _;

use num_enum::TryFromPrimitive;
use paste::paste;

use ector::{spawn_actor, ActorContext};

mod actors;
mod drivers;
use ssd1306::size::DisplaySize128x64;

use serde::{Deserialize, Serialize, Serializer};

use num_traits::float::Float;

#[macro_use]
extern crate dimensioned as dim;
use dim::si::{f32consts::V, Ampere, Kelvin, Lux, MeterPerSecond2, Pascal, Volt};
use dim::Dimensionless;

cfg_if::cfg_if! {
    if #[cfg(feature = "systemview-target")] {
        use systemview_target::SystemView;
        use rtos_trace;
        static LOGGER: systemview_target::SystemView = systemview_target::SystemView::new();
        rtos_trace::global_trace!{SystemView}

        struct TraceInfo();

        impl rtos_trace::RtosTraceApplicationCallbacks for TraceInfo {
            fn system_description() {}
            fn sysclock() -> u32 {
                64000000
            }
        }
        rtos_trace::global_application_callbacks!{TraceInfo}
    }
}

const CELSIUS_ZERO: f32 = 273.15;

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
//define_pin!(ConnLed, P1_10);
#[cfg(not(feature = "mdbt50q"))]
define_pin!(Led, P1_10);
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
define_pin!(PdmData, P0_00);
// 35 / P0.01 - PDM microphone clock
define_pin!(PdmClock, P0_01);
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
const LSM6DS33_ADDR: u8 = 0x6A;
// 0x77 - BMP280 Temperature + Pressure

#[nrf_softdevice::gatt_service(uuid = "180f")]
pub struct BatteryService {
    #[characteristic(uuid = "2a19", read, notify)]
    battery_level: u8,
}

#[nrf_softdevice::gatt_service(uuid = "2b18")]
pub struct ExternalBatteryService {
    #[characteristic(uuid = "2b18", read, notify)]
    battery_volts: u8,
}

#[nrf_softdevice::gatt_service(uuid = "00000000-1fbd-c985-0843-2e5f29538d87")]
pub struct HeadlightService {
    #[characteristic(uuid = "00000001-1fbd-c985-0843-2e5f29538d87", read, write)]
    headlight_mode: u8,
}

#[nrf_softdevice::gatt_service(uuid = "00000100-1fbd-c985-0843-2e5f29538d87")]
pub struct UnderlightService {
    #[characteristic(uuid = "00000101-1fbd-c985-0843-2e5f29538d87", read, write)]
    underlight_mode: u8,
    #[characteristic(uuid = "00000102-1fbd-c985-0843-2e5f29538d87", read, write)]
    underlight_effect: u8,
    #[characteristic(uuid = "00000103-1fbd-c985-0843-2e5f29538d87", read, write)]
    underlight_color: [u8; 4],
    #[characteristic(uuid = "00000104-1fbd-c985-0843-2e5f29538d87", read, write)]
    underlight_speed: i16,
}

#[nrf_softdevice::gatt_server]
pub struct Server {
    bas: BatteryService,
    external_battery: ExternalBatteryService,
    headlight: HeadlightService,
    underlight: UnderlightService,
}

#[embassy_executor::task]
async fn softdevice_task(sd: &'static Softdevice) {
    sd.run().await;
}

static SERVER: ThreadModeMutex<RefCell<Option<&Server>>> = ThreadModeMutex::new(RefCell::new(None));
static SERVER_FOREVER: StaticCell<Server> = StaticCell::new();

#[derive(Copy, Clone)]
pub struct EventTimer {
    last: Instant,
}
impl EventTimer {
    pub const fn new() -> Self {
        EventTimer { last: Instant::MIN }
    }
    pub fn update(&mut self) {
        self.last = Instant::now();
    }
    pub fn elapsed(&self) -> Duration {
        self.last.elapsed()
    }
}

impl Serialize for EventTimer {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        serializer.serialize_f32(self.last.elapsed().as_micros() as f32 / 1e6)
    }
}

impl fmt::Display for EventTimer {
    fn fmt(&self, formatter: &mut fmt::Formatter) -> fmt::Result {
        if self.last == Instant::MIN {
            core::write!(formatter, "NEVER")
        } else {
            core::write!(formatter, "{}s", (Instant::now() - self.last).as_secs())
        }
    }
}

#[repr(u8)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Copy, Clone, PartialEq, Debug, Serialize, Deserialize, TryFromPrimitive)]
enum HeadlightMode {
    Off = 0,
    Auto = 1,
    Day = 2,
    Night = 3,
    Blink = 4,
}

#[repr(u8)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Copy, Clone, PartialEq, Debug, Serialize, Deserialize, TryFromPrimitive)]
enum UnderlightMode {
    Off = 0,
    Auto = 1,
    On = 2,
    ForceOn = 3,
}

#[repr(u8)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Copy, Clone, PartialEq, Debug, Serialize, Deserialize, TryFromPrimitive)]
enum Effect {
    Solid = 0,
    ColorWipe,
    TheaterChase,
    Rainbow,
    TheaterChaseRainbow,
    CylonBounce,
    Fire,
    VuMeter,
    RgbVuMeter,
    Pulse,
    Traffic,
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Copy, Clone)]
pub struct DesiredState {
    headlight_mode: HeadlightMode,
    underlight_mode: UnderlightMode,
    underlight_effect: Effect,
    underlight_speed: i16,
    // underlight_color
}

pub static DESIRED_STATE: BlockingMutex<CriticalSectionRawMutex, Cell<DesiredState>> =
    BlockingMutex::new(Cell::new(DesiredState {
        headlight_mode: HeadlightMode::Auto,
        underlight_mode: UnderlightMode::ForceOn, //UnderlightMode::Auto,
        underlight_effect: Effect::Traffic,       //Effect::Rainbow,
        underlight_speed: 1024,
    }));

#[derive(Copy, Clone, Serialize)]
pub struct ActualState {
    headlight_mode: HeadlightMode,
    headlight_brightness: f32,
    taillight_brightness: f32,
    underlight_brightness: u8,
    display_on: bool,
    vbus_detected: bool,

    vbus_timer: EventTimer,
    move_timer: EventTimer,
    vext_present_timer: EventTimer,
    vext_poll_timer: EventTimer,

    vbat: Option<Volt<f32>>,
    vext: Option<Volt<f32>>,
    current: Option<Ampere<f32>>,
    accel_mag: Option<MeterPerSecond2<f32>>,
    accel_temperature: Option<Kelvin<f32>>,
    lux: Option<Lux<f32>>,
    pressure: Option<Pascal<f32>>,
    temperature: Option<Kelvin<f32>>,
}

pub static STATE: BlockingMutex<CriticalSectionRawMutex, Cell<ActualState>> =
    BlockingMutex::new(Cell::new(ActualState {
        headlight_mode: HeadlightMode::Off,
        headlight_brightness: 0.0,
        taillight_brightness: 0.0,
        underlight_brightness: 0,
        display_on: false,
        vbus_detected: false,

        vbus_timer: EventTimer::new(),
        move_timer: EventTimer::new(),
        vext_present_timer: EventTimer::new(),
        vext_poll_timer: EventTimer::new(),

        vbat: None,
        vext: None,
        current: None,
        accel_mag: None,
        accel_temperature: None,
        lux: None,
        pressure: None,
        temperature: None,
    }));

fn bluetooth_start_server(sd: &mut Softdevice) -> Result<(), gatt_server::RegisterError> {
    let server: &Server = SERVER_FOREVER.init(Server::new(sd)?);

    let v = unwrap!(server.bas.battery_level_get());
    info!("Initial battery level: {}", v);

    SERVER.borrow().replace(Some(server));
    Ok(())
}

#[embassy_executor::task]
async fn bluetooth_task(spawner: Spawner, sd: &'static Softdevice) {
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

    let server = SERVER.borrow().borrow().unwrap();

    loop {
        let config = peripheral::Config::default();
        let adv = peripheral::ConnectableAdvertisement::ScannableUndirected {
            adv_data,
            scan_data,
        };
        let conn = unwrap!(peripheral::advertise_connectable(sd, adv, &config).await);

        debug!("connection established");
        if let Err(e) = spawner.spawn(gatt_server_task(sd, conn, server)) {
            warn!("Error spawning gatt task: {:?}", e);
        }
    }
}

#[embassy_executor::task(pool_size = "4")]
pub async fn gatt_server_task(sd: &'static Softdevice, conn: Connection, server: &'static Server) {
    // Run the GATT server on the connection. This returns when the connection gets disconnected.
    let res = gatt_server::run(&conn, server, |e| match e {
        ServerEvent::Bas(_) => {}
        ServerEvent::ExternalBattery(_) => {}
        ServerEvent::Headlight(e) => match e {
            HeadlightServiceEvent::HeadlightModeWrite(val) => {
                DESIRED_STATE.lock(|c| {
                    c.update(|s| {
                        let mut s = s;
                        if let Ok(val) = val.try_into() {
                            s.headlight_mode = val;
                        }
                        s
                    })
                });
            }
        },
        ServerEvent::Underlight(e) => match e {
            UnderlightServiceEvent::UnderlightModeWrite(val) => {
                DESIRED_STATE.lock(|c| {
                    c.update(|s| {
                        let mut s = s;
                        if let Ok(val) = val.try_into() {
                            s.underlight_mode = val;
                        }
                        s
                    })
                });
            }
            UnderlightServiceEvent::UnderlightEffectWrite(val) => {
                DESIRED_STATE.lock(|c| {
                    c.update(|s| {
                        let mut s = s;
                        if let Ok(val) = val.try_into() {
                            s.underlight_effect = val;
                        }
                        s
                    })
                });
            }
            UnderlightServiceEvent::UnderlightColorWrite(val) => {
                DESIRED_STATE.lock(|c| {
                    c.update(|s| {
                        let mut s = s;
                        //TODO: s.underlight_color = val;
                        s
                    })
                });
            }
            UnderlightServiceEvent::UnderlightSpeedWrite(val) => {
                DESIRED_STATE.lock(|c| {
                    c.update(|s| {
                        let mut s = s;
                        s.underlight_speed = val;
                        s
                    })
                });
            }
        },
    })
    .await;

    if let Err(e) = res {
        info!("gatt_server run exited with error: {:?}", e);
    }
}

fn calculate_adc_one_bit(
    config: &saadc::Config,
    channel_config: &saadc::ChannelConfig,
) -> Volt<f32> {
    use saadc::*;
    // RESULT = [V(P) – V(N) ] * GAIN/REFERENCE * 2(RESOLUTION - m)
    // buf[0] = V * (1/6) / (0.6 volts) * 2^12
    // V = buf[0] * 6 * 0.6 / 2^12
    let adc_resolution = match config.resolution {
        Resolution::_8BIT => 8,
        Resolution::_10BIT => 10,
        Resolution::_12BIT => 12,
        Resolution::_14BIT => 14,
        _ => panic!("unsupported resolution"),
    };
    let adc_gain = match channel_config.gain {
        Gain::GAIN1_6 => 1.0 / 6.0,
        Gain::GAIN1_5 => 1.0 / 5.0,
        Gain::GAIN1_4 => 1.0 / 4.0,
        Gain::GAIN1_3 => 1.0 / 3.0,
        Gain::GAIN1_2 => 1.0 / 2.0,
        Gain::GAIN1 => 1.0,
        Gain::GAIN2 => 2.0,
        Gain::GAIN4 => 4.0,
        _ => panic!("unsupported gain"),
    };
    let adc_reference = match channel_config.reference {
        Reference::INTERNAL => 0.6 * V,
        Reference::VDD1_4 => 3.3 / 4.0 * V,
        _ => panic!("unsupported reference"),
    };
    adc_reference / adc_gain / 2.0.powi(adc_resolution)
}

#[embassy_executor::task]
async fn adc_task(psaadc: SAADC, pin_vbat: PinVbat, interval: Duration) {
    let config = saadc::Config::default();
    // P0.29 / AIN5 is connected to Vbat through a 100K/100K resistor divider.
    let channel_config = saadc::ChannelConfig::single_ended(pin_vbat);
    let adc_one_bit = calculate_adc_one_bit(&config, &channel_config);
    let mut saadc = saadc::Saadc::new(psaadc, interrupt::take!(SAADC), config, [channel_config]);

    loop {
        let mut buf = [0; 1];
        saadc.sample(&mut buf).await;
        // Vbat/2 is being sampled
        let vbat = (buf[0] as f32) * 2.0 * adc_one_bit;
        let vbat_percent = if vbat <= 0.0 * V {
            0
        } else {
            123 - (123.0 / ((1f32 + (vbat / (3.7 * V)).powi(80)).powf(0.165))) as u8
        };
        STATE.lock(|c| {
            c.update(|s| {
                let mut s = s;
                s.vbat = Some(vbat);
                s
            })
        });
        trace!(
            "vbat: {} = {} V = {} %",
            &buf[0],
            (vbat / V).value(),
            vbat_percent
        );
        if let Some(server) = SERVER.borrow().borrow().as_ref() {
            if let Err(e) = server.bas.battery_level_set(vbat_percent) {
                error!("battery_level_set had error: {:?}", e);
            }
        }
        Timer::after(interval).await;
    }
}

//#[cfg(feature = "mdbt50q")]
#[embassy_executor::task]
async fn blinker(mut led: Output<'static, PinLed>, interval: Duration) {
    loop {
        led.set_high();
        Timer::after(interval).await;
        led.set_low();
        Timer::after(interval).await;
    }
}

type I2cDevice = I2cBusDevice<'static, ThreadModeRawMutex, Twim<'static, TWISPI0>>;

async fn start_wdt<'l, T: embassy_nrf::pwm::Instance>(
    p_wdt: peripherals::WDT,
    neopixel: &mut NeoPixelRgb<'l, T, 1>,
) -> wdt::WatchdogHandle {
    let mut config = wdt::Config::default();
    config.timeout_ticks = 32768 * 5; // 5 seconds

    // This is needed for `probe-run` to be able to catch the panic message
    // in the WDT interrupt. The core resets 2 ticks after firing the interrupt.
    config.run_during_debug_halt = false;

    let (_wdt, [handle]) = match wdt::Watchdog::try_new(p_wdt, config) {
        Ok(x) => x,
        Err(_) => {
            info!("Watchdog already enabled; first boot after DFU? Waiting for it to timeout...");
            if let Err(e) = neopixel.set(&[rgb::Rgb8::new(0x11, 0, 0x11)]).await {
                error!("failed to set neopixel: {:?}", e);
            }
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

const BLUETOOTH: bool = true;
const WDT: bool = false;

static EXECUTOR_HIGH: StaticCell<InterruptExecutor<interrupt::SWI0_EGU0>> = StaticCell::new();

static RNG: CriticalSectionMutex<RefCell<Option<Rng>>> = CriticalSectionMutex::new(RefCell::new(None));

#[embassy_executor::main()]
async fn main(spawner: Spawner) {
    let p = embassy_nrf::init(config());
    #[cfg(all(feature = "systemview-target", feature = "log"))]
    {
        LOGGER.init();
        ::log::set_logger(&LOGGER).ok();
        ::log::set_max_level(::log::LevelFilter::Trace);
    }

    let irq = interrupt::take!(SWI0_EGU0);
    irq.set_priority(interrupt::Priority::P7);
    let executor = EXECUTOR_HIGH.init(InterruptExecutor::new(irq));
    let spawner_high = executor.start();

    info!("Booting!");
    let mut neopixel = unwrap!(NeoPixelRgb::<'_, _, 1>::new(p.PWM0, use_pin_neo_pixel!(p)));
    if let Err(e) = neopixel.set(&[rgb::BLACK]).await {
        error!("failed to set neopixel on boot: {:?}", e);
    }
    info!("neopixel set on boot");
    let wdt_handle = if WDT {
        Some(start_wdt(p.WDT, &mut neopixel).await)
    } else {
        None
    };
    info!("WDT started");

    //let mut rng = Rng::new(p.RNG, interrupt::take!(RNG));
    //RNG.lock(|c| c.replace(Some(rng)));

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

    info!("Softdevice enabled");

    //#[cfg(feature = "mdbt50q")]
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

    static I2C_BUS: StaticCell<Mutex<ThreadModeRawMutex, Twim<TWISPI0>>> = StaticCell::new();
    let i2c_bus = Mutex::<ThreadModeRawMutex, _>::new(i2c);
    let i2c_bus = I2C_BUS.init(i2c_bus);

    info!("Peripherals initialized, starting actors");

    static OUTPUT: ActorContext<actors::output::Output> = ActorContext::new();

    let power_actor = spawn_actor!(
        spawner,
        POWER,
        actors::power::Power<I2cDevice>,
        actors::power::Power::new(I2cBusDevice::new(i2c_bus))
    );

    let light = spawn_actor!(
        spawner,
        LIGHT,
        actors::light::Light<I2cDevice>,
        actors::light::Light::new(I2cBusDevice::new(i2c_bus))
            .await
            .unwrap()
    );
    light.notify(actors::light::LightMessage::On).await;

    let display = spawn_actor!(
        spawner, DISPLAY, actors::display::Display<I2cDevice, DisplaySize128x64>,
        actors::display::Display::new(I2cBusDevice::new(i2c_bus), DisplaySize128x64)
    );
    display.notify(actors::display::DisplayMessage::On).await;

    let imu = spawn_actor!(
        spawner,
        IMU,
        actors::imu::Imu<I2cDevice>,
        actors::imu::Imu::new(I2cBusDevice::new(i2c_bus))
    );

    let barometer = spawn_actor!(
        spawner,
        BAROMETER,
        actors::barometer::Barometer<I2cDevice>,
        actors::barometer::Barometer::new(I2cBusDevice::new(i2c_bus))
    );
    barometer
        .notify(actors::barometer::BarometerMessage::On)
        .await;

    if BLUETOOTH {
        unwrap!(bluetooth_start_server(sd));
        unwrap!(spawner.spawn(softdevice_task(sd)));
        unwrap!(spawner.spawn(bluetooth_task(spawner, sd)));
    }

    let sound = spawn_actor!(
        spawner,
        SOUND,
        actors::sound::Sound,
        actors::sound::Sound::new(
            p.PDM,
            use_pin_pdm_data!(p),
            use_pin_pdm_clock!(p),
            OUTPUT.address(),
        )
    );

    OUTPUT.mount(
        spawner,
        actors::output::Output::new(
            wdt_handle,
            power,
            p.PWM1,
            p.PWM2,
            p.PWM3,
            use_pin_power_enable!(p),
            use_pin_headlight_dim!(p),
            use_pin_tail_l!(p),
            use_pin_tail_c!(p),
            use_pin_tail_r!(p),
            use_pin_underlight!(p),
            sound,
        ),
    );
    unwrap!(spawner.spawn(adc_task(saadc, pin_vbat, Duration::from_millis(500))));
    //#[cfg(feature = "mdbt50q")]
    unwrap!(spawner.spawn(blinker(led, Duration::from_millis(300))));
}
