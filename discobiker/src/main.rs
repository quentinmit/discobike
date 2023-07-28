#![no_std]
#![no_main]

#![feature(type_alias_impl_trait)]
#![feature(cell_update)]
#![feature(result_option_inspect)]
#![feature(trace_macros)]
#![feature(generic_arg_infer)]

use core::cell::{Cell, RefCell, OnceCell};
use core::convert::TryInto;
use core::fmt;
use core::fmt::Write;
use core::mem;
use core::sync::atomic::{AtomicU8, Ordering};
use crate::drivers::neopixel::rgbw::{Rgbw8, RED};
use embassy_executor::Spawner;
use embassy_nrf::{self as _, bind_interrupts};
use embassy_executor::{self, InterruptExecutor};
use embassy_nrf::gpio::{Level, Output, OutputDrive};
use embassy_nrf::interrupt;
use embassy_nrf::interrupt::{InterruptExt, Priority};
use embassy_nrf::peripherals::{SAADC, TWISPI0};
use embassy_nrf::twim::{self, Twim};
use embassy_nrf::wdt;
use embassy_nrf::saadc;
use embassy_nrf::pac;
use embassy_nrf::peripherals;
use embassy_sync::blocking_mutex::ThreadModeMutex;
use embassy_time::{Duration, Instant, Ticker, Timer};
use futures::StreamExt;
use static_cell::StaticCell;

use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice as I2cBusDevice;
use embassy_sync::blocking_mutex::{
    raw::{CriticalSectionRawMutex, ThreadModeRawMutex},
    Mutex as BlockingMutex,
};
use embassy_sync::mutex::Mutex;

use discobiker::*;
use discobiker::panic;

use crate::drivers::neopixel::{rgb, rgb::NeoPixelRgb};

#[cfg(feature = "defmt")]
use defmt_rtt as _;

use nrf_softdevice::ble::{gatt_server, peripheral};
use nrf_softdevice::{ble::Connection, raw, Softdevice};
use panic_probe as _;

use num_enum::TryFromPrimitive;

use ector::{actor, spawn_context, ActorContext, ActorAddress};

use atomic_counter::Counter;

use ssd1306::size::DisplaySize128x64;

use serde::{Deserialize, Serialize, Serializer};

use num_traits::float::Float;

use dimensioned as dim;
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

bind_interrupts!(struct Irqs {
    SAADC => saadc::InterruptHandler;
    SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0 => twim::InterruptHandler<TWISPI0>;
});

mod ble;
use ble::*;

pub static DESIRED_STATE: BlockingMutex<CriticalSectionRawMutex, Cell<DesiredState>> =
    BlockingMutex::new(Cell::new(DesiredState {
        headlight_mode: HeadlightMode::AutoDim,
        underlight_mode: UnderlightMode::Auto,
        underlight_effect: Effect::Rainbow,
        underlight_speed: 256,
        underlight_color: RED,
    }));

pub static STATE: BlockingMutex<CriticalSectionRawMutex, Cell<ActualState>> =
    BlockingMutex::new(Cell::new(ActualState {
        headlight_brightness: 0.0,
        taillight_brightness: 0.0,
        underlight_brightness: 0,
        display_on: false,
        vbus_detected: false,
        vext_low: false,

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
    interrupt::SAADC.set_priority(interrupt::Priority::P6);
    let mut saadc = saadc::Saadc::new(psaadc, Irqs, config, [channel_config]);

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
        /* FIXME if let Some(server) = SERVER.borrow().borrow().as_ref() {
            if let Err(e) = server.bas.battery_level_set(&vbat_percent) {
                error!("battery_level_set had error: {:?}", e);
            }
        } */
        Timer::after(interval).await;
    }
}

//#[cfg(feature = "mdbt50q")]
#[embassy_executor::task]
async fn blinker(mut led: Output<'static, PinLed>, interval: Duration) {
    loop {
        if STATE.lock(|c| c.get().display_on) {
            led.set_high();
        }
        Timer::after(interval).await;
        if !STATE.lock(|c| c.get().display_on) || CONNECTION_COUNT.load(Ordering::Relaxed) == 0 {
            led.set_low();
        }
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

const WDT: bool = false;

#[allow(non_snake_case)]
#[interrupt]
unsafe fn SWI0_EGU0() {
    EXECUTOR_HIGH.on_interrupt()
}
static EXECUTOR_HIGH: InterruptExecutor = InterruptExecutor::new();

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_nrf::init(config());
    #[cfg(all(feature = "systemview-target", feature = "log"))]
    {
        LOGGER.init();
        ::log::set_logger(&LOGGER).ok();
        ::log::set_max_level(::log::LevelFilter::Trace);
    }

    interrupt::SWI0_EGU0.set_priority(interrupt::Priority::P7);
    let spawner_high = EXECUTOR_HIGH.start(interrupt::SWI0_EGU0);

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

    unwrap!(ble::init(spawner));

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
    interrupt::SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0.set_priority(interrupt::Priority::P6);
    let i2c = Twim::new(p.TWISPI0, Irqs, use_pin_sda!(p), use_pin_scl!(p), twimconfig);

    static I2C_BUS: StaticCell<Mutex<ThreadModeRawMutex, Twim<TWISPI0>>> = StaticCell::new();
    let i2c_bus = Mutex::<ThreadModeRawMutex, _>::new(i2c);
    let i2c_bus = I2C_BUS.init(i2c_bus);

    info!("Peripherals initialized, starting actors");

    static OUTPUT: ActorContext<actors::output::Output<CriticalSectionRawMutex>, CriticalSectionRawMutex> = ActorContext::new();

    let power_addr = actor!(
        spawner,
        power,
        actors::power::Power<'static, I2cDevice>,
        actors::power::Power::new(&STATE, I2cBusDevice::new(i2c_bus))
    );

    let light_addr = actor!(
        spawner,
        light,
        actors::light::Light<'static, I2cDevice>,
        actors::light::Light::new(&STATE, I2cBusDevice::new(i2c_bus))
            .await
            .unwrap()
    );

    let display_addr = actor!(
        spawner,
        display,
        actors::display::Display<'static, I2cDevice, DisplaySize128x64>,
        actors::display::Display::new(&STATE, &DESIRED_STATE, I2cBusDevice::new(i2c_bus), DisplaySize128x64)
    );

    let imu_addr = actor!(
        spawner,
        imu,
        actors::imu::Imu<'static, I2cDevice>,
        actors::imu::Imu::new(&STATE, I2cBusDevice::new(i2c_bus), LSM6DS33_ADDR)
    );

    let barometer_addr = actor!(
        spawner,
        barometer,
        actors::barometer::Barometer<'static, I2cDevice>,
        actors::barometer::Barometer::new(&STATE, I2cBusDevice::new(i2c_bus))
    );

    let sound_addr = actor!(
        spawner,
        sound,
        actors::sound::Sound<'static, CriticalSectionRawMutex>,
        actors::sound::Sound::new(
            p.PDM,
            use_pin_pdm_data!(p),
            use_pin_pdm_clock!(p),
            OUTPUT.address(),
        ),
        CriticalSectionRawMutex
    );

    spawn_context!(OUTPUT, spawner_high, output, actors::output::Output<'static, CriticalSectionRawMutex>,
        actors::output::Output::new(
            &STATE,
            &DESIRED_STATE,
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
            sound_addr,
        ),
        CriticalSectionRawMutex
    );
    unwrap!(spawner.spawn(adc_task(saadc, pin_vbat, Duration::from_millis(500))));
    //#[cfg(feature = "mdbt50q")]
    unwrap!(spawner.spawn(blinker(led, Duration::from_millis(500))));
    let mut ticker = Ticker::every(Duration::from_millis(100));
    let mut last_display_on = false;
    loop {
        let display_on = STATE.lock(|c| c.get().display_on);
        if display_on != last_display_on {
            futures::join!(
                power_addr.notify(display_on.into()),
                barometer_addr.notify(display_on.into()),
                display_addr.notify(display_on.into()),
                light_addr.notify(display_on.into()),
            );
            last_display_on = display_on;
        }
        ticker.next().await;
    }
}
