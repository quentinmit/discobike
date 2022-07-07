#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

mod csmutex;

use core::cell::RefCell;
use core::mem;
use cortex_m_rt::entry;
use defmt::*;
use embassy::blocking_mutex::ThreadModeMutex;
use embassy::executor::{Executor, Spawner};
use embassy::time::{Duration, Timer};
use embassy::util::Forever;
use embassy_nrf as _;
use embassy_nrf::saadc;
use embassy_nrf::interrupt;
use embassy_nrf::gpio::{Level, Pull, Input, Output, OutputDrive};
use embassy_nrf::interrupt::Priority;
use embassy_nrf::peripherals;
use embassy_nrf::peripherals::{P0_13, SAADC, TWISPI0};
use embassy_nrf::twim::{self, Twim};

use nrf_softdevice::ble::{gatt_server, peripheral};
use nrf_softdevice::{raw, Softdevice};
use nrf_softdevice_defmt_rtt as _;
use panic_probe as _;

mod ina219;
use crate::ina219::{INA219, INA219_ADDR};

use shared_bus::{BusManager};

//use self::csmutex::CriticalSectionBusMutex;

use num_traits::float::Float;

type PIN_VBAT = peripherals::P0_29;

static EXECUTOR: Forever<Executor> = Forever::new();

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

static SERVER: ThreadModeMutex<RefCell<Option<Server>>> =
    ThreadModeMutex::new(RefCell::new(None));

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
                ServerEvent::Bas(_) => {},
            })
                .await;

            if let Err(e) = res {
                info!("gatt_server run exited with error: {:?}", e);
            }
        }
    }
}

#[embassy::task]
async fn adc_task(psaadc: SAADC, pin_vbat: PIN_VBAT, interval: Duration) {
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
            123 - (
                123.0 / (
                    (1f32+
                     (vbat/3.7).powi(80)
                    ).powf(0.165)
                )
            ) as u8
        };
        info!("vbat: {=i16} = {=f32} V = {=u8} %", &buf[0], vbat, vbat_percent);
        if let Some(server) = SERVER.borrow().borrow().as_ref() {
            if let Err(e) = server.bas.battery_level_set(vbat_percent) {
                error!("battery_level_set had error: {:?}", e);
            }
        }
        Timer::after(interval).await;
    }
}

#[embassy::task]
async fn blinker(mut led: Output<'static, P0_13>, interval: Duration) {
    loop {
        led.set_high();
        Timer::after(interval).await;
        led.set_low();
        Timer::after(interval).await;
    }
}

#[entry]
fn main() -> ! {
    info!("Hello World!");
    let mut config = embassy_nrf::config::Config::default();
    config.gpiote_interrupt_priority = Priority::P2;
    config.time_interrupt_priority = Priority::P2;
    let p = embassy_nrf::init(config);

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

    let led = Output::new(p.P0_13, Level::Low, OutputDrive::Standard);

    let saadc = p.SAADC;
    let pin_vbat = p.P0_29;

    let mut twimconfig = twim::Config::default();
    twimconfig.frequency = twim::Frequency::K400;
    twimconfig.sda_pullup = true;
    twimconfig.scl_pullup = true;
    let irq = interrupt::take!(SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0);
    let twi = Twim::new(p.TWISPI0, irq, p.P0_12, p.P0_11, twimconfig);

    let twi_bus = cortex_m::singleton!(
        : BusManager<csmutex::CriticalSectionBusMutex<twim::Twim<TWISPI0>>>
            = BusManager::new(twi)
    ).unwrap();

    let mut ina219_dev = INA219::new(twi_bus.acquire_i2c(), INA219_ADDR);

    //let mut ina219_dev = INA219::new(twi_bus.acquire_i2c(), INA219_ADDR);
    // ina219_dev.i2c.write(ina219_dev.address, &[
    //     0,
        

    let executor = EXECUTOR.put(Executor::new());
    executor.run(|spawner| {
        unwrap!(spawner.spawn(softdevice_task(sd)));
        unwrap!(spawner.spawn(bluetooth_task(sd)));
        unwrap!(spawner.spawn(adc_task(saadc, pin_vbat, Duration::from_millis(500))));
        unwrap!(spawner.spawn(blinker(led, Duration::from_millis(300))));
    });
}
