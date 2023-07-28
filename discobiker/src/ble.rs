use core::cell::OnceCell;
use core::convert::TryInto;
use core::fmt::Write;
use core::sync::atomic::AtomicU8;

use dimensioned::Dimensionless;
use dimensioned::si::f32consts::V;
use static_cell::StaticCell;
use crate::atomic_counter::Counter;
use crate::drivers::neopixel::rgbw::Rgbw8;
use embassy_executor::Spawner;
use embassy_nrf::pac;
use embassy_sync::blocking_mutex::ThreadModeMutex;
use nrf_softdevice::ble::gatt_server::builder::ServiceBuilder;
use nrf_softdevice::ble::gatt_server::characteristic::{Attribute, Metadata, Properties};
use nrf_softdevice::ble::gatt_server::{CharacteristicHandles, RegisterError, WriteOp};
use nrf_softdevice::ble::{gatt_server, peripheral, Connection, Uuid, SecurityMode, DeferredReadReply};
use nrf_softdevice::{raw, Softdevice};
use derive_more::From;

use crate::{debug, trace, info, warn, unwrap, STATE, DESIRED_STATE};

#[nrf_softdevice::gatt_service(uuid = "180f")]
pub struct BatteryService {
    #[characteristic(uuid = "2a19", read, notify)]
    battery_level: u8,
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

const EXTERNAL_BATTERY_SERVICE: Uuid = Uuid::new_16(0x2b18);

pub struct ExternalBatteryService {
    battery_volts: CharacteristicHandles,
}
#[allow(unused)]
pub enum ExternalBatteryServiceEvent {
    BatteryVoltsCccdWrite { notifications: bool },
    BatteryVoltsRead { offset: usize, reply: DeferredReadReply }
}

impl ExternalBatteryService {
    pub fn new(sd: &mut Softdevice) -> Result<Self, gatt_server::RegisterError> {
        let mut service_builder = ServiceBuilder::new(sd, EXTERNAL_BATTERY_SERVICE)?;

        let battery_volts = {
            let attr = Attribute::new(&[0u8; 2])
                .deferred_read()
                .read_security(SecurityMode::Open)
                .write_security(SecurityMode::NoAccess);
            let props = Properties::new()
                .read()
                .notify()
            ;
            let metadata = Metadata::new(props);
            service_builder.add_characteristic(
                EXTERNAL_BATTERY_SERVICE,
                attr,
                metadata,
            )?.build()
        };

        let _service_handle = service_builder.build();

        Ok(ExternalBatteryService {
            battery_volts,
        })
    }
    fn on_write(&self, handle: u16, data: &[u8]) -> Option<ExternalBatteryServiceEvent> {
        if handle == self.battery_volts.cccd_handle && !data.is_empty() {
            match data[0] & 0x01 {
                0x00 => {
                    return Some(ExternalBatteryServiceEvent::BatteryVoltsCccdWrite {
                        notifications: false,
                    })
                }
                0x01 => {
                    return Some(ExternalBatteryServiceEvent::BatteryVoltsCccdWrite {
                        notifications: true,
                    })
                }
                _ => {}
            }
        }
        None
    }
    fn on_deferred_read(&self, handle: u16, offset: usize, reply: DeferredReadReply) -> Option<ExternalBatteryServiceEvent> {
        if handle == self.battery_volts.value_handle {
            return Some(ExternalBatteryServiceEvent::BatteryVoltsRead { offset, reply });
        }
        None
    }
}

pub struct Server {
    bas: BatteryService,
    external_battery: ExternalBatteryService,
    headlight: HeadlightService,
    underlight: UnderlightService,
}

impl Server {
    pub fn new(
        sd: &mut Softdevice,
    ) -> Result<Self, gatt_server::RegisterError> {
        Ok(Self {
            bas: BatteryService::new(sd)?,
            external_battery: ExternalBatteryService::new(sd)?,
            headlight: HeadlightService::new(sd)?,
            underlight: UnderlightService::new(sd)?,
        })
    }
}
pub enum ServerEvent {
    ExternalBattery(ExternalBatteryServiceEvent),
    Headlight(HeadlightServiceEvent),
    Underlight(UnderlightServiceEvent),
}
impl ::nrf_softdevice::ble::gatt_server::Server for Server {
    type Event = ServerEvent;
    fn on_write(
        &self,
        _conn: &::nrf_softdevice::ble::Connection,
        handle: u16,
        op: ::nrf_softdevice::ble::gatt_server::WriteOp,
        offset: usize,
        data: &[u8],
    ) -> Option<Self::Event> {
        use ::nrf_softdevice::ble::gatt_server::Service;
        if let Some(e) = self.external_battery.on_write(handle, data) {
            return Some(ServerEvent::ExternalBattery(e));
        }
        if let Some(e) = self.headlight.on_write(handle, data) {
            return Some(ServerEvent::Headlight(e));
        }
        if let Some(e) = self.underlight.on_write(handle, data) {
            return Some(ServerEvent::Underlight(e));
        }
        None
    }
    fn on_deferred_read(&self, handle: u16, offset: usize, reply: DeferredReadReply) -> Option<Self::Event> {
        if let Some(e) = self.external_battery.on_deferred_read(handle, offset, reply) {
            return Some(ServerEvent::ExternalBattery(e))
        }
        None
    }

}

pub static CONNECTION_COUNT: AtomicU8 = AtomicU8::new(0);

#[embassy_executor::task(pool_size = "4")]
pub async fn gatt_server_task(sd: &'static Softdevice, conn: Connection, server: &'static Server) {
    let _counter = CONNECTION_COUNT.count_raii();
    // Run the GATT server on the connection. This returns when the connection gets disconnected.
    let res = gatt_server::run(&conn, server, |e| match e {
        ServerEvent::ExternalBattery(e) => match e {
            ExternalBatteryServiceEvent::BatteryVoltsCccdWrite { notifications } => {
                // TODO: Enable notifications
            }
            ExternalBatteryServiceEvent::BatteryVoltsRead { offset, reply } => {
                let vext = STATE.lock(|c| c.get().vext);
                let value = vext.map(|vext|
                    (((vext / V).value() * 100.0) as u16).to_le_bytes()
                );
                if let Err(e) = reply.reply(Ok(value.as_ref().map(|value| value.as_slice()))) {
                    warn!("replying: {:?}", e);
                }
            }
        }
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
                if let Ok(val) = val.try_into() {
                    DESIRED_STATE.lock(|c| {
                        c.update(|s| {
                            let mut s = s;
                            s.underlight_effect = val;
                            s
                        })
                    });
                }
            }
            UnderlightServiceEvent::UnderlightColorWrite(val) => {
                let color = Rgbw8::new(val[0], val[1], val[2], val[3]);
                DESIRED_STATE.lock(|c| {
                    c.update(|s| {
                        let mut s = s;
                        s.underlight_color = color;
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

    info!("gatt_server run exited");
}

#[embassy_executor::task]
async fn softdevice_task(sd: &'static Softdevice) {
    sd.run().await;
}

#[derive(Default)]
struct AdvertisementData(heapless::Vec<u8, 31>);

impl AdvertisementData {
    fn push_field(&mut self, data_type: u8, data: &[u8]) -> Result<(), ()>{
        let len = data.len() + 1;
        if (self.0.capacity() - self.0.len()) < (2 + data.len()) {
            return Err(());
        }
        self.0.push(len as u8).expect("sufficient capacit");
        self.0.push(data_type).expect("sufficient capacity");
        self.0.extend_from_slice(data).expect("sufficient capacity");
        Ok(())
    }
}

#[embassy_executor::task]
async fn bluetooth_task(spawner: Spawner, server: &'static Server, sd: &'static Softdevice, device_name: &'static str) {
    let mut adv_data = AdvertisementData::default();
    // 0x01 = flags
    adv_data.push_field(raw::BLE_GAP_AD_TYPE_FLAGS as u8, &[raw::BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE as u8]).unwrap();
    // 0x03 = service class UUIDs
    adv_data.push_field(raw::BLE_GAP_AD_TYPE_16BIT_SERVICE_UUID_COMPLETE as u8, &[0x16, 0x18]).unwrap();
    // 0x09 = complete local name
    const MAX_NAME_LEN: usize = 12;
    if device_name.len() > MAX_NAME_LEN {
        adv_data.push_field(raw::BLE_GAP_AD_TYPE_SHORT_LOCAL_NAME as u8, &device_name.as_bytes()[..MAX_NAME_LEN]).unwrap();
    } else {
        adv_data.push_field(raw::BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME as u8, device_name.as_bytes()).unwrap();
    }
    let scan_data = &[
        0x03, 0x03, 0x16, 0x18,
    ];

    loop {
        let config = peripheral::Config::default();
        let adv = peripheral::ConnectableAdvertisement::ScannableUndirected {
            adv_data: &adv_data.0,
            scan_data,
        };
        let conn = unwrap!(peripheral::advertise_connectable(sd, adv, &config).await);

        debug!("connection established");
        if let Err(e) = spawner.spawn(gatt_server_task(sd, conn, server)) {
            warn!("Error spawning gatt task: {:?}", e);
        }
    }
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(From, Clone, Debug)]
pub enum InitError {
    GattRegistrationError(gatt_server::RegisterError),
    SpawnError(embassy_executor::SpawnError),
}

pub fn init(spawner: Spawner) -> Result<(), InitError> {
    // Requires embassy-nrf/unstable-pac
    // TODO: Replace with safe API when one exists.
    let ficr = unsafe { &*pac::FICR::ptr() };
    let deviceid = (ficr.deviceid[0].read().deviceid().bits() as u64) | (ficr.deviceid[1].read().deviceid().bits() as u64) << 32;
    info!("FICR.DEVICEID = {:X}", deviceid);

    static DEVICE_NAME: ThreadModeMutex<OnceCell<heapless::String<32>>> = ThreadModeMutex::new(OnceCell::new());
    let device_name = DEVICE_NAME.borrow().get_or_init(|| {
        let mut out: heapless::String<_> = "DiscobikeR".into();
        let _ = write!(&mut out, " {:X}", deviceid);
        out
    });

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
            p_value: device_name.as_ptr() as _,//b"DiscobikeR" as *const u8 as _,
            current_len: device_name.len() as u16,
            max_len: device_name.len() as u16,
            write_perm: nrf_softdevice::ble::SecurityMode::NoAccess.into_raw(),
            _bitfield_1: raw::ble_gap_cfg_device_name_t::new_bitfield_1(
                raw::BLE_GATTS_VLOC_USER as u8,
            ),
        }),
        ..Default::default()
    };

    let sd = Softdevice::enable(&sdconfig);

    info!("Softdevice enabled");

    static SERVER: StaticCell<Server> = StaticCell::new();
    let server = SERVER.init(Server::new(sd)?);

    spawner.spawn(softdevice_task(sd))?;
    spawner.spawn(bluetooth_task(spawner, server, sd, device_name))?;

    Ok(())
}