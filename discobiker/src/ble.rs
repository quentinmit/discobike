use nrf_softdevice::ble::gatt_server::builder::ServiceBuilder;
use nrf_softdevice::ble::gatt_server::characteristic::{Attribute, Metadata, Properties};
use nrf_softdevice::ble::gatt_server::{CharacteristicHandles, RegisterError, WriteOp};
use nrf_softdevice::ble::{gatt_server, peripheral, Connection, Uuid, SecurityMode, DeferredReadReply};
use nrf_softdevice::{raw, Softdevice};

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