use arrayvec::ArrayVec;
use atomic_traits::fetch::Add;
use byteorder::{BigEndian, ByteOrder, LittleEndian};
use embassy_hal_common::{into_ref, PeripheralRef};
use embassy_nrf::pac::FICR;
use embassy_nrf::pac::radio::rxaddresses;
use crate::{pac, peripherals};
use pac::{radio, RADIO};

#[cfg(feature = "defmt")]
use defmt::Debug2Format;
#[allow(non_snake_case)]
#[cfg(not(feature = "defmt"))]
fn Debug2Format<'a, T: ::core::fmt::Debug>(item: &'a T) -> &'a T {
    item
}

pub struct Address {
    base_address: [u8; 4],
    prefix: u8,
}

pub use radio::txpower::TXPOWER_A as TxPower;

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Copy, Clone, PartialEq, Debug)]
pub enum Bitrate {
    Mbps1,
    Mbps2,
    //Kbps250,
    Mbps1_BLE,
    Mbps2_BLE,
}

impl From<Bitrate> for radio::mode::MODE_A {
    fn from(b: Bitrate) -> Self {
        use radio::mode::MODE_A::*;
        match b {
            Bitrate::Mbps1 => NRF_1MBIT,
            Bitrate::Mbps2 => NRF_2MBIT,
            //Bitrate::Kbps250 => NRF_
            Bitrate::Mbps1_BLE => BLE_1MBIT,
            Bitrate::Mbps2_BLE => BLE_2MBIT,
        }
    }
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Copy, Clone, PartialEq, Debug)]
pub enum Protocol {
    Esb,
    EsbDpl,
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Copy, Clone, PartialEq, Debug)]
pub enum CRC {
    Off,
    Bit8,
    Bit16,
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Copy, Clone, PartialEq, Debug)]
pub enum Mode {
    /// Primary transmitter mode.
    PTX,
    /// Primary receiver mode.
    PRX,
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Copy, Clone, PartialEq, Debug)]
pub enum TxMode {
    Auto,
    Manual,
    ManualStart,
}

pub struct Config {
    pub protocol: Protocol,
    pub mode: Mode,
    //pub event_handler: Fn

    pub bitrate: Bitrate,
    pub crc: CRC,

    pub tx_output_power: TxPower,

    pub retransmit_delay: u16,
    pub retransmit_count: u16,

    pub tx_mode: TxMode,

    // radio_irq_priority
    // event_irq_priority
    pub payload_length: u8,

    pub selective_auto_ack: bool,

    pub addresses: Addresses,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            protocol: Protocol::EsbDpl,
            mode: Mode::PTX,
            bitrate: Bitrate::Mbps2,
            crc: CRC::Bit16,
            tx_output_power: TxPower::_0D_BM,
            retransmit_delay: 250,
            retransmit_count: 3,
            tx_mode: TxMode::Auto,
            payload_length: 32,
            selective_auto_ack: false,
            addresses: Default::default(),
        }
    }
}

/// Convert an address from nRF24L format to nRF5 format
fn addr_convert(addr: &[u8]) -> u32 {
    // Treat the first byte as the MSB, then reverse the whole string so that when the radio reverses it again the first byte's MSB gets transmitted first.
    BigEndian::read_u32(&addr).reverse_bits()
}

pub struct Addresses {
    /// Base address for pipe 0, MSB first.
    base_addr_p0: [u8; 4],
    /// Base address for pipes 1-7, MSB first.
    base_addr_p1: [u8; 4],
    /// Address prefix for pipes 0-7.
    pipe_prefixes: ArrayVec<u8, 8>,
    /// Channel to use (must be between 0 and 100).
    rf_channel: u8,
}

impl Default for Addresses {
    fn default() -> Self {
        Self {
            base_addr_p0: [0xE7, 0xE7, 0xE7, 0xE7],
            base_addr_p1: [0xC2, 0xC2, 0xC2, 0xC2],
            pipe_prefixes: [0xE7, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8].into(),
            rf_channel: 2,
        }
    }
}

impl Addresses {
    fn set_rxaddresses<'d>(&self, w: &'d mut radio::rxaddresses::W) -> &'d mut radio::rxaddresses::W {
        w.addr0().bit(self.pipe_prefixes.len() > 0)
        .addr1().bit(self.pipe_prefixes.len() > 1)
        .addr2().bit(self.pipe_prefixes.len() > 2)
        .addr3().bit(self.pipe_prefixes.len() > 3)
        .addr4().bit(self.pipe_prefixes.len() > 4)
        .addr5().bit(self.pipe_prefixes.len() > 5)
        .addr6().bit(self.pipe_prefixes.len() > 6)
        .addr7().bit(self.pipe_prefixes.len() > 7)
    }

    fn pipe_prefix_registers(&self) -> (u32, u32) {
        let mut prefixes = [0u8; 8];
        prefixes[..self.pipe_prefixes.len()].copy_from_slice(&self.pipe_prefixes);
        (addr_convert(&prefixes[0..4]), addr_convert(&prefixes[4..8]))
    }
}

// N.B. MAX_PACKET_LENGTH == MAX_PAYLOAD_LENGTH + 2
pub struct ESB<const MAX_PACKET_LENGTH: usize = 34> {
    //_p: PeripheralRef<'d, peripherals::RADIO>,
    config: Config,
    rx_payload_buffer: ArrayVec<u8, MAX_PACKET_LENGTH>,
}

impl <const MAX_PACKET_LENGTH: usize> ESB<MAX_PACKET_LENGTH> {
    pub fn new(
        r: &mut RADIO,
        config: Config
    ) -> Self {
        //let r = unsafe{ &*RADIO::ptr() };

        // update_tx_power
        r.txpower.write(|w| w.txpower().variant(config.tx_output_power));

        // update_radio_bitrate
        let mode = config.bitrate.into();
        r.mode.write(|w| w.mode().variant(mode));
        // TODO: set wait_for_ack_timeout
        // TODO: set retransmit_delay

        // update_radio_protocol
        
        // update_radio_crc
        use radio::crccnf::LEN_A;
        let (crcinit, crcpoly, crccnf) = match config.crc {
            CRC::Bit16 => (0xffff, 0x11021, LEN_A::TWO),
            CRC::Bit8 => (0xff, 0x107, LEN_A::ONE),
            CRC::Off => (0, 0, LEN_A::DISABLED),
        };
        r.crcinit.write(|w| unsafe { w.crcinit().bits(crcinit)});
        r.crcpoly.write(|w| unsafe { w.crcpoly().bits(crcpoly)});
        r.crccnf.write(|w| w.len().variant(crccnf));

        let addr_length = 4; // FIXME
        let balen = addr_length - 1;
        let max_payload_length = (MAX_PACKET_LENGTH as u8).saturating_sub(2);
        match config.protocol {
            Protocol::Esb => {
                r.pcnf0.write(|w| unsafe {
                    w.s0len().set_bit().lflen().bits(0).s1len().bits(1)
                });
                let payload_length = max_payload_length; // FIXME
                r.pcnf1.write(|w| unsafe {
                    w
                    .whiteen().disabled()
                    .endian().big()
                    .balen().bits(balen)
                    .statlen().bits(payload_length)
                    .maxlen().bits(payload_length)
                });
            },
            Protocol::EsbDpl => {
                r.pcnf0.write(|w| unsafe {
                    w.s0len().clear_bit().lflen().bits(if max_payload_length <= 32 { 6 } else { 8 }).s1len().bits(3)
                });
                r.pcnf1.write(|w| unsafe {
                    w.whiteen().disabled().endian().big().balen().bits(balen).statlen().bits(0).maxlen().bits(max_payload_length)
                });
            },
        }

        // Configure radio address registers
        // Note that the radio transmits the LSB of BASE first, followed by the LSB of the PREFIX byte.
        // The ESB protocol uses addresses that are MSB first, so we need to swap the bit and byte order.
        let (prefix0, prefix1) = config.addresses.pipe_prefix_registers();
        unsafe {
            r.base0.write(|w| w.base0().bits(
                addr_convert(&config.addresses.base_addr_p0)
            ));
            r.base1.write(|w| w.base1().bits(
                addr_convert(&config.addresses.base_addr_p1)
            ));
            r.prefix0.write(|w| w.bits(prefix0));
            r.prefix1.write(|w| w.bits(prefix1));
        }

        // TODO: initialize_fifos
        // TODO: sys_timer_init

        // TODO: ppi_init
        // Starting the timer
        // NRF_RADIO->EVENTS_READY -> NRF_ESB_SYS_TIMER->TASKS_START
        // Stopping the timer
        // NRF_RADIO->EVENTS_ADDRESS -> NRF_ESB_SYS_TIMER->TASKS_SHUTDOWN
        // RX timeout
        // NRF_ESB_SYS_TIMER->EVENTS_COMPARE[0] -> NRF_RADIO->TASKS_DISABLE
        // TX start
        // NRF_ESB_SYS_TIMER->EVENTS_COMPARE[1] -> NRF_RADIO->TASKS_TXEN

        // TODO: nvic_SetPriority
        
        // TODO: state = IDLE
        // TODO: initialize = true

        #[cfg(feature = "nrf52832")]
        {
            let rev = FICR::ptr().info.variant.read().bits() >> 8 & 0xFF;
            if rev == 0x45 {
                //Workaround for nRF52832 rev 2 errata 182
                unsafe {
                    *(0x4000173C as *mut u32) |= (1 << 10);
                }
            }
        }
        Self{
            config,
            rx_payload_buffer: ArrayVec::new(),
        }
    }

    fn common_shorts(w: &mut radio::shorts::W) -> &mut radio::shorts::W {
        w
        .ready_start().enabled()
        .end_disable().enabled()
        .address_rssistart().enabled()
        .disabled_rssistop().enabled()
    }

    pub fn start_rx(&mut self, r: &mut RADIO) {
        r.intenclr.write(|w| unsafe { w.bits(0xFFFFFFFF) });
        r.events_disabled.write(|w| w.events_disabled().clear_bit());

        r.shorts.modify(|_, w| Self::common_shorts(w).disabled_txen().enabled());
        r.intenset.write(|w| w.disabled().set());
        // state = PRX

        r.rxaddresses.write(|w| self.config.addresses.set_rxaddresses(w));
        r.frequency.write(|w| unsafe { w.frequency().bits(self.config.addresses.rf_channel) });

        // FIXME: Need to make sure this gets cleared before self is dropped.
        r.packetptr.write(|w| unsafe { w.packetptr().bits(self.rx_payload_buffer.as_mut_ptr() as u32) })
    }
}
