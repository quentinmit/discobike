use super::output::OutputMessage;
use crate::{Debug2Format, STATE};
use core::fmt;
use core::sync::atomic::{AtomicBool, Ordering};
use ector::{actor, Actor, Address, Inbox};
use embassy_nrf::interrupt;
use embassy_nrf::pdm::{Channels, Config, Frequency, Pdm, Ratio, SamplerState};
use embassy_nrf::peripherals;
use embassy_time::{Duration, Timer};
use embedded_hal_async::i2c;
use fixed::types::I7F1;
use futures::{pin_mut, select_biased, FutureExt};
use microfft::real::rfft_512;
use num_integer::Roots;

pub(super) type PdmBuffer = [i16; 512];

pub struct Sound<'a> {
    pdm: Pdm<'a>,
    output: Address<OutputMessage>,
}

pub enum SoundMessage {
    On,
    Off,
}
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug)]
pub struct SoundData {
    pub amplitude: u16,
    pub bands: [u16; 8],
}

impl Sound<'_> {
    pub fn new(
        p: peripherals::PDM,
        mut pin_data: crate::PinPdmData,
        mut pin_clock: crate::PinPdmClock,
        output: Address<super::output::OutputMessage>,
    ) -> Self {
        let mut config = Config::default();
        config.frequency = Frequency::_1280K; // 16 kHz sample rate
        config.ratio = Ratio::RATIO80;
        config.channels = Channels::Mono;
        config.gain_left = I7F1::from_bits(20); // 10 dB gain
        let pdm = Pdm::new(p, interrupt::take!(PDM), pin_data, pin_clock, config);
        Sound { pdm, output }
    }

    fn convert_sound_buf(buf: &PdmBuffer) -> SoundData {
        let mean = (buf.iter().map(|v| i32::from(*v)).sum::<i32>() / buf.len() as i32) as i16;
        let amplitude = (buf
            .iter()
            .map(|v| i32::from((*v).saturating_sub(mean)).pow(2))
            .fold(0i32, |a, b| a.saturating_add(b))
            / buf.len() as i32)
            .sqrt() as u16;

        let mut f = [0f32; 512];
        for i in 0..buf.len() {
            f[i] = (buf[i] as f32) / 32768.0;
        }
        // N.B. rfft_512 does the FFT in-place so result is actually also a reference to f.
        let result = rfft_512(&mut f);
        result[0].im = 0.0;
        let mut bands = [0u16; 8];
        for i in 0..bands.len() {
            let indices = 2usize.pow(i as u32)..2usize.pow((i + 1) as u32);
            let denom = indices.len();
            bands[i] = ((result[indices].iter().map(|c| c.norm_sqr()).sum::<f32>() * 32768.0
                / denom as f32) as u32)
                .sqrt() as u16;
        }
        SoundData { amplitude, bands }
    }

    async fn run_sampler<M>(&mut self, inbox: &mut M)
    where
        M: Inbox<SoundMessage>,
    {
        let mut bufs = [[0; 512]; 2];

        let mut running = &AtomicBool::new(true);

        let output = &self.output;

        let sampler_fut = self
            .pdm
            .run_task_sampler(&mut bufs, move |buf| {
                //let mean = (buf.iter().map(|v| i32::from(*v)).sum::<i32>() / buf.len() as i32) as i16;
                /* let (peak_freq_index, peak_mag) = fft_peak_freq(&buf);
                let peak_freq = peak_freq_index * 16000 / buf.len();
                info!(
                    "{} samples, min {=i16}, max {=i16}, mean {=i16}, AC RMS {=i16}, peak {} @ {} Hz",
                    buf.len(),
                    buf.iter().min().unwrap(),
                    buf.iter().max().unwrap(),
                    mean,
                    (
                        buf.iter().map(|v| i32::from(*v - mean).pow(2)).fold(0i32, |a,b| a.saturating_add(b))
                    / buf.len() as i32).sqrt() as i16,
                    peak_mag, peak_freq,
                ); */
                if let Err(e) =
                    output.try_notify(OutputMessage::SoundData(Self::convert_sound_buf(buf)))
                {
                    warn!("dropped sound frame");
                }
                match running.load(Ordering::SeqCst) {
                    true => SamplerState::Sampled,
                    false => SamplerState::Stopped,
                }
            })
            .fuse();
        pin_mut!(sampler_fut);

        loop {
            select_biased! {
                message = inbox.next().fuse() => {
                    match message {
                        SoundMessage::On => warn!("sound on while already on"),
                        SoundMessage::Off => running.store(false, Ordering::SeqCst),
                    }
                },
                _ = sampler_fut => {
                    return;
                }
            };
        }
    }
    async fn run_sound<M>(&mut self, inbox: &mut M) -> Result<(), ()>
    where
        M: Inbox<SoundMessage>,
    {
        loop {
            match inbox.next().await {
                SoundMessage::On => {
                    info!("sound on");
                    self.run_sampler(inbox).await;
                }
                SoundMessage::Off => warn!("sound off while already off"),
            };
        }
    }
}

#[actor]
impl Actor for Sound<'_> {
    type Message<'m> = SoundMessage;

    async fn on_mount<M>(&mut self, _: Address<SoundMessage>, mut inbox: M)
    where
        M: Inbox<SoundMessage>,
    {
        loop {
            info!("run_sound");
            if let Err(e) = self.run_sound(&mut inbox).await {
                error!("run_sound failed: {:?}", Debug2Format(&e));
            }
            Timer::after(Duration::from_secs(1)).await;
        }
    }
}
