use crate::{ActualState, Debug2Format, DesiredState, UnderlightMode};
use drogue_device::drivers::led::neopixel::filter::*;
use drogue_device::drivers::led::neopixel::rgbw::{NeoPixelRgbw, RED};
use ector::{actor, Actor, Address, Inbox};
use embassy_nrf::gpio::{Level, Output as GpioOutput, OutputDrive};
use embassy_nrf::pac;
use embassy_nrf::peripherals::{PWM1, PWM2, PWM3};
use embassy_nrf::pwm::{self, Prescaler, SimplePwm};
use embassy_nrf::wdt::WatchdogHandle;
use embassy_time::{Duration, Instant, Ticker, Timer};
use embedded_hal::digital::blocking::OutputPin;
use futures::{pin_mut, select_biased, FutureExt, StreamExt};
use num_traits::Float;
extern crate dimensioned as dim;
use crate::{DESIRED_STATE, STATE};
use dim::si::f32consts::{LX, V};

use super::sound::{SoundData, SoundMessage};

mod effects;
mod volume;

trait StepTowards {
    fn step_towards(self, target: Self, step: Self) -> Self;
}

impl StepTowards for u8 {
    fn step_towards(self, target: Self, step: Self) -> Self {
        if self > target {
            target.max(self.saturating_sub(step))
        } else {
            target.min(self.saturating_add(step))
        }
    }
}
impl StepTowards for f32 {
    fn step_towards(self, target: Self, step: Self) -> Self {
        if self > target {
            target.max(self - step)
        } else {
            target.min(self + step)
        }
    }
}

const TAIL_PERIOD: Duration = Duration::from_secs(2);
const LAST_MOVE_TIMEOUT: Duration = Duration::from_secs(20);
const LAST_MOVE_TAIL_TIMEOUT: Duration = Duration::from_secs(60);
const LAST_MOVE_UNDER_TIMEOUT: Duration = Duration::from_secs(60);
const DISPLAY_TIMEOUT: Duration = Duration::from_secs(60);

const PWM_MAX_HEADLIGHT: u16 = 625;
const PWM_MAX_TAILLIGHT: u16 = 255;

const UNDERLIGHT_PIXELS: usize = 35;

struct PeakRingBuffer<Item, const CAPACITY: usize> {
    items: [Item; CAPACITY],
    last_idx: usize,
}

impl<Item, const CAPACITY: usize> PeakRingBuffer<Item, CAPACITY>
where
    Item: Copy + Default + Ord,
{
    fn new() -> Self {
        Self {
            items: [Default::default(); CAPACITY],
            last_idx: 0,
        }
    }
    fn set_max(&mut self, idx: usize, value: Item) {
        let idx = idx % self.items.len();
        if idx != self.last_idx {
            self.items[idx] = Default::default();
            self.last_idx = idx;
        }
        self.items[idx] = self.items[idx].max(value);
    }
    fn max(&self) -> Item {
        self.items.iter().fold(Default::default(), |a, b| a.max(*b))
    }
}

pub struct Output<'a> {
    wdt_handle: Option<WatchdogHandle>,
    power: pac::POWER,
    power_enable: GpioOutput<'a, crate::PinPowerEnable>,
    headlight_pwm: SimplePwm<'a, PWM3>,
    taillight_pwm: SimplePwm<'a, PWM2>,
    underlight: NeoPixelRgbw<'a, PWM1, UNDERLIGHT_PIXELS>,
    sound: Address<SoundMessage>,

    underlight_frame: u32,
    need_sound: bool,
    sound_data: Option<SoundData>,
    peak_amplitudes: PeakRingBuffer<u16, 8>,
    volume_tracker: volume::VolumeTracker,
    current_underlight_effect: effects::Effect<UNDERLIGHT_PIXELS>,
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug)]
pub enum OutputMessage {
    SoundData(SoundData),
}

const FPS: u64 = 30;

impl Output<'_> {
    pub fn new(
        wdt_handle: Option<WatchdogHandle>,
        power: pac::POWER,
        pwm1: PWM1,
        pwm2: PWM2,
        pwm3: PWM3,
        pin_power_enable: crate::PinPowerEnable,
        pin_headlight_dim: crate::PinHeadlightDim,
        pin_tail_l: crate::PinTailL,
        pin_tail_c: crate::PinTailC,
        pin_tail_r: crate::PinTailR,
        pin_underlight: crate::PinUnderlight,
        sound: Address<SoundMessage>,
    ) -> Self {
        let power_enable = GpioOutput::new(pin_power_enable, Level::Low, OutputDrive::Standard);
        let mut headlight_pwm = SimplePwm::new_1ch(pwm3, pin_headlight_dim);
        headlight_pwm.set_prescaler(Prescaler::Div128); // Div128 == 125kHz
        headlight_pwm.set_max_duty(PWM_MAX_HEADLIGHT);
        headlight_pwm.set_duty(0, PWM_MAX_HEADLIGHT / 2);
        let mut taillight_pwm = SimplePwm::new_3ch(pwm2, pin_tail_c, pin_tail_l, pin_tail_r);
        taillight_pwm.set_prescaler(Prescaler::Div16); // Div16 = 1MHz
        taillight_pwm.set_max_duty(PWM_MAX_TAILLIGHT);
        taillight_pwm.set_duty(0, PWM_MAX_TAILLIGHT);
        taillight_pwm.set_duty(1, PWM_MAX_TAILLIGHT);
        taillight_pwm.set_duty(2, PWM_MAX_TAILLIGHT);
        let underlight =
            NeoPixelRgbw::<'_, _, UNDERLIGHT_PIXELS>::new(pwm1, pin_underlight).unwrap();
        Self {
            wdt_handle,
            power,
            power_enable,
            headlight_pwm,
            taillight_pwm,
            underlight,
            sound,

            underlight_frame: 0,
            need_sound: false,
            sound_data: None,
            peak_amplitudes: PeakRingBuffer::new(),
            volume_tracker: volume::VolumeTracker::new(),
            current_underlight_effect: effects::Effect::Solid,
        }
    }

    async fn run_output<M>(&mut self, inbox: &mut M) -> Result<(), ()>
    where
        M: Inbox<OutputMessage>,
    {
        let mut ticker = Ticker::every(Duration::from_millis(1000 / FPS));
        loop {
            let now = Instant::now();

            if let Some(wdt_handle) = self.wdt_handle.as_mut() {
                wdt_handle.pet();
            }
            //   // TODO: Use IMU angle for mode too?

            let vbus_detected = self
                .power
                .usbregstatus
                .read()
                .vbusdetect()
                .is_vbus_present();

            // info!("vbus detected: {:?}", vbus_detected);

            let state = STATE.lock(|c| {
                c.update(|s| {
                    let mut s = s;
                    s.vbus_detected = vbus_detected;
                    if vbus_detected {
                        s.vbus_timer.update();
                    }
                    s
                })
            });

            let desired_state = DESIRED_STATE.lock(|c| c.get());

            // Update underlight
            let mut underlight_on = state.vbus_detected;
            let mut underlight_target_brightness = 0;
            match desired_state.underlight_mode {
                UnderlightMode::ForceOn => {
                    underlight_on = true;
                    underlight_target_brightness = 255;
                }
                UnderlightMode::On => {
                    underlight_target_brightness = 255;
                }
                UnderlightMode::Auto => {
                    if state.vbus_detected && state.move_timer.elapsed() < LAST_MOVE_UNDER_TIMEOUT {
                        underlight_target_brightness = 255;
                    }
                }
                UnderlightMode::Off => (),
            }
            let state = STATE.lock(|c| {
                c.update(|s| {
                    let mut s = s;
                    s.underlight_brightness = s
                        .underlight_brightness
                        .step_towards(underlight_target_brightness, 3);
                    self.underlight_frame += 1;
                    s
                })
            });
            if state.underlight_brightness == 0 {
                // Power down completely when we reach 0 brightness
                underlight_on = false;
            }

            //trace!("UL: on {} brightness: {:02x} target: {:02x}", underlight_on, state.underlight_brightness, underlight_target_brightness);
            // TODO: underlight.setBrightness(underlight.gamma8(underlight_brightness));

            // TODO: Make sure it doesn't cause flashing if we enable power before drive is set
            if let Err(e) = self.power_enable.set_state(underlight_on.into()) {
                error!("failed to set power_enable: {}", e);
            };

            let mut taillight_on = false;
            let state = STATE.lock(|c| {
                c.update(|s| {
                    let mut s = s;
                    use crate::HeadlightMode::*;
                    s.display_on = s.move_timer.elapsed() < DISPLAY_TIMEOUT
                        || s.vbus_timer.elapsed() < DISPLAY_TIMEOUT;
                    if s.vext.map_or(true, |v| v < 11.2 * V) || !s.vbus_detected {
                        s.headlight_brightness = 0.0;
                        s.taillight_brightness = 0.0;
                        s.headlight_mode = Off;
                    } else {
                        s.vext_present_timer.update();
                        taillight_on = s.move_timer.elapsed() < LAST_MOVE_TAIL_TIMEOUT;
                        s.headlight_mode = match desired_state.headlight_mode {
                            Auto => {
                                if s.move_timer.elapsed() > LAST_MOVE_TIMEOUT {
                                    Off
                                } else if s.lux.map_or(true, |l| l > 10.0 * LX) {
                                    Day
                                } else {
                                    Night
                                }
                            }
                            _ => desired_state.headlight_mode,
                        };
                    }
                    let target_brightness = match state.headlight_mode {
                        Night => 1.0,
                        Day => 0.5, // TODO: Blink
                        _ => 0.0,
                    };
                    s.headlight_brightness =
                        s.headlight_brightness.step_towards(target_brightness, 0.01);
                    let target_taillight_brightness = if taillight_on { 1.0 } else { 0.0 };
                    s.taillight_brightness = s
                        .taillight_brightness
                        .step_towards(target_taillight_brightness, 0.05);
                    s
                })
            });
            self.headlight_pwm.set_duty(
                0,
                (PWM_MAX_HEADLIGHT as f32 * state.headlight_brightness) as u16,
            ); // TODO: Figure out how to invert polarity.
            if state.taillight_brightness == 0.0 {
                self.taillight_pwm.set_duty(0, PWM_MAX_TAILLIGHT);
                self.taillight_pwm.set_duty(1, PWM_MAX_TAILLIGHT);
                self.taillight_pwm.set_duty(2, PWM_MAX_TAILLIGHT);
            } else {
                let tail_phase = (now.as_ticks() % TAIL_PERIOD.as_ticks()) as f32
                    / TAIL_PERIOD.as_ticks() as f32;
                let tail_intensity = 2.0
                    * if tail_phase > 0.5 {
                        1.0 - tail_phase
                    } else {
                        tail_phase
                    };

                // TODO: gamma
                let tail_pwm_1 = (PWM_MAX_TAILLIGHT as f32
                    * (state.taillight_brightness * tail_intensity).powf(2.8))
                    as u16;
                let tail_pwm_2 = (PWM_MAX_TAILLIGHT as f32
                    * (state.taillight_brightness * (1.0 - tail_intensity)).powf(2.8))
                    as u16;

                self.taillight_pwm
                    .set_duty(0, PWM_MAX_TAILLIGHT - tail_pwm_1);
                self.taillight_pwm
                    .set_duty(1, PWM_MAX_TAILLIGHT - tail_pwm_2);
                self.taillight_pwm
                    .set_duty(2, PWM_MAX_TAILLIGHT - tail_pwm_2);
            }

            if underlight_on {
                // TODO: Update underlight
                self.underlight_update(&desired_state, &state)
                    .await
                    .expect("failed")
            }

            let next_tick = ticker.next().fuse();
            pin_mut!(next_tick);

            while select_biased! {
                message = inbox.next().fuse() => {
                    match message {
                        OutputMessage::SoundData(data) => self.handle_sound_data(data),
                    }
                    true
                },
                _ = next_tick => false,
            } {}
        }
    }

    fn handle_sound_data(&mut self, data: SoundData) {
        self.peak_amplitudes
            .set_max(self.underlight_frame as usize / 16, data.amplitude);
        self.volume_tracker.update(data.amplitude as f32);
        debug!(
            "new sound_data: {:?}, stats: {:?}",
            data, &self.volume_tracker
        );
        self.sound_data = Some(data);
    }

    async fn underlight_update(
        &mut self,
        desired_state: &DesiredState,
        state: &ActualState,
    ) -> Result<(), pwm::Error> {
        use effects::Effect::*;
        self.current_underlight_effect
            .set_from_desired_state(desired_state);
        let need_sound = match self.current_underlight_effect {
            VuMeter => true,
            RgbVuMeter => true,
            Pulse(_) => true,
            Traffic(_) => true,
            _ => false,
        };
        if need_sound != self.need_sound {
            self.sound
                .notify(match need_sound {
                    true => SoundMessage::On,
                    false => SoundMessage::Off,
                })
                .await;
            self.need_sound = need_sound;
        }
        let color = desired_state.underlight_color;
        let pixels = match &mut self.current_underlight_effect {
            Solid => [color; UNDERLIGHT_PIXELS],
            ColorWipe => {
                effects::color_wipe(self.underlight_frame, desired_state.underlight_speed, color)
            }
            TheaterChase => {
                effects::theater_chase(self.underlight_frame, desired_state.underlight_speed, color)
            }
            Rainbow => effects::rainbow(self.underlight_frame, desired_state.underlight_speed),
            TheaterChaseRainbow => effects::theater_chase_rainbow(
                self.underlight_frame,
                desired_state.underlight_speed,
            ),
            CylonBounce => {
                effects::cylon_bounce(self.underlight_frame, desired_state.underlight_speed, color)
            }
            Fire(ref mut fire) => fire.run(self.underlight_frame, desired_state.underlight_speed),
            VuMeter => effects::vu_meter(&self.sound_data, self.peak_amplitudes.max(), color),
            RgbVuMeter => effects::rgb_vu_meter(&self.sound_data, self.peak_amplitudes.max()),
            Pulse(ref mut pulse) => pulse.run(&self.volume_tracker),
            Traffic(ref mut traffic) => traffic.run(&self.volume_tracker),
            x => {
                error!("unsupported effect {:?}", x);
                [RED; UNDERLIGHT_PIXELS]
            }
        };
        trace!("painting neopixels: {:?}", self.current_underlight_effect);
        self.underlight
            .set_with_filter(
                &pixels,
                &mut Brightness(state.underlight_brightness).and(Gamma {}),
            )
            .await
    }
}

#[actor]
impl Actor for Output<'_> {
    type Message<'m> = OutputMessage;

    async fn on_mount<M>(&mut self, _: Address<OutputMessage>, mut inbox: M)
    where
        M: Inbox<OutputMessage>,
    {
        loop {
            info!("run_output");
            if let Err(e) = self.run_output(&mut inbox).await {
                error!("run failed: {:?}", Debug2Format(&e));
            }
            Timer::after(Duration::from_secs(1)).await;
        }
    }
}

//   // Update BLE characteristics
//   // TODO: Notify
//   //blevoltc.write16(vext*64);
// }

// uint32_t underlight_frame = 0;
// void _underlight_update() {
//   uint32_t color1 = underlight.Color(underlight_color.red, underlight_color.green, underlight_color.blue, underlight_color.white);
//   switch (underlight_effect) {
//     case SOLID:
//     underlight.fill(color1);
//     break;
//     case COLOR_WIPE:
//     colorWipe(underlight, underlight_frame, underlight_speed, color1);
//     break;
//     case THEATER_CHASE:
//     theaterChase(underlight, underlight_frame, underlight_speed, color1);
//     break;
//     case RAINBOW:
//     rainbow(underlight, underlight_frame, underlight_speed);
//     break;
//     case THEATER_CHASE_RAINBOW:
//     theaterChaseRainbow(underlight, underlight_frame, underlight_speed);
//     break;
//     case CYLON_BOUNCE:
//     cylonBounce(underlight, underlight_frame, underlight_speed, color1);
//     break;
//     case FIRE:
//     fire(underlight, underlight_frame, underlight_speed);
//     break;
//   }
//   underlight.show();
//   underlight_frame++;
// }
