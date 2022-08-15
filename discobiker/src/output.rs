use crate::{HeadlightMode, UnderlightMode, ActualState, DesiredState};
use drogue_device::drivers::led::neopixel::filter::*;
use drogue_device::drivers::led::neopixel::rgbw::{NeoPixelRgbw, Rgbw8, RED};
use embassy_executor::time::{Duration, Instant, Ticker};
use embassy_nrf::gpio::{Level, Output, OutputDrive};
use embassy_nrf::pac;
use embassy_nrf::peripherals::{PWM1, PWM2, PWM3};
use embassy_nrf::pwm::{self, Prescaler, SimplePwm, Instance};
use embassy_nrf::wdt::WatchdogHandle;
use embedded_hal::digital::blocking::OutputPin;
use futures::StreamExt;
extern crate dimensioned as dim;
use crate::{DESIRED_STATE, STATE};
use dim::si::{
    f32consts::{LX, V},
};

mod effects;

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

#[embassy_executor::task]
pub async fn output_task(
    mut wdt_handle: Option<WatchdogHandle>,
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
) {
    let mut power_enable = Output::new(pin_power_enable, Level::Low, OutputDrive::Standard);
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
    let mut underlight = NeoPixelRgbw::<'_, _, UNDERLIGHT_PIXELS>::new(pwm1, pin_underlight).unwrap();

    let mut ticker = Ticker::every(Duration::from_millis(1000/30));
    loop {
        let now = Instant::now();

        if let Some(wdt_handle) = wdt_handle.as_mut() {
            wdt_handle.pet();
        }
        //   // TODO: Use IMU angle for mode too?

        let vbus_detected = power.usbregstatus.read().vbusdetect().is_vbus_present();

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
                s.underlight_frame += 1;
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
        if let Err(e) = power_enable.set_state(underlight_on.into()) {
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
                s.taillight_brightness =
                    s.taillight_brightness.step_towards(target_taillight_brightness, 0.05);
                s
            })
        });
        headlight_pwm.set_duty(
            0,
            PWM_MAX_HEADLIGHT - (PWM_MAX_HEADLIGHT as f32 * state.headlight_brightness) as u16,
        ); // TODO: Figure out how to invert polarity.
        if state.taillight_brightness == 0.0 {
            taillight_pwm.set_duty(0, PWM_MAX_TAILLIGHT);
            taillight_pwm.set_duty(1, PWM_MAX_TAILLIGHT);
            taillight_pwm.set_duty(2, PWM_MAX_TAILLIGHT);
        } else {
            let tail_phase =
                (now.as_ticks() % TAIL_PERIOD.as_ticks()) as f32 / TAIL_PERIOD.as_ticks() as f32;
            let tail_intensity = 2.0
                * if tail_phase > 0.5 {
                    1.0 - tail_phase
                } else {
                    tail_phase
                };

            // TODO: gamma
            let tail_pwm_1 = (255.0 * state.taillight_brightness * tail_intensity) as u16;
            let tail_pwm_2 = (255.0 * state.taillight_brightness * (1.0 - tail_intensity)) as u16;

            taillight_pwm.set_duty(0, PWM_MAX_TAILLIGHT-tail_pwm_1);
            taillight_pwm.set_duty(1, PWM_MAX_TAILLIGHT-tail_pwm_2);
            taillight_pwm.set_duty(2, PWM_MAX_TAILLIGHT-tail_pwm_2);
        }

        if underlight_on {
            // TODO: Update underlight
            underlight_update(&mut underlight, &desired_state, &state).await.expect("failed")
        }

        ticker.next().await;
    }
}

async fn underlight_update<const N: usize, T: Instance>(underlight: &mut NeoPixelRgbw<'_, T, N>, desired_state: &DesiredState, state: &ActualState) -> Result<(), pwm::Error> {
    use crate::Effect::*;
    let pixels = match desired_state.underlight_effect {
        ColorWipe => effects::colorWipe(state.underlight_frame, desired_state.underlight_speed, RED),
        Rainbow => effects::rainbow(state.underlight_frame, desired_state.underlight_speed),
        _ => todo!("unsupported"),
    };
    underlight.set_with_filter(&pixels, &mut Brightness(state.underlight_brightness).and(Gamma{})).await
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
