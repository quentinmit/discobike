use crate::EventTimer;
use crate::I2cDevice;
use apds9960::{Apds9960, LightData};
use defmt::*;
use embassy::time::{Duration, Instant, Timer};
use embassy_nrf::pac;
use embassy_nrf::peripherals::PWM3;
use embassy_nrf::pwm::{SimplePwm, Prescaler};
use embassy_nrf::wdt::WatchdogHandle;
use embassy_nrf::gpio::{Level, Output, OutputDrive};
use embedded_hal::digital::blocking::OutputPin;
extern crate dimensioned as dim;
use crate::{DESIRED_STATE, STATE};
use dim::si::{
    f32consts::{V, A, LX, OHM},
    Volt, Lux,
};

trait CalculateIlluminance {
    fn calculate_lux(&self) -> Lux<f32>;
}
impl CalculateIlluminance for LightData {
    fn calculate_lux(&self) -> Lux<f32> {
        ((-0.32466 * self.red as f32)
            + (1.57837 * self.green as f32)
            + (-0.73191 * self.blue as f32))
            * LX
    }
}

#[inline]
pub fn max<T: PartialOrd>(a: T, b: T) -> T {
    if a > b {
        a
    } else if b == b {
        b
    } else {
        a
    }
}

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
            target.max(self-step)
        } else {
            target.min(self+step)
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

#[embassy::task]
pub async fn output_task(
    mut wdt_handle: WatchdogHandle,
    power: pac::POWER,
    pwm3: PWM3,
    pin_power_enable: crate::PinPowerEnable,
    pin_headlight_dim: crate::PinHeadlightDim,
    mut apds9960: Apds9960<I2cDevice>,
) {
    let mut power_enable = Output::new(pin_power_enable, Level::Low, OutputDrive::Standard);
    let mut headlight_pwm = SimplePwm::new_1ch(pwm3, pin_headlight_dim);
    headlight_pwm.set_prescaler(Prescaler::Div128); // Div128 == 125kHz
    headlight_pwm.set_max_duty(PWM_MAX_HEADLIGHT);
    headlight_pwm.set_duty(0, PWM_MAX_HEADLIGHT/2);
    loop {
        let now = Instant::now();

        wdt_handle.pet();

        let color = apds9960
            .read_light()
            .await
            .inspect_err(|e| {
                error!("failed to read light sensor: {:?}", defmt::Debug2Format(&e));
            })
            .ok();
        // 3.5 counts/lux in the c channel according to datasheet
        let lux_value =
            color.map(|color| max(color.calculate_lux(), color.clear as f32 / (3.5 / LX))); // If C is overloaded, use RGB

        //   // Update mode
        //   accel_mag = vecMag(accel_evt.acceleration);
        //   if (accel_mag > 12) {
        //     last_move_time = now;
        //   }
        //   // TODO: Use IMU angle for mode too?

        let vbus_detected = power.usbregstatus.read().vbusdetect().is_vbus_present();

        // info!("vbus detected: {:?}", vbus_detected);

        let state = STATE.lock(|c| {
            c.update(|s| {
                let mut s = s;
                s.lux = lux_value;
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
            Force => {
                underlight_on = true;
                underlight_target_brightness = 255;
            }
            On => {
                underlight_target_brightness = 255;
            }
            Auto => {
                if state.vbus_detected && state.move_timer.elapsed() < LAST_MOVE_UNDER_TIMEOUT {
                    underlight_target_brightness = 255;
                }
            }
        }
        let state = STATE.lock(|c| c.update(|s| {
            let mut s = s;
            s.underlight_brightness = s.underlight_brightness.step_towards(underlight_target_brightness, 3);
            s
        }));
        if state.underlight_brightness == 0 {
            // Power down completely when we reach 0 brightness
            underlight_on = false;
        }

        trace!("UL: on {} brightness: {:02x} target: {:02x}", underlight_on, state.underlight_brightness, underlight_target_brightness);
        // TODO: underlight.setBrightness(underlight.gamma8(underlight_brightness));

        // TODO: Make sure it doesn't cause flashing if we enable power before drive is set
        if let Err(e) = power_enable.set_state(underlight_on.into()) {
            error!("failed to set power_enable: {}", e);
        };

        let mut taillight_on = false;
        let state = STATE.lock(|c| c.update(|s| {
            let mut s = s;
            use crate::HeadlightMode::*;
            if s.vext.map_or(true, |v| v < 11.2*V) || !s.vbus_detected {
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
                        } else if s.lux.map_or(true, |l| l > 10.0*LX) {
                            Day
                        } else {
                            Night
                        }
                    }
                    _ => desired_state.headlight_mode
                };
            }
            let target_brightness = match state.headlight_mode {
                Night => 1.0,
                Day => 0.5, // TODO: Blink
                _ => 0.0,
            };
            s.headlight_brightness = s.headlight_brightness.step_towards(target_brightness, 0.01);
            s
        }));
        headlight_pwm.set_duty(0, PWM_MAX_HEADLIGHT - (PWM_MAX_HEADLIGHT as f32 * state.headlight_brightness) as u16); // TODO: Figure out how to invert polarity.

        Timer::after(Duration::from_millis(1000 / 30)).await;
    }
}

//   // Update headlight
//   float target_brightness = 0;
//   switch (actual_mode) {
//   case NIGHT:
//     target_brightness = 1;
//     break;
//   case DAY:
//     // TODO: Blink
//     target_brightness = 0.5;
//     break;
//   }
//   brightness = stepclamp(brightness, target_brightness, (float)0.01);
//   HwPWM3.writeChannel(0, PWM_MAX_HEADLIGHT * brightness, true); // Invert (high = off)

//   // Update taillight
//   float taillight_target_brightness = taillight_on ? 1 : 0;
//   taillight_brightness = stepclamp(taillight_brightness, taillight_target_brightness, (float)0.05);
//   if (taillight_brightness == 0) {
//     // Onboard red LED (on channel 2) uses up to 1 mA!
//     for (int i = 0; i < 3; i++) {
//       HwPWM2.writeChannel(i, 0);
//     }
//   } else {
//     float tail_phase = (now % TAIL_PERIOD) / (float)TAIL_PERIOD;
//     float tail_intensity = (tail_phase*2);
//     if (tail_intensity > 1) {
//       tail_intensity = 2-tail_intensity;
//     }
//     //tail_intensity *= tail_intensity*tail_intensity; // Apply gamma of 3
//     uint16_t tail_pwm_1 = gamma8[(uint8_t)(255*taillight_brightness*tail_intensity)];
//     uint16_t tail_pwm_2 = gamma8[(uint8_t)(255*taillight_brightness*(1-tail_intensity))];
//     HwPWM2.writeChannel(0, tail_pwm_1);
//     HwPWM2.writeChannel(1, tail_pwm_2);
//     if (!DEBUG_IMU) {
//       HwPWM2.writeChannel(2, tail_pwm_2);
//     }
//   }
//   if (underlight_on) {
//     _underlight_update();
//   }

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
