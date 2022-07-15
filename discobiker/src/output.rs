use crate::EventTimer;
use crate::I2cDevice;
use apds9960::{Apds9960, LightData};
use defmt::*;
use embassy::time::{Duration, Instant, Timer};
use embassy_nrf::pac;
use embassy_nrf::wdt::WatchdogHandle;
extern crate dimensioned as dim;
use crate::STATE;
use dim::si::{
    f32consts::{A, LX, OHM},
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

#[embassy::task]
pub async fn output_task(
    mut wdt_handle: WatchdogHandle,
    power: pac::POWER,
    mut apds9960: Apds9960<I2cDevice>,
) {
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

        info!("vbus detected: {:?}", vbus_detected);

        STATE.lock(|c| {
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

        Timer::after(Duration::from_millis(1000 / 1)).await;
    }
}

//   // Update underlight
//   bool underlight_on = vbus_detected;
//   uint8_t underlight_target_brightness = 0;
//   switch (underlight_mode) {
//     case UL_FORCE:
//     underlight_on = true;
//     case UL_ON:
//     underlight_target_brightness = 255;
//     break;
//     case UL_AUTO:
//     if (vbus_detected && (now - last_move_time) < LAST_MOVE_UNDER_TIMEOUT) {
//       underlight_target_brightness = 255;
//     }
//     break;
//   }
//   underlight_brightness = stepclamp(underlight_brightness, underlight_target_brightness, (uint8_t)3);
//   if (underlight_brightness == 0) {
//     // Power down completely when we reach 0 brightness
//     underlight_on = false;
//   }
//   //Serial.printf("On: %d brightness: %02x target: %02x\n", underlight_on, underlight_brightness, underlight_target_brightness);
//   underlight.setBrightness(underlight.gamma8(underlight_brightness));
//   // TODO: Make sure it doesn't cause flashing if we enable power before drive is set
//   digitalWrite(PIN_POWER_ENABLE, underlight_on);

//   bool taillight_on = false;
//   if (vext < 11.2 || !vbus_detected) {
//     brightness = 0;
//     taillight_brightness = 0;
//     actual_mode = OFF;
//   } else {
//     last_vext_time = now;
//     taillight_on = (now - last_move_time) < LAST_MOVE_TAIL_TIMEOUT;
//     switch (desired_mode) {
//       case AUTO:
//         if (lux > 10) {
//           actual_mode = DAY;
//         } else {
//           actual_mode = NIGHT;
//         }
//         if ((now - last_move_time) > LAST_MOVE_TIMEOUT) {
//           actual_mode = OFF;
//         }
//         break;
//       default:
//         actual_mode = desired_mode;
//     }
//   }

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
