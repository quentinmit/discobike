use core::cmp::min;
use core::f32::consts::PI;
use core::ops::{Range, Add, Sub};

use drogue_device::drivers::led::neopixel::rgbw::{Rgbw8, BLACK};
use drogue_device::drivers::led::neopixel::Pixel;
use embassy_time::Instant;
use micromath::F32Ext;
use nrf_softdevice::{Softdevice, random_bytes};
use num_traits::{Euclid, One, Zero};
use pareen::{Anim, Fun};
use replace_with::replace_with_or_default;

use nanorand::{Rng, WyRand};
use tween::{Tween, CubicInOut};


// Can't be a trait because impl Trait can't be used in trait methods.
// https://github.com/rust-lang/impl-trait-initiative
fn bounce<T, V>(anim: Anim<impl Fun<T = T, V = V>>, duration: T) -> Anim<impl Fun<T = T, V = V>>
where
    T: Euclid + One + Zero + Sub<Output = T> + Add<Output = T> + PartialOrd + Copy,
{
    let zero = <T as Zero>::zero();
    let one = <T as One>::one();
    let two = one.add(one);
    return anim.map_time(move |t: T| {
        let direction = t.div_euclid(&duration).rem_euclid(&two);
        let phase = t.rem_euclid(&duration);
        if direction > zero {
            duration.sub(phase)
        } else {
            phase
        }
    })
}

fn color_hsv(hue: u16, sat: u8, val: u8) -> Rgbw8 {
    // Remap 0-65535 to 0-1529. Pure red is CENTERED on the 64K rollover;
    // 0 is not the start of pure red, but the midpoint...a few values above
    // zero and a few below 65536 all yield pure red (similarly, 32768 is the
    // midpoint, not start, of pure cyan). The 8-bit RGB hexcone (256 values
    // each for red, green, blue) really only allows for 1530 distinct hues
    // (not 1536, more on that below), but the full unsigned 16-bit type was
    // chosen for hue so that one's code can easily handle a contiguous color
    // wheel by allowing hue to roll over in either direction.
    let hue = ((hue as u32 * 1530 + 32768) / 65536) as u16;
    // Because red is centered on the rollover point (the +32768 above,
    // essentially a fixed-point +0.5), the above actually yields 0 to 1530,
    // where 0 and 1530 would yield the same thing. Rather than apply a
    // costly modulo operator, 1530 is handled as a special case below.

    // So you'd think that the color "hexcone" (the thing that ramps from
    // pure red, to pure yellow, to pure green and so forth back to red,
    // yielding six slices), and with each color component having 256
    // possible values (0-255), might have 1536 possible items (6*256),
    // but in reality there's 1530. This is because the last element in
    // each 256-element slice is equal to the first element of the next
    // slice, and keeping those in there this would create small
    // discontinuities in the color wheel. So the last element of each
    // slice is dropped...we regard only elements 0-254, with item 255
    // being picked up as element 0 of the next slice. Like this:
    // Red to not-quite-pure-yellow is:        255,   0, 0 to 255, 254,   0
    // Pure yellow to not-quite-pure-green is: 255, 255, 0 to   1, 255,   0
    // Pure green to not-quite-pure-cyan is:     0, 255, 0 to   0, 255, 254
    // and so forth. Hence, 1530 distinct hues (0 to 1529), and hence why
    // the constants below are not the multiples of 256 you might expect.

    // Convert hue to R,G,B (nested ifs faster than divide+mod+switch):
    let (r, g, b) = if hue < 255 {
        // Red to Yellow-1
        (255, hue as u8, 0)
    } else if hue < 510 {
        // Yellow to Green-1
        ((510 - hue) as u8, 255, 0)
    } else if hue < 765 {
        // Green to Cyan-1
        (0, 255, (hue - 510) as u8)
    } else if hue < 1020 {
        // Cyan to Blue-1
        (0, (1020 - hue) as u8, 255)
    } else if hue < 1275 {
        // Blue to Magenta-1
        ((hue - 1020) as u8, 0, 255)
    } else if hue < 1530 {
        // Magenta to Red-1
        (255, 0, (1530 - hue) as u8)
    } else {
        // Red
        (255, 0, 0)
    };
    // Apply saturation and value to RGB to make RGBW
    let color_value = ((1 + val as u16) * (sat as u16)) >> 8;
    let white_value = ((1 + val as u16) * (255 - sat as u16)) >> 8;
    Rgbw8::new(
        ((r as u16 * color_value) >> 8) as u8,
        ((g as u16 * color_value) >> 8) as u8,
        ((b as u16 * color_value) >> 8) as u8,
        white_value as u8,
    )
}

pub fn color_wipe<const N: usize>(frame: u32, speed: i16, color: Rgbw8) -> [Rgbw8; N] {
    let frame = ((frame as i32) * (speed as i32)) as usize;
    // default speed is 1 frame per pixel
    let frame = frame / 256;
    let frame = frame % (2 * N);

    let first = frame.saturating_sub(N);
    let last = min(frame + 1, N);

    let mut out = [BLACK; N];
    for i in first..last {
        out[i] = color;
    }
    out
}

pub fn theater_chase<const N: usize>(frame: u32, speed: i16, color: Rgbw8) -> [Rgbw8; N] {
    let frame = ((frame as i32) * (speed as i32)) as usize;
    // default speed is 4 frames per pixel
    let frame = frame / 1024;
    let b = frame % 3;

    let mut out = [BLACK; N];
    for i in (b..N).step_by(3) {
        out[i] = color;
    }
    out
}

pub fn rainbow<const N: usize>(frame: u32, speed: i16) -> [Rgbw8; N] {
    let frame = ((frame as i32) * (speed as i32)) as usize;
    // default speed is 32 frames (4.2 seconds) per loop
    let frame = frame / 32;
    let first_pixel_hue = (256 * frame) % 65536;
    // Hue of first pixel runs 5 complete loops through the color wheel.
    // Color wheel has a range of 65536 but it's OK if we roll over, so
    // just count from 0 to 5*65536. Adding 256 to firstPixelHue each time
    // means we'll make 5*65536/256 = 1280 passes through this outer loop:
    let mut out = [BLACK; N];
    for i in 0..N {
        // For each pixel in strip...
        // Offset pixel hue by an amount to make one full revolution of the
        // color wheel (range of 65536) along the length of the strip
        // (strip.numPixels() steps):
        let pixel_hue = (first_pixel_hue + (i * 65536 / N)) as u16;
        // strip.ColorHSV() can take 1 or 3 arguments: a hue (0 to 65535) or
        // optionally add saturation and value (brightness) (each 0 to 255).
        // Here we're using just the single-argument hue variant. The result
        // is passed through strip.gamma32() to provide 'truer' colors
        // before assigning to each pixel:
        out[i] = color_hsv(pixel_hue, 255, 255);
    }
    out
}

pub fn theater_chase_rainbow<const N: usize>(frame: u32, speed: i16) -> [Rgbw8; N] {
    let frame = ((frame as i32) * (speed as i32)) as usize;
    let frame = frame / 256;
    let firstPixelHue = (65536 / 90) * (frame % 90); // One cycle of color wheel over 90 frames / 3 seconds
    let frame = frame >> 2; // 4 frames per move
    let b = frame % 3;

    let mut out = [BLACK; N];
    for i in (b..N).step_by(3) {
        let hue = firstPixelHue + i * 65536 / N;
        out[i] = color_hsv(hue as u16, 255, 255);
    }
    out
}

pub fn cylon_bounce<const N: usize>(frame: u32, speed: i16, color: Rgbw8) -> [Rgbw8; N] {
    let EyeSize = 4.0;
    let anim = pareen::ease_in_out::<pareen::easer::functions::Cubic, f32>(EyeSize/2.0, (N as f32) - EyeSize, 1.0);
    // TODO: Make pareen::ease_in_out Clone
    //let anim2 = pareen::ease_in_out::<pareen::easer::functions::Cubic, f32>(EyeSize/2.0, (N as f32) - EyeSize, 1.0);
    //let anim = anim.seq(1.0, anim2.backwards(1.0)).repeat(2.0);
    let anim = bounce(anim, 1.0);
    let anim = anim.squeeze(0.0..=(speed as f32)/256.0);

    let center = anim.eval(frame as f32 / 30.0);

    let mut out = [BLACK; N];

    let brightness = pareen::constant(1.0).seq_ease_in(EyeSize/2.0, pareen::easer::functions::Quad, 1.0, pareen::constant(0.0)).map_time(|t: f32| t.abs()).shift_time(center);

    for j in 0..N {
        out[j] = color.scale(brightness.eval(j as f32));
    }
    out
}

#[derive(Debug)]
pub struct Fire<const N: usize> {
    heat: [u8; N],
    last_frame: usize,
    rng: WyRand,
}

#[cfg(feature = "defmt")]
impl <const N: usize> defmt::Format for Fire<N> {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(
            f,
            "Fire<{}> {{ heat: {:?}, last_frame: {:?} }}",
            N, self.heat, self.last_frame,
        )
    }
}

impl<const N: usize> Default for Fire<N> {
    fn default() -> Self {
        let mut seed = [0u8; 8];
        let sd = unsafe { Softdevice::steal() };
        if let Err(e) = random_bytes(sd, &mut seed) {
            error!("Failed to fetch random seed: {:?}", e);
        }
        Self {
            heat: [0; _],
            last_frame: 0,
            rng: WyRand::new_seed(u64::from_ne_bytes(seed)),
        }
    }
}

impl<const N: usize> Fire<N> {
    pub fn run(&mut self, frame: u32, speed: i16) -> [Rgbw8; N] {
        let Cooling: usize = 55;
        let Sparking: u8 = 120;

        let upside_down = (speed < 0);
        let frame = ((frame as i32) * (speed as i32)) as usize;
        if ((self.last_frame - frame) > 256 || (self.last_frame < frame)) {
            // Skip frames to control speed
            self.last_frame = frame;

            // Step 1.  Cool down every cell a little
            for i in 0..N {
                let cooldown = self.rng.generate_range(0..(((Cooling * 10) / N) as u8) + 2);
                self.heat[i] = self.heat[i].saturating_sub(cooldown);
            }

            // Step 2.  Heat from each cell drifts 'up' and diffuses a little
            for i in (2..N).rev() {
                self.heat[i] = ((self.heat[i-1] as usize + 2*(self.heat[i-2] as usize)) / 3) as u8;
            }

            // Step 3.  Randomly ignite new 'sparks' near the bottom
            if self.rng.generate_range(0..255) < Sparking {
                let y = self.rng.generate_range(0..7);
                self.heat[y] = self.heat[y] + self.rng.generate_range(160..255);
            }
        }
        // Step 4.  Convert heat to LED colors
        let mut out = [BLACK; N];
        for i in 0..N {
            out[i] = self.heat_color(self.heat[i]);
        }
        out
    }

    fn heat_color(&self, temperature: u8) -> Rgbw8 {
        // Scale 'heat' down from 0-255 to 0-191
        let t192 = (temperature as usize) * 191 / 255;

        let heatramp = ((t192 & 0x3F) << 2) as u8; // 0..252

        if t192 > 0x80 {
            Rgbw8::new(255, 255, heatramp, 0)
        } else if t192 > 0x40 {
            Rgbw8::new(255, heatramp, 0, 0)
        } else {
            Rgbw8::new(heatramp, 0, 0, 0)
        }
    }
}

pub(super) fn vu_meter<const N: usize>(
    data: &Option<super::SoundData>,
    max: u16,
    color: Rgbw8,
) -> [Rgbw8; N] {
    let mut out = [BLACK; N];
    if let Some(data) = data {
        let num = data.amplitude as f32;
        let peak = max.max(100) as f32;
        let min = peak * 0.1; // 20 dB range
        let frac = (num.ln() - min.ln()) / (peak.ln() - min.ln());
        let n = ((frac * N as f32) as usize).min(N);
        for i in 0..n {
            out[i] = color;
        }
    }
    out
}

pub(super) fn rgb_vu_meter<const N: usize>(
    data: &Option<super::SoundData>,
    max: u16,
) -> [Rgbw8; N] {
    let mut out = [BLACK; N];
    if let Some(data) = data {
        let num = data.amplitude as f32;
        let peak = max.max(100) as f32;
        let min = peak * 0.1; // 20 dB range
        let offset = min.ln();
        let denom = (peak.ln() - offset);

        let calc_n = |slice: Range<usize>, div: f32| {
            (((data.bands[slice].iter().sum::<u16>() as f32 / div).ln() - offset) / denom
                * N as f32) as usize
        };

        let red = calc_n(0..3, 2.0);
        let green = calc_n(3..5, 1.0);
        let blue = calc_n(4..8, 1.0);

        info!("red {}. green {}, blue {}", red, green, blue);
        for i in 0..out.len() {
            out[i] = Rgbw8::new(
                if i < red { 255 } else { 0 },
                if i < green { 255 } else { 0 },
                if i < blue { 255 } else { 0 },
                0,
            );
        }
    }
    out
}

trait Color {
    fn brightness(&self) -> u8;
    fn scale(&self, ratio: f32) -> Self;
}

impl Color for Rgbw8 {
    fn brightness(&self) -> u8 {
        let mut sum = 0u16;
        for c in 0..4 {
            sum += self.get(c).unwrap_or(0) as u16;
        }
        (sum / 4) as u8
    }

    fn scale(&self, ratio: f32) -> Self {
        if ratio == 0.0 {
            return BLACK;
        }
        let mut out = *self;
        for k in 0..Self::CHANNELS {
            let _ = out.set(k, (self.get(k).unwrap_or(0) as f32 * ratio) as u8);
        }
        out
    }
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug)]
pub struct Pulse<const N: usize> {
    last: [Rgbw8; N],
    lastBump: Instant,
    gradient: u16,
    palette: Palette,
}

impl<const N: usize> Default for Pulse<N> {
    fn default() -> Self {
        Self {
            last: [BLACK; N],
            lastBump: Instant::MIN,
            gradient: 0,
            palette: Palette::Rainbow,
        }
    }
}

impl<const N: usize> Pulse<N> {
    pub fn run(&mut self, volume_tracker: &super::volume::VolumeTracker) -> [Rgbw8; N] {
        fade(&mut self.last, 0.75);
        if volume_tracker.lastBumpTime > self.lastBump {
            self.gradient += self.palette.len() / 24;
            self.gradient %= self.palette.len();
            self.lastBump = volume_tracker.lastBumpTime;
        }
        if volume_tracker.lastVol > 0.0 {
            let knob = 0.9;
            let color = self.palette.convert(self.gradient);
            let ratio = volume_tracker.lastVol / volume_tracker.maxVol;

            let led_half = N / 2;
            let start = led_half - (led_half as f32 * ratio) as usize;
            let finish = led_half + (led_half as f32 * ratio) as usize + N % 2;

            for i in start..finish {
                let damp = ((i - start) as f32 * PI / (finish - start) as f32).sin();
                let damp = damp.powi(2);

                let color = color.scale(damp * knob * ratio.powi(2));

                if color.brightness() > self.last[i].brightness() {
                    self.last[i] = color;
                }
            }
        }
        self.last
    }
}

const DOTS: usize = 32;

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug)]
pub struct Traffic<const N: usize> {
    last: [Rgbw8; N],
    lastBump: Instant,
    gradient: u16,
    palette: Palette,
    dots: [Option<(usize, Rgbw8)>; DOTS],
}

impl<const N: usize> Default for Traffic<N> {
    fn default() -> Self {
        Self {
            last: [BLACK; N],
            lastBump: Instant::MIN,
            gradient: 0,
            palette: Palette::Rainbow,
            dots: [None; _],
        }
    }
}

impl<const N: usize> Traffic<N> {
    pub fn run(&mut self, volume_tracker: &super::volume::VolumeTracker) -> [Rgbw8; N] {
        //fade() actually creates the trail behind each dot here, so it's important to include.
        fade(&mut self.last, 0.8);

        if volume_tracker.lastBumpTime > self.lastBump {
            self.dots.iter().position(|d| d == &None).map(|slot| {
                let pos = if slot % 2 == 0 { 0 } else { N - 1 };
                self.dots[slot] = Some((pos, self.palette.convert(self.gradient)));
                self.gradient += self.palette.len() / 24;
            });

            self.lastBump = volume_tracker.lastBumpTime;
        }
        if volume_tracker.lastVol > 0.0 {
            let knob = 0.9;
            let ratio = (volume_tracker.lastVol / volume_tracker.maxVol).powi(2) * knob;

            let last = &mut self.last;

            for slot in 0..self.dots.len() {
                replace_with_or_default(&mut self.dots[slot], |v| {
                    v.and_then(|(pos, color)| {
                        last[pos] = color.scale(ratio);
                        pos.checked_add_signed(if slot % 2 == 0 { 1 } else { -1 })
                            .filter(|pos| *pos < N)
                            .map(|pos| (pos, color))
                    })
                })
            }
        }
        self.last
    }
}

fn fade<const N: usize, P: Pixel<C>, const C: usize>(pixels: &mut [P; N], damper: f32) {
    for i in 0..N {
        for c in 0..C {
            pixels[i].set(c, (pixels[i].get(c).unwrap_or(0) as f32 * damper) as u8);
        }
    }
}

macro_rules! effects {
    (@match $name:ident $type:ty) => {
        Self::$name(_)
    };
    (@match $name:ident) => {
        Self::$name
    };
    ($($name:ident $(($type:ty))?),+ $(,)?) => {
        #[cfg_attr(feature = "defmt", derive(defmt::Format))]
        #[derive(Debug)]
        pub enum Effect<const N: usize> {
            $($name $(($type))?),+
        }
        impl <const N: usize> Effect<N> {
            pub fn set_from_desired_state(&mut self, desired_state: &crate::DesiredState) {
                match desired_state.underlight_effect {
                    $(crate::Effect::$name => {
                        match self {
                            effects!(@match $name $($type)?) => (),
                            _ => {
                                *self = Self::$name $((<$type>::default()))?;
                            }
                        }
                    }),+
                }
            }
        }
    }
}
effects! {
    Solid,
    ColorWipe,
    TheaterChase,
    Rainbow,
    TheaterChaseRainbow,
    CylonBounce,
    Fire(Fire<N>),
    VuMeter,
    RgbVuMeter,
    Pulse(Pulse<N>),
    Traffic(Traffic<N>),
}

trace_macros!(true);

macro_rules! palettes {
    (@color $i:ident, ( $r:expr, $g:expr, $b:expr, $w:expr ) ) => {
        {
            let $i = $i as u8;
            Rgbw8::new($r, $g, $b, $w)
        }
    };
    (@color $i:ident, ( $r:expr, $g:expr, $b:expr ) ) => {
        palettes!(@color $i, ($r, $g, $b, 0))
    };
    (@size ( $r:expr, $g:expr, $b:expr $(, $w:expr)? ) ) => { 255 };
    (@case $i:ident, $rule:tt, $($tail:tt),+) => {
        if $i < palettes!(@size $rule) {
            palettes!(@color $i, $rule)
        } else {
            let $i = $i - palettes!(@size $rule);
            palettes!(@case $i, $($tail),+)
        }
    };
    (@case $i:ident, $rule:tt) => {
        palettes!(@color $i, $rule)
    };
    ($i:ident, $($name:ident: { $($rule:tt),+ $(,)? }),* $(,)?) => {
        #[cfg_attr(feature = "defmt", derive(defmt::Format))]
        #[derive(Debug)]
        pub enum Palette {
            $($name),*
        }
        impl Palette {
            pub const fn convert(&self, $i: u16) -> Rgbw8 {
                let $i = $i % self.len();
                match self {
                    $(Self::$name => palettes!(@case $i, $($rule),+)),*
                }
            }
            pub const fn len(&self) -> u16 {
                match self {
                    $(Self::$name => 0 $(+ palettes!(@size $rule))+),*
                }
            }
        }
        //$(palettes!(@impl $i, $name: $body);)*
    };
}

palettes! {
    i,
    Rainbow: {
        (255, i, 0),       // red -> yellow
        (255 - i, 255, 0), // yellow -> green
        (0, 255, i),       // green -> aqua
        (0, 255 - i, 255), // aqua -> blue
        (i, 0, 255),       // blue -> violet
        (255, 0, 255 - i), // violet -> red
    },
    Sunset: {
        (255, i / 2, 0),       // red -> orange
        (255, 128 - i / 2, i), // orange -> purple
        (255 - i, 0, 255),     // purple -> blue
        (i, 0, 255 - i),       // blue -> red
    },
    Ocean: {
        (0, 255, i),       // green -> aqua
        (0, 255 - i, 255), // aqua -> blue
        (0, i, 255 - i),   // blue -> green
    },
    PinaColada: {
        (i, i, 0, 128 - i / 2), // half white -> yellow
        (255, 255 - i, 0),      // yellow -> red
        (255 - i, 0, 0, i / 2), // red -> half white
    },
    Sulfur: {
        (255 - i, 255, 0), // yellow -> green
        (0, 255, i),       // green -> aqua
        (i, 255, 255 - i), // aqua -> yellow
    },
    NoGreen: {
        (255, i, 0),       // red -> yellow
        (255 - i, 255, i), // yellow -> aqua
        (0, 255 - i, 255), // aqua -> blue
        (i, 0, 255),       // blue -> violet
        (255, 0, 255 - i), // violet -> red
    },
    USA: {
        (255, 0, 0),     // red
        (128, 128, 128), // white
        (0, 0, 255),     // blue
    },
}

trace_macros!(false);
