use core::cmp::min;
use core::ops::Range;

use drogue_device::drivers::led::neopixel::rgbw::{Rgbw8, BLACK};
use drogue_device::drivers::led::neopixel::Pixel;
use micromath::F32Ext;
use trait_enum::trait_enum;

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

/* trait_enum!{
    pub enum Effect: Default {
        ColorWipe,
        Rainbow,
        VUMeter,
        RGBVUMeter,
    }
} */

#[derive(Default)]
pub struct ColorWipe {}

pub fn color_wipe<const N: usize>(frame: u32, speed: i16, color: Rgbw8) -> [Rgbw8; N] {
    let frame = ((frame as i32) * (speed as i32)) as usize;
    // default speed is 1 frame per pixel
    let frame = frame / 256;
    let frame = frame % (2 * N);

    let first = frame.saturating_sub(N);
    let last = min(frame, N);

    let mut out = [BLACK; N];
    for i in first..last + 1 {
        out[i] = color;
    }
    out
}

pub fn rainbow<const N: usize>(frame: u32, speed: i16) -> [Rgbw8; N] {
    let frame = ((frame as i32) * (speed as i32)) as usize;
    // default speed is 128 frames (4.2 seconds) per loop
    let frame = frame / 128;
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

#[derive(Default)]
pub struct VUMeter {}
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

#[derive(Default)]
pub struct RGBVUMeter {}
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

pub struct Pulse<const N: usize> {
    last: [Rgbw8; N],
    gradient: u16,
}

impl<const N: usize> Pulse<N> {
    pub fn run(&mut self, volume_tracker: &super::volume::VolumeTracker) -> [Rgbw8; N] {
        fade(&mut self.last, 0.75);
        /* if volume_tracker.bump {

        } */
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

pub trait ColorPalette {
    fn convert(&self, i: u16) -> Rgbw8;
    fn max(&self) -> u16;
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
    (@impl $i:ident, $name:ident: { $($rule:tt),+ $(,)? }) => {
        pub struct $name {}
        impl ColorPalette for $name {
            fn convert(&self, $i: u16) -> Rgbw8 {
                let $i = $i % self.max();
                palettes!(@case $i, $($rule),+)
            }
            fn max(&self) -> u16 {
                0 $(+ palettes!(@size $rule))+
            }
        }
    };
    ($i:ident, $($name:ident: $body:tt),* $(,)?) => {
        pub enum Palette {
            $($name),*
        }
        $(palettes!(@impl $i, $name: $body);)*
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
