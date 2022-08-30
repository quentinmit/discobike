use embassy_time::{Duration, Instant};

/// Volume/beat tracker algorithm.
/// From https://github.com/oddmund/RGB-LED-Music-Sound-Visualizer-Arduino-Lib/blob/master/Visualizer.cpp
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug)]
pub struct VolumeTracker {
    pub max_volume: f32,
    pub last_volume: f32,
    /// The last time that volume was non-zero.
    pub last_volume_time: Instant,
    /// The last time the volume bumped.
    pub last_bump_time: Instant,
    /// The average change in volume for a bump.
    pub average_bump_delta: f32,
    /// A moving average of the volume.
    pub average_volume: f32,
    /// A moving average of the time between bumps.
    pub average_bump_interval: Duration,

    /// The time since the max_volume was last lowered.
    max_adjust_time: Instant,
}

impl VolumeTracker {
    pub fn new() -> Self {
        Self {
            max_volume: 0.0,
            last_volume: 0.0,
            last_volume_time: Instant::MIN,
            last_bump_time: Instant::MIN,
            average_bump_delta: 0.0,
            average_volume: 0.0,
            average_bump_interval: Duration::MIN,
            max_adjust_time: Instant::MIN,
        }
    }
    pub fn update(&mut self, mut volume: f32) {
        let now = Instant::now();
        //Sets a threshold for volume.
        //  In practice I've found noise can get up to 15, so if it's lower, the visual thinks it's silent.
        //  Also if the volume is less than average volume / 2 (essentially an average with 0), it's considered silent.
        if volume < 15.0 {
            // volume < (self.avgVol / 2.0) ||
            volume = 0.0;
        } else {
            self.average_volume = (self.average_volume + volume) / 2.0; //If non-zeo, take an "average" of volumes.
            self.last_volume_time = now;
        }
        //If the current volume is larger than the loudest value recorded, overwrite
        if volume > self.max_volume {
            self.max_volume = volume;
        }
        //Everytime a palette gets completed is a good time to readjust "maxVol," just in case
        //  the song gets quieter; we also don't want to lose brightness intensity permanently
        //  because of one stray loud sound.
        if now.duration_since(self.max_adjust_time).as_secs() > 10 {
            self.max_adjust_time = now;
            self.max_volume = (self.max_volume + volume) / 2.0;
        }

        //If there is a decent change in volume since the last pass, average it into "avgBump"
        //if (volume - self.lastVol) > 10.0 {
        if (volume / self.last_volume) > 1.3 {
            self.average_bump_delta = (self.average_bump_delta + (volume - self.last_volume)) / 2.0;
        }

        //If there is a notable change in volume, trigger a "bump"
        //  avgbump is lowered just a little for comparing to make the visual slightly more sensitive to a beat.
        let bump = volume - self.last_volume > self.average_bump_delta * 0.9;
        if bump {
            let now = Instant::now();
            self.average_bump_interval =
                (now.duration_since(self.last_bump_time) + self.average_bump_interval) / 2;
            self.last_bump_time = now;
        }
        self.last_volume = volume;
    }
}
