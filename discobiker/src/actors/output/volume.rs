use embassy_time::{Duration, Instant};

/// Volume/beat tracker algorithm.
/// From https://github.com/oddmund/RGB-LED-Music-Sound-Visualizer-Arduino-Lib/blob/master/Visualizer.cpp
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug)]
pub struct VolumeTracker {
    pub maxVol: f32,
    pub lastVol: f32,
    pub lastVolTime: Instant,
    pub lastBumpTime: Instant,
    pub avgBump: f32,
    pub avgVol: f32,
    pub avgTime: Duration,
    adjTime: Instant,
}

impl VolumeTracker {
    pub fn new() -> Self {
        Self {
            maxVol: 0.0,
            lastVol: 0.0,
            lastVolTime: Instant::MIN,
            lastBumpTime: Instant::MIN,
            avgBump: 0.0,
            avgVol: 0.0,
            avgTime: Duration::MIN,
            adjTime: Instant::MIN,
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
            self.avgVol = (self.avgVol + volume) / 2.0; //If non-zeo, take an "average" of volumes.
            self.lastVolTime = now;
        }
        //If the current volume is larger than the loudest value recorded, overwrite
        if volume > self.maxVol {
            self.maxVol = volume;
        }
        //Everytime a palette gets completed is a good time to readjust "maxVol," just in case
        //  the song gets quieter; we also don't want to lose brightness intensity permanently
        //  because of one stray loud sound.
        if now.duration_since(self.adjTime).as_secs() > 10 {
            self.adjTime = now;
            self.maxVol = (self.maxVol + volume) / 2.0;
        }

        //If there is a decent change in volume since the last pass, average it into "avgBump"
        if (volume - self.lastVol) > 10.0 {
            self.avgBump = (self.avgBump + (volume - self.lastVol)) / 2.0;
        }

        //If there is a notable change in volume, trigger a "bump"
        //  avgbump is lowered just a little for comparing to make the visual slightly more sensitive to a beat.
        let bump = volume - self.lastVol > self.avgBump * 0.9;
        if bump {
            let now = Instant::now();
            self.avgTime = (now.duration_since(self.lastBumpTime) + self.avgTime) / 2;
            self.lastBumpTime = now;
        }
        self.lastVol = volume;
    }
}
