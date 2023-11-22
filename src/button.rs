use embassy_rp::gpio::{Level, Pin, Input};
use embassy_time::Instant;

pub struct Button<'a, P: Pin> {
    pin: Input<'a, P>,
    pub attack: u16,
    last: Level,
    last_change: Instant,
    just: bool,
}

impl<'a, P: Pin> Button<'a, P> {
    pub fn new(pin: P) -> Button<'a, P> {
        Button { pin: Input::new(pin, embassy_rp::gpio::Pull::Up), attack: 10, last: Level::High, last_change: Instant::now(), just: false }
    }

    pub fn pressed(&self) -> bool {
        self.last == Level::Low
    }

    pub fn just_pressed(&mut self) -> bool {
        self.pressed() && self.just
    }
    
    pub fn just_relesed(&mut self) -> bool {
        self.released() && self.just
    }

    pub fn released(&self) -> bool {
        self.last == Level::High
    }

    pub fn poll(&mut self) {
        if self.last_change.elapsed().as_millis() < self.attack as u64 {return;}
        let level = self.pin.get_level();
        if level != self.last {
            self.just = true;
        } else {
            self.just = false;
        }
        self.last = level;
    }
}