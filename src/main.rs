//! This example uses the RP Pico W board Wifi chip (cyw43).
//! Connects to specified Wifi network and creates a TCP endpoint on port 1234.

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(async_fn_in_trait)]
#![allow(incomplete_features)]
#![feature(sync_unsafe_cell)]

use defmt::*;
use embassy_embedded_hal::shared_bus::blocking::spi::SpiDeviceWithConfig;
use embassy_executor::Spawner;
use embassy_rp::{bind_interrupts, pwm};
use embassy_rp::gpio::{Level, Output, Input};
use embassy_rp::i2c::I2c;
use embassy_rp::peripherals::{DMA_CH0, PIN_23, PIN_25, PIO0, SPI0, PIN_5, PIN_7, PIN_6, I2C1};
use embassy_rp::pio::{InterruptHandler, Pio};
use embassy_rp::spi::Spi;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time::{Duration, Timer, Instant, Delay, Ticker};
use static_cell::make_static;
use {defmt_rtt as _, panic_probe as _};
use embedded_graphics::mono_font::ascii::FONT_10X20;
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;
use embedded_graphics::text::Text;

const ADDR_OFFSET: u32 = 0x100000;
const FLASH_SIZE: usize = 2 * 1024 * 1024;

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
});

mod sht3x;
mod display;
mod button;

const STYLE_GREEN:embedded_graphics::mono_font::MonoTextStyle<Rgb565> = embedded_graphics::mono_font::MonoTextStyle::new(&FONT_10X20, Rgb565::GREEN);
const STYLE_RED:embedded_graphics::mono_font::MonoTextStyle<Rgb565> = embedded_graphics::mono_font::MonoTextStyle::new(&FONT_10X20, Rgb565::RED);
const STYLE_WHITE:embedded_graphics::mono_font::MonoTextStyle<Rgb565> = embedded_graphics::mono_font::MonoTextStyle::new(&FONT_10X20, Rgb565::WHITE);

static RTC: once_cell::sync::OnceCell<embassy_rp::rtc::Rtc<'_, embassy_rp::peripherals::RTC>> = once_cell::sync::OnceCell::new();
const ZERO_TIME: embassy_rp::rtc::DateTime = embassy_rp::rtc::DateTime{ year: 2000, month: 1, day: 1, day_of_week: embassy_rp::rtc::DayOfWeek::Sunday, hour: 0, minute: 0, second: 0 };

static STATE: embassy_sync::mutex::Mutex<embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex, SaveState> = embassy_sync::mutex::Mutex::new(SaveState::new());

#[repr(C)]
struct SaveState {
    init: bool,
    space: u8,
    time: (rp_pac::rtc::regs::Rtc1, rp_pac::rtc::regs::Rtc0),
    last_hour: u8,
    servo_state: ServoState,
}

impl SaveState {
    const fn new() -> SaveState {
        SaveState { init: false, space: 255, time: (rp_pac::rtc::regs::Rtc1(0), rp_pac::rtc::regs::Rtc0(0)), last_hour: 255, servo_state: ServoState::NeutralLeft}
    }

    unsafe fn as_slice(&self) -> &[u8] {
        core::slice::from_raw_parts((self as *const Self) as *const u8, ::core::mem::size_of::<Self>())
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    static_assertions::const_assert!(core::mem::size_of::<SaveState>() < 256);
    info!("Hello World!");

    let p = embassy_rp::init(Default::default());

    let mut display_config = embassy_rp::spi::Config::default();
    display_config.phase = embassy_rp::spi::Phase::CaptureOnSecondTransition;
    display_config.polarity = embassy_rp::spi::Polarity::IdleHigh;
    display_config.frequency = 64000000;
    let spi: Spi<'static, _, embassy_rp::spi::Blocking> = Spi::new_blocking(p.SPI0, p.PIN_2, p.PIN_3, p.PIN_4, display_config.clone());
    let spi_bus: embassy_sync::blocking_mutex::Mutex<_, _> = embassy_sync::blocking_mutex::Mutex::<embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex, _>::new(core::cell::RefCell::new(spi));
    let spi_bus = make_static!(spi_bus);
    let rst = Output::new(p.PIN_6, Level::Low);
    let display_dc = Output::new(p.PIN_7, Level::Low);
    
    let display_spi = SpiDeviceWithConfig::new(&*spi_bus, Output::new(p.PIN_5, Level::High), display_config);
    
    let di = display::SPIDeviceInterface::new(display_spi, display_dc);
    
    let mut st7789 = mipidsi::Builder::st7789(di)
    .with_orientation(mipidsi::Orientation::Landscape(true))
    .with_invert_colors(mipidsi::ColorInversion::Inverted)
    .init(&mut Delay, Some(rst))
    .unwrap();
    let _ = st7789.clear(Rgb565::BLACK);

    let mut button_1 = button::Button::new(p.PIN_16);
    let mut button_2 = button::Button::new(p.PIN_17);
    let mut humidity_pin = Output::new(p.PIN_18, Level::Low);
    
    let mut flash: embassy_rp::flash::Flash<'_, embassy_rp::peripherals::FLASH, embassy_rp::flash::Blocking, FLASH_SIZE> = embassy_rp::flash::Flash::new_blocking(p.FLASH);
    Timer::after_millis(100).await;
    button_1.poll();
    button_2.poll();
    if !(button_1.pressed() && button_2.pressed()) {
        let mut load_state = [0; core::mem::size_of::<SaveState>()];
        let _ = flash.blocking_read(ADDR_OFFSET, &mut load_state);
        if load_state[0] == 1 {
            unsafe {
            let point = load_state.as_mut_ptr().cast::<SaveState>();
                core::mem::swap(&mut *STATE.lock().await, &mut *point);
            }
        }
    } else {
        let _ = Text::new("State Reset in 10 Seconds", Point { x: 10, y: 150 }, STYLE_GREEN).draw(&mut st7789);
        Timer::after_secs(10).await;
        backup_state(&mut flash).await;
    }

    init_rtc(p.RTC).await;

    let mut i2c = I2c::new_blocking(p.I2C1, p.PIN_27, p.PIN_26, embassy_rp::i2c::Config::default());

    let mut ticker = Ticker::every(Duration::from_secs(5));
    let mut msg = heapless::String::<100>::new();
    let _cs = Output::new(p.PIN_22, Level::High);

    embedded_graphics::text::Text::new("
    Temperature: \n
    Humidity: \n
    Pressure: \n",
        embedded_graphics::prelude::Point::new(10, 10),
        STYLE_WHITE,
    )
    .draw(&mut st7789)
    .unwrap();
    st7789.clear(Rgb565::BLACK);
    let mut pwm_config = embassy_rp::pwm::Config::default();
    pwm_config.top = 10000;
    pwm_config.divider = fixed::FixedU16::from_num(255);
    pwm_config.compare_a = ServoState::NeutralLeft.to_comp();
    pwm_config.enable = true;
    let mut pwm = embassy_rp::pwm::Pwm::new_output_a(p.PWM_CH6, p.PIN_28, pwm_config.clone());
    let mut humidity_on = false;
    let mut phase = if let Some(rtc) = RTC.get() {
        EggStage::from_day(rtc.now().unwrap_or(ZERO_TIME).day) } else {
            EggStage::Incubating
        };
    loop {
        button_1.poll();
        button_2.poll();
        st7789.clear(Rgb565::BLACK);
        use core::fmt::Write;
        let Ok(read) = sht3x::read_temperature_humidity(&mut i2c).await else {
            Text::new("Sht30 Read Error", Point { x: 10, y: 200 }, STYLE_RED).draw(&mut st7789);
            continue;
        };
        msg.clear();
        let now = if let Some(rtc) = RTC.get() {
            rtc.now().unwrap_or(ZERO_TIME) } else {
                ZERO_TIME
            };
        msg.write_fmt(format_args!("Temperature: {:2.02}c \nHumidity: {:2.0}%\nPhase: {:?}\nRunning For {} days {} Hours\nDays Remaining {}", read.temperature, read.humidity, phase, now.day, now.hour, 18 - now.day as i8));
        Text::new(&msg, Point { x: 10, y: 20 }, STYLE_WHITE).draw(&mut st7789);
        if read.temperature > 38. {
            Text::new("Temp TO High", Point { x: 10, y: 125 }, STYLE_RED).draw(&mut st7789);
        }
        if read.temperature < 37. {
            Text::new("Temp TO Low", Point { x: 10, y: 125 }, STYLE_RED).draw(&mut st7789);
        }
        let mut state =  STATE.lock().await;
        Text::new("1", Point { x: 10, y: 150 }, STYLE_RED).draw(&mut st7789);
        // check once per hour
        if state.last_hour != now.hour {
            Text::new("2", Point { x: 20, y: 150 }, STYLE_RED).draw(&mut st7789);
            //update the stage .. lazy day check
            phase = EggStage::from_day(now.day);
            // roll the eggs if the current phase needs this
            if phase.roll_eggs() {
                state.servo_state = match state.servo_state {
                ServoState::NeutralLeft |
                ServoState::Left => ServoState::Right,
                ServoState::NeutralRight |
                ServoState::Right => ServoState::Left,
                };
                state.last_hour = now.hour;
                slow_servo(&mut pwm, &mut pwm_config, state.servo_state.to_comp(), 1000).await;
            }
            drop(state);
            backup_state(&mut flash).await;
        } else if button_2.just_pressed() {
            Text::new("4", Point { x: 40, y: 150 }, STYLE_RED).draw(&mut st7789);
            drop(state);
            backup_state(&mut flash).await;
            Text::new("State Saved", Point { x: 10, y: 175 }, STYLE_GREEN).draw(&mut st7789);
        }
        Text::new("3", Point { x: 30, y: 150 }, STYLE_RED).draw(&mut st7789);
        let (humd_min, humd_max) = phase.humidity_target();
        if read.humidity < humd_min && !humidity_on {
            humidity_pin.set_high();
            Timer::after_millis(10).await;
            humidity_pin.set_low();
            humidity_on = true;
        } else if humidity_on && read.humidity > humd_max {
            humidity_pin.set_high();
            Timer::after_millis(10).await;
            humidity_pin.set_low();
            Timer::after_millis(10).await;
            humidity_pin.set_high();
            Timer::after_millis(10).await;
            humidity_pin.set_low();
            humidity_on = false;
        }
        Text::new("5", Point { x: 50, y: 150 }, STYLE_RED).draw(&mut st7789);
        ticker.next().await
    }

}

async fn slow_servo<T: embassy_rp::pwm::Channel>(servo: &mut embassy_rp::pwm::Pwm<'_, T>, config: &mut embassy_rp::pwm::Config, target: u16, time: u16) {
    const MILLIS_PER_STEP: u16 = 20;
    let forward = target > config.compare_a;
    let steps = time / MILLIS_PER_STEP;
    let step_diff = if forward {
        target - config.compare_a 
    } else {
        config.compare_a - target
    } / steps;
    for _ in 0..steps {
        if forward {
            config.compare_a += step_diff;
        } else {
            config.compare_a -= step_diff;
        }
        servo.set_config(&config);
        Timer::after_millis(MILLIS_PER_STEP as u64).await;
    }
    config.compare_a = target;
    servo.set_config(&config);
}

async fn backup_state(flash: &mut embassy_rp::flash::Flash<'_, embassy_rp::peripherals::FLASH, embassy_rp::flash::Blocking, FLASH_SIZE>) {
    let mut state = STATE.lock().await;
    state.init = true;
    state.time = if let Some(rtc) = RTC.get() {
        unsafe {
            let rtc = rtc as *const embassy_rp::rtc::Rtc<'_, embassy_rp::peripherals::RTC>;
            let rtc = rtc as *mut embassy_rp::rtc::Rtc<'_, embassy_rp::peripherals::RTC>;
            let rtc = &mut *rtc;
            rtc.save()
        }
    } else {return;};
    flash.blocking_erase(ADDR_OFFSET, ADDR_OFFSET + embassy_rp::flash::ERASE_SIZE as u32);
    flash.blocking_write(ADDR_OFFSET, unsafe {state.as_slice()});
}

async fn init_rtc(rtc: embassy_rp::peripherals::RTC) {
    let state = STATE.lock().await;
    let mut rtc = embassy_rp::rtc::Rtc::new(rtc);
    let _ = rtc.restore(state.time.0, state.time.1);
    let _ = RTC.set(rtc);
}

enum ServoState {
    NeutralLeft,
    Left,
    NeutralRight,
    Right,
}

impl ServoState {
    fn to_comp(&self) -> u16 {
        match self {
            ServoState::NeutralRight |
            ServoState::NeutralLeft => 750,
            ServoState::Left => 560,
            ServoState::Right => 940,
        }
    }
}

#[derive(Debug, PartialEq, Eq)]
enum EggStage {
    Incubating,
    Candle,
    Lockdown,
    Hatching,
}

impl EggStage {
    fn humidity_target(&self) -> (f32, f32) {
        match self {
            EggStage::Incubating | EggStage::Candle => (45., 55.),
            EggStage::Lockdown | EggStage::Hatching => (65., 75.),
        }
    }

    fn temperature_target(&self) -> (f32, f32) {
        (37.2, 37.5)
    }

    fn roll_eggs(&self) -> bool {
        match self {
            EggStage::Incubating | EggStage::Candle => true,
            EggStage::Lockdown | EggStage::Hatching => false,
        }
    }

    fn from_day(day: u8) -> EggStage {
           if day < 14 { EggStage::Incubating }
           else if day == 14 {EggStage::Candle}
           else if day < 18{EggStage::Lockdown}
           else {EggStage::Hatching}
    }
}