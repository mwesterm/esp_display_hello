#![no_std]
#![no_main]

use embassy_executor::*;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::channel::{Channel, Receiver, Sender};
use embassy_time::{Duration, Timer};

use embedded_graphics::mono_font::MonoTextStyle;
use embedded_graphics::primitives::{PrimitiveStyle, Rectangle};
use embedded_graphics::text;
use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};

use esp_backtrace as _;
use esp_hal::gpio::{Input, Pull};
use esp_hal::peripherals::I2C0;
use esp_hal::timer::timg::TimerGroup;
use esp_hal::Async;
use esp_hal::{
    clock::ClockControl, gpio::Io, i2c::I2C, peripherals::Peripherals, prelude::*,
    system::SystemControl,
};
use heapless::String;
use log::info;

use core::fmt::Write as _;

use ssd1306::mode::DisplayConfig;
use ssd1306::prelude::DisplayRotation;
use ssd1306::size::DisplaySize128x32;
use ssd1306::{I2CDisplayInterface, Ssd1306};
use static_cell::StaticCell;

mod domain;
use self::domain::Reading;

/// A channel between sensor sampler and display updater
static CHANNEL: StaticCell<Channel<NoopRawMutex, Reading, 3>> = StaticCell::new();

#[main]
async fn main(spawner: Spawner) {
    //Take peripherals
    let peripherals = Peripherals::take();
    let system = SystemControl::new(peripherals.SYSTEM);
    let clocks = ClockControl::max(system.clock_control).freeze();
    let timer_group0 = TimerGroup::new(peripherals.TIMG0, &clocks);

    esp_hal_embassy::init(&clocks, timer_group0.timer0);

    esp_println::logger::init_logger_from_env();

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
    info!("Create I²C bus");
    let sda = io.pins.gpio6;
    let scl = io.pins.gpio7;

    let i2c = I2C::new_async(peripherals.I2C0, sda, scl, 100_u32.kHz(), &clocks);

    info!("Create channel");
    let channel: &'static mut _ = CHANNEL.init(Channel::new());
    let receiver = channel.receiver();
    let sender = channel.sender();

    info!("Create button");
    let button: Input<'_, esp_hal::gpio::GpioPin<2>> = Input::new(io.pins.gpio2, Pull::Up);

    spawner.must_spawn(worker_task(i2c, receiver));
    spawner.must_spawn(button_task(button, sender));

    let mut button_counter: u32 = 0;
    loop {
        Timer::after(Duration::from_millis(6000)).await;
    }
}

#[embassy_executor::task]
async fn worker_task(
    i2c: I2C<'static, I2C0, Async>,
    receiver: Receiver<'static, NoopRawMutex, Reading, 3>,
) {
    info!("worker Task created");
    let interface = I2CDisplayInterface::new(i2c);
    let mut display = Ssd1306::new(interface, DisplaySize128x32, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();
    display.init().unwrap();

    let text_style = MonoTextStyle::new(&FONT_6X10, BinaryColor::On);

    Text::with_baseline("Running!", Point::zero(), text_style, Baseline::Top)
        .draw(&mut display)
        .unwrap();
    display.flush().unwrap();
    let text_position = Point::new(0, 22);
    loop {
        info!("Wait for message from sensor");
        let reading = receiver.receive().await;
        let mut output_string: String<10> = String::new();
        write!(&mut output_string, " value:{}", reading).unwrap();
        info!("{}", output_string.as_str());

        let text_dimensions = Text::new(&output_string, text_position, text_style).bounding_box();
        let background = Rectangle::new(text_dimensions.top_left, text_dimensions.size)
            .into_styled(PrimitiveStyle::with_fill(BinaryColor::Off));

        // Draw the background rectangle to clear the area
        background.draw(&mut display).unwrap();
        display.flush().unwrap();
        Text::with_baseline(
            output_string.as_str(),
            text_position,
            text_style,
            Baseline::Bottom,
        )
        .draw(&mut display)
        .unwrap();
        display.flush().unwrap();
    }
}

#[embassy_executor::task]
async fn button_task(
    mut button: Input<'static, esp_hal::gpio::GpioPin<2>>,
    sender: Sender<'static, NoopRawMutex, Reading, 3>,
) {
    info!("button Task created");
    let mut button_counter: u32 = 0;
    loop {
        button.wait_for_rising_edge().await;

        button_counter += 1;
        info!("Button pressed: {}", button_counter);
        sender.send(button_counter).await;
    }
}
