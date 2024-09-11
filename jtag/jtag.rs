#![no_std]
#![no_main]

extern crate alloc;
use alloc::borrow::ToOwned;
use alloc::vec;
use esp_hal::gpio::{Input, Level, Output, Pull};
use jtag_taps::statemachine::{JtagSM, Register};
use jtag_taps::taps::Taps;
use core::mem::MaybeUninit;
use esp_backtrace as _;
use esp_hal::spi::master::HalfDuplexReadWrite;
use esp_hal::{
    clock::ClockControl,
    delay::Delay,
    gpio::Io,
    peripherals::Peripherals,
    prelude::*,
    spi::{
        master::{Address, Command, Spi},
        SpiDataMode, SpiMode,
    },
    system::SystemControl,
};
use embedded_hal::digital::{OutputPin, InputPin};
use esp_println::{print, println};

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = SystemControl::new(peripherals.SYSTEM);

    let clocks = ClockControl::max(system.clock_control).freeze();
    let delay = Delay::new(&clocks);

    #[global_allocator]
    static ALLOCATOR: esp_alloc::EspHeap = esp_alloc::EspHeap::empty();
    const HEAP_SIZE: usize = 32 * 1024;
    static mut HEAP: MaybeUninit<[u8; HEAP_SIZE]> = MaybeUninit::uninit();

    unsafe {
        ALLOCATOR.init(HEAP.as_mut_ptr() as *mut u8, HEAP_SIZE);
    }

    println!("Hello");

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

    let clock = Output::new(io.pins.gpio0, Level::Low);
    let tdi = Output::new(io.pins.gpio1, Level::Low);
    let tdo = Input::new(io.pins.gpio2, Pull::Down);
    let tms = Output::new(io.pins.gpio3, Level::Low);

    let mut cable = jtag_taps::cable::gpio::Gpio::new(500, clock, tdi, tdo, tms, delay);
    let jtag = JtagSM::new(&mut cable);
    let mut taps = Taps::new(jtag);

    // taps.detect();
    
    let ir = vec![235, 0];
    taps.select_tap(0, &ir);
    let readback = taps.read_ir();
    print!("ir: ");
    for x in readback {
        print!("{:x} ", x);
    }

    // let mut spi = Spi::new_half_duplex(peripherals.SPI2, 100.kHz(), SpiMode::Mode0, &clocks)
    //     .with_pins(
    //         Some(sclk),
    //         Some(mosi),
    //         Some(miso),
    //         Some(sio2),
    //         Some(sio3),
    //         Some(cs),
    //     );

    loop {
        // println!("{}", "hello, world!".to_owned());

        // let mut data = [0u8; 2];
        // spi.read(
        //     SpiDataMode::Single,
        //     Command::Command8(0x90, SpiDataMode::Single),
        //     Address::Address24(0x000000, SpiDataMode::Single),
        //     0,
        //     &mut data,
        // )
        // .unwrap();
        // println!("Single {:x?}", data);

        // delay.delay(500.millis());
    }
}
