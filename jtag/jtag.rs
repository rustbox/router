#![no_std]
#![no_main]

extern crate alloc;
use alloc::borrow::ToOwned;
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
use esp_println::println;

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

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

    let sclk = io.pins.gpio0;
    let miso = io.pins.gpio1;
    let mosi = io.pins.gpio2;
    let sio2 = io.pins.gpio3;
    let sio3 = io.pins.gpio4;
    let cs = io.pins.gpio5;

    let mut spi = Spi::new_half_duplex(peripherals.SPI2, 100.kHz(), SpiMode::Mode0, &clocks)
        .with_pins(
            Some(sclk),
            Some(mosi),
            Some(miso),
            Some(sio2),
            Some(sio3),
            Some(cs),
        );

    loop {
        println!("{}", "hello, world!".to_owned());

        let mut data = [0u8; 2];
        spi.read(
            SpiDataMode::Single,
            Command::Command8(0x90, SpiDataMode::Single),
            Address::Address24(0x000000, SpiDataMode::Single),
            0,
            &mut data,
        )
        .unwrap();
        println!("Single {:x?}", data);

        delay.delay(500.millis());
    }
}
