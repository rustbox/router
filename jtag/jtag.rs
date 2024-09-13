#![no_std]
#![no_main]
#![feature(riscv_ext_intrinsics)]
#![feature(generic_const_exprs)]

extern crate alloc;
use alloc::borrow::ToOwned;
use alloc::vec::{self, Vec};
use esp_hal::i2c::I2C;
use mdio::{Controller, MDIOFrame};
use core::arch::riscv32::wfi;
use core::mem::MaybeUninit;
use esp_backtrace as _;
use esp_hal::gpio::{Input, Level, Output, Pull};
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
use esp_println::{print, println};
use jtag_taps::cable::Cable;
use jtag_taps::statemachine::{JtagSM, Register};
use jtag_taps::taps::Taps;

mod mdio;

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
    println!("Clock: {}", &clocks.cpu_clock);

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

    
    let mdc = io.pins.gpio0;
    let mdio = io.pins.gpio1;

    let mut control = Controller::new(100, delay, mdc, mdio);

    let PHY = 0b00000010;
    let h = MDIOFrame::<2>::new(PHY, 0x02);
    println!("header: {:?}", h.header());
    println!("size: {}", h.header().len());
    let v = control.frame_read(h);
    println!("STD_PHYID1: {}", v);


    loop {

        unsafe {
            wfi();
        }
    }
}

fn device_id<T: Cable>(cable: &mut T) -> Vec<u8> {
    println!("Resetting");
    cable.change_mode(&[1, 1, 1, 1, 1], true);
    println!("Set to Shift IR");
    cable.change_mode(&[0, 1, 1, 0, 0], false);
    println!("Shift in 0b10001 command");
    cable.write_data(&[0b10001000], 5, true);
    println!("Set to Read data");
    cable.change_mode(&[1, 1, 1, 0, 0], false);
    cable.read_data(32)
}
