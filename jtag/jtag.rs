#![no_std]
#![no_main]
#![feature(riscv_ext_intrinsics)]
#![feature(generic_const_exprs)]

extern crate alloc;
use alloc::borrow::ToOwned;
use alloc::vec::{self, Vec};
use embedded_hal::digital::v2::IoPin;
use esp_hal::delay::MicrosDurationU64;
use esp_hal::i2c::I2C;
use esp_hal::timer::timg::{Timer, TimerGroup};
use esp_hal::timer::{ErasedTimer, PeriodicTimer};
use mdio::miim::{Read, Write};
use core::arch::riscv32::wfi;
use core::convert::Infallible;
use core::mem::MaybeUninit;
use core::time::Duration;
use esp_backtrace as _;
use esp_hal::gpio::{GpioPin, Input, Level, Output, Pull};
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

use mdio::bb::Mdio;

mod md;

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
    let mut mdio = io.pins.gpio1;
    // mdio.set_to_open_drain_output(
    //     unsafe { core::mem::transmute(()) }
    // );

    let g = TimerGroup::new(peripherals.TIMG0, &clocks, None);
    let d = MicrosDurationU64::kHz(100);
    let mut t = PeriodicTimer::new(ErasedTimer::Timg0Timer0(g.timer0));
    t.start(d).unwrap();

    // let mut m = Mdio::new(IOPin{pin: mdio}, OutPin{pin: mdc}, t);

    let PHY = 0x03;
    let status_reg = 0x02;
    
    // for phy in 0..32 {
    //     let x = m.read(phy, 0b00010).unwrap();
    //     println!("x = {x}");
    // }
    let led_reg: u8 = 0x1b;
    let ledvalue_original: u16= 0x1e01;
    
    let led: u16 = 0b0001_0111_0000_0000;

    
    let mut cont = md::Controller::new(100, delay, mdc, mdio);
    // for reg in 0..32 {

    //     let mm  = md::MDIOFrame::<2>::new(PHY, reg);
    //     let x = cont.frame_read(mm);
    //     println!("addr: {:x} -> x = {:x}", reg, x);
    // }
    const PERF_TEST_DATA: u16 = 0b010_1_0000_0000_0001;
    const TEST_REG: u8 =  0x13;
    cont.frame_write(md::MDIOFrame::<2>::new(PHY, TEST_REG), PERF_TEST_DATA);
    // let ledv = cont.frame_read(md::MDIOFrame::<2>::new(PHY, led_reg));
    // println!("LED Register: {:x}", ledv);

    loop {

        unsafe {
            wfi();
        }
    }
}

struct IOPin {
    pin: GpioPin<1>
}

struct OutPin {
    pin: GpioPin<0>
}

impl embedded_hal::digital::v2::OutputPin for OutPin {
    type Error = ();
    
    fn set_high(&mut self) -> Result<(), Self::Error> {
        Ok(self.pin.set_high())
    }

    fn set_low(&mut self) -> Result<(), Self::Error> {
        Ok(self.pin.set_low())
    }

    fn set_state(&mut self, state: embedded_hal::digital::v2::PinState) -> Result<(), Self::Error> {
        let b = match state {
            embedded_hal::digital::v2::PinState::High => true,
            embedded_hal::digital::v2::PinState::Low => false,
        };
        Ok(self.pin.set_state(b))
    }
}

impl embedded_hal::digital::v2::InputPin for IOPin {
    type Error = ();

    fn is_high(&self) -> Result<bool, Self::Error> {
        Ok(self.pin.is_high())
    }

    fn is_low(&self) -> Result<bool, Self::Error> {
        Ok(self.pin.is_low())
    }
}

impl embedded_hal::digital::v2::OutputPin for IOPin {
    type Error = ();
    
    fn set_high(&mut self) -> Result<(), Self::Error> {
        Ok(self.pin.set_high())
    }

    fn set_low(&mut self) -> Result<(), Self::Error> {
        Ok(self.pin.set_low())
    }

    fn set_state(&mut self, state: embedded_hal::digital::v2::PinState) -> Result<(), Self::Error> {
        let b = match state {
            embedded_hal::digital::v2::PinState::High => true,
            embedded_hal::digital::v2::PinState::Low => false,
        };
        Ok(self.pin.set_state(b))
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
