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
use md::Controller;
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

const PHY: u8 = 0x03;


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
    print_standard_regs(&mut cont);
    const TX_COUNT_FRAMES: u16 = 0b0000_0101_00000000;
    const TX_ERROR_COUNT: u16 = 0b0000_0100_00000000;
    const RX_COUNT_FRAMES: u16 = 0b0000_0001_00000000;
    const RX_ERROR_COUNT: u16 = 0b0000_0000_00000000;
    const TPG_EN: u16 = 0x0001;
    const TPG_START: u16 = 0x0003;
    const FETL: u16 = 0b101_0_0000_0000_0001;
    const PERF_TEST_DATA: u16 = 0xa001; //(0b010_0_0000_0000_0001_u16.wrapping_sub(1)) >> 1 ;
    const TEST_REG: u8 =  0x13;
    const COUNT_REG: u8 = 0x15;
    const TEST_PACKET_CTRL: u8 = 0x1C;
    const TEST_PACKET_DATA: u8 = 0x1D;

    // println!("Using Default test data");
    // cont.frame_write(md::MDIOFrame::<2>::new(PHY, COUNT_REG), TX_COUNT_FRAMES);
    // let x = cont.frame_read(md::MDIOFrame::<2>::new(PHY, COUNT_REG));
    // println!("Wrote: {:x}, {:x}", TX_COUNT_FRAMES, x);

    // cont.frame_write(md::MDIOFrame::<2>::new(PHY, TEST_PACKET_CTRL), TPG_EN);
    // let x = cont.frame_read(md::MDIOFrame::<2>::new(PHY, TEST_PACKET_CTRL));
    // println!("Wrote: {:x}, {:x}", TPG_EN, x);

    // cont.frame_write(md::MDIOFrame::<2>::new(PHY, TEST_PACKET_CTRL), TPG_START);
    // let x = cont.frame_read(md::MDIOFrame::<2>::new(PHY, TEST_PACKET_CTRL));
    // println!("Wrote: {:x}, {:x}", TPG_START, x);
    // for t in 0..8 {
    //     let hi3 = t << 13;
    //     let reg_value = hi3 + 1;
    //     let f = md::MDIOFrame::<2>::new(PHY, TEST_REG);
        
    //     cont.frame_write(f.clone(), reg_value);
    //     let x = cont.frame_read(f);
    //     println!("Wrote: {:x}, {:x}", reg_value, x);
    //     delay.delay_millis(2500);
    // }
    

    loop {

        unsafe {
            wfi();
        }
    }
}

fn print_standard_regs(cont: &mut Controller) {
    for r in 0..32 {
        let f = md::MDIOFrame::<2>::new(PHY, r);
        let x = cont.frame_read(f);
        println!("Reg {:x} = {:x}", r, x);
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
