#![no_std]
#![no_main]
#![feature(riscv_ext_intrinsics)]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]
#![allow(clippy::unusual_byte_groupings)]
#![allow(unused_variables)]

extern crate alloc;
use alloc::format;
use alloc::string::String;
use alloc::vec::Vec;
use core::mem::MaybeUninit;
use esp_backtrace as _;
use esp_hal::timer::timg::TimerGroup;
use esp_hal::timer::{ErasedTimer, PeriodicTimer};
use esp_hal::uart;
use esp_hal::uart::Uart;
use esp_hal::uart::{UartRx, UartTx};
use esp_hal::{
    clock::ClockControl, delay::Delay, gpio::Io, peripherals::Peripherals, prelude::*,
    system::SystemControl,
};
use esp_hal::{gpio::GpioPin, Blocking};
use esp_println::println;
use fugit::TimerRateU64;
use md::Controller;
use nb::block;
use noline::builder::EditorBuilder;

// use mdio::miim::{Read, Write};
// use jtag_taps::cable::Cable;
// use jtag_taps::statemachine::{JtagSM, Register};
// use jtag_taps::taps::Taps;

pub mod md;

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

    let mut io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

    println!();
    // gotta do this before we `println` any more than one byte (above), else we drop bytes in flight.
    // (alternatively, we could block on the uart's fifos being drained, I guess?)
    // this still messes with one of the UART registers somehow, and we get a weird byte
    // printed (`�` in a utf-8 terminal) if we don't pre-println! once.
    let uart0 = Uart::new_with_default_pins(
        peripherals.UART0,
        &clocks,
        &mut io.pins.gpio21,
        &mut io.pins.gpio20,
    )
    .unwrap();

    println!("Hello");
    println!("Clock: {}", &clocks.cpu_clock);

    let mdc = io.pins.gpio0;
    let mdio = io.pins.gpio1;

    let g = TimerGroup::new(peripherals.TIMG0, &clocks, None);
    let t = PeriodicTimer::new(ErasedTimer::Timg0Timer0(g.timer0));

    // let mut m = Mio::new(IOPin{pin: mdio}, OutPin{pin: mdc}, t);

    let status_reg = 0x02;

    // for phy in 0..32 {
    //     let x = m.read(phy, 0b00010).unwrap();
    //     println!("x = {x}");
    // }
    let led_reg: u8 = 0x1b;
    let ledvalue_original: u16 = 0x1e01;

    let led: u16 = 0b0001_0111_0000_0000;

    let mut cont = md::Controller::new(TimerRateU64::kHz(100), t, mdc, mdio);
    // for reg in 0..32 {
    //     let mm  = md::MDIOFrame::<2>::new(PHY, reg);
    //     let x = cont.frame_read(mm);
    //     println!("addr: {:x} -> x = {:x}", reg, x);
    // }
    const TX_COUNT_FRAMES: u16 = 0b0000_0101_00000000;
    const TX_ERROR_COUNT: u16 = 0b0000_0100_00000000;
    const RX_COUNT_FRAMES: u16 = 0b0000_0001_00000000;
    const RX_ERROR_COUNT: u16 = 0b0000_0000_00000000;
    const TPG_EN: u16 = 0x0001;
    const TPG_START: u16 = 0x0003;
    const FETL: u16 = 0b101_0_0000_0000_0001;
    const PERF_TEST_DATA: u16 = 0xa001; //(0b010_0_0000_0000_0001_u16.wrapping_sub(1)) >> 1 ;
    const TEST_REG: u8 = 0x13;
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

    println!("Waiting for input...");

    let mut uart0 = UartWrapper::from(uart0);

    loop {
        let prompt = "> ";

        let mut buffer = [0; 128];
        let mut history = [0; 256];

        let mut editor = EditorBuilder::from_slice(&mut buffer)
            .with_slice_history(&mut history)
            .build_sync(&mut uart0)
            .unwrap();

        let mut phy_addr = 0xff;
        while let Ok(line) = editor.readline(prompt, &mut uart0) {
            const N: usize = 4;
            let mut split = [""; N];
            let n = line.splitn(N, ' ').collect_slice(&mut split[..]);
            let split = &split[..n];
            match split {
                ["reset"] => println!("todo: reset!"),
                ["detect"] => {
                    let phys = detect(&mut cont);
                    if phys.is_empty() {
                        println!("No PHY detected");
                    } else {
                        let v: Vec<_> = phys.into_iter().map(|a| format!("0x{:x}", a)).collect();
                        println!("Detected PHYS: [{:?}]", v.join(", "));
                    }
                },
                ["status"] => {
                    println!("Connected to PHY at 0x{:x}", phy_addr);
                }
                ["setphy", addr] => {
                    let phy = match addr {
                        r if r.starts_with("0x") => u8::from_str_radix(&r[2..], 16)
                            .map_err(|err| format!("couldn't read {:?} as hex: {}", addr, err)),

                        r if r.starts_with("0o") => u8::from_str_radix(&r[2..], 8)
                            .map_err(|err| format!("couldn't read {:?} as octal: {}", addr, err)),

                        r if r.starts_with("0b") => u8::from_str_radix(&r[2..], 2)
                            .map_err(|err| format!("couldn't read {:?} as binary: {}", addr, err)),

                        #[allow(clippy::from_str_radix_10)]
                        _ => u8::from_str_radix(addr, 10)
                            .map_err(|err| format!("couldn't read {:?}: {}", addr, err)),
                    };

                    if let Err(err) = &phy {
                        println!("error: {}", err);
                        continue;
                    }

                    let phy = phy.unwrap() & 0b00011111;
                    phy_addr = phy;
                }

                ["read", "all"] => print_standard_regs(&mut cont, phy_addr),
                ["read", reg] => {
                    let reg = match reg {
                        r if r.starts_with("0x") => u8::from_str_radix(&r[2..], 16)
                            .map_err(|err| format!("couldn't read {:?} as hex: {}", reg, err)),

                        r if r.starts_with("0o") => u8::from_str_radix(&r[2..], 8)
                            .map_err(|err| format!("couldn't read {:?} as octal: {}", reg, err)),

                        r if r.starts_with("0b") => u8::from_str_radix(&r[2..], 2)
                            .map_err(|err| format!("couldn't read {:?} as binary: {}", reg, err)),

                        #[allow(clippy::from_str_radix_10)]
                        _ => u8::from_str_radix(reg, 10)
                            .map_err(|err| format!("couldn't read {:?}: {}", reg, err)),
                    };

                    if let Err(err) = &reg {
                        println!("error: {}", err);
                        continue;
                    }

                    let reg = reg.unwrap();
                    let read = cont.frame_read(md::MDIOFrame::<2>::new(phy_addr, reg));

                    println!("read 0x{:x}: 0x{:x}", reg, read);
                }
                ["read", ..] => {
                    println!("error: read: unrecognized arguments: {:?}", &split[1..]);
                    println!("usage: read [all|REG])");
                }

                [other, ..] => {
                    println!("error: unrecognized: {}", other)
                }
                [] => {}
            }
        }
    }
}

fn detect(cont: &mut Controller) -> Vec<u8> {
    let phy_id1_addr = 0x02;
    let mut phys = Vec::new();
    for p in 0..(1<<5) {
        let x = cont.frame_read(md::MDIOFrame::<2>::new(p, phy_id1_addr));
        if x != 0xffff {
            println!("x == {:x}", x);
            // If the data is all 1s then that means the PHY was not detected at that address
            // This could be that the PHY is under reset or just the wrong PHY address
            phys.push(p);
        }
    }
    phys
}

struct UartWrapper<'d, T> {
    pub rx: UartRx<'d, T, Blocking>,
    pub tx: UartTx<'d, T, Blocking>,
}

impl<'d, T> From<Uart<'d, T, Blocking>> for UartWrapper<'d, T>
where
    T: uart::Instance,
{
    fn from(value: Uart<'d, T, Blocking>) -> Self {
        let (tx, rx) = value.split();

        Self { tx, rx }
    }
}

fn print_standard_regs(cont: &mut Controller, phy: u8) {
    for r in 0..32 {
        let f = md::MDIOFrame::<2>::new(phy, r);
        let x = cont.frame_read(f);
        println!("Reg {:x} = {:x}", r, x);
    }
}

#[derive(Debug)]
struct Error(esp_hal::uart::Error);

impl From<esp_hal::uart::Error> for Error {
    fn from(value: esp_hal::uart::Error) -> Self {
        Self(value)
    }
}

impl embedded_io::Error for Error {
    fn kind(&self) -> embedded_io::ErrorKind {
        embedded_io::ErrorKind::Other
    }
}

impl<'a, T> embedded_io::ErrorType for UartWrapper<'a, T> {
    type Error = Error;
}

impl<'a, T: uart::Instance> embedded_io::Read for UartWrapper<'a, T> {
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        buf[0] = block!(self.rx.read_byte())?;

        Ok(1 + self.rx.drain_fifo(&mut buf[1..]))
    }
}

impl<'a, T: uart::Instance> embedded_io::Write for UartWrapper<'a, T> {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        self.tx.write_bytes(buf).map_err(Self::Error::from)
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        block!(self.tx.flush_tx()).map_err(Self::Error::from)
    }
}

#[allow(dead_code)]
struct IOPin {
    pin: GpioPin<1>,
}

#[allow(dead_code)]
struct OutPin {
    pin: GpioPin<0>,
}

impl embedded_hal::digital::v2::OutputPin for OutPin {
    type Error = ();

    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.pin.set_high();
        Ok(())
    }

    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.pin.set_low();
        Ok(())
    }

    fn set_state(&mut self, state: embedded_hal::digital::v2::PinState) -> Result<(), Self::Error> {
        let b = match state {
            embedded_hal::digital::v2::PinState::High => true,
            embedded_hal::digital::v2::PinState::Low => false,
        };
        self.pin.set_state(b);
        Ok(())
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
        self.pin.set_high();
        Ok(())
    }

    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.pin.set_low();
        Ok(())
    }

    fn set_state(&mut self, state: embedded_hal::digital::v2::PinState) -> Result<(), Self::Error> {
        let b = match state {
            embedded_hal::digital::v2::PinState::High => true,
            embedded_hal::digital::v2::PinState::Low => false,
        };
        self.pin.set_state(b);
        Ok(())
    }
}

// fn device_id<T: Cable>(cable: &mut T) -> Vec<u8> {
//     println!("Resetting");
//     cable.change_mode(&[1, 1, 1, 1, 1], true);
//     println!("Set to Shift IR");
//     cable.change_mode(&[0, 1, 1, 0, 0], false);
//     println!("Shift in 0b10001 command");
//     cable.write_data(&[0b10001000], 5, true);
//     println!("Set to Read data");
//     cable.change_mode(&[1, 1, 1, 0, 0], false);
//     cable.read_data(32)
// }

// see archived: https://github.com/kchmck/collect_slice
pub trait CollectSlice: Iterator {
    fn collect_slice(&mut self, slice: &mut [Self::Item]) -> usize;
}

impl<I: ?Sized> CollectSlice for I
where
    I: Iterator,
{
    fn collect_slice(&mut self, slice: &mut [Self::Item]) -> usize {
        slice.iter_mut().zip(self).fold(0, |count, (dest, item)| {
            *dest = item;
            count + 1
        })
    }
}
