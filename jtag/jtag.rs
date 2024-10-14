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
use esp_hal::clock::Clocks;
use esp_hal::gpio::{AnyOutputOpenDrain, Level, Pull};
use esp_hal::timer::systimer::SystemTimer;
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
use esp_println::{print, println};
use fugit::{HertzU64, MicrosDurationU64, TimerRateU64};
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
    let ticker = Ticker::new(&clocks);

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

    let mdc = io.pins.gpio1;
    let mdio = io.pins.gpio0;

    let mut rstn = AnyOutputOpenDrain::new(io.pins.gpio2, Level::Low, Pull::Up);
    let mut reset = || {
        print!("resetting...");
        rstn.set_low();
        delay.delay_millis(2000);
        rstn.set_high();
        delay.delay_millis(300);
        println!("");
    };
    reset();

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

    let mdc_freq = TimerRateU64::kHz(100);
    let mut cont = md::Controller::new(mdc_freq, t, mdc, mdio);
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

    let mut phy_addr = 0xff;
    fn auto_detect(cont: &mut Controller, phy_addr: &mut u8) {
        let phys = detect(cont);
        if phys.is_empty() {
            println!("No PHY detected");
        } else {
            *phy_addr = phys[0];
            let v: Vec<_> = phys.into_iter().map(|a| format!("0x{:x}", a)).collect();
            println!(
                "Detected PHYS: [{:?}] and using 0x{:x}",
                v.join(", "),
                phy_addr
            );
        }
    };
    auto_detect(&mut cont, &mut phy_addr);

    println!("Waiting for input...");

    fn parse_num<T: Num>(str: &str) -> Result<T, String> {
        let str = &str.replace("_", "").replace("'", "");
        match str {
            s if s.starts_with("0x") => T::from_str_radix(&s[2..], 16)
                .map_err(|err| format!("couldn't read {:?} as hex: {}", str, err)),

            s if s.starts_with("0o") => T::from_str_radix(&s[2..], 8)
                .map_err(|err| format!("couldn't read {:?} as octal: {}", str, err)),

            s if s.starts_with("0b") => T::from_str_radix(&s[2..], 2)
                .map_err(|err| format!("couldn't read {:?} as binary: {}", str, err)),

            #[allow(clippy::from_str_radix_10)]
            _ => T::from_str_radix(str, 10)
                .map_err(|err| format!("couldn't read {:?}: {}", str, err)),
        }
    }

    let mut uart0 = UartWrapper::from(uart0);

    loop {
        let prompt = "> ";

        let mut buffer = [0; 128];
        let mut history = [0; 256];

        let mut editor = EditorBuilder::from_slice(&mut buffer)
            .with_slice_history(&mut history)
            .build_sync(&mut uart0)
            .unwrap();

        while let Ok(line) = {
            editor.reinit(&mut uart0).unwrap(); // On every newline, re-initialize the cursor/terminal state

            editor.readline(prompt, &mut uart0)
        } {
            const N: usize = 4;
            let mut split = [""; N];
            let n = line
                .splitn(N, ' ')
                .filter(|s| !s.is_empty())
                .collect_slice(&mut split[..]);
            let split = &split[..n];
            match split {
                ["help"] => {
                    println!("Interactive PHY MDIO tool! Available Commands:");
                    println!("* help: Prints this message");
                    println!("* reset: Resets the PHY by asserting the reset line for some time");
                    println!("* detect [auto]: Detects and reports addresses of connected PHYs");
                    println!("                 If `auto` then the lowest PHY address will be automatically set");
                    println!("* status: Prints the PHY address that reads and writes will be directed to");
                    println!("* setphy ADDR: Set the PHY address that reads and writes will use");
                    println!("* read all|REG: read a PHY register at REG address, or print all standard register values");
                    println!("* write REG DATA: write DATA to register REG");
                }
                ["reset"] => reset(),
                ["detect"] => {
                    let phys = detect(&mut cont);
                    if phys.is_empty() {
                        println!("No PHY detected");
                    } else {
                        let v: Vec<_> = phys.into_iter().map(|a| format!("0x{:x}", a)).collect();
                        println!("Detected PHYS: [{:?}]", v.join(", "));
                    }
                }
                ["detect", "auto"] => auto_detect(&mut cont, &mut phy_addr),
                ["status"] => {
                    println!("Connected to PHY at 0x{:x}", phy_addr);
                }
                ["setphy", addr] => {
                    let phy = parse_num::<u8>(addr);

                    if let Err(err) = &phy {
                        println!("error: {}", err);
                        continue;
                    }

                    phy_addr = phy.unwrap() & 0b00011111;
                }

                ["read", "all"] => print_standard_regs(&mut cont, phy_addr),
                ["read", reg] => {
                    let reg = parse_num(reg);

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
                    println!("usage: read all|REG)");
                }
                ["write", reg, val] => {
                    let reg = parse_num(reg);
                    if let Err(err) = &reg {
                        println!("error: {}", err);
                        continue;
                    }
                    let val = parse_num(val);
                    if let Err(err) = &reg {
                        println!("error: {}", err);
                        continue;
                    }

                    let reg = reg.unwrap();
                    let val = val.unwrap();
                    cont.frame_write(md::MDIOFrame::<2>::new(PHY, reg), val);

                    println!("wrote to 0x{:x}: 0x{:x}", reg, val);
                }
                ["write", ..] => {
                    println!("error: write: unrecognized arguments: {:?}", &split[1..]);
                    println!("usage: write REG DATA)");
                }
                ["relink"] => {
                    let CONT = 0x00;
                    let RESET_LINK = 0x1200;
                    cont.frame_write(md::MDIOFrame::<2>::new(PHY, CONT), RESET_LINK);
                }

                ["count"] => {
                    let count = cont.frame_read(md::MDIOFrame::<2>::new(PHY, COUNT_REG));
                    let event = (count >> 8) & 0b0000_1111;
                    let event_name = match event {
                        0b0000 => "rxerr",
                        0b0001 => "rx",
                        0b0010 => "esd",
                        0b0011 => "ssd",
                        0b0100 => "txerr",
                        0b0101 => "tx",
                        0b0110 => "col",
                        0b1000 => "ld",
                        0b1001 => "ds",
                        _ => {
                            println!("Found unexpected event: `{:b}`", event);
                            continue;
                        }
                    };
                    println!("{} count: {}", event_name, count & 0xff);
                }

                ["count", event] => {
                    let ev = match event {
                        &"rxerr" =>  0b0000,
                        &"rx" =>     0b0001,
                        &"esd" =>    0b0010,
                        &"ssd" =>    0b0011,
                        &"txerr" =>  0b0100,
                        &"tx" =>     0b0101,
                        &"col" =>    0b0110,
                        &"ld" =>     0b1000,
                        &"ds" =>     0b1001,
                        _ => {
                            println!("Invalid event! Should be one of `rxerr`, `rx`, `esd`, `ssd`, `txerr`, `tx`, `col`, `ld`, `ds`");
                            continue;
                        }
                    };

                    cont.frame_write(md::MDIOFrame::<2>::new(PHY, COUNT_REG), ev << 8);
                }

                ["skew", dir, val] => {
                    let shift = match dir {
                        &"tx" => 8,
                        &"rx" => 12,
                        _ => {
                            println!("Invalid `direction`: \"{}\". Must be either `tx` or `rx`", dir);
                            continue
                        }
                    };
                    
                    let v = match val {
                        &"0"     => 0b000,
                        &"0.5"   => 0b001,
                        &"1.0"   => 0b010,
                        &"1.5"   => 0b011,
                        &"2.0"   => 0b100,
                        &"2.5"   => 0b101,
                        &"3.0"   => 0b110,
                        &"3.5"   => 0b111,
                        _ => {
                            println!("Invalid skew amount: \"{}\". Must be one of 0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0, or 3.5", dir);
                            continue
                        }
                    };

                    // 0x17
                    cont.frame_write(md::MDIOFrame::<2>::new(PHY, 0x17), v << shift);
                }

                ["skew"] => {
                    let skew = cont.frame_read(md::MDIOFrame::<2>::new(PHY, 0x17));
                    let rx = (skew >> 12) & 0b111;
                    let tx = (skew >> 8) & 0b111;

                    let rx_skew = match rx {
                        0b000 => "0"  ,
                        0b001 => "0.5",
                        0b010 => "1.0",
                        0b011 => "1.5",
                        0b100 => "2.0",
                        0b101 => "2.5",
                        0b110 => "3.0",
                        0b111 => "3.5",
                        _ => unreachable!()
                    };

                    let tx_skew = match tx {
                        0b000 => "0"  ,
                        0b001 => "0.5",
                        0b010 => "1.0",
                        0b011 => "1.5",
                        0b100 => "2.0",
                        0b101 => "2.5",
                        0b110 => "3.0",
                        0b111 => "3.5",
                        _ => unreachable!()
                    };

                    println!("skew Tx: {}, Rx: {}", tx_skew, rx_skew);
                }

                ["test", mode] => {
                    let m = match mode {
                        &"off" =>    0b000,
                        &"netl" =>   0b001,
                        &"fetl" =>   0b010,
                        &"echo" =>   0b011,
                        &"rjtl" =>   0b100,
                        &"fetls" =>  0b101,
                        _ => {
                            println!("Invalid test mode: `{}`. Must be one of \"off, netl, fetl, echo, rjtl, fetls\"", mode);
                            continue;
                        }
                    };
                    cont.frame_write(md::MDIOFrame::<2>::new(PHY, 0x13), (m << 13) + 1);
                }

                ["test"] => {
                    let test = cont.frame_read(md::MDIOFrame::<2>::new(PHY, 0x13));
                    let mode = (test >> 13) & 0b111;
                    let m = match mode {
                        0b000 => "off",
                        0b001 => "netl",
                        0b010 => "fetl",
                        0b011 => "echo",
                        0b100 => "rjtl",
                        0b101 => "fetls",
                        _ => {
                            println!("Unknown TLOOP field: `{:b}` in PHY_PHYCTL1 (offsete 0x13) register", mode);
                            continue;
                        }
                    };
                    println!("Test mode: `{}`", m);
                }

                [] => {}
                [other, ..] => {
                    println!("error: unrecognized: {}", other)
                }
            }
        }
    }
}

fn detect(cont: &mut Controller) -> Vec<u8> {
    let phy_id1_addr = 0x02;
    let mut phys = Vec::new();
    for p in 0..(1 << 5) {
        let x = cont.frame_read(md::MDIOFrame::<2>::new(p, phy_id1_addr));
        if x != 0xffff {
            // println!("x == {:x}", x);
            // If the data is all 1s then that means the PHY was not detected at that address
            // This could be that the PHY is under reset or just the wrong PHY address
            phys.push(p);
        }
    }
    phys
}

#[derive(Clone, Copy)]
pub struct Ticker {
    pub freq: HertzU64,
}

impl Ticker {
    pub fn new(clocks: &Clocks) -> Self {
        Self {
            freq: HertzU64::MHz(clocks.xtal_clock.to_MHz() as u64 * 10 / 25),
        }
    }

    pub fn now(&self) -> fugit::TimerInstantU64<1_000_000> {
        let t0 = SystemTimer::now();
        let rate: HertzU64 = MicrosDurationU64::from_ticks(1).into_rate();

        fugit::TimerInstantU64::from_ticks(t0 / (self.freq / rate))
    }

    pub fn since(
        &self,
        start: fugit::TimerInstantU64<1_000_000>,
    ) -> fugit::TimerDurationU64<1_000_000> {
        self.now() - start
    }
}

// hoOrAy for NomInaL TypEs
// cf. https://github.com/rust-num/num-traits
trait Num: Sized + Clone + Copy {
    fn from_str_radix(str: &str, radix: u32) -> Result<Self, core::num::ParseIntError>;
}

impl Num for u8 {
    fn from_str_radix(str: &str, radix: u32) -> Result<Self, core::num::ParseIntError> {
        u8::from_str_radix(str, radix)
    }
}
impl Num for u16 {
    fn from_str_radix(str: &str, radix: u32) -> Result<Self, core::num::ParseIntError> {
        u16::from_str_radix(str, radix)
    }
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
