#![no_std]
#![no_main]
#![feature(riscv_ext_intrinsics)]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]
#![allow(clippy::unusual_byte_groupings)]
#![allow(unused_variables)]

extern crate alloc;
use alloc::{format, string::String};
use core::{mem::MaybeUninit, num};
use esp_backtrace as _;
use esp_hal::uart;
use esp_hal::uart::Uart;
use esp_hal::uart::{UartRx, UartTx};
use esp_hal::{
    clock::ClockControl, delay::Delay, gpio::Io, peripherals::Peripherals, prelude::*,
    system::SystemControl,
};
use esp_hal::{
    clock::Clocks,
    timer::{systimer::SystemTimer, timg::TimerGroup},
};
use esp_hal::{gpio::GpioPin, Blocking};
use esp_hal::{
    gpio::Level,
    timer::{ErasedTimer, PeriodicTimer},
};
use esp_println::println;
use fugit::{Duration, HertzU32, HertzU64, MicrosDurationU64, NanosDurationU64, TimerRateU64};
use md::{Controller, MDIOFrame};
use nb::block;
use noline::builder::EditorBuilder;

// use mdio::miim::{Read, Write};
// use jtag_taps::cable::Cable;
// use jtag_taps::statemachine::{JtagSM, Register};
// use jtag_taps::taps::Taps;

pub mod md;

const PHY: u8 = 0x03;

#[link_section = ".rwtext"]
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

    let mdc_freq = TimerRateU64::kHz(1);
    let mut cont = md::Controller::new(mdc_freq, t, mdc, mdio, ticker);
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

    fn parse_num<T: Num>(str: &str) -> Result<T, String> {
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

        while let Ok(line) = editor.readline(prompt, &mut uart0) {
            const N: usize = 4;
            let mut split = [""; N];
            let n = line.splitn(N, ' ').collect_slice(&mut split[..]);
            let split = &split[..n];
            match split {
                ["reset"] => println!("todo: reset!"),

                ["read", "all"] => print_standard_regs(&mut cont),
                ["read", reg] => {
                    let reg = parse_num(reg);

                    if let Err(err) = &reg {
                        println!("error: {}", err);
                        continue;
                    }

                    let reg = reg.unwrap();

                    let start0 = SystemTimer::now();
                    let start = ticker.now();
                    let read = cont.frame_read(md::MDIOFrame::<2>::new(PHY, reg));
                    let dur = ticker.since(start);

                    println!("read 0x{:x}: 0x{:x}", reg, read);
                    println!(
                        "  (took {}, or {} periods)",
                        dur,
                        dur / mdc_freq.into_duration::<1, 1_000_000>(),
                    );
                }
                ["read", ..] => {
                    println!("error: read: unrecognized arguments: {:?}", &split[1..]);
                    println!("usage: read [all|REG])");
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
                    println!("error: read: unrecognized arguments: {:?}", &split[1..]);
                    println!("usage: read [all|REG])");
                }

                ["wat"] => {
                    println!("lol");

                    let mdc = &mut cont.mdc;
                    let mdio = &mut cont.mdio;
                    let clock = &mut cont.clock;

                    mdio.set_state((Level::Low).into());
                    block!(clock.wait()).unwrap();
                    block!(clock.wait()).unwrap();
                    mdc.set_low();

                    // .....
                    mdio.set_state((Level::High).into());

                    block!(clock.wait()).unwrap();
                    mdc.set_high();
                    block!(clock.wait()).unwrap();
                    mdc.set_low();

                    mdio.set_state((Level::Low).into());

                    block!(clock.wait()).unwrap();
                    mdc.set_high();
                    block!(clock.wait()).unwrap();
                    // block!(clock.wait()).unwrap();
                    // block!(clock.wait()).unwrap();
                    mdc.set_low();
                }
                ["wat2"] => {
                    println!("ummmmmm");

                    let mdc = &mut cont.mdc;
                    let mdio = &mut cont.mdio;
                    let clock = &mut cont.clock;

                    mdio.set_state((Level::Low).into());
                    block!(clock.wait()).unwrap();
                    block!(clock.wait()).unwrap();

                    // start of write
                    mdio.set_state((Level::High).into());

                    block!(clock.wait()).unwrap();
                    while unsafe {
                        core::mem::transmute::<
                            HertzU32,
                            esp_hal::timer::timg::Timer<
                                esp_hal::timer::timg::Timer0<esp_hal::peripherals::TIMG0>,
                                Blocking,
                            >,
                        >(clocks.apb_clock)
                    }
                    .is_interrupt_set()
                    {}
                    mdc.set_high();
                    block!(clock.wait()).unwrap();
                    while unsafe {
                        core::mem::transmute::<
                            HertzU32,
                            esp_hal::timer::timg::Timer<
                                esp_hal::timer::timg::Timer0<esp_hal::peripherals::TIMG0>,
                                Blocking,
                            >,
                        >(clocks.apb_clock)
                    }
                    .is_interrupt_set()
                    {}
                    mdc.set_low();

                    // end of write

                    block!(clock.wait()).unwrap();
                    mdc.set_high();

                    mdio.set_state((Level::Low).into());

                    block!(clock.wait()).unwrap();
                    // block!(clock.wait()).unwrap();
                    // block!(clock.wait()).unwrap();
                    // mdc.set_low();
                }
                ["wat3"] => {
                    println!("ummmmmm");

                    let mdc = &mut cont.mdc;
                    let mdio = &mut cont.mdio;
                    let clock = &mut cont.clock;

                    mdio.set_state((Level::Low).into());
                    block!(clock.wait()).unwrap();
                    block!(clock.wait()).unwrap();

                    // start of write
                    mdio.set_state((Level::High).into());

                    block!(clock.wait()).unwrap();
                    mdc.set_high();
                    block!(clock.wait()).unwrap();
                    mdc.set_low();
                    // end of write

                    block!(clock.wait()).unwrap();
                    mdc.set_high();

                    mdio.set_state((Level::Low).into());

                    block!(clock.wait()).unwrap();
                    // block!(clock.wait()).unwrap();
                    // block!(clock.wait()).unwrap();
                    // mdc.set_low();
                }
                ["watd"] => {
                    println!("delay with micros to ensure the interrupt bit is cleared by the next call into `clock.wait()`");

                    let mdc = &mut cont.mdc;
                    let mdio = &mut cont.mdio;
                    let clock = &mut cont.clock;

                    let mut wait = || {
                        block!(clock.wait()).unwrap();
                        delay.delay_micros(12);
                    };

                    mdio.set_state((Level::Low).into());
                    wait();
                    wait();

                    // start of write
                    mdio.set_state((Level::High).into());

                    // block!(clock.wait()).unwrap();
                    wait();
                    mdc.set_high();
                    wait();
                    mdc.set_low();
                    // end of write

                    wait();
                    mdc.set_high();

                    mdio.set_state((Level::Low).into());

                    wait();
                    // block!(clock.wait()).unwrap();
                    // block!(clock.wait()).unwrap();
                    // mdc.set_low();
                }

                ["wat-cf"] => {
                    println!(
                        "can we set two gpios at the same time or do they stomp on each other?"
                    );

                    let mdc = &mut cont.mdc;
                    let mdio = &mut cont.mdio;
                    let clock = &mut cont.clock;

                    // given both signals start as high:
                    // how long will this code take to run?
                    //      my guess: on the order of 300µs
                    // when will they fall relative to each other?
                    //      my guess: with 100µs of spacing between them
                    // when will they rise relative to each other?
                    //      my guess: at about the same time (+/- ~10µs)
                    //
                    // observed (ooooohhh, hmm: with DIN pull = "mid"):
                    //      takes anywhere from ~600µs to 1200µs measured from falling edge to second signal rise (mebe, rise time?)
                    //      though the "on CPU" parts do claim to finish in ~300µs once things are cached, +/- 5-6µs
                    //
                    //      nope, they fall right at the same instant, very reliably
                    //
                    //      as seen above, it varies quite a lot when the analyzer "sees" a 1 on both lines
                    //
                    // with (DIN pull = "up")
                    //      takes a fairly consistent ~214µs from fall to second rise
                    //
                    //      fall is separated by about 101µs, as expected
                    //
                    //      rise is ~12µs separate

                    let start = ticker.now();

                    mdc.set_low();
                    delay.delay_micros(100);

                    mdio.set_low();
                    delay.delay_micros(100);

                    if mdc.is_set_high() || mdio.is_set_high() {
                        panic!("bad start");
                    }

                    mdc.set_high();
                    mdio.set_high();

                    // delay.delay_micros(100);
                    // let res = unsafe { &*esp_hal::peripherals::GPIO::PTR }
                    //     .out()
                    //     .read()
                    //     .bits();
                    // println!("0b{}", res);

                    if !(mdc.is_set_high() && mdio.is_set_high()) {
                        panic!("lololololololol");
                    }

                    let dur = ticker.since(start);
                    println!("took: {}", dur);
                }
                ["hmm"] => {
                    println!("ok so given that the memory write to the 'W1TS'/'W1TC' registers seem to stall (?) for an indeterminate(?) amount of time");
                    println!("then what?");

                    let mdc = &mut cont.mdc;
                    let mdio = &mut cont.mdio;
                    let clock = &mut cont.clock;

                    let start = ticker.now();

                    mdio.set_state((Level::Low).into());
                    delay.delay_micros(100);
                    mdc.set_low();

                    // start of write
                    mdio.set_state((Level::High).into());

                    delay.delay_micros(100);
                    mdc.set_high();
                    delay.delay_micros(100);
                    mdc.set_low();
                    // end of write

                    delay.delay_micros(100);
                    mdc.set_high();

                    mdio.set_state((Level::Low).into());

                    delay.delay_micros(100);

                    mdc.set_high();
                    mdio.set_high();

                    if !(mdc.is_set_high() && mdio.is_set_high()) {
                        panic!("lololololololol");
                    }

                    let dur = ticker.since(start);
                    println!("took: {}", dur);
                }

                [other, ..] => {
                    println!("error: unrecognized: {}", other)
                }
                [] => {}
            }
        }
    }
}

// cf. esp_hal::delay::Delay
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

fn print_standard_regs(cont: &mut Controller) {
    for r in 0..32 {
        let f = md::MDIOFrame::<2>::new(PHY, r);
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
