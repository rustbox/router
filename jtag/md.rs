use alloc::vec::Vec;
use esp_hal::gpio::{AnyFlex, AnyOutput, GpioPin, Level, Pull};
use esp_hal::{
    prelude::*,
    timer::{ErasedTimer, PeriodicTimer},
};
use esp_println::println;
use fugit::{HertzU64, KilohertzU64};
use nb::block;

// Taken from Table 25, page 56 of GPY111 datasheet
const START_FRAME: &[u8] = &[0, 1];
const WRITE_OP: &[u8] = &[0, 1];
const READ_OP: &[u8] = &[1, 0];
const TURNAROUND: u8 = 2;

#[derive(Debug, Clone)]
pub enum Op {
    Read,
    Write,
}

impl Op {
    pub fn signal(&self) -> &[u8] {
        match self {
            Op::Read => READ_OP,
            Op::Write => WRITE_OP,
        }
    }
}

#[derive(Debug, Clone)]
/// PRE: How many cycles of 1s from the controller to be issued before a frame
pub struct MDIOFrame<const PRE: u8 = 2> {
    /// PHY address: This is 5 bits, and we will use bits [4, 0] of the internal u8
    /// when actually transmitting this value
    phy_addr: u8,
    /// Register is the address of the register to read or write to.
    /// Again register is actually only 5 bits, so we use bits [4, 0] of the given u8
    register: u8,
}

impl<const PRE: u8> MDIOFrame<PRE> {
    pub fn new(phy_addr: u8, register: u8) -> Self {
        MDIOFrame {
            phy_addr: phy_addr & 0b00011111,
            register: register & 0b00011111,
        }
    }

    pub fn header(&self, op: Op) -> impl IntoIterator<Item = Level>
    where
        [(); PRE as usize]:,
    {
        let pre = [1_u8; PRE as usize];
        let start = START_FRAME;
        let op = op.signal();

        let mut d: Vec<u8> = Vec::new();

        d.extend_from_slice(&pre);
        d.extend_from_slice(start);
        d.extend_from_slice(op);

        // PHY Address
        for b in 0..5 {
            // When b = 0, we want to grab bit 4, to go from high to low
            let h = 4 - b;
            d.push(self.phy_addr >> h & 1);
        }

        // MDIO Register Address
        for b in 0..5 {
            // When b = 0, we want to grab bit 4, to go from high to low
            let h = 4 - b;
            d.push(self.register >> h & 1);
        }

        d.iter()
            .map(|v| {
                // cheese it
                match v & 1 {
                    1 => Level::High,
                    0 => Level::Low,
                    _ => unreachable!(),
                }
            })
            .collect::<Vec<_>>()
    }
}

pub struct Controller<'a> {
    freq: HertzU64,
    clock: PeriodicTimer<ErasedTimer>,
    mdc: AnyOutput<'a>,
    mdio: AnyFlex<'a>,
}

impl<'a> Controller<'a> {
    pub fn new(
        freq: HertzU64,
        mut clock: PeriodicTimer<ErasedTimer>,
        mdc: GpioPin<0>,
        mdio: GpioPin<1>,
    ) -> Controller<'a> {
        clock.start(freq.into_duration() / 2).unwrap();

        let mut mdc = AnyOutput::new(mdc, Level::Low);
        let mdio = AnyFlex::new(mdio);

        mdc.set_low();

        Controller {
            freq,
            clock,
            mdc,
            mdio,
        }
    }

    #[link_section = ".rwtext"]
    fn pulse_clock(&mut self) {
        block!(self.clock.wait()).unwrap();
        self.mdc.set_high();
        block!(self.clock.wait()).unwrap();
        self.mdc.set_low();
    }

    #[inline]
    fn set_write_mode(&mut self) {
        self.mdio.set_as_open_drain(Pull::Up);
    }

    #[inline]
    fn set_read_mode(&mut self) {
        self.mdio.set_as_input(Pull::Up);
    }

    #[inline]
    pub fn turnaround(&mut self) {
        for _ in 0..TURNAROUND {
            self.pulse_clock();
        }
    }

    #[inline]
    pub fn write_bit(&mut self, v: Level) {
        // Set the MDIO value
        self.mdio.set_level(v);

        // println!("v: {:?}", v);

        // Clock cycle
        self.pulse_clock();
    }

    #[inline]
    pub fn read_bit(&mut self) -> Level {
        self.pulse_clock();

        // Sample the value of MDIO
        self.mdio.get_level()
    }

    #[link_section = ".rwtext"]
    pub fn frame_read<const PRE: u8>(&mut self, frame: MDIOFrame<PRE>, ticker: crate::Ticker) -> u16
    where
        [(); PRE as usize]:,
    {
        // Just assert that the frame is a read if we're in the read function
        let header = frame.header(Op::Read);

        self.set_write_mode();
        block!(self.clock.wait()).unwrap(); // else we'll foreshorten the first cycle

        // block!(self.clock.wait()).unwrap();
        // block!(self.clock.wait()).unwrap();
        // block!(self.clock.wait()).unwrap();
        // block!(self.clock.wait()).unwrap();
        // block!(self.clock.wait()).unwrap();

        let start = ticker.now();

        // self.clock.clear_interrupt(); // else we'll foreshorten the first cycle

        // Write the header bits
        let mut wrote = 0;
        for b in header {
            // println!("W: {:?}", b);
            self.write_bit(b);
            wrote += 1;
        }

        let start1 = ticker.now();

        self.set_read_mode();
        self.turnaround();

        let mid = ticker.now();

        let mut counter2 = 0;
        let mut value = 0;
        for b in 0..16 {
            // Read bits High to Low
            let h = 15 - b;

            let x: u16 = match self.read_bit() {
                Level::Low => 0,
                Level::High => 1,
            };
            value |= x << h;
            counter2 += 1;
        }

        let end = ticker.now();
        println!(
            "total: {} ; inner bits: {} {} {}",
            (end - start) / self.freq.into_duration::<1, 1_000_000>(),
            (start1 - start) / self.freq.into_duration::<1, 1_000_000>(),
            (end - mid) / self.freq.into_duration::<1, 1_000_000>(),
            (mid - start1) / self.freq.into_duration::<1, 1_000_000>(),
        );
        println!(
            "headers: want {}, got: {} (wrote: {})",
            (14 + PRE as u32) * self.freq.into_duration::<1, 1_000_000>(),
            (start1 - start),
            wrote,
        );
        println!(
            "turnaround: want {}, got: {}",
            2 * self.freq.into_duration::<1, 1_000_000>(),
            (mid - start1),
        );
        println!(
            "reading: want {}, got: {} (read: {})",
            16 * self.freq.into_duration::<1, 1_000_000>(),
            (end - mid),
            counter2,
        );

        value
    }

    #[link_section = ".rwtext"]
    pub fn frame_write(&mut self, frame: MDIOFrame, data: u16) {
        let header = frame.header(Op::Write);

        self.set_write_mode();
        self.clock.clear_interrupt(); // else we'll foreshorten the first cycle

        // Write the header bits
        for b in header {
            self.write_bit(b);
        }

        self.turnaround();

        for b in 0..16 {
            // Write bits high to low
            let h = 15 - b;

            self.write_bit(match data >> h & 1 {
                0 => Level::Low,
                1 => Level::High,

                _ => unreachable!(),
            });
        }
    }
}
