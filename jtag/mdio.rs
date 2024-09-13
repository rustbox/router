use alloc::vec::{self, Vec};
use esp_hal::{delay::Delay, gpio::{AnyFlex, AnyOutput, Flex, GpioPin, Input, Level, Output, Pull}};
use esp_hal::prelude::*;
use esp_println::println;


// Taken from Table 25, page 56 of GPY111 datasheet
const START_FRAME: &[u8] = &[0, 1];
const WRITE_OP: &[u8] = &[0, 1];
const READ_OP: &[u8] = &[1, 0];
const TURNAROUND: u8 = 2;
const Z: u8 = 2;

#[derive(Debug, Clone, Copy)]
pub enum TriState {
    High,
    Low,
    Z,
}

impl TriState {
    fn from_bslice(bs: &[u8]) -> Vec<TriState> {
        bs.into_iter().map(|b| TriState::from(*b)).collect()
    }
}

impl From<u8> for TriState {
    fn from(value: u8) -> Self {
        match value {
            0 => TriState::Low,
            1 => TriState::High,
            Z => TriState::Z,
            _ => TriState::High,
        }
    }
}

impl From<TriState> for u16 {
    fn from(value: TriState) -> Self {
        match value {
            TriState::Low => 0,
            TriState::High => 1,
            TriState::Z => Z as u16
        }
    }
}

#[derive(Debug, Clone)]
pub enum Op {
    Read,
    Write
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
pub struct MDIOFrame<const PRE: u8 = 2> {
    /// How many cycles of 1s from the controller to be issued before a frame
    preamble: u8,
    op: Op,
    /// PHY address: This is 5 bits, and we will use bits [4, 0] of the internal u8
    /// when actually transmitting this value
    phy_addr: u8,
    /// Register is the address of the register to read or write to.
    /// Again register is actually only 5 bits, so we use bits [4, 0] of the given u8
    register: u8,

    data: u16
}


impl<const PRE: u8> MDIOFrame<PRE> {
    pub fn new(phy_addr: u8, register: u8) -> MDIOFrame {
        MDIOFrame {
            preamble: PRE,
            op: Op::Read,
            phy_addr: phy_addr & 0b00011111,
            register: register & 0b00011111,
            data: 0,
        }
    }

    pub fn read(self) -> MDIOFrame {
        let mut frame = MDIOFrame::<PRE>::new(self.phy_addr, self.register);
        frame.op = Op::Read;
        frame
    }

    pub fn write(self, data: u16) -> MDIOFrame {
        let mut frame = MDIOFrame::<PRE>::new(self.phy_addr, self.register);
        frame.op = Op::Write;
        frame.data = data;
        frame
    }

    pub fn header(&self) -> Vec<TriState> where [(); PRE as usize]: {
        let pre = [1_u8; PRE as usize];
        let start = START_FRAME;
        let op = self.op.signal();

        let mut d = Vec::new();
        d.extend_from_slice(&TriState::from_bslice(&pre));
        d.extend_from_slice(&TriState::from_bslice(start));
        d.extend_from_slice(&TriState::from_bslice(&op));
        
        // PHY Address
        for b in 0..5 {
            // When b = 0, we want to grab bit 3, to go from high to low
            let h = 4 - b;
            d.push(TriState::from(self.phy_addr >> h & 1));
        }

        // MDIO Register Address
        for b in 0..5 {
            // When b = 0, we want to grab bit 3, to go from high to low
            let h = 4 - b;
            d.push(TriState::from(self.register >> h & 1));
        }

        d
    }

    fn turnaround(&self) -> Vec<TriState> {
        TriState::from_bslice(match self.op {
            Op::Read => &[Z, Z],
            Op::Write => &[0, 0]
        })
    }
}

enum RWState {
    Reading,
    Writing
}

pub struct Controller<'a> {
    half_period_ns: u32,
    delay: Delay,
    mdc: AnyOutput<'a>,
    mdio: AnyFlex<'a>,
    rw_state: RWState,
}

impl<'a> Controller<'a> {
    pub fn new(freq_kh: u32, delay: Delay, mdc: GpioPin<0>, mdio: GpioPin<1>) -> Controller<'a> {
        let half_period_ns = (1_000_000 / freq_kh) / 2;
        
        let mdc = AnyOutput::new(mdc, Level::Low);
        let mdio = AnyFlex::new(mdio);
        Controller { half_period_ns, delay, mdc, mdio, rw_state: RWState::Writing }
    }

    fn half_cycle_high(&mut self) {
        self.mdc.set_high();
        self.delay.delay_nanos(self.half_period_ns);
    }

    fn half_cycle_low(&mut self) {
        self.mdc.set_low();
        self.delay.delay_nanos(self.half_period_ns);
    }

    fn set_write_mode(&mut self) {
        println!("Setting to write");
        self.rw_state = RWState::Writing;
        self.mdio.set_as_output();
    }

    fn set_read_mode(&mut self) {
        println!("Setting to read");
        self.rw_state = RWState::Reading;
        self.mdio.set_as_input(Pull::None);
    }

    /// Write a "High Z". This means we activate the clock but we don't sample any values
    /// from MDIO, we simply activate the clock
    pub fn write_z(&mut self, times: u8) {
        // Set MDIO as HIGH Z by putting pin in Read mode
        // if let RWState::Writing = self.rw_state {
        //     self.rw_state = RWState::Reading;
        //     self.mdio.set_as_input(Pull::None);
        // }
        // Clock `times`, high low, each high will clock out a "Z"
        for _ in 0..times {
            self.half_cycle_high();
            self.half_cycle_low();
        }
    }

    pub fn write_bit(&mut self, v: TriState) {
        // If we're Reading, then prep MDIO for writing
        // if let RWState::Reading = self.rw_state {
        //     println!("Plz write");
        //     self.rw_state = RWState::Writing;
        //     self.mdio.set_as_output();
        // }

        // Set the MDIO value
        match v {
            // Treat High Z as input, and "write" an input I guess
            // What happens if we're set as input from this Z and then we set an output?
            TriState::Z => {},
            TriState::Low => self.mdio.set_low(),
            TriState::High => self.mdio.set_high()
        }
        // Clock cycle
        self.half_cycle_high();
        self.half_cycle_low();
    }

    pub fn read_bit(&mut self) -> TriState {
        // Setup MDIO for reading
        // if let RWState::Writing = self.rw_state {
        //     self.rw_state = RWState::Reading;
        //     self.mdio.set_as_input(Pull::None);
        // }

        // Clock High to grab a bit from the device
        self.half_cycle_high();

        // Sample the value of MDIO
        let x = match self.mdio.get_level() {
            Level::High => TriState::High,
            Level::Low => TriState::Low,
        };

        // Clock low again
        self.half_cycle_low();
        x
    }

    pub fn frame_read(&mut self, frame: MDIOFrame) -> u16 {
        // Just assert that the frame is a read if we're in the read function
        let frame = frame.read();
        let header = frame.header();

        self.set_write_mode();
        // Write the header bits
        for b in header {
            println!("W: {:?}", b);
            self.write_bit(b);
        }

        self.set_read_mode();
        // Just "write" a Z for the turnaround
        self.write_z(TURNAROUND);

        let mut value = 0;
        for b in 0..16 {
            // Read bits High to Low
            let h = 15 - b;

            // We can assume this is either 0 or 1, never Z (2)
            let x: u16 = self.read_bit().into();
            value = value | (x << h);
        }

        value
    }

    pub fn frame_write(&mut self, frame: MDIOFrame, data: u16) {
        let frame = frame.write(data); // Sort of redundant I guess, but assert write at least
        let header = frame.header();

        self.set_write_mode();
        // Write the header bits
        for b in header {
            self.write_bit(b);
        }

        self.set_read_mode();
        // Just "write" a Z for the turnaround
        self.write_z(TURNAROUND);

        self.set_write_mode();
        for b in 0..16 {
            // Write bits high to low
            let h = 15 - b;

            let x = TriState::from(((data >> h) & 1) as u8);
            self.write_bit(x);
        }
    }
}
