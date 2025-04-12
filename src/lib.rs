#![cfg_attr(not(test), no_std)]

// This mod MUST go first, so that the others see its macros.
pub(crate) mod fmt;

use core::num::NonZeroU32;

use dap_rs::{
    dap::DelayNs,
    jtag,
    swd::{self, APnDP, DPRegister, Swd},
    swj::{Dependencies, Pins},
};

pub trait InputOutputPin {
    fn set_as_output(&mut self);
    fn set_high(&mut self, high: bool);

    fn set_as_input(&mut self);
    fn is_high(&mut self) -> bool;
}

pub trait DelayCycles: DelayNs {
    fn cpu_clock(&self) -> u32;
    fn delay_cycles(&mut self, cycles: u32);
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Dir {
    Write = 0,
    Read = 1,
}

pub struct BitbangAdapter<IO: InputOutputPin, D: DelayCycles> {
    nreset: IO,
    tdi: IO,
    tms_swdio: IO,
    tck_swclk: IO,
    tdo: IO,
    delay: D,
    bit_cycles: u32,
    target_clock: Option<NonZeroU32>,
}

impl<IO, D> BitbangAdapter<IO, D>
where
    IO: InputOutputPin,
    D: DelayCycles,
{
    pub fn new(
        mut nreset: IO,
        mut tdi: IO,
        mut tms_swdio: IO,
        mut tck_swclk: IO,
        mut tdo: IO,
        delay: D,
    ) -> Self {
        nreset.set_high(true);
        nreset.set_as_output();

        //io.set_pull(Pull::Up);
        tms_swdio.set_high(true);
        tms_swdio.set_as_output();

        //ck.set_pull(Pull::None);
        tck_swclk.set_as_output();

        tdi.set_as_input();
        tdo.set_as_output();

        Self {
            nreset,
            tdi,
            tms_swdio,
            tck_swclk,
            tdo,
            delay,
            bit_cycles: 0,
            target_clock: None,
        }
    }

    fn req(&mut self, port: APnDP, dir: Dir, addr: DPRegister) {
        let req = (port as u32) | (dir as u32) << 1 | (addr as u32) << 2;
        let parity = req.count_ones() % 2;
        self.shift_out(0b10000001 | req << 1 | parity << 5, 8);
    }

    pub fn read(&mut self, port: APnDP, addr: DPRegister) -> swd::Result<u32> {
        self.req(port, Dir::Read, addr);

        let ack = self.shift_in(4); // turnaround + ack
        match (ack >> 1) & 0b111 {
            0b001 => {} // ok
            0b010 => {
                self.shift_in(1); // turnaround
                return Err(swd::Error::AckWait);
            }
            0b100 => {
                self.shift_in(1); // turnaround
                return Err(swd::Error::AckFault);
            }
            _ => {
                self.shift_in(1); // turnaround
                return Err(swd::Error::AckUnknown(ack as u8));
            }
        }

        let data = self.shift_in(32);
        let parity = self.shift_in(1);
        if parity != data.count_ones() % 2 {
            return Err(swd::Error::BadParity);
        }

        self.shift_in(1); // turnaround

        Ok(data)
    }

    pub fn write(&mut self, port: APnDP, addr: DPRegister, data: u32) -> swd::Result<()> {
        self.req(port, Dir::Write, addr);

        let ack = self.shift_in(5); // turnaround + ack + turnaround
        match (ack >> 1) & 0b111 {
            0b001 => {} // ok
            0b010 => return Err(swd::Error::AckWait),
            0b100 => return Err(swd::Error::AckFault),
            _ => return Err(swd::Error::AckUnknown(ack as _)),
        }

        self.shift_out(data, 32);
        self.shift_out(data.count_ones() % 2, 1);

        Ok(())
    }

    fn shift_out(&mut self, val: u32, num_bits: usize) {
        self.tms_swdio.set_as_output();
        for i in 0..num_bits {
            self.tms_swdio.set_high(val & (1 << i) != 0);
            self.clock_pulse();
        }
    }

    fn shift_in(&mut self, num_bits: usize) -> u32 {
        self.tms_swdio.set_as_input();
        let mut val = 0;
        for i in 0..num_bits {
            val |= (self.tms_swdio.is_high() as u32) << i;
            self.clock_pulse();
        }
        val
    }

    fn clock_pulse(&mut self) {
        self.wait();
        self.tck_swclk.set_high(true);
        self.wait();
        self.tck_swclk.set_high(false);
    }

    fn wait(&mut self) {
        self.delay.delay_cycles(self.bit_cycles);
    }

    fn apply_clock(&mut self) {
        if let Some(target_clock) = self.target_clock {
            self.bit_cycles = (self.delay.cpu_clock() / 2)
                .div_ceil(target_clock.get())
                .max(1);

            self.target_clock = None;
        }
    }

    fn set_target_clock(&mut self, max_frequency: u32) -> bool {
        debug!("set frequency({})", max_frequency);
        match NonZeroU32::new(max_frequency) {
            Some(frequency) => {
                self.target_clock = Some(frequency);
                true
            }
            _ => false,
        }
    }
}

impl<IO: InputOutputPin, D: DelayCycles> Dependencies<Self, Self> for BitbangAdapter<IO, D> {
    fn high_impedance_mode(&mut self) {
        self.tms_swdio.set_as_input();
        self.tck_swclk.set_as_input();
        self.nreset.set_as_input();
        self.tdi.set_as_input();
        self.tdo.set_as_input();
    }

    fn process_swj_clock(&mut self, max_frequency: u32) -> bool {
        self.set_target_clock(max_frequency)
    }

    fn process_swj_pins(&mut self, output: Pins, mask: Pins, wait_us: u32) -> Pins {
        if mask.contains(Pins::SWCLK) {
            self.tck_swclk.set_high(output.contains(Pins::SWCLK));
        }
        if mask.contains(Pins::SWDIO) {
            self.tms_swdio.set_high(output.contains(Pins::SWDIO));
        }
        if mask.contains(Pins::NRESET) {
            self.nreset.set_high(output.contains(Pins::NRESET));
        }
        if mask.contains(Pins::TDO) {
            self.tdo.set_high(output.contains(Pins::TDO));
        }

        if wait_us != 0 {
            self.delay.delay_us(wait_us);
        }

        let mut read = Pins::empty();

        read.set(Pins::SWCLK, self.tck_swclk.is_high());
        read.set(Pins::SWDIO, self.tms_swdio.is_high());
        read.set(Pins::NRESET, self.nreset.is_high());
        read.set(Pins::TDO, self.tdo.is_high());
        read.set(Pins::TDI, self.tdi.is_high());
        read.set(Pins::NTRST, true);

        read
    }

    fn process_swj_sequence(&mut self, data: &[u8], num_bits: usize) {
        _ = self.write_sequence(num_bits, data);
    }
}

impl<IO, D> swd::Swd<Self> for BitbangAdapter<IO, D>
where
    IO: InputOutputPin,
    D: DelayCycles,
{
    const AVAILABLE: bool = true;

    fn read_inner(&mut self, port: APnDP, addr: DPRegister) -> swd::Result<u32> {
        debug!("read_inner({:?}, {:?})", port, addr);
        self.read(port, addr)
    }

    fn write_inner(&mut self, port: APnDP, addr: DPRegister, data: u32) -> swd::Result<()> {
        debug!("write_inner({:?}, {:?}, {:x})", port, addr, data);
        self.write(port, addr, data)
    }

    fn set_clock(&mut self, max_frequency: u32) -> bool {
        self.set_target_clock(max_frequency)
    }

    /// Write a sequence of bits using SWDIO and the clock line running at the configured freq.
    fn write_sequence(&mut self, mut num_bits: usize, data: &[u8]) -> swd::Result<()> {
        self.apply_clock();
        debug!("write_sequence({})", num_bits);
        for b in data.iter().copied() {
            if num_bits == 0 {
                break;
            }
            let bits = num_bits.min(8);
            self.shift_out(b as u32, bits);
            num_bits -= bits;
        }
        Ok(())
    }

    /// Read a sequence of bits using SWDIO and the clock line running at the configured freq.
    fn read_sequence(&mut self, mut num_bits: usize, data: &mut [u8]) -> swd::Result<()> {
        self.apply_clock();
        debug!("read_sequence({})", num_bits);
        for b in data.iter_mut() {
            if num_bits == 0 {
                break;
            }
            let bits = num_bits.min(8);
            *b = self.shift_in(bits) as u8;
            num_bits -= bits;
        }
        Ok(())
    }
}

impl<IO, D> jtag::Jtag<Self> for BitbangAdapter<IO, D>
where
    IO: InputOutputPin,
    D: DelayCycles,
{
    const AVAILABLE: bool = false;

    fn sequences(&mut self, data: &[u8], rxbuf: &mut [u8]) -> u32 {
        self.apply_clock();

        todo!()
    }

    fn set_clock(&mut self, max_frequency: u32) -> bool {
        self.set_target_clock(max_frequency)
    }
}
