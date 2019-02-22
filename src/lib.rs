#![no_std]
#![allow(dead_code)]
#![allow(non_upper_case_globals)]
extern crate embedded_hal as hal;
extern crate nb;

use hal::blocking::i2c::{Read, Write};

pub enum PhysicalRegisters {
    Status = 0x00,
    Write = 0x01,
    Read = 0x02,
}

pub struct VirtualRegister {
    address: u8,
    size: u8,
}

const HWVersion: VirtualRegister = VirtualRegister { address: 0x00, size: 2 };
const FWVersion: VirtualRegister = VirtualRegister { address: 0x02, size: 2 };
const ControlSetup: VirtualRegister = VirtualRegister { address: 0x04, size: 1 };
const INT_T: VirtualRegister = VirtualRegister { address: 0x05, size: 1 };
const DeviceTemp: VirtualRegister = VirtualRegister { address: 0x06, size: 1 };
const LEDControl: VirtualRegister = VirtualRegister { address: 0x07, size: 1 };

const R_Raw: VirtualRegister = VirtualRegister { address: 0x08, size: 2 };
const S_Raw: VirtualRegister = VirtualRegister { address: 0x0a, size: 2 };
const T_Raw: VirtualRegister = VirtualRegister { address: 0x0c, size: 2 };
const U_Raw: VirtualRegister = VirtualRegister { address: 0x0e, size: 2 };
const V_Raw: VirtualRegister = VirtualRegister { address: 0x10, size: 2 };
const W_Raw: VirtualRegister = VirtualRegister { address: 0x12, size: 2 };

const R_Cal: VirtualRegister = VirtualRegister { address: 0x14, size: 4 };
const S_Cal: VirtualRegister = VirtualRegister { address: 0x18, size: 4 };
const T_Cal: VirtualRegister = VirtualRegister { address: 0x1c, size: 4 };
const U_Cal: VirtualRegister = VirtualRegister { address: 0x20, size: 4 };
const V_Cal: VirtualRegister = VirtualRegister { address: 0x24, size: 4 };
const W_Cal: VirtualRegister = VirtualRegister { address: 0x28, size: 4 };

const RX_VALID: u8 = 0x01;
const TX_VALID: u8 = 0x02;
const WRITE_FLAG: u8 = 0x80;

pub struct SpectralResponse {
    r: f32,
    s: f32,
    t: f32,
    u: f32,
    v: f32,
    w: f32,
}

pub enum BankMode {
    Mode0 = 0x00,
    Mode1 = 0x01,
    Mode2 = 0x02,
    OneShot = 0x03,
}

pub enum Gain {
    Gain1x = 0x00,
    Gain3_7x = 0x01,
    Gain16x = 0x02,
    Gain64x = 0x03,
}

pub struct Config {
    enable_interrupts: bool,
    gain: Gain,
    bank: BankMode,
    integration_time: u8,
}

const DEFAULT_CONFIG: Config = Config {
    enable_interrupts: false,
    gain: Gain::Gain1x,
    bank: BankMode::OneShot,
    integration_time: 0x01,
};

pub struct As7263<I2C: Write + Read> {
    i2c: I2C,
}

impl<I2C, E> As7263<I2C>
    where I2C: Write<Error = E> + Read<Error = E>
{
    fn apply_config(&mut self, config: Config) -> Result<(), E> {
        let control_setup_mask: u8 = (config.bank as u8) << 2
                                   | if config.enable_interrupts { 1 } else { 0 } << 6
                                   | (config.gain as u8) << 4;
        let existing_config = self.read(ControlSetup)? as u8;
        self.write(ControlSetup, existing_config | control_setup_mask)?;
        self.write(INT_T, config.integration_time)?;
        Ok(())
    }

    fn get_spectral_measurement(&mut self) -> Result<SpectralResponse, E> {
        Ok(SpectralResponse {
            r: f32::from_bits(self.read(R_Cal)?),
            s: f32::from_bits(self.read(S_Cal)?),
            t: f32::from_bits(self.read(T_Cal)?),
            u: f32::from_bits(self.read(U_Cal)?),
            v: f32::from_bits(self.read(V_Cal)?),
            w: f32::from_bits(self.read(W_Cal)?),
        })
    }

    fn oneshot_measurement(&mut self) -> Result<SpectralResponse, E> {
        let oneshot_request = 0x01;
        let mut existing_config = self.read(ControlSetup)? as u8;
        self.write(ControlSetup, existing_config | oneshot_request)?;
        existing_config = self.read(ControlSetup)? as u8;
        while existing_config & oneshot_request == 1 {
            existing_config = self.read(ControlSetup)? as u8;
        }
        self.get_spectral_measurement()
    }

    fn write_ready(&mut self) -> Result<bool, E> {
        let status: u8 = TX_VALID;
        self.i2c.read(PhysicalRegisters::Status as u8, &mut [status])?;
        Ok(status & TX_VALID == 0)
    }

    fn read_ready(&mut self) -> Result<bool, E> {
        let status = 0;
        self.i2c.read(PhysicalRegisters::Status as u8, &mut [status])?;
        Ok(status & RX_VALID != 0)
    }

    fn access_virtual_address(&mut self, virtual_addr: u8, write: bool) -> Result<(), E> {
        let access_flag = if write { WRITE_FLAG } else { 0x00 };
        while !self.write_ready()? {}
        self.i2c.write(PhysicalRegisters::Write as u8, &[virtual_addr | access_flag])?;
        Ok(())
    }

    fn write(&mut self, virtual_reg: VirtualRegister, data: u8) -> Result<(), E> {
        self.access_virtual_address(virtual_reg.address, true)?;
        while !self.write_ready()? {}
        self.i2c.write(PhysicalRegisters::Write as u8, &[data])?;
        Ok(())
    }

    fn read(&mut self, virtual_reg: VirtualRegister) -> Result<u32, E> {
        let mut result: u32 = 0;
        for i in 0..virtual_reg.size {
            self.access_virtual_address(virtual_reg.address + i, false)?;
            while !self.read_ready()? {}
            let buffer: u8 = 0;
            self.i2c.read(PhysicalRegisters::Read as u8, &mut [buffer])?;
            result |= (buffer as u32) << (virtual_reg.size - 1 - i);
        }
        Ok(result)
    }
}
