#![no_std]

use core::cell::RefCell;

use bitfield_struct::bitfield;
use embedded_hal::{delay::DelayNs, digital::OutputPin, spi::SpiBus};
use snafu::Snafu;

#[cfg(feature = "graphics")]
pub mod display;

#[derive(Snafu, Debug)]
pub enum Error {
    #[snafu(display("Unknown Error"))]
    Unknown,

    #[snafu(display("Pin Error"))]
    Pin,

    #[snafu(display("Spi Error"))]
    Spi,

    #[snafu(display("Command Length Error"))]
    CommandLen,

    #[snafu(display("Data Length Error"))]
    DataLen,
}

pub type Result<T, E = Error> = core::result::Result<T, E>;

pub enum DataCommandMode {
    Command,
    Data,
}

#[repr(u8)]
#[derive(Debug, Copy, Clone)]
pub enum EntryMode {
    Horizontal,
    Vertical,
}

impl EntryMode {
    const fn into_bits(self) -> u8 {
        match self {
            EntryMode::Horizontal => 0b0,
            EntryMode::Vertical => 0b1,
        }
    }

    const fn from_bits(value: u8) -> Self {
        match value {
            0b0 => Self::Horizontal,
            _ => Self::Vertical,
        }
    }
}

#[bitfield(u8)]
pub struct FunctionSet {
    /// The first field occupies the least significant bits
    extended_instruction_set: bool,
    #[bits(1)]
    entry_mode: EntryMode,
    power_down_control: bool,
    #[bits(5)]
    _padding: u8
}

#[repr(u8)]
#[derive(Debug, Copy, Clone)]
pub enum DisplayMode {
    Blank,
    Normal,
    AllOn,
    Inverse,
}

impl DisplayMode {
    const fn into_bits(self) -> u8 {
        match self {
            DisplayMode::Blank => 0b0,
            DisplayMode::Normal => 0b100,
            DisplayMode::AllOn => 0b1,
            DisplayMode::Inverse => 0b101,
        }
    }

    const fn from_bits(value: u8) -> Self {
        match value {
            0b0 => Self::Blank,
            0b100 => Self::Normal,
            0b1 => Self::AllOn,
            _ => Self::Inverse,
        }
    }
}

#[bitfield(u8)]
pub struct DisplayControl {
    #[bits(3)]
    display_mode: DisplayMode,
    #[bits(5)]
    _padding: u8,
}

#[bitfield(u8)]
pub struct YAddress {
    #[bits(3)]
    address: u8,
    #[bits(5)]
    _padding: u8,
}

#[bitfield(u8)]
pub struct XAddress {
    #[bits(7)]
    address: u8,
    #[bits(1)]
    _padding: u8,
}

#[bitfield(u8)]
pub struct TemperatureCoefficient {
    #[bits(2)]
    tc: u8,
    #[bits(6)]
    _padding: u8,
}

#[bitfield(u8)]
pub struct BiasSystem {
    #[bits(3)]
    bs: u8,
    #[bits(5)]
    _padding: u8,
}

#[bitfield(u8)]
pub struct Vop {
    #[bits(7)]
    vop: u8,
    #[bits(1)]
    _padding: u8,
}

#[repr(u8)]
pub enum Command {
    Nop,
    FunctionSet(FunctionSet),
    DisplayControl(DisplayControl),
    SetYAddress(YAddress),
    SetXAddress(XAddress),
}

impl Command {
    const fn into_bits(self) -> u8 {
        match self {
            Self::FunctionSet(fs) => 0b10_0000 | fs.into_bits(),
            Self::DisplayControl(dc) => 0b1000 | dc.into_bits(),
            Self::SetYAddress(addr) => 0b100_0000 | addr.into_bits(),
            Self::SetXAddress(addr) => 0b1000_0000 | addr.into_bits(),
            _ => 0b0,
        }
    }

    #[allow(unused)]
    const fn from_bits(value: u8) -> Self {
        if (value & 0b1000_0000) != 0 {
            return Self::SetXAddress(XAddress::new().with_address(value));
        }
        
        if (value & 0b100_0000) != 0 {
            return Self::SetYAddress(YAddress::new().with_address(value));
        }

        if (value & 0b1000) != 0 {
            return Self::DisplayControl(DisplayControl::new().with_display_mode(DisplayMode::from_bits(value & 0b111)));
        }

        return Self::Nop;
    }
}

#[repr(u8)]
pub enum ExtendedCommand {
    Nop,
    SetTemperatureCoefficient(TemperatureCoefficient),
    SetBiasSystem(BiasSystem),
    WriteVop(Vop),
}


impl ExtendedCommand {
    const fn into_bits(self) -> u8 {
        match self {
            Self::SetTemperatureCoefficient(tc) => 0b100 | tc.into_bits(),
            Self::SetBiasSystem(bs) => 0b1_0000 | bs.into_bits(),
            Self::WriteVop(vop) => 0b1000_0000 | vop.into_bits(),
            _ => 0b0,
        }
    }

    #[allow(unused)]
    const fn from_bits(value: u8) -> Self {
        if (value & 0b1000_0000) != 0 {
            return Self::WriteVop(Vop::new().with_vop(value));
        }

        if (value & 0b1_0000) != 0 {
            return Self::SetBiasSystem(BiasSystem::new().with_bs(value));
        }

        if (value & 0b100) != 0 {
            return Self::SetTemperatureCoefficient(TemperatureCoefficient::new().with_tc(value));
        }

        return Self::Nop;
    }
}

pub struct PCD8544<'a, Bus, DcPin, RstPin, Delay> {
    bus: &'a RefCell<Bus>,
    dc: DcPin,
    rst: RstPin,
    delay: Delay,
}

impl<'a, Bus, DcPin, RstPin, Delay> PCD8544<'a, Bus, DcPin, RstPin, Delay>
where
    Bus: SpiBus,
    DcPin: OutputPin,
    RstPin: OutputPin,
    Delay: DelayNs,
{
    pub fn new(bus: &'a RefCell<Bus>, dc: DcPin, rst: RstPin, delay: Delay) -> Result<Self> {
        Ok(PCD8544 {
            bus,
            dc,
            rst,
            delay,
        })
    }

    // reset should be pulsed low within 30ms of power up
    // reset may be held low at the time of power up
    // minimum T_WL pulse width is 100ns
    // TODO: see if the controller is tolerant of reset without power cycle
    // TODO: if vdd needs to be software controlled then confirm current required or add driver fet
    pub fn reset(&mut self) -> Result<()> {
        self.rst.set_low().map_err(|_| Error::Pin)?;
        //self.vdd.set_low().map_err(|_| Error::Pin)?;
        //self.delay.delay_ns(200);
        //self.vdd.set_high().map_err(|_| Error::Pin)?;

        self.delay.delay_ns(200);

        self.rst.set_high().map_err(|_| Error::Pin)?;

        Ok(())
    }

    pub fn write_byte(&mut self, mode: DataCommandMode, data_command: u8) -> Result<()> {
        self.write(mode, &[data_command])?;

        Ok(())
    }

    pub fn write(&mut self, mode: DataCommandMode, data: &[u8]) -> Result<()> {
        let bus = &mut *self.bus.borrow_mut();

        match mode {
            DataCommandMode::Command => self.dc.set_low().map_err(|_| Error::Pin)?,
            DataCommandMode::Data => self.dc.set_high().map_err(|_| Error::Pin)?,
        }

        bus.write(data).map_err(|_| Error::Spi)?;
        bus.flush().map_err(|_| Error::Spi)?;

        Ok(())
    }

    pub fn command(&mut self, command: Command) -> Result<()> {
        self.write_byte(DataCommandMode::Command, command.into_bits())?;
        Ok(())
    }

    pub fn extended_command(&mut self, command: ExtendedCommand) -> Result<()> {
        self.write_byte(DataCommandMode::Command, command.into_bits())?;
        Ok(())
    }
}

pub struct Nokia5110<'a, Bus, DcPin, RstPin, Delay> {
    pcd8544: PCD8544<'a, Bus, DcPin, RstPin, Delay>,
    function_set: FunctionSet,
    display_mode: DisplayMode,
}

impl <'a, Bus, DcPin, RstPin, Delay> Nokia5110<'a, Bus, DcPin, RstPin, Delay>
where
    Bus: SpiBus,
    DcPin: OutputPin,
    RstPin: OutputPin,
    Delay: DelayNs,
{
    pub fn new(pcd8544: PCD8544<'a, Bus, DcPin, RstPin, Delay>) -> Self {
        Nokia5110 {
            pcd8544,
            function_set: FunctionSet::new(),
            display_mode: DisplayMode::Normal,
        }
    }

    pub fn init(&mut self) -> Result<()> {
        self.pcd8544.reset()?;

        self.function_set.set_power_down_control(false);
        self.function_set.set_entry_mode(EntryMode::Horizontal);
        self.function_set.set_extended_instruction_set(true);
        self.pcd8544.command(Command::FunctionSet(self.function_set))?;
        self.pcd8544.extended_command(ExtendedCommand::WriteVop(Vop::new().with_vop(56)))?;
        self.pcd8544.extended_command(ExtendedCommand::SetTemperatureCoefficient(TemperatureCoefficient::new().with_tc(0x03)))?;
        self.pcd8544.extended_command(ExtendedCommand::SetBiasSystem(BiasSystem::new().with_bs(0b100)))?;
        self.set_extended_instruction_set(false)?;

        self.set_display_mode(DisplayMode::Normal)?;

        Ok(())
    }

    fn set_extended_instruction_set(&mut self, enabled: bool) -> Result<()> {
        self.function_set.set_extended_instruction_set(enabled);
        self.pcd8544.command(Command::FunctionSet(self.function_set))
    }

    pub fn is_enabled(&self) -> bool {
        self.function_set.power_down_control()
    }

    pub fn set_enabled(&mut self, enabled: bool) -> Result<()> {
        self.function_set.set_power_down_control(!enabled);
        self.pcd8544.command(Command::FunctionSet(self.function_set))
    }

    pub fn set_vop(&mut self, vop: u8) -> Result<()> {
        self.set_extended_instruction_set(true)?;
        self.pcd8544.extended_command(ExtendedCommand::WriteVop(Vop::new().with_vop(vop)))?;
        self.set_extended_instruction_set(false)?;
        Ok(())
    }

    pub fn display_mode(&self) -> DisplayMode {
        self.display_mode
    }

    pub fn set_display_mode(&mut self, display_mode: DisplayMode) -> Result<()> {
        self.display_mode = display_mode;
        self.pcd8544.command(Command::DisplayControl(DisplayControl::new().with_display_mode(display_mode)))?;
        Ok(())
    }

    pub fn set_address(&mut self, x: u8, y: u8) -> Result<()> {
        self.pcd8544.command(Command::SetXAddress(XAddress::new().with_address(x)))?;
        self.pcd8544.command(Command::SetYAddress(YAddress::new().with_address(y)))?;
        Ok(())
    }

    pub fn write_data_byte(&mut self, data: u8) -> Result<()> {
        self.pcd8544.write_byte(DataCommandMode::Data, data)?;
        Ok(())
    }

    pub fn write_data(&mut self, data: &[u8]) -> Result<()> {
        self.pcd8544.write(DataCommandMode::Data, data)?;
        Ok(())
    }
}