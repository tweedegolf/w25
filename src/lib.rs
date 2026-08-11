#![no_std]
#![doc = include_str!("../README.md")]
#![deny(unsafe_code)]
#![warn(missing_docs)]

use core::{fmt::Debug, marker::PhantomData};
use derive_more::TryFrom;
use embedded_hal::digital::{OutputPin, PinState};
use embedded_storage::nor_flash::{ErrorType, NorFlashError, NorFlashErrorKind};

mod commands_impl;

/// The Q series
pub struct Q;
/// The X series
pub struct X;

/// Any series that is a NOR flash implements this trait
pub trait NorSeries {
    /// The size of a page in bytes
    const PAGE_SIZE: u32;
    /// The size of a sector in bytes
    const SECTOR_SIZE: u32;
}

impl NorSeries for Q {
    const PAGE_SIZE: u32 = 256;
    const SECTOR_SIZE: u32 = Self::PAGE_SIZE * 16;
}

impl NorSeries for X {
    const PAGE_SIZE: u32 = 256;
    const SECTOR_SIZE: u32 = Self::PAGE_SIZE * 16;
}

/// This trait is implemented when the flash supports the reset commands
pub trait Reset {}

impl Reset for Q {}

/// Easily readable representation of the command bytes used by the flash chip.
#[repr(u8)]
enum Command {
    PageProgram = 0x02,
    ReadData = 0x03,
    ReadStatusRegister1 = 0x05,
    WriteEnable = 0x06,
    SectorErase = 0x20,
    JedecId = 0x9F,
    UniqueId = 0x4B,
    Block32Erase = 0x52,
    Block64Erase = 0xD8,
    ChipErase = 0xC7,
    EnableReset = 0x66,
    PowerDown = 0xB9,
    ReleasePowerDown = 0xAB,
    Reset = 0x99,
}

/// Low level driver for the w25 flash memory chip.
pub struct W25<Series, SPI, HOLD, WP> {
    spi: SPI,
    hold: HOLD,
    wp: WP,
    capacity: u32,
    _pantom: PhantomData<Series>,
}

impl<Series: NorSeries, SPI, HOLD, WP> W25<Series, SPI, HOLD, WP> {
    /// Get the total capacity of the flash in bytes
    pub fn capacity(&self) -> u32 {
        self.capacity
    }

    fn n_sectors(&self) -> u32 {
        self.capacity / Series::SECTOR_SIZE
    }

    fn n_blocks_32k(&self) -> u32 {
        self.capacity / 32768
    }

    fn n_blocks_64k(&self) -> u32 {
        self.capacity / 65536
    }
}

impl<Series: NorSeries, SPI, S: Debug, P: Debug, HOLD, WP> W25<Series, SPI, HOLD, WP>
where
    SPI: embedded_hal::spi::ErrorType<Error = S>,
    HOLD: OutputPin<Error = P>,
    WP: OutputPin<Error = P>,
{
    /// Create a new instance of the flash.
    ///
    /// The capacity must be the total chip capacity.
    /// Weird things can happen if you provide the wrong value.
    /// No checks are done, you're believed at your word.
    pub fn new(spi: SPI, hold: HOLD, wp: WP, capacity: u32) -> Result<Self, P> {
        let mut flash = W25 {
            spi,
            hold,
            wp,
            capacity,
            _pantom: PhantomData,
        };

        flash.hold.set_high()?;
        flash.wp.set_high()?;

        Ok(flash)
    }

    /// Set the hold pin state.
    ///
    /// The driver doesn't do anything with this pin. When using the chip, make sure the hold pin is not asserted.
    /// By default this means the pin needs to be high (true).
    ///
    /// This function sets the pin directly and can cause the chip to not work.
    pub fn set_hold(&mut self, value: PinState) -> Result<(), P> {
        self.hold.set_state(value)
    }

    /// Set the write protect pin state.
    ///
    /// The driver doesn't do anything with this pin. When using the chip, make sure the hold pin is not asserted.
    /// By default this means the pin needs to be high (true).
    ///
    /// This function sets the pin directly and can cause the chip to not work.
    pub fn set_wp(&mut self, value: PinState) -> Result<(), P> {
        self.wp.set_state(value)
    }
}

impl<Series: NorSeries, SPI, S: Debug> W25<Series, SPI, (), ()>
where
    SPI: embedded_hal::spi::ErrorType<Error = S>,
{
    /// Create a new instance of the flash, but without the nHold and nWP pins.
    ///
    /// The capacity must be the total chip capacity.
    /// Weird things can happen if you provide the wrong value.
    /// No checks are done, you're believed at your word.
    pub fn new_no_pins(spi: SPI, capacity: u32) -> Self {
        Self {
            spi,
            hold: (),
            wp: (),
            capacity,
            _pantom: PhantomData,
        }
    }
}

impl<Series: NorSeries, SPI, S: Debug, HOLD, WP> ErrorType for W25<Series, SPI, HOLD, WP>
where
    SPI: embedded_hal::spi::ErrorType<Error = S>,
{
    type Error = Error<S>;
}

/// Custom error type for the various errors that can be thrown by driver.
/// Can be converted into a NorFlashError.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum Error<S: Debug> {
    /// Something went wrong with the SPI
    SpiError(S),
    /// An operation was not aligned
    NotAligned,
    /// An operation was out of bounds
    OutOfBounds,
    /// Setting the write enable bit failed for some reason
    WriteEnableFail,
}

impl<S: Debug> NorFlashError for Error<S> {
    fn kind(&self) -> NorFlashErrorKind {
        match self {
            Error::NotAligned => NorFlashErrorKind::NotAligned,
            Error::OutOfBounds => NorFlashErrorKind::OutOfBounds,
            _ => NorFlashErrorKind::Other,
        }
    }
}

#[allow(clippy::identity_op)]
fn command_and_address(command: u8, address: u32) -> [u8; 4] {
    [
        command,
        // MSB, BE
        ((address & 0xFF0000) >> 16) as u8,
        ((address & 0x00FF00) >> 8) as u8,
        ((address & 0x0000FF) >> 0) as u8,
    ]
}

/// Major byte of the device identification (ID7-ID0) denoting chip capacity.
///
/// Note that the repr value corresponds to the Manufacturer/DeviceID command (0x90) and not the JEDEC ID (0x9F).
/// The latter is the same value, but incremented by one.
#[derive(Debug, Clone, Copy, TryFrom)]
#[repr(u8)]
#[try_from(repr)]
pub enum MajorDeviceId {
    /// W25X10 512kb device
    W25_05 = 0x05,
    /// W25[QX]10 1Mb device
    W25_10 = 0x10,
    /// W25[QX]20 2Mb device
    W25_20 = 0x11,
    /// W25[QX]40 4Mb device
    W25_40 = 0x12,
    /// W25[QX]80 8Mb device
    W25_80 = 0x13,
    /// W25[QX]16 16Mb device
    W25_16 = 0x14,
    /// W25[QX]32 32Mb device
    W25_32 = 0x15,
    /// W25[QX]64 64Mb device
    W25_64 = 0x16,
    /// W25Q128 128Mb device
    W25_128 = 0x17,
    /// W25Q256 256Mb device
    W25_256 = 0x18,
    /// W25Q512 512Mb device
    W25_512 = 0x19,
    /// W25Q01 1Gb device
    W25_01 = 0x20,
    /// W25Q02 2Gb device
    W25_02 = 0x21,
}

/// Result from the
pub struct JedecId([u8; 3]);

impl JedecId {
    /// Manufacturer byte of the [JedecId].
    ///
    /// Should always be 0xEF for Winbond.
    pub const fn manufacturer(&self) -> u8 {
        self.0[0]
    }

    /// Try to get the major device identifier ID7-0, denoting the chip capacity.
    ///
    /// If the ID does not match any known device, returns the **original** JEDEC Major Device ID,
    /// which is incremented by one compared to the value returned by the Manufacturer/DeviceID command (0x90).
    pub fn major_device_id(&self) -> Result<MajorDeviceId, u8> {
        let b = self.0[2];
        MajorDeviceId::try_from(b.checked_sub(1).ok_or(b)?).map_err(|_e| b)
    }

    /// Return the minor device identifier ID8-15, denoting the package and variant of the chip.
    pub fn minor_device_id(&self) -> u8 {
        self.0[1]
    }
}
