#![no_std]
#![doc = include_str!("../README.md")]
#![deny(unsafe_code)]
#![warn(missing_docs)]

use core::{fmt::Debug, marker::PhantomData};
use derive_more::TryFrom;
use embedded_hal::digital::{OutputPin, PinState};
use embedded_hal_async::spi::SpiDevice;
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
    /// The capacity must be the total chip capacity in bytes.
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

/// Errors that can occur during autodetect initialization.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum AutodetectError<S: Debug, P: Debug> {
    /// Something went wrong with the flash device
    DeviceError(Error<S>),
    /// Something went wrong with a pin
    PinError(P),
    /// Device reported that it was not manufactured by Winbond
    ManufacturerNotRecognized(u8),
    /// The device ID did not match any known devices
    DeviceNotRecognized(u8),
}

impl<S: Debug, P: Debug> From<Error<S>> for AutodetectError<S, P> {
    fn from(value: Error<S>) -> Self {
        Self::DeviceError(value)
    }
}

impl<Series: NorSeries, SPI, S: Debug, P: Debug, HOLD, WP> W25<Series, SPI, HOLD, WP>
where
    SPI: SpiDevice<Error = S>,
    HOLD: OutputPin<Error = P>,
    WP: OutputPin<Error = P>,
{
    /// Create a new instance of the flash, autodetecting the chip variant and capacity.
    pub async fn new_autodetect(
        spi: SPI,
        hold: HOLD,
        wp: WP,
    ) -> Result<Self, AutodetectError<S, P>> {
        let mut flash = W25 {
            spi,
            hold,
            wp,
            capacity: 0, // For now do not set the capacity of the device, as we do not know yet.
            _pantom: PhantomData,
        };

        flash.hold.set_high().map_err(AutodetectError::PinError)?;
        flash.wp.set_high().map_err(AutodetectError::PinError)?;

        let jedec_id = flash.jedec_id().await?;
        let manufacturer = jedec_id.manufacturer();
        if jedec_id.manufacturer() != JedecId::MANUFACTURER {
            return Err(AutodetectError::ManufacturerNotRecognized(manufacturer));
        }

        let major_device_id = jedec_id
            .major_device_id()
            .map_err(AutodetectError::DeviceNotRecognized)?;

        // Update the capacity.
        flash.capacity = major_device_id.capacity();

        Ok(flash)
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
/// The latter is the same value, but incremented by one (with exception of W25_512, which is incremented by seven).
#[derive(Debug, Clone, Copy, TryFrom)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
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

impl MajorDeviceId {
    /// Capacity of a device that has the identifier assigned, in bytes.
    pub const fn capacity(&self) -> u32 {
        let capacity_kilobits = match *self {
            MajorDeviceId::W25_05 => 512,
            MajorDeviceId::W25_10 => 1024,
            MajorDeviceId::W25_20 => 2 * 1024,
            MajorDeviceId::W25_40 => 4 * 1024,
            MajorDeviceId::W25_80 => 8 * 1024,
            MajorDeviceId::W25_16 => 16 * 1024,
            MajorDeviceId::W25_32 => 32 * 1024,
            MajorDeviceId::W25_64 => 64 * 1024,
            MajorDeviceId::W25_128 => 128 * 1024,
            MajorDeviceId::W25_256 => 256 * 1024,
            MajorDeviceId::W25_512 => 512 * 1024,
            MajorDeviceId::W25_01 => 1024 * 1024,
            MajorDeviceId::W25_02 => 2 * 1024 * 1024,
        };

        const FACTOR_KILOBITS_BYTES: u32 = 1024 / 8;
        capacity_kilobits * FACTOR_KILOBITS_BYTES
    }
}

/// Result from the
pub struct JedecId([u8; 3]);

impl JedecId {
    /// ID assigned to Winbond.
    pub const MANUFACTURER: u8 = 0xEF;

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
        let b_jedec = self.0[2];
        let b_device_id = match b_jedec {
            0x20 => 0x19, // Note W25_512 has JEDEC ID 0x20 and not 0x1F.
            _ => b_jedec.checked_sub(1).ok_or(b_jedec)?,
        };
        MajorDeviceId::try_from(b_device_id).map_err(|_e| b_jedec)
    }

    /// Return the minor device identifier ID8-15, denoting the package and variant of the chip.
    pub const fn minor_device_id(&self) -> u8 {
        self.0[1]
    }
}
