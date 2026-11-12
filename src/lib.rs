#![no_std]
#![doc = include_str!("../README.md")]
#![deny(unsafe_code)]
#![warn(missing_docs)]

use core::{fmt::Debug, marker::PhantomData};
use derive_more::TryFrom;

#[cfg(feature = "async")]
mod commands_async;

#[cfg(feature = "blocking")]
mod commands_blocking;

#[cfg(any(feature = "blocking", feature = "async"))]
mod hal_common {
    use super::*;
    use core::convert::Infallible;
    use embedded_hal::digital::{ErrorType as PinErrorType, OutputPin, PinState};
    use embedded_storage::nor_flash::{
        ErrorType as StorageErrorType, NorFlashError, NorFlashErrorKind,
    };

    /// Dummy pin implementation when HOLD or WP pins are not connected to the MCU.
    pub struct NoPin {}

    impl PinErrorType for NoPin {
        type Error = Infallible;
    }

    impl OutputPin for NoPin {
        fn set_low(&mut self) -> Result<(), Self::Error> {
            Ok(())
        }
        fn set_high(&mut self) -> Result<(), Self::Error> {
            Ok(())
        }
        fn set_state(&mut self, _state: PinState) -> Result<(), Self::Error> {
            Ok(())
        }
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

    impl<Series: NorSeries, SPI, S: Debug, HOLD, WP> StorageErrorType for W25<Series, SPI, HOLD, WP>
    where
        SPI: embedded_hal::spi::ErrorType<Error = S>,
    {
        type Error = Error<S>;
    }

    impl<Series: NorSeries, SPI, S: Debug, P: Debug, HOLD, WP> W25<Series, SPI, HOLD, WP>
    where
        SPI: embedded_hal::spi::ErrorType<Error = S>,
        HOLD: OutputPin<Error = P>,
        WP: OutputPin<Error = P>,
    {
        /// Internal constructor for initializing the W25 flash driver.
        ///
        /// Configures the `HOLD` and `WP` control pins to a high state
        /// (inactive/deselected) and wraps the provided SPI peripheral, pins,
        /// and memory capacity into the driver instance.
        pub(crate) fn _new(
            spi: SPI,
            mut hold: HOLD,
            mut wp: WP,
            capacity: u32,
        ) -> Result<Self, InitError<S, P>> {
            hold.set_high().map_err(InitError::PinError)?;
            wp.set_high().map_err(InitError::PinError)?;
            Ok(Self {
                spi,
                hold,
                wp,
                capacity,
                address_mode: AddressMode::ThreeByte,
                _phantom: PhantomData,
            })
        }

        /// Set the hold pin state.
        ///
        /// The driver doesn't do anything with this pin. When using the chip, make
        /// sure the hold pin is not asserted.
        /// By default this means the pin needs to be high (true).
        ///
        /// This function sets the pin directly and can cause the chip to not work.
        pub fn set_hold(&mut self, value: PinState) -> Result<(), P> {
            self.hold.set_state(value)
        }

        /// Set the write protect pin state.
        ///
        /// The driver doesn't do anything with this pin. When using the chip, make
        /// sure the hold pin is not asserted.
        /// By default this means the pin needs to be high (true).
        ///
        /// This function sets the pin directly and can cause the chip to not work.
        pub fn set_wp(&mut self, value: PinState) -> Result<(), P> {
            self.wp.set_state(value)
        }
    }
}

#[cfg(any(feature = "blocking", feature = "async"))]
pub use hal_common::NoPin;

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
    /// True if the series has a second status register (SR2) and supports Quad/Suspend features.
    const HAS_SR2: bool;
}

/// Addressing mode used by the flash driver.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AddressMode {
    /// Traditional 24-bit addressing. For w25(q/x)128 or smaller
    ThreeByte,

    /// 32-bit addressing. For w25(q/x)256 or bigger up to
    FourByte,
}

impl NorSeries for Q {
    const PAGE_SIZE: u32 = 256;
    const SECTOR_SIZE: u32 = Self::PAGE_SIZE * 16;
    const HAS_SR2: bool = true;
}

impl NorSeries for X {
    const PAGE_SIZE: u32 = 256;
    const SECTOR_SIZE: u32 = Self::PAGE_SIZE * 16;
    const HAS_SR2: bool = false;
}

/// This trait is implemented when the flash supports the reset commands
pub trait Reset {}
impl Reset for Q {}

/// Easily readable representation of the command bytes used by the flash chip.
///
/// # Warning
///
/// It is rather hard to really adapt to global series "Q" or "X" since, even in
/// a single serie, the commands compatibilities vary. For that reason, the
/// available commands for your specific model could vary from whats written
/// below. In this case, since Winbond apparently uses the same instruction set
/// for its commands, every command is implemented for both series, and it is up
/// to you to call the ones compatible with your hardware when using the driver.
/// Also, the X/Q Series are only partially implemented by this driver, things
/// like Security Registers, and commands that require treating I/O pins
/// differently from their "standard" modes are not supported (because the
/// `SpiDevice` trait is not supporting those either).
#[cfg_attr(not(any(feature = "blocking", feature = "async")), allow(dead_code))]
#[repr(u8)]
enum Command {
    WriteEnable = 0x06,
    ReadStatusRegister1 = 0x05,
    ReadData = 0x03,
    FastRead = 0x0B,
    PageProgram = 0x02,
    Block64Erase = 0xD8,
    Block32Erase = 0x52,
    SectorErase = 0x20,
    ChipErase = 0xC7,
    PowerDown = 0xB9,
    ReleasePowerDown = 0xAB, // Or Device ID.
    ManufacturerDeviceId = 0x90,
    JedecId = 0x9F,
    UniqueId = 0x4B,
    ReadStatusRegister2 = 0x35, // Q only.
    EraseSuspend = 0x75,        // Q only.
    EraseResume = 0x7A,         // Q only.
    HighPerformanceMode = 0xA3, // Q only.
    ModeBitReset = 0xFF,        // Q only (0xFFFF for Dual I/O).
    EnableReset = 0x66,         // Q only.
    Reset = 0x99,               // Q only.
    Enter4ByteAddressMode = 0xB7,
    Exit4ByteAddressMode = 0xE9,
}

/// Low level driver for the w25 flash memory chip.
#[cfg_attr(not(any(feature = "blocking", feature = "async")), allow(dead_code))]
pub struct W25<Series, SPI, HOLD, WP> {
    pub(crate) spi: SPI,
    pub(crate) hold: HOLD,
    pub(crate) wp: WP,
    pub(crate) capacity: u32,
    pub(crate) address_mode: AddressMode,
    _phantom: PhantomData<Series>,
}

#[cfg_attr(not(any(feature = "blocking", feature = "async")), allow(dead_code))]
impl<Series: NorSeries, SPI, HOLD, WP> W25<Series, SPI, HOLD, WP> {
    /// Get the total capacity of the flash in bytes
    pub fn capacity(&self) -> u32 {
        self.capacity
    }

    pub(crate) fn n_sectors(&self) -> u32 {
        self.capacity / Series::SECTOR_SIZE
    }

    pub(crate) fn n_blocks_32k(&self) -> u32 {
        self.capacity / 32768
    }

    pub(crate) fn n_blocks_64k(&self) -> u32 {
        self.capacity / 65536
    }

    #[inline]
    pub(crate) fn check_read<S: Debug>(&self, offset: u32, len: usize) -> Result<(), Error<S>> {
        if offset.saturating_add(len as u32) > self.capacity {
            Err(Error::OutOfBounds)
        } else {
            Ok(())
        }
    }

    #[inline]
    pub(crate) fn check_write<S: Debug>(&self, offset: u32, len: usize) -> Result<(), Error<S>> {
        if offset.saturating_add(len as u32) > self.capacity {
            Err(Error::OutOfBounds)
        } else {
            Ok(())
        }
    }

    #[inline]
    pub(crate) fn check_erase<S: Debug>(&self, from: u32, to: u32) -> Result<(), Error<S>> {
        if from > to || to > self.capacity {
            Err(Error::OutOfBounds)
        } else if !from.is_multiple_of(Series::SECTOR_SIZE)
            || !to.is_multiple_of(Series::SECTOR_SIZE)
        {
            Err(Error::NotAligned)
        } else {
            Ok(())
        }
    }
}

/// Errors that can occur during autodetect initialization.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum InitError<S: Debug, P: Debug> {
    /// Something went wrong with the flash device
    DeviceError(Error<S>),
    /// Something went wrong with a pin
    PinError(P),
    /// Device reported that it was not manufactured by Winbond
    ManufacturerNotRecognized(u8),
    /// The device ID did not match any known devices
    DeviceNotRecognized(u8),
}

impl<S: Debug, P: Debug> From<Error<S>> for InitError<S, P> {
    fn from(value: Error<S>) -> Self {
        Self::DeviceError(value)
    }
}

/// Custom error type for the various errors that can be thrown by driver.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum Error<S: Debug> {
    /// Something went wrong with the SPI
    SpiError(S),
    /// Something went wrong with a control pin.
    PinError,
    /// An operation was not aligned
    NotAligned,
    /// An operation was out of bounds
    OutOfBounds,
    /// Setting the write enable bit failed for some reason
    WriteEnableFail,
    /// Attempted to enable Quad Enable (QE) while WP/HOLD are tied to fixed
    /// rails (NoPin) or attempted to execute a command while the chip is in an
    /// invalid state (e.g. not busy when suspending).
    InvalidState,
}

#[allow(dead_code)]
#[cfg_attr(not(any(feature = "blocking", feature = "async")), allow(dead_code))]
#[allow(clippy::identity_op)]
fn command_and_address(command: u8, address: u32) -> [u8; 4] {
    [
        command,
        // MSB, BE.
        ((address & 0xFF0000) >> 16) as u8,
        ((address & 0x00FF00) >> 8) as u8,
        ((address & 0x0000FF) >> 0) as u8,
    ]
}

#[allow(dead_code)]
#[cfg_attr(not(any(feature = "blocking", feature = "async")), allow(dead_code))]
#[allow(clippy::identity_op)]
fn command_address_dummy(command: u8, address: u32) -> [u8; 5] {
    [
        command,
        // MSB, BE.
        ((address & 0xFF0000) >> 16) as u8,
        ((address & 0x00FF00) >> 8) as u8,
        ((address & 0x0000FF) >> 0) as u8,
        0x00, // 8 dummy clocks.
    ]
}

/// Build a command header using the selected address mode.
///
/// Returns the header and its actual length.
pub(crate) fn command_and_address_mode(
    command: u8,
    address: u32,
    mode: AddressMode,
) -> ([u8; 5], usize) {
    let bytes = address.to_be_bytes();

    match mode {
        AddressMode::ThreeByte => ([command, bytes[1], bytes[2], bytes[3], 0], 4),

        AddressMode::FourByte => ([command, bytes[0], bytes[1], bytes[2], bytes[3]], 5),
    }
}

/// Build a fast-read command header using the selected address mode.
///
/// Returns the header and its actual length.
pub(crate) fn command_address_dummy_mode(
    command: u8,
    address: u32,
    mode: AddressMode,
) -> ([u8; 6], usize) {
    let bytes = address.to_be_bytes();

    match mode {
        AddressMode::ThreeByte => (
            [
                command, bytes[1], bytes[2], bytes[3], 0x00, // 8 dummy clocks
                0,
            ],
            5,
        ),

        AddressMode::FourByte => (
            [
                command, bytes[0], bytes[1], bytes[2], bytes[3], 0x00, // 8 dummy clocks
            ],
            6,
        ),
    }
}

/// Major byte of the device identification (ID7-ID0) denoting chip capacity.
///
/// Note that the repr value corresponds to the Manufacturer/DeviceID command
/// (0x90) and not the JEDEC ID (0x9F). The latter is the same value, but
/// incremented by one (with exception of W25_512, which is incremented by
/// seven).
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

/// Result from the JEDEC ID (0x9F) command.
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

    /// Try to get the major device identifier ID7-0, denoting the chip
    /// capacity.
    ///
    /// If the ID does not match any known device, returns the **original**
    /// JEDEC Major Device ID, which is incremented by one compared to the value
    /// returned by the Manufacturer/DeviceID command (0x90).
    pub fn major_device_id(&self) -> Result<MajorDeviceId, u8> {
        let b_jedec = self.0[2];
        let b_device_id = match b_jedec {
            0x20 => 0x19, // Note W25_512 has JEDEC ID 0x20 and not 0x1F.
            _ => b_jedec.checked_sub(1).ok_or(b_jedec)?,
        };
        MajorDeviceId::try_from(b_device_id).map_err(|_e| b_jedec)
    }

    /// Return the minor device identifier ID8-15, denoting the package and
    /// variant of the chip.
    pub const fn minor_device_id(&self) -> u8 {
        self.0[1]
    }
}
