use super::*;
use core::convert::Infallible;
use embedded_hal::{
    digital::OutputPin,
    spi::{Operation, SpiDevice},
};
use embedded_storage::nor_flash::{MultiwriteNorFlash, NorFlash, ReadNorFlash};

impl<Series: NorSeries, SPI, S: Debug, HOLD, WP> ReadNorFlash for W25<Series, SPI, HOLD, WP>
where
    SPI: SpiDevice<Error = S>,
{
    const READ_SIZE: usize = 1;

    fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
        self.read_blocking(offset, bytes)
    }

    fn capacity(&self) -> usize {
        self.capacity() as usize
    }
}

impl<Series: NorSeries, SPI, S: Debug, HOLD, WP> NorFlash for W25<Series, SPI, HOLD, WP>
where
    SPI: SpiDevice<Error = S>,
{
    const WRITE_SIZE: usize = 1;
    const ERASE_SIZE: usize = Series::SECTOR_SIZE as usize;

    fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
        self.erase_range_blocking(from, to)
    }

    fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
        self.write_blocking(offset, bytes)
    }
}

impl<Series: NorSeries, SPI, S: Debug, HOLD, WP> MultiwriteNorFlash for W25<Series, SPI, HOLD, WP> where
    SPI: SpiDevice<Error = S>
{
}

impl<Series: NorSeries, SPI, S: Debug, HOLD, WP, P: Debug> W25<Series, SPI, HOLD, WP>
where
    SPI: SpiDevice<Error = S>,
    HOLD: OutputPin<Error = P>,
    WP: OutputPin<Error = P>,
{
    /// Create a new *blocking* instance of the flash.
    ///
    /// The capacity must be the total chip capacity in bytes.
    /// Weird things can happen if you provide the wrong value.
    /// No checks are done, you're believed at your word.
    pub fn new_blocking(
        spi: SPI,
        hold: HOLD,
        wp: WP,
        capacity: u32,
    ) -> Result<Self, InitError<S, P>> {
        let mut flash = Self::_new(spi, hold, wp, capacity)?;

        while flash.busy_blocking()? {}

        Ok(flash)
    }

    /// Create a new *blocking* instance of the flash, autodetecting the chip
    /// variant and capacity.
    pub fn new_autodetect_blocking(spi: SPI, hold: HOLD, wp: WP) -> Result<Self, InitError<S, P>> {
        let mut flash = Self::new_blocking(spi, hold, wp, 0)?;

        let jedec = flash.jedec_id_blocking()?;
        if jedec.manufacturer() != JedecId::MANUFACTURER {
            return Err(InitError::ManufacturerNotRecognized(jedec.manufacturer()));
        }

        let major_id = jedec
            .major_device_id()
            .map_err(InitError::DeviceNotRecognized)?;
        flash.capacity = major_id.capacity();

        Ok(flash)
    }
}

impl<Series: NorSeries, SPI, S: Debug> W25<Series, SPI, NoPin, NoPin>
where
    SPI: SpiDevice<Error = S>,
{
    /// Create a new *blocking* instance of the flash, but without the nHold and
    /// nWP pins.
    ///
    /// The capacity must be the total chip capacity.
    /// Weird things can happen if you provide the wrong value.
    /// No checks are done, you're believed at your word.
    pub fn new_no_pins_blocking(spi: SPI, capacity: u32) -> Result<Self, InitError<S, Infallible>> {
        Self::new_blocking(spi, NoPin {}, NoPin {}, capacity)
    }

    /// Create a new *blocking* instance of the flash, autodetecting the chip
    /// variant and capacity, but without the nHold and nWP pins.
    pub fn new_autodetect_no_pins_blocking(spi: SPI) -> Result<Self, InitError<S, Infallible>> {
        Self::new_autodetect_blocking(spi, NoPin {}, NoPin {})
    }
}

impl<Series: NorSeries, SPI, S: Debug, HOLD, WP> W25<Series, SPI, HOLD, WP>
where
    SPI: SpiDevice<Error = S>,
{
    /// Read Status Register 1 (0x05, blocking).
    fn read_status_register1_blocking(&mut self) -> Result<u8, Error<S>> {
        let mut buf: [u8; 2] = [Command::ReadStatusRegister1 as u8, 0];

        self.spi
            .transfer_in_place(&mut buf)
            .map_err(Error::SpiError)?;

        Ok(buf[1])
    }

    /// The flash chip is unable to perform new commands while it is still
    /// working on a previous one. Especially erases take a long time.
    /// This function returns true while the chip is unable to respond to
    /// commands (with the exception of the busy command).
    pub(crate) fn busy_blocking(&mut self) -> Result<bool, Error<S>> {
        Ok((self.read_status_register1_blocking()? & 0x01) != 0)
    }

    fn write_enabled_blocking(&mut self) -> Result<bool, Error<S>> {
        Ok((self.read_status_register1_blocking()? & 0x02) != 0)
    }

    /// Request the 64 bit id that is unique to this chip (0x4B, blocking).
    pub fn device_id_blocking(&mut self) -> Result<[u8; 8], Error<S>> {
        let mut buf: [u8; 13] = [0; 13];
        buf[0] = Command::UniqueId as u8;

        self.spi
            .transfer_in_place(&mut buf)
            .map_err(Error::SpiError)?;

        Ok(TryFrom::try_from(&buf[5..]).unwrap())
    }

    /// Request the 24-bit JEDEC ID that is specific to this chip variant
    /// (0x9F, blocking).
    pub fn jedec_id_blocking(&mut self) -> Result<JedecId, Error<S>> {
        let mut buf = [Command::JedecId as u8, 0, 0, 0];

        self.spi
            .transfer_in_place(&mut buf)
            .map_err(Error::SpiError)?;

        Ok(JedecId(TryFrom::try_from(&buf[1..]).unwrap()))
    }

    /// Read Manufacturer and Device Id (0x90, blocking).
    /// Returns (`manufacturer_id, device_id`).
    pub fn manufacturer_device_id_blocking(&mut self) -> Result<(u8, u8), Error<S>> {
        let mut buf: [u8; 6] = [Command::ManufacturerDeviceId as u8, 0, 0, 0, 0, 0];

        self.spi
            .transfer_in_place(&mut buf)
            .map_err(Error::SpiError)?;

        Ok((buf[4], buf[5]))
    }

    /// Reset the chip (0x66 followed by 0x99, blocking).
    pub fn reset_blocking(&mut self) -> Result<(), Error<S>>
    where
        Series: Reset,
    {
        self.spi
            .write(&[Command::EnableReset as u8])
            .map_err(Error::SpiError)?;
        self.spi
            .write(&[Command::Reset as u8])
            .map_err(Error::SpiError)
    }

    /// Reset Continuous Read Mode (0xFFFF, blocking) to return the device to normal SPI
    /// command recognition.
    ///
    /// Sending 16 clocks of all 1s (`[0xFF, 0xFF]`) safely resets both Quad I/O
    /// (which needs 8 clocks) and Dual I/O (which needs 16) Continuous Read
    /// Modes accross Q and X series.
    pub fn mode_bit_reset_blocking(&mut self) -> Result<(), Error<S>> {
        self.spi
            .write(&[Command::ModeBitReset as u8, Command::ModeBitReset as u8])
            .map_err(Error::SpiError)
    }

    /// Reads a chunk of bytes from the flash chip (0x03, up to 33/50 MHz,
    /// blocking).
    ///
    /// The number of bytes read is equal to the length of the buf slice.
    /// The first byte is read from the provided address. This address is then
    /// incremented for each following byte.
    ///
    /// # Arguments
    /// * `address` - Address where the first byte of the buf will be read.
    /// * `buf` - Slice that is going to be filled with the read bytes.
    pub fn read_blocking(&mut self, address: u32, buf: &mut [u8]) -> Result<(), Error<S>> {
        self.check_read(address, buf.len())?;

        if buf.is_empty() {
            return Ok(());
        }

        self.spi
            .transaction(&mut [
                Operation::Write(&command_and_address(Command::ReadData as u8, address)),
                Operation::Read(buf),
            ])
            .map_err(Error::SpiError)
    }

    /// Reads a chunk of bytes at the highest supported clock frequency (0x0B,
    /// up to 104 MHz, blocking).
    ///
    /// The number of bytes read is equal to the length of the buf slice.  The
    /// first byte is read from the provided address. This address is then
    /// incremented for each following byte. Inserts 8 dummy clocks after the
    /// 24-bit address to allow internal address setup.
    ///
    /// # Arguments
    /// * `address` - Address where the first byte of the buf will be read.
    /// * `buf` - Slice that is going to be filled with the read bytes.
    pub fn fast_read_blocking(&mut self, address: u32, buf: &mut [u8]) -> Result<(), Error<S>> {
        self.check_read(address, buf.len())?;

        if buf.is_empty() {
            return Ok(());
        }

        self.spi
            .transaction(&mut [
                Operation::Write(&command_address_dummy(Command::FastRead as u8, address)),
                Operation::Read(buf),
            ])
            .map_err(Error::SpiError)
    }

    /// Sets the enable_write flag on the flash chip to true (0x06, blocking).
    ///
    /// Writes and erases to the chip only have effect when this flag is true.
    /// Each write and erase clears the flag, requiring it to be set to true
    /// again for the next command.
    pub(crate) fn enable_write_blocking(&mut self) -> Result<(), Error<S>> {
        self.spi
            .write(&[Command::WriteEnable as u8])
            .map_err(Error::SpiError)?;

        if !self.write_enabled_blocking()? {
            return Err(Error::WriteEnableFail);
        }

        Ok(())
    }

    /// Writes a chunk of bytes to the flash chip (blocking).
    ///
    /// The first byte is written to the provided address. This address is then
    /// incremented for each following byte.
    ///
    /// # Arguments
    /// * `address` - Address where the first byte of the buf will be written.
    /// * `buf` - Slice of bytes that will be written.
    pub fn write_blocking(&mut self, mut address: u32, mut buf: &[u8]) -> Result<(), Error<S>> {
        self.check_write(address, buf.len())?;

        let chunk_len = (Series::PAGE_SIZE - (address & 0x000000FF)) as usize;
        let mut chunk_len = chunk_len.min(buf.len());
        self.write_page_blocking(address, &buf[..chunk_len])?;

        loop {
            buf = &buf[chunk_len..];
            address += chunk_len as u32;
            chunk_len = buf.len().min(Series::PAGE_SIZE as usize);
            if chunk_len == 0 {
                break;
            }
            self.write_page_blocking(address, &buf[..chunk_len])?;
        }

        Ok(())
    }

    /// Execute a write on a single page (0x02, blocking).
    fn write_page_blocking(&mut self, address: u32, buf: &[u8]) -> Result<(), Error<S>> {
        if (address & 0x000000FF) + buf.len() as u32 > Series::PAGE_SIZE {
            return Err(Error::OutOfBounds);
        }

        self.enable_write_blocking()?;

        self.spi
            .transaction(&mut [
                Operation::Write(&command_and_address(Command::PageProgram as u8, address)),
                Operation::Write(buf),
            ])
            .map_err(Error::SpiError)?;

        while self.busy_blocking()? {}

        Ok(())
    }

    /// Erases a range of sectors (blocking).
    ///
    /// The range is expressed in bytes. These bytes need to be a multiple of
    /// SECTOR_SIZE. If the range starts at SECTOR_SIZE * 3 then the erase
    /// starts at the fourth sector.All sectors are erased in the range
    /// [start_sector..end_sector].  The start address may not be a higher value
    /// than the end address.
    ///
    /// # Arguments
    /// * `start_address` - Address of the first byte of the start of the range
    ///   of sectors that need to be erased.
    /// * `end_address` - Address of the first byte of the end of the range of
    ///   sectors that need to be erased.
    pub fn erase_range_blocking(
        &mut self,
        start_address: u32,
        end_address: u32,
    ) -> Result<(), Error<S>> {
        self.check_erase(start_address, end_address)?;

        let start_sector = start_address / Series::SECTOR_SIZE;
        let end_sector = end_address / Series::SECTOR_SIZE;

        for sector in start_sector..end_sector {
            self.erase_sector_blocking(sector)?;
        }

        Ok(())
    }

    /// Erases a single sector of flash memory with the size of SECTOR_SIZE
    /// (0x20, blocking).
    ///
    /// # Arguments
    /// * `index` - the index of the sector that needs to be erased. The address
    ///   of the first byte of the sector is the provided index * SECTOR_SIZE.
    pub fn erase_sector_blocking(&mut self, index: u32) -> Result<(), Error<S>> {
        if index >= self.n_sectors() {
            return Err(Error::OutOfBounds);
        }

        self.enable_write_blocking()?;

        let address: u32 = index * Series::SECTOR_SIZE;

        self.spi
            .write(&command_and_address(Command::SectorErase as u8, address))
            .map_err(Error::SpiError)?;

        while self.busy_blocking()? {}

        Ok(())
    }

    /// Erases a single block of flash memory with the size of BLOCK_32K_SIZE
    /// (0xC7, blocking).
    ///
    /// # Arguments
    /// * `index` - the index of the block that needs to be erased. The address
    ///   of the first byte of the block is the provided index * BLOCK_32K_SIZE.
    pub fn erase_block_32k_blocking(&mut self, index: u32) -> Result<(), Error<S>> {
        if index >= self.n_blocks_32k() {
            return Err(Error::OutOfBounds);
        }

        self.enable_write_blocking()?;

        let address: u32 = index * 32768;

        self.spi
            .write(&command_and_address(Command::Block32Erase as u8, address))
            .map_err(Error::SpiError)?;

        while self.busy_blocking()? {}

        Ok(())
    }

    /// Erases a single block of flash memory with the size of BLOCK_64K_SIZE
    /// (0xD8, blocking).
    ///
    /// # Arguments
    /// * `index` - the index of the block that needs to be erased. The address
    ///   of the first byte of the block is the provided index * BLOCK_64K_SIZE.
    pub fn erase_block_64k_blocking(&mut self, index: u32) -> Result<(), Error<S>> {
        if index >= self.n_blocks_64k() {
            return Err(Error::OutOfBounds);
        }

        self.enable_write_blocking()?;

        let address: u32 = index * 65536;

        self.spi
            .write(&command_and_address(Command::Block64Erase as u8, address))
            .map_err(Error::SpiError)?;

        while self.busy_blocking()? {}

        Ok(())
    }

    /// Erases all sectors on the flash chip (0xC7, blocking).
    ///
    /// NOTE: This is a very expensive operation, the order of magnitude is the
    /// second.
    pub fn erase_chip_blocking(&mut self) -> Result<(), Error<S>> {
        self.enable_write_blocking()?;

        self.spi
            .write(&[Command::ChipErase as u8])
            .map_err(Error::SpiError)?;

        while self.busy_blocking()? {}

        Ok(())
    }

    /// Puts the chip into power down mode (blocking, 0xB9).
    ///
    /// While in the power-down state, only the Release Power-down/Device ID
    /// (0xAB) instruction will be recognized. This instruction restores the
    /// device to normal operation. All other instructions are ignored.
    ///
    /// NOTE: The chip takes `tDP` (max 3µs) after `/CS` rises to fully enter
    /// power-down mode.  Wait at least 3µs before issuing
    /// `disable_power_down_mode`.
    pub fn enable_power_down_mode_blocking(&mut self) -> Result<(), Error<S>> {
        self.spi
            .write(&[Command::PowerDown as u8])
            .map_err(Error::SpiError)?;

        Ok(())
    }

    /// Releases the chip from power-down mode (0xAB, blocking).
    ///
    /// Restores operation from power-down mode by reading the deviceID from the
    /// device.
    ///
    /// NOTE: The chip takes `tRES1` (max 3µs) after `/CS` rises to return to
    /// standby mode.
    pub fn disable_power_down_mode_blocking(&mut self) -> Result<(), Error<S>> {
        self.spi
            .write(&[Command::ReleasePowerDown as u8])
            .map_err(Error::SpiError)?;

        Ok(())
    }

    /// Releases the chip from power-down mode and reads the 8-bit Device Id
    /// (0xAB, blocking).
    ///
    /// Restores operation from power-down mode by reading the deviceID from the
    /// device.
    ///
    /// NOTE: When reading the Electric Signature, the chip requires `tRES2`
    /// (max 1.8µs) after `/CS` rises to resume normal standby operation.
    pub fn release_power_down_and_device_id_blocking(&mut self) -> Result<u8, Error<S>> {
        let mut buf: [u8; 5] = [Command::ReleasePowerDown as u8, 0, 0, 0, 0];

        self.spi
            .transfer_in_place(&mut buf)
            .map_err(Error::SpiError)?;

        Ok(buf[4])
    }
}

// Q-Series exclusive commands
impl<SPI, S: Debug, HOLD, WP> W25<Q, SPI, HOLD, WP>
where
    SPI: SpiDevice<Error = S>,
{
    /// Read Status Register 2 (0x35, Q-series only, blocking).
    fn read_status_register2_blocking(&mut self) -> Result<u8, Error<S>> {
        let mut buf: [u8; 2] = [Command::ReadStatusRegister2 as u8, 0];

        self.spi
            .transfer_in_place(&mut buf)
            .map_err(Error::SpiError)?;

        Ok(buf[1])
    }

    /// Checks the Erase/Program Suspend status bit (`SUS`, bit 7 of Status
    /// Register 2, blocking).
    pub fn is_suspended_blocking(&mut self) -> Result<bool, Error<S>> {
        Ok((self.read_status_register2_blocking()? & 0x80) != 0)
    }

    /// Suspends an ongoing Sector/Block Erase or Page Program operation (0x75,
    /// Q-series only, blocking).
    ///
    /// Edge cases & Timings:
    /// * Valid only when `BUSY == 1` and `SUS == 0` (returns
    ///   `Error::InvalidState` otherwise).
    /// * Polls until `BUSY` clears to 0 (within `tSUS` <= 20µs), guaranteeing
    ///   the chip is ready to accept read/program commands immediately upon
    ///   return.
    pub fn erase_program_suspend_blocking(&mut self) -> Result<(), Error<S>> {
        let sr1 = self.read_status_register1_blocking()?;
        let sr2 = self.read_status_register2_blocking()?;

        // Must be busy (BUSY == 1) and not already suspended (SUS == 0).
        if (sr1 & 0x01) == 0 || (sr2 & 0x80) != 0 {
            return Err(Error::InvalidState);
        }

        self.spi
            .write(&[Command::EraseSuspend as u8])
            .map_err(Error::SpiError)?;

        // Wait for BUSY to clear (takes up to tSUS = 20µs).
        while self.busy_blocking()? {}

        Ok(())
    }

    /// Resumes a previously suspended Erase or Page Program operation (0x7A,
    /// Q-series only, blocking).
    ///
    /// Edge cases & Timings:
    /// * Valid only when `SUS == 1` and `BUSY == 0` (returns
    ///   `Error::InvalidState` otherwise).
    /// * NOTE: After resuming, a subsequent suspend (`0x75`) must not be issued
    ///   for at least `tSUS` (20µs).
    pub fn erase_program_resume_blocking(&mut self) -> Result<(), Error<S>> {
        let sr1 = self.read_status_register1_blocking()?;
        let sr2 = self.read_status_register2_blocking()?;

        // Must not be busy (BUSY == 0) and must be suspended (SUS == 1).
        if (sr1 & 0x01) != 0 || (sr2 & 0x80) == 0 {
            return Err(Error::InvalidState);
        }

        self.spi
            .write(&[Command::EraseResume as u8])
            .map_err(Error::SpiError)
    }

    /// Puts the Q-series device into High Performance Mode (0xA3, blocking).
    ///
    /// Requires the command byte followed by 3 dummy bytes (24 clock cycles).
    pub fn enable_high_performance_mode_blocking(&mut self) -> Result<(), Error<S>> {
        self.spi
            .write(&[Command::HighPerformanceMode as u8, 0, 0, 0])
            .map_err(Error::SpiError)
    }
}
