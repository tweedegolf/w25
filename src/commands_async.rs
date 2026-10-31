use super::*;
use core::convert::Infallible;
use embassy_futures::yield_now;
use embedded_hal::digital::OutputPin;
use embedded_hal_async::spi::{Operation, SpiDevice};
use embedded_storage_async::nor_flash::{MultiwriteNorFlash, NorFlash, ReadNorFlash};

impl<Series: NorSeries, SPI, S: Debug, HOLD, WP> ReadNorFlash for W25<Series, SPI, HOLD, WP>
where
    SPI: SpiDevice<Error = S>,
{
    const READ_SIZE: usize = 1;

    async fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
        self.read(offset, bytes).await
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

    async fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
        self.erase_range(from, to).await
    }

    async fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
        self.write(offset, bytes).await
    }
}

impl<Series: NorSeries, SPI, S: Debug, HOLD, WP> MultiwriteNorFlash for W25<Series, SPI, HOLD, WP>
where
    SPI: SpiDevice<Error = S>,
    S: Debug,
{
}

impl<Series: NorSeries, SPI, S: Debug, P: Debug, HOLD, WP> W25<Series, SPI, HOLD, WP>
where
    SPI: SpiDevice<Error = S>,
    HOLD: OutputPin<Error = P>,
    WP: OutputPin<Error = P>,
{
    /// Create a new instance of the flash.
    ///
    /// The capacity must be the total chip capacity in bytes.
    /// Weird things can happen if you provide the wrong value.
    /// No checks are done, you're believed at your word.
    pub async fn new(spi: SPI, hold: HOLD, wp: WP, capacity: u32) -> Result<Self, InitError<S, P>> {
        let mut flash = Self::_new(spi, hold, wp, capacity)?;

        // Ensure the device is not busy from before a MCU restart
        while flash.busy().await? {
            // Avoid starving the executor when the SPI is
            // fast enough for the busy check to not yield
            yield_now().await;
        }

        Ok(flash)
    }

    /// Create a new instance of the flash, autodetecting the chip variant and
    /// capacity.
    pub async fn new_autodetect(spi: SPI, hold: HOLD, wp: WP) -> Result<Self, InitError<S, P>> {
        let mut flash = Self::new(spi, hold, wp, 0).await?;

        let jedec = flash.jedec_id().await?;
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
    /// Create a new instance of the flash, but without the nHold and nWP pins.
    ///
    /// The capacity must be the total chip capacity.
    /// Weird things can happen if you provide the wrong value.
    /// No checks are done, you're believed at your word.
    pub async fn new_no_pins(spi: SPI, capacity: u32) -> Result<Self, InitError<S, Infallible>> {
        Self::new(spi, NoPin {}, NoPin {}, capacity).await
    }

    /// Create a new instance of the flash, autodetecting the chip variant and
    /// capacity, but without the nHold and nWP pins.
    pub async fn new_autodetect_no_pins(spi: SPI) -> Result<Self, InitError<S, Infallible>> {
        Self::new_autodetect(spi, NoPin {}, NoPin {}).await
    }
}

impl<Series: NorSeries, SPI, S: Debug, HOLD, WP> W25<Series, SPI, HOLD, WP>
where
    SPI: SpiDevice<Error = S>,
{
    /// Read Status Register 1 (0x05).
    pub async fn read_status_register1(&mut self) -> Result<u8, Error<S>> {
        let mut buf: [u8; 2] = [Command::ReadStatusRegister1 as u8; 2];

        self.spi
            .transfer_in_place(&mut buf)
            .await
            .map_err(Error::SpiError)?;

        Ok(buf[1])
    }

    /// The flash chip is unable to perform new commands while it is still
    /// working on a previous one. Especially erases take a long time.
    /// This function returns true while the chip is unable to respond to
    /// commands (with the exception of the busy command).
    pub(crate) async fn busy(&mut self) -> Result<bool, Error<S>> {
        Ok((self.read_status_register1().await? & 0x01) != 0)
    }

    async fn write_enabled(&mut self) -> Result<bool, Error<S>> {
        Ok((self.read_status_register1().await? & 0x02) != 0)
    }

    /// Explicitely resets the Write Enable Latch (WEL) bit in Status Register 1
    /// to 0 (0x04).
    pub async fn disable_write(&mut self) -> Result<(), Error<S>> {
        self.spi
            .write(&[Command::WriteDisable as u8])
            .await
            .map_err(Error::SpiError)
    }

    /// Request the 64 bit id that is unique to this chip (0x4B).
    pub async fn device_id(&mut self) -> Result<[u8; 8], Error<S>> {
        let mut buf: [u8; 13] = [0; 13];
        buf[0] = Command::UniqueId as u8;

        self.spi
            .transfer_in_place(&mut buf)
            .await
            .map_err(Error::SpiError)?;

        Ok(TryFrom::try_from(&buf[5..]).unwrap())
    }

    /// Request the 24-bit JEDEC ID that is specific to this chip variant
    /// (0x9F).
    pub async fn jedec_id(&mut self) -> Result<JedecId, Error<S>> {
        let mut buf = [0; 4];
        buf[0] = Command::JedecId as u8;

        self.spi
            .transfer_in_place(&mut buf)
            .await
            .map_err(Error::SpiError)?;

        Ok(JedecId(TryFrom::try_from(&buf[1..]).unwrap()))
    }

    /// Read Manufacturer and Device Id (0x90).
    /// Returns (`manufacturer_id, device_id`).
    pub async fn manufacturer_device_id(&mut self) -> Result<(u8, u8), Error<S>> {
        let mut buf: [u8; 6] = [Command::ManufacturerDeviceId as u8, 0, 0, 0, 0, 0];

        self.spi
            .transfer_in_place(&mut buf)
            .await
            .map_err(Error::SpiError)?;

        Ok((buf[4], buf[5]))
    }

    /// Reset the chip (0x66 followed by 0x99).
    pub async fn reset(&mut self) -> Result<(), Error<S>>
    where
        Series: Reset,
    {
        self.spi
            .write(&[Command::EnableReset as u8])
            .await
            .map_err(Error::SpiError)?;
        self.spi
            .write(&[Command::Reset as u8])
            .await
            .map_err(Error::SpiError)
    }

    /// Reset Continuous Read Mode (0xFFFF) to return the device to normal SPI
    /// command recognition.
    ///
    /// Sending 16 clocks of all 1s (`[0xFF, 0xFF]`) safely resets both Quad I/O
    /// (which needs 8 clocks) and Dual I/O (which needs 16) Continuous Read
    /// Modes accross Q and X series.
    pub async fn mode_bit_reset(&mut self) -> Result<(), Error<S>> {
        self.spi
            .write(&[Command::ModeBitReset as u8, Command::ModeBitReset as u8])
            .await
            .map_err(Error::SpiError)
    }

    /// Reads a chunk of bytes from the flash chip (0x03, up to 33/50 MHz).
    ///
    /// The number of bytes read is equal to the length of the buf slice.
    /// The first byte is read from the provided address. This address is then
    /// incremented for each following byte.
    ///
    /// # Arguments
    /// * `address` - Address where the first byte of the buf will be read.
    /// * `buf` - Slice that is going to be filled with the read bytes.
    pub async fn read(&mut self, address: u32, buf: &mut [u8]) -> Result<(), Error<S>> {
        self.check_read(address, buf.len())?;

        if buf.is_empty() {
            return Ok(());
        }

        self.spi
            .transaction(&mut [
                Operation::Write(&command_and_address(Command::ReadData as u8, address)),
                Operation::Read(buf),
            ])
            .await
            .map_err(Error::SpiError)
    }

    /// Reads a chunk of bytes at the highest supported clock frequency (0x0B,
    /// up to 104 MHz).
    ///
    /// The number of bytes read is equal to the length of the buf slice.  The
    /// first byte is read from the provided address. This address is then
    /// incremented for each following byte. Inserts 8 dummy clocks after the
    /// 24-bit address to allow internal address setup.
    ///
    /// # Arguments
    /// * `address` - Address where the first byte of the buf will be read.
    /// * `buf` - Slice that is going to be filled with the read bytes.
    pub async fn fast_read(&mut self, address: u32, buf: &mut [u8]) -> Result<(), Error<S>> {
        self.check_read(address, buf.len())?;

        if buf.is_empty() {
            return Ok(());
        }

        self.spi
            .transaction(&mut [
                Operation::Write(&command_address_dummy(Command::FastRead as u8, address)),
                Operation::Read(buf),
            ])
            .await
            .map_err(Error::SpiError)
    }

    /// Sets the enable_write flag on the flash chip to true (0x06).
    ///
    /// Writes and erases to the chip only have effect when this flag is true.
    /// Each write and erase clears the flag, requiring it to be set to true
    /// again for the next command.
    pub(crate) async fn enable_write(&mut self) -> Result<(), Error<S>> {
        self.spi
            .write(&[Command::WriteEnable as u8])
            .await
            .map_err(Error::SpiError)?;

        if !self.write_enabled().await? {
            return Err(Error::WriteEnableFail);
        }

        Ok(())
    }

    /// Writes a chunk of bytes to the flash chip.
    ///
    /// The first byte is written to the provided address. This address is then
    /// incremented for each following byte.
    ///
    /// # Arguments
    /// * `address` - Address where the first byte of the buf will be written.
    /// * `buf` - Slice of bytes that will be written.
    pub async fn write(&mut self, mut address: u32, mut buf: &[u8]) -> Result<(), Error<S>> {
        self.check_write(address, buf.len())?;

        // Write first chunk, taking into account that given addres might
        // point to a location that is not on a page boundary.
        let chunk_len = (Series::PAGE_SIZE - (address & 0x000000FF)) as usize;
        let chunk_len = chunk_len.min(buf.len());
        self.write_page(address, &buf[..chunk_len]).await?;

        // Write rest of the chunks.
        let mut chunk_len = chunk_len;
        loop {
            buf = &buf[chunk_len..];
            address += chunk_len as u32;
            chunk_len = buf.len().min(Series::PAGE_SIZE as usize);
            if chunk_len == 0 {
                break;
            }
            self.write_page(address, &buf[..chunk_len]).await?;
        }

        Ok(())
    }

    /// Execute a write on a single page (0x02).
    async fn write_page(&mut self, address: u32, buf: &[u8]) -> Result<(), Error<S>> {
        // We don't support wrapping writes. They're scary.
        if (address & 0x000000FF) + buf.len() as u32 > Series::PAGE_SIZE {
            return Err(Error::OutOfBounds);
        }

        self.enable_write().await?;

        self.spi
            .transaction(&mut [
                Operation::Write(&command_and_address(Command::PageProgram as u8, address)),
                Operation::Write(buf),
            ])
            .await
            .map_err(Error::SpiError)?;

        while self.busy().await? {
            // Avoid starving the executor when the SPI is
            // fast enough for the busy check to not yield.
            yield_now().await;
        }

        Ok(())
    }

    /// Erases a range of sectors.
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
    pub async fn erase_range(
        &mut self,
        start_address: u32,
        end_address: u32,
    ) -> Result<(), Error<S>> {
        self.check_erase(start_address, end_address)?;

        let start_sector = start_address / Series::SECTOR_SIZE;
        let end_sector = end_address / Series::SECTOR_SIZE;

        for sector in start_sector..end_sector {
            self.erase_sector(sector).await?;
        }

        Ok(())
    }

    /// Erases a single sector of flash memory with the size of SECTOR_SIZE
    /// (0x20).
    ///
    /// # Arguments
    /// * `index` - the index of the sector that needs to be erased. The address
    ///   of the first byte of the sector is the provided index * SECTOR_SIZE.
    pub async fn erase_sector(&mut self, index: u32) -> Result<(), Error<S>> {
        if index >= self.n_sectors() {
            return Err(Error::OutOfBounds);
        }

        self.enable_write().await?;

        let address: u32 = index * Series::SECTOR_SIZE;

        self.spi
            .write(&command_and_address(Command::SectorErase as u8, address))
            .await
            .map_err(Error::SpiError)?;

        while self.busy().await? {
            // Avoid starving the executor when the SPI is
            // fast enough for the busy check to not yield
            yield_now().await;
        }

        Ok(())
    }

    /// Erases a single block of flash memory with the size of BLOCK_32K_SIZE
    /// (0xC7).
    ///
    /// # Arguments
    /// * `index` - the index of the block that needs to be erased. The address
    ///   of the first byte of the block is the provided index * BLOCK_32K_SIZE.
    pub async fn erase_block_32k(&mut self, index: u32) -> Result<(), Error<S>> {
        if index >= self.n_blocks_32k() {
            return Err(Error::OutOfBounds);
        }

        self.enable_write().await?;

        let address: u32 = index * 32768;

        self.spi
            .write(&command_and_address(Command::Block32Erase as u8, address))
            .await
            .map_err(Error::SpiError)?;

        while self.busy().await? {
            // Avoid starving the executor when the SPI is
            // fast enough for the busy check to not yield
            yield_now().await;
        }

        Ok(())
    }

    /// Erases a single block of flash memory with the size of BLOCK_64K_SIZE.
    /// (0xD8).
    ///
    /// # Arguments
    /// * `index` - the index of the block that needs to be erased. The address
    ///   of the first byte of the block is the provided index * BLOCK_64K_SIZE.
    pub async fn erase_block_64k(&mut self, index: u32) -> Result<(), Error<S>> {
        if index >= self.n_blocks_64k() {
            return Err(Error::OutOfBounds);
        }

        self.enable_write().await?;

        let address: u32 = index * 65536;

        self.spi
            .write(&command_and_address(Command::Block64Erase as u8, address))
            .await
            .map_err(Error::SpiError)?;

        while self.busy().await? {
            // Avoid starving the executor when the SPI is
            // fast enough for the busy check to not yield
            yield_now().await;
        }

        Ok(())
    }

    /// Erases all sectors on the flash chip (0xC7).
    ///
    /// NOTE: This is a very expensive operation, the order of magnitude is the
    /// second.
    pub async fn erase_chip(&mut self) -> Result<(), Error<S>> {
        self.enable_write().await?;

        self.spi
            .write(&[Command::ChipErase as u8])
            .await
            .map_err(Error::SpiError)?;

        while self.busy().await? {
            // Avoid starving the executor when the SPI is
            // fast enough for the busy check to not yield
            yield_now().await;
        }

        Ok(())
    }

    /// Puts the chip into power down mode (0xB9).
    ///
    /// While in the power-down state, only the Release Power-down/Device ID
    /// (0xAB) instruction will be recognized. This instruction restores the
    /// device to normal operation. All other instructions are ignored.
    ///
    /// NOTE: The chip takes `tDP` (max 3µs) after `/CS` rises to fully enter
    /// power-down mode.  Wait at least 3µs before issuing
    /// `disable_power_down_mode`.
    pub async fn enable_power_down_mode(&mut self) -> Result<(), Error<S>> {
        self.spi
            .write(&[Command::PowerDown as u8])
            .await
            .map_err(Error::SpiError)
    }

    /// Releases the chip from power-down mode (0xAB).
    ///
    /// Restores operation from power-down mode by reading the deviceID from the
    /// device.
    ///
    /// NOTE: The chip takes `tRES1` (max 3µs) after `/CS` rises to return to
    /// standby mode.
    pub async fn disable_power_down_mode(&mut self) -> Result<(), Error<S>> {
        self.spi
            .write(&[Command::ReleasePowerDown as u8])
            .await
            .map_err(Error::SpiError)
    }

    /// Releases the chip from power-down mode and reads the 8-bit Device Id
    /// (0xAB).
    ///
    /// Restores operation from power-down mode by reading the deviceID from the
    /// device.
    ///
    /// NOTE: When reading the Electric Signature, the chip requires `tRES2`
    /// (max 1.8µs) after `/CS` rises to resume normal standby operation.
    pub async fn release_power_down_and_device_id(&mut self) -> Result<u8, Error<S>> {
        let mut buf: [u8; 5] = [Command::ReleasePowerDown as u8, 0, 0, 0, 0];

        self.spi
            .transfer_in_place(&mut buf)
            .await
            .map_err(Error::SpiError)?;

        Ok(buf[4])
    }
}

// Commands that interact with Write Protect (/WP) or Status Register 1 & 2
// writing.
impl<Series: NorSeries, SPI, S: Debug, HOLD, WP> W25<Series, SPI, HOLD, WP>
where
    SPI: SpiDevice<Error = S>,
    HOLD: PinCapabilities,
    WP: PinCapabilities,
{
    /// Writes Status Register 1 (0x01) on X-series, or preserves SR2 on
    /// Q-series.
    ///
    /// Handles hardware `/WP` unprotection automatically if `WP` is a connected
    /// pin, and prevents writing 1-byte on Q-series chips (which would
    /// accidentally clear `CMP` and `QE` in SR2).
    pub async fn write_status_register1(&mut self, sr1: u8) -> Result<(), Error<S>> {
        if Series::HAS_SR2 {
            // On Q-series, clocking only 8 bits into 0x01 clears CMP and QE in
            // SR2 to 0! Read SR2 first so we can write all 16 bits and
            // preserve SR2.
            let mut buf = [Command::ReadStatusRegister2 as u8, 0];
            self.spi
                .transfer_in_place(&mut buf)
                .await
                .map_err(Error::SpiError)?;
            let sr2 = buf[1];
            return self
                .write_status_registers_internal(&[Command::WriteStatusRegister as u8, sr1, sr2])
                .await;
        }

        self.write_status_registers_internal(&[Command::WriteStatusRegister as u8, sr1])
            .await
    }

    async fn write_status_registers_internal(&mut self, payload: &[u8]) -> Result<(), Error<S>> {
        // If QE (bit 1 of SR2) is being set to 1, datasheet warns:
        // "WARNING: If /WP or /HOLD pins are tied directly to power supply or
        // ground, QE should never be set to 1."
        if payload.len() == 3
            && (payload[2] & 0x02) != 0
            && (!HOLD::IS_IMPLEMENTED || !WP::IS_IMPLEMENTED)
        {
            return Err(Error::InvalidState);
        }

        // Ensure /WP is high before /CS falls (tWHSL = 20ns min when SRP0=1, SRP1=0)
        self.wp.set_high().map_err(|_| Error::PinError)?;
        self.enable_write().await?;

        self.spi.write(payload).await.map_err(Error::SpiError)?;

        // Poll BUSY until tw (10ms typ / 15ms max) completes
        while self.busy().await? {
            yield_now().await;
        }

        Ok(())
    }
}

// Q-Series exclusive commands
impl<SPI, S: Debug, HOLD, WP> W25<Q, SPI, HOLD, WP>
where
    SPI: SpiDevice<Error = S>,
{
    /// Read Status Register 2 (0x35, Q-series only).
    pub async fn read_status_register2(&mut self) -> Result<u8, Error<S>> {
        let mut buf: [u8; 2] = [Command::ReadStatusRegister2 as u8, 0];

        self.spi
            .transfer_in_place(&mut buf)
            .await
            .map_err(Error::SpiError)?;

        Ok(buf[1])
    }

    /// Checks the Erase/Program Suspend status bit (`SUS`, bit 7 of Status
    /// Register 2).
    pub async fn is_suspended(&mut self) -> Result<bool, Error<S>> {
        Ok((self.read_status_register2().await? & 0x80) != 0)
    }

    /// Writes both Status Register 1 and Status Register 2 (0x01, Q-series
    /// only).
    ///
    /// Returns `Error::InvalidState` if the caller attempts to set the `QE` bit
    /// (bit 1 of `sr2`) when `HOLD` or `WP` is `NoPin`, preventing a hardware
    /// bus short-circuit.
    pub async fn write_status_registers(&mut self, sr1: u8, sr2: u8) -> Result<(), Error<S>>
    where
        HOLD: PinCapabilities,
        WP: PinCapabilities,
    {
        self.write_status_registers_internal(&[Command::WriteStatusRegister as u8, sr1, sr2])
            .await
    }

    /// Suspends an ongoing Sector/Block Erase or Page Program operation (0x75,
    /// Q-series only).
    ///
    /// Edge cases & Timings:
    /// * Valid only when `BUSY == 1` and `SUS == 0` (returns
    ///   `Error::InvalidState` otherwise).
    /// * Polls until `BUSY` clears to 0 (within `tSUS` <= 20µs), guaranteeing
    ///   the chip is ready to accept read/program commands immediately upon
    ///   return.
    pub async fn erase_program_suspend(&mut self) -> Result<(), Error<S>> {
        let sr1 = self.read_status_register1().await?;
        let sr2 = self.read_status_register2().await?;

        // Must be busy (BUSY == 1) and not already suspended (SUS == 0).
        if (sr1 & 0x01) == 0 || (sr2 & 0x80) != 0 {
            return Err(Error::InvalidState);
        }

        self.spi
            .write(&[Command::EraseSuspend as u8])
            .await
            .map_err(Error::SpiError)?;

        // Wait for BUSY to clear (takes up to tSUS = 20µs).
        while self.busy().await? {
            yield_now().await;
        }

        Ok(())
    }

    /// Resumes a previously suspended Erase or Page Program operation (0x7A,
    /// Q-series only).
    ///
    /// Edge cases & Timings:
    /// * Valid only when `SUS == 1` and `BUSY == 0` (returns
    ///   `Error::InvalidState` otherwise).
    /// * NOTE: After resuming, a subsequent suspend (`0x75`) must not be issued
    ///   for at least `tSUS` (20µs).
    pub async fn erase_program_resume(&mut self) -> Result<(), Error<S>> {
        let sr1 = self.read_status_register1().await?;
        let sr2 = self.read_status_register2().await?;

        // Must not be busy (BUSY == 0) and must be suspended (SUS == 1).
        if (sr1 & 0x01) != 0 || (sr2 & 0x80) == 0 {
            return Err(Error::InvalidState);
        }

        self.spi
            .write(&[Command::EraseResume as u8])
            .await
            .map_err(Error::SpiError)
    }

    /// Puts the Q-series device into High Performance Mode (0xA3).
    ///
    /// Requires the command byte followed by 3 dummy bytes (24 clock cycles).
    pub async fn enable_high_performance_mode(&mut self) -> Result<(), Error<S>> {
        self.spi
            .write(&[Command::HighPerformanceMode as u8, 0, 0, 0])
            .await
            .map_err(Error::SpiError)
    }
}
